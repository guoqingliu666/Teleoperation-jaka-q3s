"""Expand real ROS launch actions without starting any nodes or loading the SDK."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from operator_settings import ROOT


def hardware_parameters(root, runtime):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.actions import Node
    from hardware_hold_check import launch_source as hold
    from hardware_quest_commissioning import launch_source as commissioning

    source=root/'src/v3_moveit/launch/bringup.launch.py'
    spec=importlib.util.spec_from_file_location('operator_bringup',source)
    bringup=importlib.util.module_from_spec(spec);spec.loader.exec_module(bringup)
    descriptions=[]
    node_init=Node.__init__
    def capture(node,**kwargs):
        if kwargs.get('package')=='robot_state_publisher':
            descriptions.append(kwargs['parameters'][0]['robot_description'])
        node_init(node,**kwargs)

    def expand(description,overrides):
        context=LaunchContext();context.launch_configurations.update(overrides)
        descriptions.clear()
        for action in description.entities:
            if isinstance(action,(DeclareLaunchArgument,OpaqueFunction)):action.execute(context)
        assert len(descriptions)==1
        hw=ET.fromstring(descriptions[0]).find('ros2_control/hardware')
        assert hw.findtext('plugin')=='v3_moveit/JakaSystem'
        return {param.attrib['name']:param.text for param in hw.findall('param')}

    payload=root/'src/v3_moveit/config/payload.json'
    config=dict(mode='hardware',runtime_config=str(runtime),rviz='false')
    audit=root/'.tmp/launch-test.jsonl'
    with patch.object(Node,'__init__',capture):
        result={'teleop':expand(bringup.generate_launch_description(),dict(config,payload_config=str(payload)))}
        for name,code in [('hold',hold(root,True,audit)),('commission',commissioning(root,True,runtime,audit,False))]:
            namespace={};exec(code,namespace)
            result[name]=expand(namespace['generate_launch_description'](),{})
    return result


@unittest.skipUnless(importlib.util.find_spec('launch_ros'),'ROS Jazzy environment required')
class LaunchPayload(unittest.TestCase):
    def test_refreshed_payload_reaches_every_driver_launch_without_reinstall(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'.tmp') as tmp:
            root=Path(tmp);pkg=root/'src/v3_moveit';(pkg/'config').mkdir(parents=True)
            (pkg/'launch').symlink_to(ROOT/'src/v3_moveit/launch')
            site=root/'site_ubuntu.yaml';site.write_text((ROOT/'site_ubuntu.yaml').read_text())
            fresh=dict(tool_id=3,tcp_mm_rad=[1,2,180,0,0,0],payload_kg=1.234567891234,cog_mm=[-1.5,2.25,60.1234567])
            (pkg/'config/payload.json').write_text(json.dumps(fresh))
            with patch.dict(os.environ,V3_PAYLOAD_CONFIG='/unused/stale-payload.json'):
                params=hardware_parameters(root,site)
            for name,p in params.items():
                with self.subTest(launch=name):
                    self.assertEqual(float(p['payload_kg']),fresh['payload_kg'])
                    self.assertEqual([float(p[f'cog_{i}']) for i in range(3)],fresh['cog_mm'])
                    self.assertEqual(int(p['tool_id']),fresh['tool_id'])
                    self.assertEqual([float(p[f'tool_tcp_{i}']) for i in range(6)],fresh['tcp_mm_rad'])
