"""跨平台离线验证。不会导入 JAKA DLL/SO、ROS 驱动或连接硬件。"""
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path(__file__).resolve().parents[1]
PKG=ROOT/'src/v3_moveit'
sys.path.insert(0,str(PKG))
from v3_moveit.input import Clutch
from v3_moveit.coordinator import Epoch


def source_digest():
    digest=hashlib.sha256()
    files=list(PKG.rglob('*'))+list((ROOT/'vendor/jaka').rglob('*'))+list((ROOT/'quest').glob('*'))
    for path in sorted(files):
        if path.is_file() and '__pycache__' not in path.parts:
            digest.update(path.relative_to(ROOT).as_posix().encode())
            if path == PKG/'config/payload.json':
                # Load calibration is operating data. It is checked against fresh
                # cabinet reads and bound separately to physical acceptance.
                # TCP geometry still invalidates the model/software evidence.
                from v3_moveit.tool_config import tool_values
                tool,tcp=tool_values(json.loads(path.read_text()))
                digest.update(json.dumps(dict(tool_id=tool,tcp_mm_rad=tcp),sort_keys=True).encode())
            else:digest.update(path.read_bytes())
    return digest.hexdigest()


def calibration_digest(root=ROOT):
    from v3_moveit.tool_config import tool_values
    payload=json.loads((root/'src/v3_moveit/config/payload.json').read_text())
    tool,tcp=tool_values(payload)
    mass=payload['payload_kg'];cog=payload['cog_mm']
    if type(mass) not in (int,float) or not math.isfinite(mass) or not 0<=mass<=5 or not isinstance(cog,list) or len(cog)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in cog):
        raise ValueError('负载或质心配置无效，请重新读取 JAKA 校准数据')
    data=dict(tool_id=tool,tcp_mm_rad=tcp,payload_kg=mass,cog_mm=cog)
    return hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()


def packet(seq,stamp,grip=0,position=(0,0,0),rotation=(0,0,0,1)):
    return dict(sequence=seq,unity_time_s=stamp,right=dict(connected=True,tracked=True,pose_valid=True,
        position_m=list(position),rotation_xyzw=list(rotation),grip=grip,thumbstick_click=False))


def origin(e):
    t=np.eye(4)
    if e is not None:
        t[:3,3]=np.fromstring(e.get('xyz','0 0 0'),sep=' ')
        t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(e.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return t


def fk(robot,q):
    transforms={'world':np.eye(4)}
    joints=list(robot.findall('joint'))
    while joints:
        before=len(joints)
        for joint in joints[:]:
            parent=joint.find('parent').get('link')
            if parent not in transforms:continue
            t=transforms[parent]@origin(joint.find('origin'))
            if joint.get('type')=='revolute':
                axis=np.fromstring(joint.find('axis').get('xyz'),sep=' ')
                turn=np.eye(4);turn[:3,:3]=Rotation.from_rotvec(axis*q[int(joint.get('name').split('_')[-1])-1]).as_matrix()
                t=t@turn
            transforms[joint.find('child').get('link')]=t;joints.remove(joint)
        if len(joints)==before:raise ValueError('disconnected model')
    return np.linalg.inv(transforms['base_link'])@transforms['tcp']


class InputTests(unittest.TestCase):
    def setUp(self):self.c=Clutch([[0,0,1],[-1,0,0],[0,1,0]])
    def feed(self,p,now=None):return self.c.consume(p,('127.0.0.1',1),p['unity_time_s'] if now is None else now,[.4,.1,.3],np.eye(3))
    def arm(self):self.feed(packet(1,1.,0));self.feed(packet(2,1.01,1))
    def test_start_requires_release(self):self.assertIsNone(self.feed(packet(1,1,1)));self.assertFalse(self.c.held)
    def test_mapping_and_anchor(self):
        self.arm();t=self.feed(packet(3,1.02,1,[0,0,.02]));np.testing.assert_allclose(t.position,[.42,.1,.3]);np.testing.assert_allclose(t.rotation,np.eye(3))
    def test_release_reanchor(self):
        self.arm();self.feed(packet(3,1.02,0,[.02,0,0]));t=self.feed(packet(4,1.03,1,[.02,0,0]));np.testing.assert_allclose(t.position,[.4,.1,.3])
    def test_tracking_loss(self):
        self.arm();p=packet(3,1.02,1);p['right']['tracked']=False;self.assertIsNone(self.feed(p));self.assertFalse(self.c.held)
        self.assertIsNone(self.feed(packet(4,1.03,1)))
    def test_timeout_requires_release(self):self.arm();self.c.expire(1.2);self.assertIsNone(self.feed(packet(3,1.21,1)))
    def test_replay_rejected(self):
        self.arm()
        with self.assertRaises(ValueError):self.feed(packet(2,1.02,1))
        self.assertFalse(self.c.held)
    def test_delay_not_freshened(self):
        self.arm()
        with self.assertRaises(ValueError):self.feed(packet(3,1.02,1),now=1.2)
    def test_quaternion_and_nonfinite(self):
        self.arm()
        with self.assertRaises(ValueError):self.feed(packet(3,1.02,1,rotation=[0,0,0,0]))
        with self.assertRaises(ValueError):self.feed(packet(4,1.03,1,position=[float('nan'),0,0]))
    def test_jump_drops_clutch(self):self.arm();self.assertIsNone(self.feed(packet(3,1.02,1,[.2,0,0])));self.assertFalse(self.c.held)
    def test_software_stop(self):
        self.arm();p=packet(3,1.02,1);p['right']['thumbstick_click']=True;self.feed(p)
        self.assertFalse(self.c.held);self.assertTrue(self.c.stopped)
    def test_reflection_rotation_is_proper(self):
        self.arm();t=self.feed(packet(3,1.02,1,rotation=Rotation.from_rotvec([.1,.2,0]).as_quat()))
        self.assertAlmostEqual(np.linalg.det(t.rotation),1.);self.assertAlmostEqual(Rotation.from_matrix(t.rotation).magnitude(),math.sqrt(.05))


class EpochTests(unittest.TestCase):
    def test_stale_plan_rejected(self):
        e=Epoch();ticket=e.ticket([0]*6);self.assertTrue(e.accepts(ticket,[0]*6,True));e.invalidate();self.assertFalse(e.accepts(ticket,[0]*6,True))
    def test_scene_and_start_state(self):
        e=Epoch();ticket=e.ticket([0]*6);self.assertFalse(e.accepts(ticket,[.02]*6,True));self.assertFalse(e.accepts(ticket,[0]*6,False))
        e.scene_changed();self.assertFalse(e.accepts(ticket,[0]*6,True))


class LaunchPolicyTests(unittest.TestCase):
    def test_authorized_tracking_profile_and_fake_override_separation(self):
        from v3_moveit.launch_policy import tracking_config
        cfg=yaml.safe_load((PKG/'config/runtime.yaml').read_text())
        self.assertTrue(tracking_config(cfg,'hardware'))
        self.assertTrue(tracking_config(cfg,'fake'))
        self.assertFalse(tracking_config({'velocity_tracking':False},'hardware'))
        self.assertTrue(tracking_config({'velocity_tracking':False},'fake',True))
        with self.assertRaises(ValueError):tracking_config(cfg,'hardware',True)
        for value in ('true',1,None):
            with self.assertRaises(ValueError):tracking_config({'velocity_tracking':value},'hardware')


    def test_fake_threshold_does_not_change_hardware_default_or_scene(self):
        from v3_moveit.launch_policy import servo_config
        config=yaml.safe_load((PKG/'config/servo.yaml').read_text())
        before=copy.deepcopy(config)
        self.assertEqual(servo_config(config,'hardware'),before)
        tuned=servo_config(config,'fake','0.02')
        self.assertEqual(tuned['self_collision_proximity_threshold'],.02)
        self.assertEqual(tuned['scene_collision_proximity_threshold'],before['scene_collision_proximity_threshold'])
        self.assertEqual(config,before)
        for bad in ('0','-1','nan','inf','invalid'):
            with self.subTest(bad=bad),self.assertRaises(ValueError):servo_config(config,'fake',bad)
        for value in ('0.03','0.02','nan'):
            with self.subTest(hardware=value),self.assertRaises(ValueError):servo_config(config,'hardware',value)
        with self.assertRaises(ValueError):servo_config(dict(config,check_collisions=False),'fake','.02')


class ModelTests(unittest.TestCase):
    def test_exact_fk(self):
        robot=ET.parse(PKG/'description/s5.urdf').getroot()
        evidence=json.loads((ROOT/'reference/official_fk_evidence.json').read_text())['model']
        meta=json.loads((PKG/'config/model_metadata.json').read_text())
        screws=np.array(evidence['space_screws_v_w']);q0=np.array(evidence['q_reference_rad']);home=np.array(meta['tcp_home'])
        rng=np.random.default_rng(20261112);worst_p=0.;worst_r=0.
        for q in [q0]+list(rng.uniform(meta['joint_lower'],meta['joint_upper'],size=(64,6))):
            expected=np.eye(4)
            for i in range(6):
                x,y,z=screws[3:,i];a=np.zeros((4,4));a[:3,:3]=[[0,-z,y],[z,0,-x],[-y,x,0]];a[:3,3]=screws[:3,i]
                expected=expected@expm(a*(q[i]-q0[i]))
            expected=expected@home;actual=fk(robot,q)
            worst_p=max(worst_p,float(np.linalg.norm(expected[:3,3]-actual[:3,3])))
            worst_r=max(worst_r,float(Rotation.from_matrix(expected[:3,:3]@actual[:3,:3].T).magnitude()))
        self.assertLess(worst_p,1e-8);self.assertLess(worst_r,1e-8)
        print(f'FK: 65 configurations, max position error {worst_p:.3g} m, rotation {worst_r:.3g} rad')
    def test_assets_and_scenes(self):
        robot=ET.parse(PKG/'description/s5.urdf').getroot()
        self.assertEqual(len(robot.findall("joint[@type='revolute']")),6)
        self.assertGreaterEqual(len(robot.findall('.//collision')),20)
        for mesh in robot.findall('.//mesh'):
            self.assertTrue((PKG/mesh.get('filename').replace('package://v3_moveit/','')).is_file())
        scenes=json.loads((PKG/'config/scenes.json').read_text());self.assertEqual(len(scenes['s5_only']),1);self.assertGreater(len(scenes['workcell']),100)
        srdf=ET.parse(PKG/'description/s5.srdf').getroot();self.assertEqual(len(srdf.findall('disable_collisions')),7)
    def test_hand_visuals_are_not_collision_envelopes(self):
        robot=ET.parse(PKG/'description/s5.urdf').getroot()
        link=robot.find("link[@name='Link_06']")
        visuals={x.get('name') for x in link.findall('visual')}
        collisions={x.get('name') for x in link.findall('collision')}
        for n in range(1,6):
            self.assertIn(f'DH_finger{n}_sweep',collisions)
            self.assertNotIn(f'DH_finger{n}_sweep',visuals)
        self.assertEqual(len([x for x in visuals if x.startswith('DH_static_')]),16)
    def test_configs_and_python_syntax(self):
        for path in list(PKG.rglob('*.py'))+list((ROOT/'quest').glob('*.py')):ast.parse(path.read_text('utf8'),filename=str(path))
        for path in (PKG/'config').glob('*.yaml'):yaml.safe_load(path.read_text('utf8'))
        servo=yaml.safe_load((PKG/'config/servo.yaml').read_text())
        self.assertTrue(servo['check_collisions']);self.assertFalse(servo['use_smoothing'])
        self.assertEqual(servo['publish_period'],.008)
    def test_hand_mount_matches_native_1116_assembly(self):
        from update_hand_mount_20261116 import derive, CONFIG, REF, affected
        expected, evidence = derive()
        self.assertEqual((PKG/'description/s5.urdf').read_bytes(), expected)
        self.assertEqual(json.loads(CONFIG.read_text()), evidence)
        before = ET.parse(REF/'before_s5.urdf').getroot()
        after = ET.fromstring(expected)
        # Outside the hand origins every joint, TCP, rigid attachment, mesh,
        # and envelope dimension must remain identical to the audited model.
        for robot in (before, after):
            for element in robot.find("link[@name='Link_06']"):
                if affected(element):
                    element.find('origin').attrib.clear()
        self.assertEqual(ET.tostring(before), ET.tostring(after))
        self.assertAlmostEqual(evidence['rotation_change_deg'], 90., places=8)
        self.assertEqual(evidence['unchanged_components'], 8)
        self.assertEqual(len(evidence['changed_components']),58)

    def test_old_files_unchanged(self):
        manifest=json.loads((ROOT/'reference/source_manifest.json').read_text('utf8'))
        checked=0
        for name,digest in manifest.items():
            path=Path(name)
            if not path.is_file():continue # Linux 不依赖旧的 Windows 路径。
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),digest,name);checked+=1
        print(f'Read-only source identities checked: {checked}')


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__]))
    out=ROOT/'artifacts';out.mkdir(exist_ok=True)
    report=dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
        package_sha256=source_digest(),robot_connections=0,robot_commands=0,ros_built=False,
        note='Pure logic/model checks; no ROS compilation or physical acceptance implied')
    (out/'offline_verification.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    sys.exit(0 if result.wasSuccessful() else 1)
