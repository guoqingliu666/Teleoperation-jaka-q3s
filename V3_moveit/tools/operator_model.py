"""Build a display-only articulated hand model; leave the control URDF untouched."""
import copy,hashlib,json,os,xml.etree.ElementTree as ET
from pathlib import Path
from operator_settings import ROOT,atomic
from prepare_assets import origin,set_origin
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
HAND=ROOT/'vendor/dh116/DH116-R000-A1'
def generate():
    robot=ET.parse(ROOT/'src/v3_moveit/description/s5.urdf').getroot();hand=ET.parse(HAND/'urdf/DH116-R000-A1.urdf').getroot();wrist=robot.find("link[@name='Link_06']")
    from operator_tooling import apply_tool
    robot,_=apply_tool(robot,json.loads((ROOT/'src/v3_moveit/config/model_metadata.json').read_text()),json.loads((ROOT/'src/v3_moveit/config/payload.json').read_text()))
    wrist=robot.find("link[@name='Link_06']")
    palm=wrist.find("visual[@name='DH_base_link_visual']");vendor_palm=hand.find("link[@name='base_link']/visual")
    mount=origin(palm.find('origin'))@np.linalg.inv(origin(vendor_palm.find('origin')))
    for visual in list(wrist.findall('visual')):
        if visual.get('name','').startswith('DH_static_') or visual is palm:wrist.remove(visual)
    for link in hand.findall('link'):
        link=copy.deepcopy(link);link.set('name','DH_'+link.get('name'))
        for collision in list(link.findall('collision')):link.remove(collision)
        for mesh in link.findall('.//mesh'):
            mesh.set('filename',(HAND/mesh.get('filename').split('package://DH116-R000-A1/')[1]).as_uri())
        robot.append(link)
    for joint in hand.findall('joint'):
        joint=copy.deepcopy(joint);joint.set('name','DH_'+joint.get('name'))
        for tag in ('parent','child'):joint.find(tag).set('link','DH_'+joint.find(tag).get('link'))
        mimic=joint.find('mimic')
        if mimic is not None:mimic.set('joint','DH_'+mimic.get('joint'))
        robot.append(joint)
    joint=ET.SubElement(robot,'joint',name='DH_display_mount',type='fixed');ET.SubElement(joint,'parent',link='Link_06');ET.SubElement(joint,'child',link='DH_base_link');set_origin(joint,mount)
    scene_id=yaml.safe_load((ROOT/'src/v3_moveit/config/runtime.yaml').read_text())['scene']
    for obj in json.loads((ROOT/'src/v3_moveit/config/scenes.json').read_text())[scene_id]:
        name='display_scene_'+obj['id'];link=ET.SubElement(robot,'link',name=name);visual=ET.SubElement(link,'visual');visual.append(ET.fromstring(obj['geometry']))
        material=ET.SubElement(visual,'material',name='environment');ET.SubElement(material,'color',rgba='0.21 0.29 0.37 0.45')
        joint=ET.SubElement(robot,'joint',name=name+'_fixed',type='fixed');ET.SubElement(joint,'parent',link='world');ET.SubElement(joint,'child',link=name)
        transform=np.eye(4);transform[:3,3]=obj['xyz'];transform[:3,:3]=Rotation.from_quat(obj['xyzw']).as_matrix();set_origin(joint,transform)
    path=ROOT/'.cache/operator/display.urdf';path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.urdf.tmp');ET.ElementTree(robot).write(temporary,encoding='utf-8',xml_declaration=True);os.replace(temporary,path)
    atomic(ROOT/'artifacts/operator_model.json',dict(display_only=True,hand_urdf=str(HAND/'urdf/DH116-R000-A1.urdf'),control_urdf_sha256=hashlib.sha256((ROOT/'src/v3_moveit/description/s5.urdf').read_bytes()).hexdigest(),mount_transform=mount.tolist()))
    return path
if __name__=='__main__':print(generate())
