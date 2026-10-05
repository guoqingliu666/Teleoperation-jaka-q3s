"""从 V2 已有 CAD 派生缓存构建双引擎资产；不写 F: 原始 CAD。"""
import hashlib,json,shutil,struct,sys,copy,math,xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]; TWIN=ROOT/'digital_twin'
sys.path.insert(0,str(TWIN/'.deps')); sys.path.insert(0,str(ROOT/'src'))
sys.dont_write_bytecode=True
import numpy as np
import trimesh,fast_simplification

def load_json(path): return json.loads(path.read_text(encoding='utf-8'))
def put_json(path,obj): path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf-8')
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def material_mesh(name,points,faces,visual_dir):
    points=np.asarray(points,np.float32); faces=np.asarray(faces,np.int32)
    obj=trimesh.Trimesh(vertices=points,faces=faces,process=False)
    obj.export(visual_dir/f'{name}.obj')
    with (visual_dir/f'{name}.v2mesh').open('wb') as out:
        out.write(b'V2M1'); out.write(struct.pack('<II',len(points),len(faces)))
        out.write(points.astype('<f4').tobytes()); out.write(faces.astype('<u4').tobytes())
    return obj

def extract(archive,keep,target_count):
    mask=np.isin(archive['part_index'],keep); faces=archive['triangles'][mask]
    unique,inverse=np.unique(faces,return_inverse=True)
    points=archive['vertices_mm'][unique]/1000.; faces=inverse.reshape(-1,3).astype(np.int32)
    before=len(faces)
    if before>target_count:
        points,faces=fast_simplification.simplify(points,faces,target_count=target_count,agg=5)
    return points,faces,before

def main():
    for folder in ('meshes/visual','meshes/collision','config','calibration','materials','unity','mujoco','urdf','usd','gltf','docs','tests','figures','data'):
        (TWIN/folder).mkdir(parents=True,exist_ok=True)
    parts=load_json(ROOT/'models/scene/assembly_parts.json')['parts']
    binding=load_json(ROOT/'config/scene_binding.json')
    source=ROOT/'models/scene/assembly_mesh.npz'
    archive=np.load(source)
    assert len(parts)==443 and len(archive['triangles'])==1490441
    classes={'cabinet_shell':[],'workcell_fixtures':[],'legacy_neighbor_placeholder':[],'excluded_left_TX2':[]}
    mapping=[]
    for part in parts:
        name=part['name']; idx=part['index']
        if '/TX2-90-VB-WS____instance_2/' in name: kind='legacy_neighbor_placeholder'
        elif '/TX2-90-VB-WS__/' in name: kind='excluded_left_TX2'
        elif idx in (0,442): kind='cabinet_shell'
        else: kind='workcell_fixtures'
        classes[kind].append(idx)
        mapping.append({'index':idx,'source_name':name,'derived_class':kind,'bbox_mm':part['bbox_mm'],
                        'source_triangles':part['triangles'],'brep_valid':part['valid_brep'],
                        'physical_identity_confirmed':False,'collision_approved':False})
    # Full near-field geometry; the prior global decimation erased thin frames and neighboring-arm details.
    visuals={}; meshes={}; limits={'cabinet_shell':90000,'workcell_fixtures':1000000,'legacy_neighbor_placeholder':500000}
    for category,budget in limits.items():
        points,faces,before=extract(archive,classes[category],budget)
        meshes[category]=material_mesh(category,points,faces,TWIN/'meshes/visual')
        visuals[category]={'part_indices':classes[category],'source_triangles':before,'export_triangles':len(faces),
            'bounds_m':meshes[category].bounds.tolist(),'sha256_obj':sha(TWIN/'meshes/visual'/f'{category}.obj')}
        print(category,before,'->',len(faces),flush=True)
        if category in ('workcell_fixtures','legacy_neighbor_placeholder'):
            lod_budget=190000 if category=='workcell_fixtures' else 50000
            low_points,low_faces,_=extract(archive,classes[category],lod_budget)
            material_mesh(category+'_lod1',low_points,low_faces,TWIN/'meshes/visual')
            visuals[category]['lod1_triangles']=len(low_faces)
    arm=[]; cad_mesh=TWIN/'cad/derived/s5_player_source'
    if not (cad_mesh/'source_manifest.json').is_file():
        raise FileNotFoundError('Copy the old ① Player JakaS5 first: python digital_twin/cad/copy_s5_player_source.py')
    for i in range(7):
        name=f'Link_{i:02}'; src=cad_mesh/f'{name}.STL'
        if sha(src)!=sha(ROOT/'models/arm'/f'{name}.STL'):
            raise RuntimeError(f'Old V2 {name} differs from user-specified ① Player; inspect first')
        dst=TWIN/'meshes/visual'/f'{name}.stl'; shutil.copy2(src,dst)
        # Physically separate file, same source geometry; MuJoCo convexifies mesh contacts.
        shutil.copy2(src,TWIN/'meshes/collision'/f'{name}.stl')
        mesh=trimesh.load_mesh(src,process=True) # weld STL duplicate vertices for visual normals only
        material_mesh(name,mesh.vertices,mesh.faces,TWIN/'meshes/visual')
        arm.append({'name':name,'source_sha256':sha(src),'triangles':len(mesh.faces),
                    'render_vertices_welded':len(mesh.vertices),'bbox_m':mesh.bounds.tolist(),
                    'collision_type':'MuJoCo convex hull of copied source STL; unverified site clearance'})
    pose=np.load(ROOT/'models/scene/left_s5_preview_mesh.npz')
    pose_mesh=material_mesh('S5_saved_pose_preview',pose['vertices_mm']/1000,pose['triangles'],TWIN/'meshes/visual')
    original=ET.parse(cad_mesh/'jaka_s5.urdf').getroot()
    preview=ET.parse(ROOT/'models/left_s5_preview.urdf').getroot()
    joints=[]; inertials={}
    for i in range(7):
        link=original.find(f"link[@name='Link_{i:02}']")
        ine=link.find('inertial')
        if ine is None: raise ValueError(f'missing sourced inertial Link_{i:02}')
        inertials[f'Link_{i:02}']={'origin_xyz_m':[float(x) for x in ine.find('origin').get('xyz').split()],
            'mass_kg':float(ine.find('mass').get('value')),'matrix_kg_m2':ine.find('inertia').attrib,
            'provenance':'old ① Player JakaS5/jaka_s5.urdf; copied for simulation, not site-verified'}
    for i in range(1,7):
        j=preview.find(f"joint[@name='joint_{i}']")
        source_joint=original.find(f"joint[@name='joint_{i}']")
        for field,keys in [('origin',('xyz','rpy')),('axis',('xyz',)),('limit',('lower','upper'))]:
            if source_joint is None or source_joint.find(field) is None:
                raise RuntimeError(f'Old ① Player URDF missing joint_{i} {field}')
            for key in keys:
                a=[float(x) for x in j.find(field).get(key).split()]
                b=[float(x) for x in source_joint.find(field).get(key).split()]
                if not np.allclose(a,b,atol=1e-7):
                    raise RuntimeError(f'V2 joint_{i} {field}.{key} differs from old ① Player')
        joints.append({'name':j.get('name'),'parent':j.find('parent').get('link'),'child':j.find('child').get('link'),
            'origin_xyz_m':[float(x) for x in j.find('origin').get('xyz').split()],
            'origin_rpy_rad':[float(x) for x in j.find('origin').get('rpy').split()],
            'axis_local':[float(x) for x in j.find('axis').get('xyz').split()],
            'lower_rad':float(j.find('limit').get('lower')),'upper_rad':float(j.find('limit').get('upper')),
            'source':'old ① Player StreamingAssets/JakaS5/jaka_s5.urdf; V2 preview base orientation unverified'})
    base=np.array(binding['preview_S5_base_to_cad_matrix_m'],float)
    shell=visuals['cabinet_shell']['bounds_m']
    # CAD shell bounds determine camera placement; these are viewpoints, not measured camera installations.
    cams={'inside_operator':{'position_cad_m':[.15,-.65,1.38],'look_at_cad_m':[.15,.02,.95]},
          'inside_fixed':{'position_cad_m':[.70,-.62,1.46],'look_at_cad_m':[.10,.00,.90]},
          'free_start':{'position_cad_m':[-.40,-.60,1.40],'look_at_cad_m':[.10,.00,.90]},
          'tcp_follow_local_placeholder':{'position_link06_m':[0.,-.35,.18],'look_at_link06_m':[0.,0.,0.]}}
    cfg={'schema':1,'units':'m','cad_world':'right-handed XYZ; Z up','unity_basis_from_cad':[[1,0,0],[0,0,1],[0,1,0]],
         'scope':'one actuated leftmost S5; other robot geometry is a static unverified legacy TX2 placeholder',
         'cad_source':'models/scene/source/布局装配.STEP','cad_source_sha256':sha(ROOT/'models/scene/source/布局装配.STEP'),
         'cad_mesh_cache_sha256':sha(source),'part_count':len(parts),'s5_serial':'S5160217',
         's5_player_urdf_sha256':sha(cad_mesh/'jaka_s5.urdf'),
         's5_base_preview_cad_m':base.tolist(),'base_translation_user_confirmed':True,'base_rotation_verified':False,
         'initial_joint_sample_rad':load_json(ROOT/'models/scene/left_s5_binding_report.json')['q_rad_from_saved_readonly'],
         'joint_order':[j['name'] for j in joints],'joints':joints,'inertials':inertials,
         'camera_views':cams,'cabinet_shell_bounds_m':shell,
         'tool1_tcp_mm_rad':load_json(ROOT/'config/site_observed_20260928.json')['tcp_mm_rad'],
         'tool1_tcp_relative_to_model_link06_verified':False,
         'zed':{'model':'ZED-M','serial':12474260,'firmware':1523,'mount':'eye-in-hand',
                'link06_to_camera_m':None,'calibration_transform_direction_and_units_verified':False},
         'site_collision_approved':False,'sim_collision_proxy':'cabinet bounding surfaces + arm source-mesh convex hull; not real safety geometry',
         'sim_actuator_kp':'simulation-only 80 N*m/rad position gain; not identified hardware gain'}
    put_json(TWIN/'config/twin.json',cfg)
    put_json(TWIN/'config/import_map.json',{'source_step_sha256':cfg['cad_source_sha256'],
        'duplicate_aliases':load_json(ROOT/'models/scene/assembly_parts.json')['instance_aliases'],
        'unresolved_brep_indices':[p['index'] for p in parts if not p['valid_brep']],
        'left_TX2_removed_from_visual':classes['excluded_left_TX2'],'parts':mapping})
    put_json(TWIN/'calibration/status.json',{'base_position':'user-confirmed reused old left mount CAD datum',
        'base_orientation':'preview assumption +90deg Z; not site validated','s5_joint_zero':'source URDF only, site zero unverified',
        'link06_to_controller_flange':'unknown','tool1_tcp':'read-only controller value, Link06 relation unverified',
        'zed_hand_eye':'3 user matrices present; direction/units unknown; disabled'})
    twin_urdf=copy.deepcopy(preview); twin_urdf.set('name','V2_left_S5_cabinet_preview_not_site_collision_approved')
    for i in range(7):
        name=f'Link_{i:02}'; link=twin_urdf.find(f"link[@name='{name}']")
        link.insert(0,copy.deepcopy(original.find(f"link[@name='{name}']").find('inertial')))
        for visual in link.findall('visual'):
            visual.find('geometry/mesh').set('filename',f'../meshes/visual/{name}.stl')
        for collision in link.findall('collision'):
            collision.find('geometry/mesh').set('filename',f'../meshes/collision/{name}.stl')
    world=twin_urdf.find("link[@name='world']")
    for category,rgba in [('cabinet_shell','0.55 0.73 0.8 0.18'),('workcell_fixtures','0.6 0.65 0.7 1'),
                          ('legacy_neighbor_placeholder','0.95 0.56 0.18 1')]:
        visual=ET.SubElement(world,'visual',name=category+'_CAD_preview'); ET.SubElement(visual,'origin',xyz='0 0 0',rpy='0 0 0')
        ET.SubElement(ET.SubElement(visual,'geometry'),'mesh',filename=f'../meshes/visual/{category}.obj')
        ET.SubElement(ET.SubElement(visual,'material',name=category),'color',rgba=rgba)
    lo,hi=np.array(shell); floor=ET.SubElement(world,'collision',name='cabinet_floor_sim_proxy')
    ET.SubElement(floor,'origin',xyz=' '.join(map(str,[(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2]-.01])),rpy='0 0 0')
    ET.SubElement(ET.SubElement(floor,'geometry'),'box',size=' '.join(map(str,[hi[0]-lo[0],hi[1]-lo[1],.02])))
    ET.indent(twin_urdf); ET.ElementTree(twin_urdf).write(TWIN/'urdf/workcell.urdf',encoding='utf-8',xml_declaration=True)
    def xyz(values): return dict(zip(('x','y','z'),[float(v) for v in values]))
    unity={'schema':1,'cad_source_sha256':cfg['cad_source_sha256'],'s5_serial':cfg['s5_serial'],
        'base_position_cad_m':xyz(base[:3,3]),
        'base_rpy_cad_rad':xyz([0.,0.,math.atan2(base[1,0],base[0,0])]),
        'initial_joint_sample_rad':cfg['initial_joint_sample_rad'],
        'joints':[{'name':j['name'],'origin_xyz_cad_m':xyz(j['origin_xyz_m']),
                   'origin_rpy_cad_rad':xyz(j['origin_rpy_rad']),
                   'axis_local_cad':xyz(j['axis_local']),'lower_rad':j['lower_rad'],'upper_rad':j['upper_rad']}
                  for j in joints],
        'views':[{'name':name,'position_cad_m':xyz(value['position_cad_m']),
                  'look_at_cad_m':xyz(value['look_at_cad_m'])}
                 for name,value in cams.items() if 'position_cad_m' in value],
        'shell_min_cad_m':xyz(lo),'shell_max_cad_m':xyz(hi),
        'status_udp_port':5007,'vr_input_udp_port':5006,'site_collision_approved':False,
        'zed_serial':12474260,'zed_mount':'eye-in-hand','zed_extrinsic_verified':False}
    put_json(TWIN/'unity/Assets/V2Twin/Data/twin_unity.json',unity)
    qdu_sender=Path('F:/规划/工程/设备/VR/QuestPoseBridge/Assets/VrPoseUdpSender.cs')
    if qdu_sender.is_file():
        sender=qdu_sender.read_text(encoding='utf-8').replace('private int port = 5005;', 'private int port = 5006;')
        if 'private int port = 5006;' not in sender: raise RuntimeError('QDU sender UDP port declaration changed; inspect before adapting')
        sender_dest=TWIN/'unity/Assets/V2Twin/Scripts/VrPoseUdpSender.cs';sender_dest.parent.mkdir(parents=True,exist_ok=True)
        sender_dest.write_text(sender,encoding='utf-8')
        put_json(TWIN/'unity/Assets/V2Twin/Data/qdu_sender_provenance.json',
                 {'source':str(qdu_sender),'sha256_original':sha(qdu_sender),'adaptation':'default UDP port 5005 -> 5006',
                  'license':'not located; local research use pending provenance/license confirmation'})
    (TWIN/'unity/ProjectSettings').mkdir(parents=True,exist_ok=True)
    (TWIN/'unity/ProjectSettings/ProjectVersion.txt').write_text('m_EditorVersion: 2022.3.62f3\n',encoding='utf-8')
    put_json(TWIN/'unity/Packages/manifest.json',{'dependencies':{
        'com.unity.xr.management':'4.7.0','com.unity.xr.openxr':'1.14.3',
        'com.unity.ugui':'1.0.0','com.unity.modules.xr':'1.0.0',
        'com.unity.modules.screencapture':'1.0.0'}})
    # A static glTF is for interchange/visual verification. Dynamic articulation is defined by twin.json/URDF/MJCF.
    gltf=trimesh.Scene()
    for category,mesh in meshes.items(): gltf.add_geometry(mesh,geom_name=category,node_name=category)
    gltf.add_geometry(pose_mesh,geom_name='S5_saved_pose_preview',node_name='S5_saved_pose_preview')
    gltf.export(TWIN/'gltf/workcell_static.glb')
    from build_mujoco import build as build_mujoco
    from build_usd import build as build_usd
    if (TWIN/'cad/derived/complete_end_effector/assembly.json').exists():
        from hand_assets import prepare,export_static
        prepare();build_mujoco(cfg,TWIN);export_static()
    else:build_mujoco(cfg,TWIN);build_usd(cfg,TWIN)
    report={'source_parts':len(parts),'source_triangles':len(archive['triangles']),'source_invalid_brep':[p['index'] for p in parts if not p['valid_brep']],
        'duplicate_aliases':len(load_json(ROOT/'models/scene/assembly_parts.json')['instance_aliases']),
        'visual_assets':visuals,'arm':arm,'controlled_chains':1,'site_collision_approved':False,
        'sha256_config':sha(TWIN/'config/twin.json')}
    put_json(TWIN/'data/build_report.json',report)
    print(json.dumps({'parts':len(parts),'faces':{k:v['export_triangles'] for k,v in visuals.items()},'arm_links':len(arm)},ensure_ascii=False))

if __name__=='__main__': main()
