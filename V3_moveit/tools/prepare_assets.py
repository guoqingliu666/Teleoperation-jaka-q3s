"""从旧工程只读复制资产到 V3。不会改动 V2，也不连接机器人。"""
import argparse
import copy
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/v3_moveit'


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def transform(pos=(0, 0, 0), rotation=None):
    t = np.eye(4)
    t[:3, 3] = pos
    if rotation is not None:
        t[:3, :3] = rotation
    return t


def origin(element):
    if element is None:
        return np.eye(4)
    return transform(np.fromstring(element.get('xyz', '0 0 0'), sep=' '),
                     Rotation.from_euler('xyz', np.fromstring(element.get('rpy', '0 0 0'), sep=' ')).as_matrix())


def set_origin(element, t):
    ET.SubElement(element, 'origin', xyz=' '.join(format(x, '.16g') for x in t[:3, 3]),
                  rpy=' '.join(format(x, '.16g') for x in Rotation.from_matrix(t[:3, :3]).as_euler('xyz')))


def add_hand_visuals(robot, v2, evidence):
    """Render the original hand at its static open pose; collision stays intact.

    The five full-travel envelopes are collision geometry only. This is a
    visual preview, not measured hand joint feedback or finger actuation.
    """
    source = v2 / 'digital_twin/urdf/workcell.urdf'
    old = ET.parse(source).getroot()
    evidence[str(source)] = sha(source)
    assembly = v2 / 'digital_twin/cad/derived/complete_end_effector/assembly.json'
    evidence[str(assembly)] = sha(assembly)
    evidence[str(v2 / 'digital_twin/hand_assets.py')] = sha(v2 / 'digital_twin/hand_assets.py')
    target = robot.find("link[@name='Link_06']")
    # Reuse the exact existing CAD-to-SDK registration from the palm visual.
    base = target.find("visual[@name='DH_base_link_visual']")
    old_base = old.find("link[@name='DH_base_link']/visual")
    mount = old.find("joint[@name='DH_hand_mount']")
    registration = origin(base.find('origin')) @ np.linalg.inv(
        origin(mount.find('origin')) @ origin(old_base.find('origin')))
    angles = {'DH_finger13': .004592, **{f'DH_finger{n}2': .011072 for n in (2,3,4,5)}}
    frames = {'Link_06': np.eye(4)}
    pending = [j for j in old.findall('joint') if j.get('name', '').startswith('DH_')]
    while pending:
        count = len(pending)
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in frames: continue
            local = origin(joint.find('origin'))
            if joint.get('type') != 'fixed':
                q = angles.get(joint.get('name'), 0.)
                limits = joint.find('limit')
                if not float(limits.get('lower')) <= q <= float(limits.get('upper')):
                    raise ValueError('Open hand pose outside source visual joint bounds')
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                local = local @ transform(rotation=Rotation.from_rotvec(axis*q).as_matrix())
            frames[joint.find('child').get('link')] = frames[parent] @ local
            pending.remove(joint)
        if count == len(pending): raise ValueError('Disconnected source hand visual chain')
    for elem in list(target.findall('visual')):
        name = elem.get('name', '')
        if name in {f'DH_finger{i}_sweep' for i in range(1,6)} or name.startswith('DH_static_'):
            target.remove(elem)
    added = []
    for link in old.findall('link'):
        name = link.get('name', '')
        if not name.startswith('DH_finger'): continue
        for visual in link.findall('visual'):
            elem = copy.deepcopy(visual)
            elem.set('name', 'DH_static_'+name.removeprefix('DH_'))
            local = registration @ frames[name] @ origin(visual.find('origin'))
            if elem.find('origin') is not None: elem.remove(elem.find('origin'))
            set_origin(elem, local)
            for mesh in elem.findall('.//mesh'):
                src = (source.parent / mesh.get('filename')).resolve()
                filename = sha(src)[:12]+'_'+src.name
                dest = PKG / 'description/meshes' / filename
                dest.parent.mkdir(parents=True, exist_ok=True)
                if not dest.exists(): shutil.copy2(src, dest)
                if sha(src) != sha(dest): raise ValueError('Hand visual mesh copy differs')
                evidence[str(src)] = sha(src)
                mesh.set('filename', 'package://v3_moveit/description/meshes/'+filename)
            material = elem.find('material')
            if material is not None: elem.remove(material)
            ET.SubElement(ET.SubElement(elem, 'material', name='hand_dark'), 'color', rgba='.18 .20 .24 1')
            target.append(elem); added.append(elem.get('name'))
    if len(added) != 16: raise ValueError('Expected 16 original finger visual meshes')
    return added


def export_visuals_only(v2):
    path = PKG / 'description/s5.urdf'
    robot = ET.parse(path).getroot()
    def protected(tree):
        result = copy.deepcopy(tree)
        for link in result.findall('link'):
            for elem in list(link.findall('visual')):
                name = elem.get('name', '')
                if name in {f'DH_finger{i}_sweep' for i in range(1,6)} or name.startswith('DH_static_'):
                    link.remove(elem)
        ET.indent(result)
        return ET.tostring(result)
    before = protected(robot)
    manifest_path = ROOT / 'reference/source_manifest.json'
    evidence = json.loads(manifest_path.read_text())
    added = add_hand_visuals(robot, v2, evidence)
    if before != protected(robot): raise ValueError('Visual export changed protected robot elements')
    ET.indent(robot)
    ET.ElementTree(robot).write(path, encoding='utf-8', xml_declaration=True)
    manifest_path.write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
    print('Restored static open-hand visuals:', len(added), '; collision/joints/TCP/other visuals unchanged')


def export(v2, sdk):
    evidence = {}
    def remember(path):
        evidence[str(path)] = sha(path)
    def copied(source, dest):
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
        remember(source)
        if sha(source) != sha(dest):
            raise ValueError('Copy hash mismatch: ' + str(source))

    model_ref = json.loads((v2 / 'config/local/continuous_model.json').read_text('utf8'))
    model_path = v2 / model_ref['path']
    if sha(model_path) != model_ref['sha256']:
        raise ValueError('V2 identified-model hash mismatch')
    report = json.loads(model_path.read_text('utf8'))
    if not report['passed_math_only'] or report['motion_commands'] != 0:
        raise ValueError('Unverified source kinematics')
    copied(model_path, ROOT / 'reference/official_fk_evidence.json')
    nominal_path = v2 / 'digital_twin/urdf/workcell.urdf'
    remember(nominal_path)
    old = ET.parse(nominal_path).getroot()
    qref = np.array(report['model']['q_reference_rad'])
    screws = np.array(report['model']['space_screws_v_w'])
    old_tool = report['tcp_parameters']
    old_tool_t = transform(np.array(old_tool[:3]) / 1000, Rotation.from_euler('xyz', old_tool[3:]).as_matrix())
    tcp = np.array(report['model']['T_reference_m']) @ np.linalg.inv(old_tool_t) @ transform([0, 0, .2])
    # 求 nominal CAD 基坐标到厂家基坐标的刚体配准。它只是模型配准，不是现场标定。
    cad = [np.eye(4)]
    cad_axes, cad_points = [], []
    for i in range(6):
        joint = old.find(f"joint[@name='joint_{i+1}']")
        t = cad[-1] @ origin(joint.find('origin'))
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        cad_axes.append(t[:3, :3] @ axis)
        cad_points.append(t[:3, 3].copy())
        cad.append(t @ transform(rotation=Rotation.from_rotvec(axis * qref[i]).as_matrix()))
    ca = np.array(cad_axes).T
    oa = screws[3:]
    u, _, vt = np.linalg.svd(oa @ ca.T)
    reg_r = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    def skew(w):
        x, y, z = w
        return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    lhs, rhs = [], []
    for i in range(6):
        lhs.append(-skew(oa[:, i]))
        rhs.append(screws[:3, i] - reg_r @ np.cross(cad_points[i], ca[:, i]))
    reg_t, *_ = np.linalg.lstsq(np.vstack(lhs), np.concatenate(rhs), rcond=None)
    registration = transform(reg_t, reg_r)
    axis_error = float(np.max(np.linalg.norm(reg_r @ ca - oa, axis=0)))
    line_error = float(np.max(np.abs(np.vstack(lhs) @ reg_t - np.concatenate(rhs))))
    if axis_error > .03 or line_error > .02:
        raise ValueError(f'CAD/SDK registration needs investigation: axes={axis_error}, lines={line_error}')
    # 精确重建已用官方 FK 验证的 PoE 关节链，不把 CAD 预览尺寸当控制运动学。
    frames = [np.eye(4)]
    for i in range(6):
        w, v = screws[3:, i], screws[:3, i]
        nominal = registration @ cad[i+1]
        point = np.cross(w, v) + w * np.dot(w, nominal[:3, 3])
        x = nominal[:3, 0] - w * np.dot(w, nominal[:3, 0])
        if np.linalg.norm(x) < 1e-6:
            x = nominal[:3, 1] - w * np.dot(w, nominal[:3, 1])
        x /= np.linalg.norm(x)
        frames.append(transform(point, np.column_stack([x, np.cross(w, x), w])))

    robot = ET.Element('robot', name='v3_s5')
    links = {name: ET.SubElement(robot, 'link', name=name)
             for name in ['world', 'base_link'] + [f'Link_{i:02d}' for i in range(1, 7)] + ['tcp']}
    world_from_cad = origin(old.find("joint[@name='world_to_link_00']/origin"))
    world_from_robot = world_from_cad @ np.linalg.inv(registration)
    fixed = ET.SubElement(robot, 'joint', name='world_to_base', type='fixed')
    ET.SubElement(fixed, 'parent', link='world'); ET.SubElement(fixed, 'child', link='base_link')
    set_origin(fixed, world_from_robot)
    cfg = json.loads((v2 / 'config/default.json').read_text('utf8'))
    limits = cfg['limits']
    for i in range(6):
        j = ET.SubElement(robot, 'joint', name=f'joint_{i+1}', type='revolute')
        ET.SubElement(j, 'parent', link='base_link' if i == 0 else f'Link_{i:02d}')
        ET.SubElement(j, 'child', link=f'Link_{i+1:02d}')
        t = np.linalg.inv(frames[i]) @ frames[i+1] @ transform(rotation=Rotation.from_rotvec([0, 0, -qref[i]]).as_matrix())
        set_origin(j, t)
        ET.SubElement(j, 'axis', xyz='0 0 1')
        ET.SubElement(j, 'limit', lower=str(limits['lower_rad'][i]), upper=str(limits['upper_rad'][i]),
                      velocity=str(np.deg2rad(10)), effort='2000')
    j = ET.SubElement(robot, 'joint', name='tool_mount', type='fixed')
    ET.SubElement(j, 'parent', link='Link_06'); ET.SubElement(j, 'child', link='tcp')
    set_origin(j, np.linalg.inv(frames[6]) @ tcp)

    # 旧场景的凸分解、手指全行程包络和末端安装件保持，不从视觉图片猜尺寸。
    scene_files = {}
    count = {}
    for scene_id, rel in [('s5_only', 's5_only/workcell_runtime.xml'), ('workcell', 'workcell_runtime.xml')]:
        scene_path = v2 / 'digital_twin/collision/derived' / rel
        remember(scene_path)
        scene = ET.parse(scene_path).getroot()
        assets = {x.get('name'): x for x in scene.findall('asset/mesh')}
        mesh_root = (scene_path.parent / scene.find('compiler').get('meshdir')).resolve()
        objects = []
        def geometry(g):
            typ = g.get('type', 'sphere')
            size = np.fromstring(g.get('size', '0'), sep=' ')
            geo = ET.Element('geometry')
            if typ == 'mesh':
                asset = assets[g.get('mesh')]
                source = mesh_root / asset.get('file')
                filename = sha(source)[:12] + '_' + source.name
                dest = PKG / 'description/meshes' / filename
                if not dest.exists(): copied(source, dest)
                else: remember(source)
                ET.SubElement(geo, 'mesh', filename='package://v3_moveit/description/meshes/' + filename,
                              scale=asset.get('scale', '1 1 1'))
            elif typ == 'box': ET.SubElement(geo, 'box', size=' '.join(map(str, 2 * size)))
            elif typ == 'sphere': ET.SubElement(geo, 'sphere', radius=str(size[0]))
            elif typ == 'cylinder': ET.SubElement(geo, 'cylinder', radius=str(size[0]), length=str(2 * size[1]))
            else: raise ValueError('Unsupported collision primitive: ' + typ)
            return geo
        def geom_t(g):
            qw = np.fromstring(g.get('quat', '1 0 0 0'), sep=' ')
            return transform(np.fromstring(g.get('pos', '0 0 0'), sep=' '), Rotation.from_quat(qw[[1,2,3,0]]).as_matrix())
        def visit(body, nominal_t, owner):
            name = body.get('name', '')
            if name.startswith('Link_') and len(name) == 7:
                owner = int(name[-2:])
                nominal_t = cad[owner]
            for g in body.findall('geom'):
                gn = g.get('name', '')
                active = ((gn.startswith('Link_') and gn.endswith('_contact_proxy')) or gn.startswith('DH_'))
                if not active or scene_id != 's5_only': continue
                target = links['base_link' if owner == 0 else f'Link_{owner:02d}']
                local = np.linalg.inv(frames[owner]) @ registration @ nominal_t @ geom_t(g)
                for tag in (('collision',) if gn in {f'DH_finger{i}_sweep' for i in range(1,6)} else ('collision', 'visual')):
                    elem = ET.SubElement(target, tag, name=gn)
                    set_origin(elem, local); elem.append(geometry(g))
                    if tag == 'visual':
                        mat = ET.SubElement(elem, 'material', name='arm_blue')
                        ET.SubElement(mat, 'color', rgba='0.15 0.50 0.72 1')
            for child in body.findall('body'):
                visit(child, nominal_t @ geom_t(child), owner)
        for body in scene.findall('worldbody/body'): visit(body, np.eye(4), 0)
        # 世界障碍独立进入 PlanningScene，可更换场景而不改变关节链。
        def collect_world(body, parent_t):
            t = parent_t @ geom_t(body)
            for g in body.findall('geom'):
                gn = g.get('name', '')
                if gn.startswith(('cabinet_', 'environment_part_')):
                    gt = t @ geom_t(g)
                    objects.append(dict(id=gn, xyz=gt[:3,3].tolist(),
                        xyzw=Rotation.from_matrix(gt[:3,:3]).as_quat().tolist(),
                        geometry=ET.tostring(geometry(g), encoding='unicode')))
            for child in body.findall('body'): collect_world(child, t)
        collect_world(scene.find('worldbody'), np.eye(4))
        scene_files[scene_id] = objects
        count[scene_id] = len(objects)
    add_hand_visuals(robot, v2, evidence)
    (PKG / 'description').mkdir(parents=True, exist_ok=True)
    ET.indent(robot)
    ET.ElementTree(robot).write(PKG / 'description/s5.urdf', encoding='utf-8', xml_declaration=True)
    (PKG / 'config/scenes.json').write_text(json.dumps(scene_files, indent=2), 'utf8')
    joints = {f'joint_{i+1}': dict(has_velocity_limits=True, max_velocity=float(np.deg2rad(10)),
              has_acceleration_limits=True, max_acceleration=float(np.deg2rad(40)),
              has_jerk_limits=True, max_jerk=float(np.deg2rad(160))) for i in range(6)}
    import yaml
    (PKG / 'config/joint_limits.yaml').write_text(yaml.safe_dump({'joint_limits': joints}), 'utf8')
    srdf = ET.Element('robot', name='v3_s5')
    group = ET.SubElement(srdf, 'group', name='arm')
    ET.SubElement(group, 'chain', base_link='base_link', tip_link='tcp')
    state = ET.SubElement(srdf, 'group_state', name='reference', group='arm')
    for i, q in enumerate(qref): ET.SubElement(state, 'joint', name=f'joint_{i+1}', value=str(q))
    chain = ['base_link'] + [f'Link_{i:02d}' for i in range(1,7)] + ['tcp']
    for a, b in zip(chain, chain[1:]): ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason='Adjacent')
    ET.indent(srdf); ET.ElementTree(srdf).write(PKG / 'description/s5.srdf', encoding='utf-8', xml_declaration=True)
    for name in ('jakaAPI.h', 'jktypes.h', 'jkerr.h'):
        copied(sdk / 'c&c++/inc_of_c' / name, ROOT / 'vendor/jaka/include' / name)
    copied(sdk / 'c&c++/x86_64-linux-gnu/shared/libjakaAPI.so', ROOT / 'vendor/jaka/lib/libjakaAPI.so')
    copied(v2 / 'config/current_payload.json', PKG / 'config/payload.json')
    metadata = dict(q_reference_rad=qref.tolist(), tcp_home=tcp.tolist(),
        base_world=world_from_robot.tolist(), registration_cad_to_sdk=registration.tolist(),
        registration_axis_error=axis_error, registration_line_error_m=line_error,
        joint_lower=limits['lower_rad'], joint_upper=limits['upper_rad'],
        kinematics='identified official FK model; exact PoE to URDF conversion',
        collision_registration='nominal CAD registration, physical validation pending',
        scene_objects=count, sdk_version='V2.3.1_beta3 Linux x86_64', robot_commands=0)
    (PKG / 'config/model_metadata.json').write_text(json.dumps(metadata, indent=2), 'utf8')
    (ROOT / 'reference/source_manifest.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2), 'utf8')
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--v2', type=Path, required=True); p.add_argument('--sdk', type=Path)
    p.add_argument('--visuals-only', action='store_true')
    a = p.parse_args()
    if a.visuals_only: export_visuals_only(a.v2.resolve())
    else:
        if a.sdk is None: p.error('--sdk is required for a full export')
        export(a.v2.resolve(), a.sdk.resolve())
