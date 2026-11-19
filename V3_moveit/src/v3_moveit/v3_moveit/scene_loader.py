"""将资产中的场景送入 MoveIt，并通过读回核对实际已加载的对象。"""
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from std_msgs.msg import String


def mesh_message(path):
    # 场景障碍的源格式为 OBJ；URDF 连杆网格由 MoveIt/Assimp 读取。
    vertices=[]; triangles=[]
    for line in Path(path).read_text('utf8').splitlines():
        parts=line.split()
        if not parts: continue
        if parts[0]=='v': vertices.append(Point(x=float(parts[1]),y=float(parts[2]),z=float(parts[3])))
        elif parts[0]=='f':
            ids=[int(x.split('/')[0])-1 for x in parts[1:]]
            for i in range(1,len(ids)-1): triangles.append(MeshTriangle(vertex_indices=[ids[0],ids[i],ids[i+1]]))
    if not vertices or not triangles: raise ValueError('empty/unsupported scene mesh '+str(path))
    return Mesh(vertices=vertices,triangles=triangles)


class SceneLoader(Node):
    def __init__(self):
        super().__init__('scene_loader'); self.declare_parameter('scene','s5_only')
        self.root=Path(get_package_share_directory('v3_moveit'))
        self.scene_id=self.get_parameter('scene').value
        self.objects=json.loads((self.root/'config/scenes.json').read_text())[self.scene_id]
        self.apply=self.create_client(ApplyPlanningScene,'/apply_planning_scene')
        self.get=self.create_client(GetPlanningScene,'/get_planning_scene')
        self.ready=self.create_publisher(String,'/v3/scene_ready',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pending=None; self.applied=False; self.confirmed=False
        self.create_timer(.2,self.tick)

    def scene(self):
        scene=PlanningScene(); scene.is_diff=True
        for obj in self.objects:
            co=CollisionObject(); co.header.frame_id='world'; co.id=obj['id']; co.operation=CollisionObject.ADD
            pose=Pose(); pose.position.x,pose.position.y,pose.position.z=obj['xyz']
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=obj['xyzw']
            geom=ET.fromstring(obj['geometry'])[0]
            if geom.tag=='mesh':
                rel=geom.get('filename').replace('package://v3_moveit/','')
                co.meshes=[mesh_message(self.root/rel)]; co.mesh_poses=[pose]
            else:
                p=SolidPrimitive()
                if geom.tag=='box': p.type=p.BOX; p.dimensions=list(map(float,geom.get('size').split()))
                elif geom.tag=='sphere': p.type=p.SPHERE; p.dimensions=[float(geom.get('radius'))]
                elif geom.tag=='cylinder': p.type=p.CYLINDER; p.dimensions=[float(geom.get('length')),float(geom.get('radius'))]
                else: raise ValueError('unsupported primitive')
                co.primitives=[p];co.primitive_poses=[pose]
            scene.world.collision_objects.append(co)
        return scene

    def tick(self):
        if self.pending is not None:
            if not self.pending.done(): return
            reply=self.pending.result();self.pending=None
            if not self.applied:
                self.applied=bool(reply.success)
            else:
                expected={x['id'] for x in self.objects}
                actual={x.id for x in reply.scene.world.collision_objects}
                self.confirmed=expected.issubset(actual)
                if not self.confirmed:self.get_logger().error('PlanningScene lost required obstacles')
        self.ready.publish(String(data=json.dumps(dict(ready=self.confirmed,scene=self.scene_id,
            objects=len(self.objects)))))
        if not self.apply.service_is_ready() or not self.get.service_is_ready(): return
        if not self.applied:
            request=ApplyPlanningScene.Request();request.scene=self.scene();self.pending=self.apply.call_async(request)
        else:
            request=GetPlanningScene.Request();request.components.components=PlanningSceneComponents.WORLD_OBJECT_NAMES
            self.pending=self.get.call_async(request)


def main():
    rclpy.init();node=SceneLoader()
    try:rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:node.destroy_node();rclpy.try_shutdown()
