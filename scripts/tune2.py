import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningScene, CollisionObject, RobotState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

POSES = {
 "home":     [0.0,0.0,0.0,0.0,0.0,0.0],
 "pre_pick": [-0.558598,0.343884,-0.158649,0.0,-1.068262,0.558598],
 "pick":     [-0.558598,0.603141,-0.301604,0.0,-0.666052,0.558598],
}
JOINTS=[f"joint_{i}" for i in range(1,7)]
BASE=[("mesa_pick",(0.25,0.25,0.02),(0.40,-0.25,0.10)),
      ("pieza",(0.04,0.04,0.05),(0.40,-0.25,0.135)),
      ("mesa_place",(0.25,0.25,0.02),(0.40,0.30,0.10))]
# (lado_x, lado_y, alto, centro_x, centro_y)
CAND=[(0.05,0.05,0.85,0.36,-0.14),
      (0.05,0.05,0.85,0.45,-0.15),
      (0.05,0.05,0.85,0.50,-0.12),
      (0.05,0.05,0.70,0.45,-0.15),
      (0.04,0.04,0.90,0.48,-0.18)]

def box(oid,size,c):
    co=CollisionObject(); co.header.frame_id="base_link"; co.id=oid
    p=SolidPrimitive(); p.type=SolidPrimitive.BOX; p.dimensions=[float(v) for v in size]
    ps=Pose(); ps.position.x,ps.position.y,ps.position.z=[float(v) for v in c]; ps.orientation.w=1.0
    co.primitives=[p]; co.primitive_poses=[ps]; co.operation=CollisionObject.ADD
    return co

class T(Node):
    def __init__(self):
        super().__init__("tune2")
        self.sc=self.create_client(ApplyPlanningScene,"/apply_planning_scene"); self.sc.wait_for_service()
        self.vl=self.create_client(GetStateValidity,"/check_state_validity"); self.vl.wait_for_service()
    def scene(self,poste):
        s=PlanningScene(); s.is_diff=True
        objs=[box(*o) for o in BASE]
        if poste:
            sx,sy,h,x,y=poste
            objs.append(box("poste",(sx,sy,h),(x,y,h/2)))
        s.world.collision_objects=objs
        r=ApplyPlanningScene.Request(); r.scene=s
        f=self.sc.call_async(r); rclpy.spin_until_future_complete(self,f)
    def valid(self,q):
        r=GetStateValidity.Request()
        rs=RobotState(); js=JointState(); js.name=JOINTS; js.position=[float(v) for v in q]
        rs.joint_state=js; r.robot_state=rs; r.group_name="manipulator"
        f=self.vl.call_async(r); rclpy.spin_until_future_complete(self,f)
        res=f.result()
        bodies=set(f"{c.contact_body_1}/{c.contact_body_2}" for c in res.contacts)
        return res.valid, bodies

def main():
    rclpy.init(); n=T()
    for cand in [None]+CAND:
        n.scene(cand)
        tag = "SIN POSTE" if cand is None else f"poste {cand}"
        print(f"\n--- {tag} ---")
        for nm,q in POSES.items():
            v,b=n.valid(q)
            extra = "" if v else "  choques: "+", ".join(sorted(b))
            print(f"   {nm:9s} valido={v}{extra}")
    rclpy.shutdown()

main()
