import rclpy, numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (PlanningScene, CollisionObject, Constraints,
                             JointConstraint, PlanningOptions)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

PRE_PICK = [-0.558598, 0.343884, -0.158649, 0.0, -1.068262, 0.558598]
JOINTS = [f"joint_{i}" for i in range(1, 7)]
BASE = [("mesa_pick",(0.25,0.25,0.02),(0.40,-0.25,0.10)),
        ("pieza",(0.04,0.04,0.05),(0.40,-0.25,0.135)),
        ("mesa_place",(0.25,0.25,0.02),(0.40,0.30,0.10))]
CAND = [ (0.36,-0.14,0.70), (0.34,-0.12,0.70), (0.38,-0.16,0.70),
         (0.36,-0.14,0.55), (0.34,-0.12,0.55), (0.38,-0.16,0.55) ]

def box(oid,size,c):
    co=CollisionObject(); co.header.frame_id="base_link"; co.id=oid
    p=SolidPrimitive(); p.type=SolidPrimitive.BOX; p.dimensions=list(size)
    ps=Pose(); ps.position.x,ps.position.y,ps.position.z=c; ps.orientation.w=1.0
    co.primitives=[p]; co.primitive_poses=[ps]; co.operation=CollisionObject.ADD
    return co

class T(Node):
    def __init__(self):
        super().__init__("tune")
        self.sc=self.create_client(ApplyPlanningScene,"/apply_planning_scene"); self.sc.wait_for_service()
        self.ac=ActionClient(self,MoveGroup,"/move_action"); self.ac.wait_for_server()
    def scene(self,x,y,h):
        s=PlanningScene(); s.is_diff=True
        objs=[box(*o) for o in BASE]
        objs.append(box("poste",(0.30,0.05,h),(x,y,h/2)))
        s.world.collision_objects=objs
        r=ApplyPlanningScene.Request(); r.scene=s
        f=self.sc.call_async(r); rclpy.spin_until_future_complete(self,f)
    def test(self):
        g=MoveGroup.Goal(); rq=g.request
        rq.group_name="manipulator"; rq.pipeline_id="ompl"
        rq.planner_id="RRTConnectkConfigDefault"
        rq.allowed_planning_time=5.0; rq.num_planning_attempts=3
        c=Constraints()
        for nm,v in zip(JOINTS,PRE_PICK):
            jc=JointConstraint(); jc.joint_name=nm; jc.position=v
            jc.tolerance_above=jc.tolerance_below=0.001; jc.weight=1.0
            c.joint_constraints.append(jc)
        rq.goal_constraints=[c]
        po=PlanningOptions(); po.plan_only=True
        po.planning_scene_diff.is_diff=True; po.planning_scene_diff.robot_state.is_diff=True
        g.planning_options=po
        f=self.ac.send_goal_async(g); rclpy.spin_until_future_complete(self,f)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(self,rf)
        res=rf.result().result
        if res.error_code.val!=1: return None
        Q=np.array([p.positions for p in res.planned_trajectory.joint_trajectory.points])
        return np.abs(np.diff(Q,axis=0)).sum()

def main():
    rclpy.init(); n=T()
    print("SIN poste (referencia):")
    n.scene(9.0,9.0,0.01); print("   Lq =", n.test())
    for (x,y,h) in CAND:
        n.scene(x,y,h)
        r=[n.test() for _ in range(3)]
        ok=[v for v in r if v]
        s=f"{np.mean(ok):.3f}" if ok else "FALLA"
        print(f"poste x={x} y={y} h={h}  exitos={len(ok)}/3  Lq={s}")
    rclpy.shutdown()

main()
