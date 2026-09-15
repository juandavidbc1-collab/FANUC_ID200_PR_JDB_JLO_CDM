import numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningOptions

GOAL = [-0.558598, 0.343884, -0.158649, 0.0, -1.068262, 0.558598]
JOINTS = [f"joint_{i}" for i in range(1, 7)]

class C(Node):
    def __init__(self):
        super().__init__("diag3")
        self.ac = ActionClient(self, MoveGroup, "/move_action"); self.ac.wait_for_server()
    def plan(self, planner, t):
        g = MoveGroup.Goal(); r = g.request
        r.group_name = "manipulator"; r.pipeline_id = "ompl"; r.planner_id = planner
        r.allowed_planning_time = t; r.num_planning_attempts = 1
        con = Constraints()
        for nm, v in zip(JOINTS, GOAL):
            jc = JointConstraint(); jc.joint_name = nm; jc.position = v
            jc.tolerance_above = jc.tolerance_below = 0.001; jc.weight = 1.0
            con.joint_constraints.append(jc)
        r.goal_constraints = [con]
        po = PlanningOptions(); po.plan_only = True
        po.planning_scene_diff.is_diff = True
        po.planning_scene_diff.robot_state.is_diff = True
        g.planning_options = po
        f = self.ac.send_goal_async(g); rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(self, rf)
        return rf.result().result

def main():
    rclpy.init(); n = C()
    for pl in ["RRTConnectkConfigDefault", "RRTstarkConfigDefault"]:
        for t in [1.0, 5.0, 15.0]:
            res = n.plan(pl, t)
            if res.error_code.val != 1:
                print(f"{pl:28s} t={t:5.1f}  FALLO {res.error_code.val}"); continue
            Q = np.array([p.positions for p in res.planned_trajectory.joint_trajectory.points])
            print(f"{pl:28s} t={t:5.1f}  plan={res.planning_time:7.4f}s  "
                  f"puntos={len(Q):4d}  Lq={np.abs(np.diff(Q,axis=0)).sum():7.4f}")
    rclpy.shutdown()

main()
