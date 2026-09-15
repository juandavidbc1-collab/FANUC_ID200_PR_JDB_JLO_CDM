import sys, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningOptions

POSES = {
    "home":      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pre_pick":  [-0.558598, 0.343884, -0.158649, 0.0, -1.068262, 0.558598],
    "pick":      [-0.558598, 0.603141, -0.301604, 0.0, -0.666052, 0.558598],
    "pre_place": [0.643493, 0.423444, -0.047642, 0.0, -1.099710, -0.643493],
    "place":     [0.643494, 0.664997, -0.189464, 0.0, -0.716335, -0.643493],
}
JOINTS = [f"joint_{i}" for i in range(1, 7)]

class M(Node):
    def __init__(self):
        super().__init__("mover")
        self.ac = ActionClient(self, MoveGroup, "/move_action"); self.ac.wait_for_server()

    def goto(self, name):
        g = MoveGroup.Goal(); r = g.request
        r.group_name = "manipulator"; r.pipeline_id = "ompl"
        r.planner_id = "RRTConnectkConfigDefault"
        r.allowed_planning_time = 10.0
        r.num_planning_attempts = 3
        r.max_velocity_scaling_factor = 0.05
        r.max_acceleration_scaling_factor = 0.05
        con = Constraints()
        for nm, v in zip(JOINTS, POSES[name]):
            jc = JointConstraint(); jc.joint_name = nm; jc.position = v
            jc.tolerance_above = jc.tolerance_below = 0.001; jc.weight = 1.0
            con.joint_constraints.append(jc)
        r.goal_constraints = [con]
        po = PlanningOptions()
        po.plan_only = False                      # <-- planea Y ejecuta
        po.planning_scene_diff.is_diff = True
        po.planning_scene_diff.robot_state.is_diff = True
        g.planning_options = po
        f = self.ac.send_goal_async(g); rclpy.spin_until_future_complete(self, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self, rf)
        print(f"{name}: error = {rf.result().result.error_code.val}")

def main():
    rclpy.init(); n = M()
    seq = sys.argv[1:] or ["home", "pre_pick", "pick", "pre_pick", "pre_place", "place", "pre_place", "home"]
    for s in seq:
        n.goto(s)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
