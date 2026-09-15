import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

PRE_PICK = [-0.480175, 0.994814, 0.010089, 0.558713, -2.083512, 0.297563]
JOINTS = [f"joint_{i}" for i in range(1, 7)]

def main():
    rclpy.init(); n = Node("diag2")
    cli = n.create_client(GetMotionPlan, "/plan_kinematic_path")
    cli.wait_for_service()

    r = MotionPlanRequest()
    r.group_name = "manipulator"
    r.pipeline_id = "ompl"
    r.planner_id = "RRTConnectkConfigDefault"
    r.allowed_planning_time = 5.0
    r.num_planning_attempts = 1
    r.start_state.is_diff = True          # <-- usar el estado actual del robot
    con = Constraints()
    for name, val in zip(JOINTS, PRE_PICK):
        jc = JointConstraint()
        jc.joint_name = name; jc.position = val
        jc.tolerance_above = jc.tolerance_below = 0.001; jc.weight = 1.0
        con.joint_constraints.append(jc)
    r.goal_constraints = [con]
    req = GetMotionPlan.Request(); req.motion_plan_request = r
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f)
    res = f.result().motion_plan_response
    print("error =", res.error_code.val, " tiempo =", res.planning_time)
    print("puntos =", len(res.trajectory.joint_trajectory.points))
    rclpy.shutdown()

main()
