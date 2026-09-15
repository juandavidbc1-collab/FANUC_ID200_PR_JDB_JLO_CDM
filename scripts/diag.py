import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState

PRE_PICK = [-0.480175, 0.994814, 0.010089, 0.558713, -2.083512, 0.297563]
JOINTS = [f"joint_{i}" for i in range(1, 7)]

class D(Node):
    def __init__(self):
        super().__init__("diag")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.val_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.plan_cli.wait_for_service(); self.val_cli.wait_for_service()

    def valid(self, q, label):
        req = GetStateValidity.Request()
        rs = RobotState(); js = JointState()
        js.name = JOINTS; js.position = list(q)
        rs.joint_state = js
        req.robot_state = rs; req.group_name = "manipulator"
        f = self.val_cli.call_async(req); rclpy.spin_until_future_complete(self, f)
        res = f.result()
        print(f"{label}: valido = {res.valid}")
        for c in res.contacts:
            print(f"   choque: {c.contact_body_1} <-> {c.contact_body_2}")

    def plan(self, planner, pipeline, label):
        r = MotionPlanRequest()
        r.group_name = "manipulator"
        if planner: r.planner_id = planner
        if pipeline: r.pipeline_id = pipeline
        r.allowed_planning_time = 5.0
        r.num_planning_attempts = 1
        con = Constraints()
        for name, val in zip(JOINTS, PRE_PICK):
            jc = JointConstraint()
            jc.joint_name = name; jc.position = val
            jc.tolerance_above = jc.tolerance_below = 0.001; jc.weight = 1.0
            con.joint_constraints.append(jc)
        r.goal_constraints = [con]
        req = GetMotionPlan.Request(); req.motion_plan_request = r
        f = self.plan_cli.call_async(req); rclpy.spin_until_future_complete(self, f)
        print(f"{label}: error = {f.result().motion_plan_response.error_code.val}")

def main():
    rclpy.init(); n = D()
    print("--- validez de estados ---")
    n.valid([0]*6, "HOME")
    n.valid(PRE_PICK, "PICK")
    print("\n--- variantes de peticion ---")
    n.plan(None, None, "sin planner, sin pipeline")
    n.plan(None, "ompl", "sin planner, pipeline=ompl")
    n.plan("RRTConnectkConfigDefault", "ompl", "RRTConnect + pipeline=ompl")
    n.plan("RRTConnect", "ompl", "RRTConnect (nombre corto)")
    rclpy.shutdown()

main()
