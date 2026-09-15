import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

POSES = {"pick": (0.40, -0.25, 0.25), "place": (0.40, 0.30, 0.25)}

class IK(Node):
    def __init__(self):
        super().__init__("ik_poses")
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("esperando /compute_ik ...")

    def solve(self, name, xyz):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "manipulator"
        req.ik_request.ik_link_name = "flange"
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = True
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 1.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 0.0
        req.ik_request.pose_stamped = p

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res.error_code.val == 1:
            q = list(res.solution.joint_state.position[:6])
            print(f"\n{name}  ->  q = [{', '.join(f'{v:.6f}' for v in q)}]")
            print(f"MATLAB:  q = [{' '.join(f'{v:.6f}' for v in q)}];")
        else:
            print(f"\n{name}: SIN SOLUCION (error {res.error_code.val})")

def main():
    rclpy.init()
    n = IK()
    for k, v in POSES.items():
        n.solve(k, v)
    n.destroy_node()
    rclpy.shutdown()

main()
