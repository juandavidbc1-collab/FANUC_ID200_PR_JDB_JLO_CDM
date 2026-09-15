import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

POSES = {
    "pre_pick":  (0.40, -0.25, 0.43),
    "pick":      (0.40, -0.25, 0.28),
    "pre_place": (0.40,  0.30, 0.43),
    "place":     (0.40,  0.30, 0.28),
}

class IK(Node):
    def __init__(self):
        super().__init__("ik4")
        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        self.cli.wait_for_service()

    def solve(self, name, xyz):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "manipulator"
        req.ik_request.ik_link_name = "flange"
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = True
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.pose.position.x, p.pose.position.y, p.pose.position.z = xyz
        p.pose.orientation.y = 1.0
        req.ik_request.pose_stamped = p
        f = self.cli.call_async(req); rclpy.spin_until_future_complete(self, f)
        res = f.result()
        if res.error_code.val == 1:
            q = list(res.solution.joint_state.position[:6])
            print(f'{name:10s} = [{", ".join(f"{v:.6f}" for v in q)}]')
        else:
            print(f"{name:10s} : SIN SOLUCION ({res.error_code.val})")

def main():
    rclpy.init(); n = IK()
    for k, v in POSES.items(): n.solve(k, v)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
