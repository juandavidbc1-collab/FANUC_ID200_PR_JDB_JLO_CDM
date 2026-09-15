import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

OBJS = [
    ("mesa_pick",  (0.25, 0.25, 0.02), (0.40, -0.25, 0.10)),
    ("pieza",      (0.07, 0.07, 0.09), (0.40, -0.25, 0.155)),
    ("poste",      (0.05, 0.05, 0.85), (0.45, -0.15, 0.425)),
    ("mesa_place", (0.25, 0.25, 0.02), (0.40,  0.30, 0.10)),
]

def make(oid, size, c):
    co = CollisionObject()
    co.header.frame_id = "base_link"
    co.id = oid
    p = SolidPrimitive(); p.type = SolidPrimitive.BOX; p.dimensions = list(size)
    ps = Pose(); ps.position.x, ps.position.y, ps.position.z = c; ps.orientation.w = 1.0
    co.primitives = [p]; co.primitive_poses = [ps]; co.operation = CollisionObject.ADD
    return co

def main():
    rclpy.init(); n = Node("scene")
    cli = n.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service()
    sc = PlanningScene(); sc.is_diff = True
    sc.world.collision_objects = [make(*o) for o in OBJS]
    req = ApplyPlanningScene.Request(); req.scene = sc
    fut = cli.call_async(req); rclpy.spin_until_future_complete(n, fut)
    print("escena aplicada:", fut.result().success)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
