import numpy as np, rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

PRE_PICK=[-0.558598,0.343884,-0.158649,0.0,-1.068262,0.558598]
JOINTS=[f"joint_{i}" for i in range(1,7)]

def pose(p):
    ps=Pose(); ps.position.x,ps.position.y,ps.position.z=[float(v) for v in p]
    ps.orientation.y=1.0; return ps

def main():
    rclpy.init(); n=Node("ts")
    cli=n.create_client(GetCartesianPath,"/compute_cartesian_path"); cli.wait_for_service()
    for step in [0.05, 0.01, 0.005, 0.002, 0.001]:
        r=GetCartesianPath.Request()
        r.header.frame_id="base_link"; r.group_name="manipulator"; r.link_name="flange"
        rs=RobotState(); js=JointState(); js.name=JOINTS; js.position=[float(v) for v in PRE_PICK]
        rs.joint_state=js; r.start_state=rs
        r.waypoints=[pose([0.40,-0.25,0.28])]     # solo el destino
        r.max_step=step; r.jump_threshold=0.0; r.avoid_collisions=False
        f=cli.call_async(r); rclpy.spin_until_future_complete(n,f)
        res=f.result()
        print(f"max_step={step:.3f}  fraccion={res.fraction:.3f}  "
              f"puntos={len(res.solution.joint_trajectory.points)}")
    rclpy.shutdown()

main()
