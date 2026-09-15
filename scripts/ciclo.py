import numpy as np, rclpy, time
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, JointConstraint, PlanningOptions,
                             RobotTrajectory)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS=[f"joint_{i}" for i in range(1,7)]
POSES={
 "home":      [0.0,0.0,0.0,0.0,0.0,0.0],
 "pre_pick":  [-0.558598,0.343884,-0.158649,0.0,-1.068262,0.558598],
 "pre_place": [0.643493,0.423444,-0.047642,0.0,-1.099710,-0.643493],
}

class Ciclo(Node):
    def __init__(self):
        super().__init__("ciclo")
        self.mg=ActionClient(self,MoveGroup,"/move_action"); self.mg.wait_for_server()
        self.ex=ActionClient(self,ExecuteTrajectory,"/execute_trajectory"); self.ex.wait_for_server()

    def libre(self,destino,etiqueta):
        """Tramo 4A / 4C: movimiento libre con RRTConnect, evade el obstaculo."""
        g=MoveGroup.Goal(); r=g.request
        r.group_name="manipulator"; r.pipeline_id="ompl"
        r.planner_id="RRTConnectkConfigDefault"
        r.allowed_planning_time=5.0; r.num_planning_attempts=5
        r.max_velocity_scaling_factor=0.10
        r.max_acceleration_scaling_factor=0.10
        c=Constraints()
        for nm,v in zip(JOINTS,POSES[destino]):
            jc=JointConstraint(); jc.joint_name=nm; jc.position=float(v)
            jc.tolerance_above=jc.tolerance_below=0.001; jc.weight=1.0
            c.joint_constraints.append(jc)
        r.goal_constraints=[c]
        po=PlanningOptions(); po.plan_only=False
        po.planning_scene_diff.is_diff=True
        po.planning_scene_diff.robot_state.is_diff=True
        g.planning_options=po
        f=self.mg.send_goal_async(g); rclpy.spin_until_future_complete(self,f)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(self,rf)
        print(f"  {etiqueta}: error={rf.result().result.error_code.val}")

    def fino(self,npz,etiqueta,reverso=False):
        """Tramo 4B / 4D: trayectoria cartesiana con perfil quintico."""
        d=np.load(npz)
        t=d["quintico_t"]; Q=d["quintico_Q"]; V=d["quintico_V"]
        if reverso:
            t=t; Q=Q[::-1]; V=-V[::-1]
        jt=JointTrajectory(); jt.joint_names=JOINTS
        for k in range(len(t)):
            p=JointTrajectoryPoint()
            p.positions=[float(v) for v in Q[k]]
            p.velocities=[float(v) for v in V[k]]
            p.time_from_start.sec=int(t[k])
            p.time_from_start.nanosec=int((t[k]%1)*1e9)
            jt.points.append(p)
        rt=RobotTrajectory(); rt.joint_trajectory=jt
        g=ExecuteTrajectory.Goal(); g.trajectory=rt
        f=self.ex.send_goal_async(g); rclpy.spin_until_future_complete(self,f)
        rf=f.result().get_result_async(); rclpy.spin_until_future_complete(self,rf)
        print(f"  {etiqueta}: error={rf.result().result.error_code.val}")

def main():
    rclpy.init(); n=Ciclo()
    B="/home/juan-david-botero/ws_fanuc/scripts/traj_4B.npz"
    D="/home/juan-david-botero/ws_fanuc/scripts/traj_4D.npz"
    print("Posicionando en HOME...");        n.libre("home","HOME")
    print("\n4A  HOME -> pre_pick (RRTConnect, evade poste)")
    n.libre("pre_pick","4A")
    print("\n4B  pre_pick -> pick (cartesiano, quintico, tramo rojo)")
    n.fino(B,"4B"); time.sleep(0.5)
    print("\n    pick -> pre_pick (retirada)")
    n.fino(B,"retirada",reverso=True)
    print("\n4C  pre_pick -> pre_place (RRTConnect)")
    n.libre("pre_place","4C")
    print("\n4D  pre_place -> place (cartesiano, quintico, tramo azul)")
    n.fino(D,"4D"); time.sleep(0.5)
    print("\n    place -> pre_place (retirada)")
    n.fino(D,"retirada",reverso=True)
    print("\n    regreso a HOME")
    n.libre("home","HOME")
    print("\nCiclo completo terminado.")
    rclpy.shutdown()

main()
