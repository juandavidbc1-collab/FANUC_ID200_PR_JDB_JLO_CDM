import numpy as np, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PlanningOptions

PRE_PICK = [-0.558598, 0.343884, -0.158649, 0.0, -1.068262, 0.558598]
JOINTS = [f"joint_{i}" for i in range(1, 7)]
PLANNERS = ["RRTConnectkConfigDefault", "RRTstarkConfigDefault"]
N = 10

P = [(0,0,0.330),(0.050,0,0),(0,0,0.330),(0,0,0.035),(0.335,0,0),(0.080,0,0)]
K = [(0,0,1),(0,1,0),(0,-1,0),(-1,0,0),(0,-1,0),(-1,0,0)]
def fk(q):
    T = np.eye(4)
    for i in range(6):
        k = np.array(K[i], float); c, s = np.cos(q[i]), np.sin(q[i])
        Kx = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
        A = np.eye(4); A[:3,3] = P[i]
        B = np.eye(4); B[:3,:3] = np.eye(3)+s*Kx+(1-c)*(Kx@Kx)
        T = T@A@B
    return T[:3,3]

class C(Node):
    def __init__(self):
        super().__init__("cmp2")
        self.ac = ActionClient(self, MoveGroup, "/move_action")
        self.ac.wait_for_server()

    def plan(self, planner):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = "manipulator"
        r.pipeline_id = "ompl"
        r.planner_id = planner
        r.allowed_planning_time = 2.0
        r.num_planning_attempts = 1
        r.max_velocity_scaling_factor = 0.1
        r.max_acceleration_scaling_factor = 0.1
        con = Constraints()
        for name, val in zip(JOINTS, PRE_PICK):
            jc = JointConstraint()
            jc.joint_name = name; jc.position = val
            jc.tolerance_above = jc.tolerance_below = 0.001; jc.weight = 1.0
            con.joint_constraints.append(jc)
        r.goal_constraints = [con]
        po = PlanningOptions()
        po.plan_only = True              # solo planear, no ejecutar
        po.planning_scene_diff.is_diff = True
        po.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = po

        f = self.ac.send_goal_async(goal); rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if not gh.accepted:
            print("  goal rechazado"); return None
        rf = gh.get_result_async(); rclpy.spin_until_future_complete(self, rf)
        return rf.result().result

def metrics(traj):
    Q = np.array([p.positions for p in traj.joint_trajectory.points])
    Lq = np.abs(np.diff(Q, axis=0)).sum()
    X = np.array([fk(q) for q in Q])
    Lx = np.linalg.norm(np.diff(X, axis=0), axis=1).sum()
    sm = (np.diff(Q, 2, axis=0)**2).sum() if len(Q) > 2 else 0.0
    return Lq, Lx, sm

def main():
    rclpy.init(); n = C()
    for pl in PLANNERS:
        T, Lq, Lx, S = [], [], [], []
        for _ in range(N):
            res = n.plan(pl)
            if res is None or res.error_code.val != 1:
                print(f"{pl}: fallo {res.error_code.val if res else 'sin resultado'}")
                continue
            a, b, c = metrics(res.planned_trajectory)
            T.append(res.planning_time); Lq.append(a); Lx.append(b); S.append(c)
        if T:
            print(f"\n=== {pl}  ({len(T)}/{N} exitos) ===")
            print(f"  tiempo planeacion : {np.mean(T):.4f} +/- {np.std(T):.4f} s")
            print(f"  long. articular   : {np.mean(Lq):.4f} rad")
            print(f"  long. cartesiana  : {np.mean(Lx):.4f} m")
            print(f"  rugosidad (menor=mas suave) : {np.mean(S):.5f}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
