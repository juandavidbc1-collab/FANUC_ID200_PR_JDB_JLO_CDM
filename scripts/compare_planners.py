import numpy as np, rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

PRE_PICK = [-0.480175, 0.994814, 0.010089, 0.558713, -2.083512, 0.297563]
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
        super().__init__("cmp")
        self.cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cli.wait_for_service()

    def plan(self, planner):
        r = MotionPlanRequest()
        r.group_name = "manipulator"
        r.planner_id = planner
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
        fut = self.cli.call_async(req); rclpy.spin_until_future_complete(self, fut)
        return fut.result().motion_plan_response

def metrics(traj):
    Q = np.array([p.positions for p in traj.joint_trajectory.points])
    Lq = np.abs(np.diff(Q, axis=0)).sum()
    X = np.array([fk(q) for q in Q])
    Lx = np.linalg.norm(np.diff(X, axis=0), axis=1).sum()
    sm = (np.diff(Q, 2, axis=0)**2).sum() if len(Q) > 2 else 0.0
    return Lq, Lx, sm, len(Q)

def main():
    rclpy.init(); n = C()
    for pl in PLANNERS:
        T, Lq, Lx, S = [], [], [], []
        for _ in range(N):
            res = n.plan(pl)
            if res.error_code.val != 1:
                print(f"{pl}: fallo {res.error_code.val}"); continue
            a, b, c, _ = metrics(res.trajectory)
            T.append(res.planning_time); Lq.append(a); Lx.append(b); S.append(c)
        if T:
            print(f"\n=== {pl}  ({len(T)}/{N} exitos) ===")
            print(f"  tiempo planeacion : {np.mean(T):.4f} +/- {np.std(T):.4f} s")
            print(f"  long. articular   : {np.mean(Lq):.4f} rad")
            print(f"  long. cartesiana  : {np.mean(Lx):.4f} m")
            print(f"  rugosidad (menor=mas suave) : {np.mean(S):.5f}")
    rclpy.shutdown()

main()
