import numpy as np, rclpy, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rclpy.node import Node
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

PRE_PICK = [-0.558598, 0.343884, -0.158649, 0.0, -1.068262, 0.558598]
JOINTS = [f"joint_{i}" for i in range(1, 7)]
P0 = np.array([0.40, -0.25, 0.43]); P1 = np.array([0.40, -0.25, 0.28])
T = 1.7321

def cubico(x):   return 3*x**2 - 2*x**3
def quintico(x): return 10*x**3 - 15*x**4 + 6*x**5

def pose(p):
    ps = Pose()
    ps.position.x, ps.position.y, ps.position.z = [float(v) for v in p]
    ps.orientation.y = 1.0
    return ps

class C(Node):
    def __init__(self):
        super().__init__("cart")
        self.cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.cli.wait_for_service()

    def path(self, wps):
        r = GetCartesianPath.Request()
        r.header.frame_id = "base_link"
        r.group_name = "manipulator"
        r.link_name = "flange"
        rs = RobotState(); js = JointState()
        js.name = JOINTS; js.position = [float(v) for v in PRE_PICK]
        rs.joint_state = js; r.start_state = rs
        r.waypoints = [pose(w) for w in wps]
        r.max_step = 0.002
        r.jump_threshold = 0.0
        r.avoid_collisions = False
        f = self.cli.call_async(r); rclpy.spin_until_future_complete(self, f)
        return f.result()

def reparam(traj, sfun, T):
    """Asigna time_from_start segun el perfil: s(t) = fraccion recorrida."""
    pts = traj.joint_trajectory.points
    n = len(pts)
    # fraccion de camino acumulada (por longitud articular)
    Q = np.array([p.positions for p in pts])
    dl = np.linalg.norm(np.diff(Q, axis=0), axis=1)
    L = np.concatenate([[0], np.cumsum(dl)]); L /= L[-1]
    # invertir s(t)=L  ->  t
    tt = np.linspace(0, T, 2000); ss = sfun(tt/T)
    tpts = np.interp(L, ss, tt)
    # derivar velocidad y aceleracion articular
    V = np.gradient(Q, tpts, axis=0)
    A = np.gradient(V, tpts, axis=0)
    return tpts, Q, V, A

def main():
    rclpy.init(); n = C()
    NW = 6
    out = {}
    for nombre, sf in [("cubico", cubico), ("quintico", quintico)]:
        x = np.linspace(0, 1, 61)[1:]
        wps = [P0 + sf(xi)*(P1-P0) for xi in x]
        res = n.path(wps)
        print(f"{nombre}: fraccion completada = {res.fraction:.3f}  "
              f"puntos = {len(res.solution.joint_trajectory.points)}")
        if res.fraction < 0.99:
            print("   ADVERTENCIA: no completo el camino"); continue
        t, Q, V, A = reparam(res.solution, sf, T)
        out[nombre] = (t, Q, V, A)
        print(f"   |q_dot| max = {np.abs(V).max():.4f} rad/s")
        print(f"   |q_ddot| max = {np.abs(A).max():.4f} rad/s2")
        print(f"   suavidad articular (int a^2) = {np.trapz((A**2).sum(1), t):.4f}\n")
    if len(out) == 2:
        fig, ax = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
        for nombre, st in [("cubico","--"), ("quintico","-")]:
            t, Q, V, A = out[nombre]
            ax[0].plot(t, np.abs(V).max(1), st, label=nombre)
            ax[1].plot(t, np.abs(A).max(1), st, label=nombre)
        ax[0].set_ylabel("max |q_dot| [rad/s]"); ax[1].set_ylabel("max |q_ddot| [rad/s2]")
        ax[1].set_xlabel("t [s]")
        for a_ in ax: a_.grid(True); a_.legend()
        fig.suptitle("Tramo 4B: aceleracion articular, cubico vs quintico")
        fig.tight_layout()
        fig.savefig("/home/juan-david-botero/ws_fanuc/scripts/articular_4B.png", dpi=130)
        np.savez("/home/juan-david-botero/ws_fanuc/scripts/traj_4B.npz",
                 **{f"{k}_{v}": out[k][i] for k in out for i, v in enumerate(["t","Q","V","A"])})
        print("Grafica: ~/ws_fanuc/scripts/articular_4B.png")
        print("Datos guardados en traj_4B.npz (los usaras en la Parte 5)")
    rclpy.shutdown()

main()
