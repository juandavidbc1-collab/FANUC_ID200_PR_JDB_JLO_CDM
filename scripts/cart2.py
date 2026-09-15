import numpy as np, rclpy, matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

JOINTS=[f"joint_{i}" for i in range(1,7)]
PRE_PICK=[-0.558598,0.343884,-0.158649,0.0,-1.068262,0.558598]
P0=np.array([0.40,-0.25,0.43]); P1=np.array([0.40,-0.25,0.28])
T=1.7321; N=60

def cubico(x):   return 3*x**2 - 2*x**3
def quintico(x): return 10*x**3 - 15*x**4 + 6*x**5

class K(Node):
    def __init__(self):
        super().__init__("cart2")
        self.cli=self.create_client(GetPositionIK,"/compute_ik"); self.cli.wait_for_service()
    def ik(self,p,seed):
        r=GetPositionIK.Request()
        r.ik_request.group_name="manipulator"
        r.ik_request.ik_link_name="flange"
        r.ik_request.timeout.sec=1
        r.ik_request.avoid_collisions=False
        rs=RobotState(); js=JointState()
        js.name=JOINTS; js.position=[float(v) for v in seed]
        rs.joint_state=js; r.ik_request.robot_state=rs      # semilla = punto anterior
        ps=PoseStamped(); ps.header.frame_id="base_link"
        ps.pose.position.x,ps.pose.position.y,ps.pose.position.z=[float(v) for v in p]
        ps.pose.orientation.y=1.0
        r.ik_request.pose_stamped=ps
        f=self.cli.call_async(r); rclpy.spin_until_future_complete(self,f)
        res=f.result()
        if res.error_code.val!=1: return None
        return np.array(res.solution.joint_state.position[:6])

def main():
    rclpy.init(); n=K()
    out={}
    for nombre,sf in [("cubico",cubico),("quintico",quintico)]:
        t=np.linspace(0,T,N)
        s=sf(t/T)
        X=P0+np.outer(s,P1-P0)
        Q=[]; seed=np.array(PRE_PICK); fail=0
        for p in X:
            q=n.ik(p,seed)
            if q is None: fail+=1; q=seed
            Q.append(q); seed=q
        Q=np.array(Q)
        V=np.gradient(Q,t,axis=0); A=np.gradient(V,t,axis=0)
        out[nombre]=(t,Q,V,A,X)
        print(f"--- {nombre} ---  puntos={N}  fallos_IK={fail}")
        print(f"   |q_dot| max  = {np.abs(V).max():.4f} rad/s")
        print(f"   |q_ddot| max = {np.abs(A).max():.4f} rad/s2")
        print(f"   suavidad articular (int a^2) = {np.trapz((A**2).sum(1),t):.4f}\n")
    fig,ax=plt.subplots(2,1,figsize=(9,8),sharex=True)
    for nombre,st in [("cubico","--"),("quintico","-")]:
        t,Q,V,A,X=out[nombre]
        ax[0].plot(t,np.abs(V).max(1),st,label=nombre)
        ax[1].plot(t,np.abs(A).max(1),st,label=nombre)
    ax[0].set_ylabel("max |q_dot| [rad/s]"); ax[1].set_ylabel("max |q_ddot| [rad/s2]")
    ax[1].set_xlabel("t [s]")
    for a_ in ax: a_.grid(True); a_.legend()
    fig.suptitle("Tramo 4B: velocidad y aceleracion articular")
    fig.tight_layout(); fig.savefig("/home/juan-david-botero/ws_fanuc/scripts/articular_4B.png",dpi=130)
    np.savez("/home/juan-david-botero/ws_fanuc/scripts/traj_4B.npz",
             **{f"{k}_{v}":out[k][i] for k in out for i,v in enumerate(["t","Q","V","A","X"])})
    print("Grafica: ~/ws_fanuc/scripts/articular_4B.png")
    rclpy.shutdown()

main()
plt.show()
