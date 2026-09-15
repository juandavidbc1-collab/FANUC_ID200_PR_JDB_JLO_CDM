import numpy as np
np.set_printoptions(precision=5, suppress=True, linewidth=140)

# ---------- FK desde el URDF (verdad de terreno) ----------

def main():
    P=[(0,0,0.330),(0.050,0,0),(0,0,0.330),(0,0,0.035),(0.335,0,0),(0.080,0,0)]
    K=[(0,0,1),(0,1,0),(0,-1,0),(-1,0,0),(0,-1,0),(-1,0,0)]

    def frames(q):
        """Devuelve lista de T_0i (i=0..6) y los ejes de giro en base."""
        T=np.eye(4); Ts=[T.copy()]; Z=[]
        for i in range(6):
            A=np.eye(4); A[:3,3]=P[i]
            T=T@A
            Z.append(T[:3,:3]@np.array(K[i],float))   # eje del joint i en base
            k=np.array(K[i],float); c,s=np.cos(q[i]),np.sin(q[i])
            Kx=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
            B=np.eye(4); B[:3,:3]=np.eye(3)+s*Kx+(1-c)*(Kx@Kx)
            T=T@B
            Ts.append(T.copy())
        return Ts,Z

    def fk(q):
        Ts,_=frames(q); return Ts[-1]

    # ---------- Jacobiano geometrico analitico ----------
    def jacobiano(q):
        """J[:,i] = [ z_i x (p_e - p_i) ; z_i ]  con z_i, p_i del eje del joint i."""
        Ts,Z=frames(q)
        pe=Ts[-1][:3,3]
        J=np.zeros((6,6))
        for i in range(6):
            pi=Ts[i][:3,3] + Ts[i][:3,:3]@np.array(P[i],float)   # origen del eje i
            zi=Z[i]
            J[:3,i]=np.cross(zi,pe-pi)
            J[3:,i]=zi
        return J

    # ---------- Jacobiano numerico (verificacion independiente) ----------
    def jac_numerico(q,h=1e-6):
        """Diferencias finitas sobre la FK: valida el analitico sin depender de KDL."""
        J=np.zeros((6,6)); T0=fk(q); p0=T0[:3,3]; R0=T0[:3,:3]
        for i in range(6):
            qp=np.array(q,float); qp[i]+=h
            Tp=fk(qp)
            J[:3,i]=(Tp[:3,3]-p0)/h
            dR=(Tp[:3,:3]-R0)/h
            W=dR@R0.T                                  # matriz antisimetrica de omega
            J[3:,i]=[W[2,1],W[0,2],W[1,0]]
        return J

    # ---------- Comparacion en varias configuraciones ----------
    POSES={
     "HOME":     [0,0,0,0,0,0],
     "pre_pick": [-0.558598,0.343884,-0.158649,0.0,-1.068262,0.558598],
     "pick":     [-0.558598,0.603141,-0.301604,0.0,-0.666052,0.558598],
     "place":    [0.643494,0.664997,-0.189464,0.0,-0.716335,-0.643493],
    }
    print("="*70)
    print("JACOBIANO ANALITICO vs NUMERICO (diferencias finitas sobre la FK)")
    print("="*70)
    for nombre,q in POSES.items():
        q=np.array(q,float)
        Ja=jacobiano(q); Jn=jac_numerico(q)
        print(f"\n--- {nombre} ---")
        print("J analitico:"); print(Ja)
        print(f"error max vs numerico = {np.abs(Ja-Jn).max():.3e}")

    # ---------- Verificacion de velocidades sobre 4B ----------
    print("\n"+"="*70)
    print("VERIFICACION x_punto = J * q_punto  SOBRE EL TRAMO 4B (quintico)")
    print("="*70)
    d=np.load("/tmp/taller_ri/traj_4D.npz")
    t=d["quintico_t"]; Q=d["quintico_Q"]; V=d["quintico_V"]; X=d["quintico_X"]
    vel_j=[]      # velocidad del extremo via Jacobiano
    for k in range(len(t)):
        J=jacobiano(Q[k])
        vel_j.append(J[:3,:]@V[k])
    vel_j=np.array(vel_j)
    mag_j=np.linalg.norm(vel_j,axis=1)

    # velocidad del perfil teorico
    T=t[-1]; x=t/T
    mag_teo=np.abs((30*x**2-60*x**3+30*x**4)/T)*0.15

    print(f"{'t[s]':>7} {'|v| Jacobiano':>15} {'|v| perfil':>12} {'error':>10}")
    for k in range(0,len(t),6):
        print(f"{t[k]:7.3f} {mag_j[k]:15.5f} {mag_teo[k]:12.5f} {abs(mag_j[k]-mag_teo[k]):10.2e}")
    print(f"\nv_max via Jacobiano = {mag_j.max():.5f} m/s")
    print(f"v_max del perfil    = {mag_teo.max():.5f} m/s")
    print(f"limite tramo rojo   = 0.20000 m/s   -> {'CUMPLE' if mag_j.max()<=0.2 else 'EXCEDE'}")
    print(f"error RMS = {np.sqrt(((mag_j-mag_teo)**2).mean()):.3e} m/s")


if __name__ == "__main__":
    main()
