import numpy as np, matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# --- Tramo 4B: pre_pick -> pick (bajada recta en Z) ---
P0 = np.array([0.40, -0.25, 0.43])
P1 = np.array([0.40, -0.25, 0.28])
L  = np.linalg.norm(P1 - P0)          # 0.15 m
VMAX, AMAX = 0.200, 0.300             # tramo rojo (ida)

# --- Duracion minima que respeta ambas restricciones ---
# cubico:  vmax = 1.5*L/T      amax = 6*L/T^2
T_cub = max(1.5*L/VMAX, np.sqrt(6*L/AMAX))
# quintico: vmax = 1.875*L/T   amax = 5.7735*L/T^2
T_qui = max(1.875*L/VMAX, np.sqrt(5.7735*L/AMAX))
T = max(T_cub, T_qui)                 # mismo T para comparar en igualdad
print(f"Longitud tramo L = {L:.4f} m")
print(f"T minimo cubico  = {T_cub:.4f} s")
print(f"T minimo quintico= {T_qui:.4f} s")
print(f"T adoptado       = {T:.4f} s\n")

def cubico(t, T):
    s   = 3*(t/T)**2 - 2*(t/T)**3
    ds  = (6*t/T**2) - (6*t**2/T**3)
    dds = (6/T**2) - (12*t/T**3)
    return s, ds, dds

def quintico(t, T):
    x = t/T
    s   = 10*x**3 - 15*x**4 + 6*x**5
    ds  = (30*x**2 - 60*x**3 + 30*x**4)/T
    dds = (60*x - 180*x**2 + 120*x**3)/T**2
    return s, ds, dds

t = np.linspace(0, T, 400)
res = {}
for nombre, f in [("cubico", cubico), ("quintico", quintico)]:
    s, ds, dds = f(t, T)
    pos = P0 + np.outer(s, P1 - P0)
    vel = np.outer(ds, P1 - P0)
    acc = np.outer(dds, P1 - P0)
    vmag = np.linalg.norm(vel, axis=1)
    amag = np.linalg.norm(acc, axis=1)
    jerk = np.gradient(amag, t)
    res[nombre] = (pos, vmag, amag, jerk)
    print(f"--- {nombre} ---")
    print(f"  v_max = {vmag.max():.4f} m/s   (limite {VMAX})  {'OK' if vmag.max()<=VMAX*1.001 else 'EXCEDE'}")
    print(f"  a_max = {amag.max():.4f} m/s2  (limite {AMAX})  {'OK' if amag.max()<=AMAX*1.001 else 'EXCEDE'}")
    print(f"  jerk max = {np.abs(jerk).max():.4f} m/s3")
    print(f"  suavidad (int a^2 dt) = {np.trapz(amag**2, t):.6f}\n")

# --- Waypoints intermedios (4 puntos) para computeCartesianPath ---
NW = 4
tw = np.linspace(0, T, NW+2)[1:-1]     # excluye extremos
print("Waypoints intermedios (x, y, z, t):")
for nombre, f in [("cubico", cubico), ("quintico", quintico)]:
    s, _, _ = f(tw, T)
    W = P0 + np.outer(s, P1 - P0)
    print(f"  {nombre}:")
    for i in range(NW):
        print(f"    {W[i,0]:.4f} {W[i,1]:.4f} {W[i,2]:.4f}   t={tw[i]:.4f}")
    np.savetxt(f"/home/juan-david-botero/ws_fanuc/scripts/wp_{nombre}.csv",
               np.column_stack([W, tw]), delimiter=",",
               header="x,y,z,t", comments="")

# --- Graficas ---
fig, ax = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
for nombre, st in [("cubico","--"), ("quintico","-")]:
    pos, v, a, j = res[nombre]
    ax[0].plot(t, pos[:,2], st, label=nombre)
    ax[1].plot(t, v, st, label=nombre)
    ax[2].plot(t, a, st, label=nombre)
ax[1].axhline(VMAX, color="r", ls=":", label="limite 0.200 m/s")
ax[2].axhline(AMAX, color="r", ls=":", label="limite 0.300 m/s2")
ax[0].set_ylabel("posicion Z [m]"); ax[1].set_ylabel("|v| [m/s]")
ax[2].set_ylabel("|a| [m/s2]");     ax[2].set_xlabel("t [s]")
for a_ in ax: a_.grid(True); a_.legend()
fig.suptitle("Tramo 4B: pre_pick -> pick   perfil cubico vs quintico")
fig.tight_layout()
fig.savefig("/home/juan-david-botero/ws_fanuc/scripts/perfiles_4B.png", dpi=130)
print("Grafica guardada en ~/ws_fanuc/scripts/perfiles_4B.png")
plt.show()
