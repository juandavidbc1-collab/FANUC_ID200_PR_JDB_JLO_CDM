# Guía de ejecución — Taller ROS2/MoveIt2 (FANUC LR Mate 200iD)

Universidad EIA — Robótica Industrial — Parcial No. 2
Workspace: `~/ws_fanuc` — ROS 2 Jazzy

---

## 0. Preparación (una sola vez)

Si las gráficas no se abren en ventana:

```bash
cd ~/ws_fanuc/scripts
for f in perfiles.py cart2.py cart4d.py; do
  sed -i 's/matplotlib.use("Agg")/matplotlib.use("TkAgg")/' $f
  grep -q "plt.show()" $f || echo "plt.show()" >> $f
done
```

Si sale `No module named 'tkinter'`:

```bash
sudo apt install python3-tk
```

---

## 1. Arranque — Terminal 1 (dejar abierta)

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fanuc_lrmate200id_moveit_config demo.launch.py
```

Esperar a que abra RViz y los logs se calmen.

---

## 2. Terminal 2 — preparación

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/scene.py
```

Debe imprimir `escena aplicada: True` y aparecer las cajas verdes en RViz.

> **Importante:** la escena se borra cada vez que se reinicia `demo.launch.py`.
> Si hay fallos raros de planeación, lo primero es volver a correr `scene.py`.

---

## Parte 2 — Transformación homogénea en HOME

```bash
python3 scripts/mover.py home
ros2 run tf2_ros tf2_echo base_link flange
```

Resultado esperado: `Translation: [0.465, 0.000, 0.695]` con rotación identidad.
Cortar con `Ctrl+C`.

Luego en MATLAB, con `q = [0 0 0 0 0 0]` y
`T_ROS = [1 0 0 0.465; 0 1 0 0; 0 0 1 0.695; 0 0 0 1]`.
Los errores deben salir del orden de 1e-16.

**Captura para el video:** `tf2_echo` y el Command Window de MATLAB lado a lado.

---

## Parte 3 — Cinemática inversa de las poses clave

```bash
cd ~/ws_fanuc
python3 scripts/ik4.py
```

Devuelve los cuatro vectores articulares:

| Pose | Coordenadas (x, y, z) |
|---|---|
| pre_pick | 0.40, −0.25, 0.43 |
| pick | 0.40, −0.25, 0.28 |
| pre_place | 0.40, 0.30, 0.43 |
| place | 0.40, 0.30, 0.28 |

Verificación en MATLAB para cada pose, cambiando dos líneas:

```matlab
q = [ ... los 6 angulos que devolvio el IK ... ];
T_ROS = [-1 0 0 0.40; 0 1 0 -0.25; 0 0 -1 0.28; 0 0 0 1];   % ejemplo: pick
```

La orientación `diag(−1, 1, −1)` corresponde al giro de 180° sobre Y
(cuaternión xyzw = 0, 1, 0, 0) con el efector apuntando hacia abajo.

---

## Parte 4A — Comparación de planeadores

```bash
python3 scripts/cmp2.py
```

Tarda aproximadamente un minuto (10 repeticiones por planeador).

Resultados de referencia:

| | RRTConnect | RRT* |
|---|---|---|
| Éxitos | 10/10 | 4/10 |
| Tiempo | 0.0139 s | 2.0011 s |
| Long. articular | 7.744 rad | 3.890 rad |
| Long. cartesiana | 0.940 m | 0.678 m |
| Rugosidad | 0.00563 | 0.00298 |

**Planeador elegido: RRTConnect**, por confiabilidad. RRT* produce caminos más
cortos y suaves, pero falla el 60% de los intentos con el obstáculo presente.

---

## Parte 4B — Perfiles cúbico y quíntico

```bash
python3 scripts/perfiles.py
```

Abre una ventana con posición, velocidad y aceleración cartesiana.
**Cerrar la ventana para que el script termine.**

Restricciones tramo rojo: 0.200 m/s, 0.300 m/s² → T = 1.7321 s

```bash
python3 scripts/cart2.py
```

Genera `traj_4B.npz` y abre la gráfica de velocidad/aceleración articular.

Resultados de referencia:

| | cúbico | quíntico |
|---|---|---|
| \|q̇\| máx | 0.3509 rad/s | 0.4384 rad/s |
| \|q̈\| máx | 0.7827 rad/s² | 0.7971 rad/s² |
| ∫a² dt | 0.5670 | 0.8775 |

**Perfil elegido: quíntico.** El cúbico tiene menores magnitudes, pero presenta
discontinuidad de aceleración en los extremos (jerk no acotado), lo que produce
impacto mecánico justo en el instante de agarre. El quíntico arranca y termina
con aceleración nula.

---

## Parte 4D — Tramo azul

```bash
python3 scripts/cart4d.py
```

Restricciones tramo azul: 0.100 m/s, 0.020 m/s² → T = 6.5804 s

Manda la aceleración por amplio margen (6.58 s contra 2.81 s por velocidad).
El mismo desplazamiento de 15 cm dura casi 4× más que en 4B.

---

## Parte 5 — Jacobiano

Requiere que `traj_4B.npz` exista (correr `cart2.py` antes).

```bash
python3 scripts/jacobiano.py
```

Resultados de referencia:

- Jacobiano analítico vs numérico: error 2.3e-7 en las cuatro configuraciones
- Verificación ẋ = J·q̇ sobre 4B: error RMS 1.1e-4 m/s
- v_max = 0.16215 m/s < 0.200 m/s → cumple

En MATLAB, bloque adicional antes de `function T_ij = T_DH(...)`:

```matlab
J = jacobiano_dh(T, T_corr, frame);
J(:,3:6) = -J(:,3:6);        % convencion: theta_i = -q_i para i=3..6
disp('Jacobiano geometrico (MATLAB, desde DH):'); disp(J)
fprintf('Manipulabilidad w = %.6f\n', sqrt(det(J*J')));
fprintf('Numero de condicion = %.3e\n', cond(J));
```

Y la función al final del archivo:

```matlab
function J = jacobiano_dh(T, T_corr, frame)
% Craig (DH modificado): z_i y p_i salen de T_0i
T_e = T(:,:,frame) * T_corr;
p_e = T_e(1:3,4);
J = zeros(6, frame);
for i = 1:frame
    z = T(1:3,3,i);
    p = T(1:3,4,i);
    J(1:3,i) = cross(z, p_e - p);
    J(4:6,i) = z;
end
end
```

---

## Ciclo completo — animación para el video

```bash
python3 scripts/ciclo.py
```

Secuencia: HOME → 4A → 4B → retirada → 4C → 4D → retirada → HOME.
Ocho `error=1` significan ciclo limpio. Dura unos 30 s.

Para ralentizar la grabación:

```bash
sed -i 's/= 0.10/= 0.05/g' scripts/ciclo.py
```

---

## Ejecución encadenada (sin gráficas)

Para verificar todo de una pasada, volviendo temporalmente a modo `Agg`:

```bash
cd ~/ws_fanuc/scripts
for f in perfiles.py cart2.py cart4d.py; do
  sed -i 's/matplotlib.use("TkAgg")/matplotlib.use("Agg")/' $f
done
cd ~/ws_fanuc
python3 scripts/scene.py && python3 scripts/ik4.py && \
python3 scripts/perfiles.py && python3 scripts/cart2.py && \
python3 scripts/cart4d.py && python3 scripts/jacobiano.py && \
python3 scripts/cmp2.py
```

Si algo falla, la cadena se detiene en ese punto.

---

## Tabla de referencia rápida

| Script | Parte | Genera |
|---|---|---|
| `scene.py` | 4 | Escena de colisión |
| `mover.py` | — | Movimiento libre a una pose |
| `ik4.py` | 3 | Ángulos de las 4 poses |
| `cmp2.py` | 4A | Comparación de planeadores |
| `perfiles.py` | 4B | `perfiles_4B.png`, waypoints CSV |
| `cart2.py` | 4B | `traj_4B.npz`, `articular_4B.png` |
| `cart4d.py` | 4D | `traj_4D.npz`, `articular_4D.png` |
| `jacobiano.py` | 5 | Jacobiano y verificación de velocidades |
| `ciclo.py` | 4 | Animación completa |

---

## Problemas frecuentes

| Síntoma | Causa | Solución |
|---|---|---|
| `error = 99999` o `-2` al planear | Escena no aplicada o meta en colisión | `python3 scripts/scene.py` |
| `GOAL_STATE_INVALID` en el log | El obstáculo toca al robot en la meta | Revisar posición del poste |
| `FileNotFoundError: traj_4B.npz` | No se corrió `cart2.py` antes | Correr `cart2.py` |
| El script parece colgado tras la gráfica | `plt.show()` espera | Cerrar la ventana |
| RRT* siempre tarda ~12 ms | Falta `ompl_planning.yaml` | Verificar que exista en `config/` |
| `error = 1` | **No es error: es éxito** | MoveIt usa SUCCESS = 1 |

---

## Códigos de error de MoveIt

| Código | Significado |
|---|---|
| 1 | Éxito |
| −1 | Falla general |
| −2 | Planeación fallida |
| −10 | Estado inicial en colisión |
| −12 | Meta en colisión |
| −31 | No se encontró IK |
| 99999 | Falla genérica |
