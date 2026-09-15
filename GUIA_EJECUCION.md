# Guía de ejecución — Taller ROS2/MoveIt2 (FANUC LR Mate 200iD)

Universidad EIA — Robótica Industrial — Parcial No. 2
Workspace: `~/ws_fanuc` — ROS 2 Jazzy

Todos los nodos están empaquetados en `fanuc_lrmate200id_taller` y se ejecutan
con `ros2 run`.

---

## 0. Preparación (una sola vez)

Compilar el workspace:

```bash
cd ~/ws_fanuc
colcon build
source install/setup.bash
```

Si sale `No module named 'tkinter'` al abrir gráficas:

```bash
sudo apt install python3-tk
```

Para no repetir los `source` en cada terminal nueva:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ws_fanuc/install/setup.bash" >> ~/.bashrc
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
ros2 run fanuc_lrmate200id_taller ros2_scene
```

Debe imprimir `escena aplicada: True` y aparecer las cajas verdes en RViz.

> **Importante:** la escena vive en la memoria de `move_group` y se borra cada
> vez que se reinicia `demo.launch.py`. Ante fallos raros de planeación, lo
> primero es volver a ejecutar `ros2_scene`.

---

## Parte 2 — Transformación homogénea en HOME

```bash
ros2 run fanuc_lrmate200id_taller ros2_mover home
ros2 run tf2_ros tf2_echo base_link flange
```

Resultado esperado: `Translation: [0.465, 0.000, 0.695]` con rotación identidad.
Cortar con `Ctrl+C`.

Luego en MATLAB (`matlab/MatrixTDH.m`), con `q = [0 0 0 0 0 0]` y
`T_ROS = [1 0 0 0.465; 0 1 0 0; 0 0 1 0.695; 0 0 0 1]`.
Los errores deben salir del orden de 1e-16.

**Captura para el video:** `tf2_echo` y el Command Window de MATLAB lado a lado.

---

## Parte 3 — Cinemática inversa de las poses clave

```bash
ros2 run fanuc_lrmate200id_taller ros2_ik
```

Poses definidas (efector apuntando hacia abajo):

| Pose | x | y | z |
|---|---|---|---|
| pre_pick | 0.40 | −0.25 | 0.43 |
| pick | 0.40 | −0.25 | 0.28 |
| pre_place | 0.40 | 0.30 | 0.43 |
| place | 0.40 | 0.30 | 0.28 |

Verificación en MATLAB para cada pose, cambiando dos líneas:

```matlab
q = [ ... los 6 angulos que devolvio el IK ... ];
T_ROS = [-1 0 0 0.40; 0 1 0 -0.25; 0 0 -1 0.28; 0 0 0 1];   % ejemplo: pick
```

La orientación `diag(−1, 1, −1)` corresponde al giro de 180° sobre Y
(cuaternión xyzw = 0, 1, 0, 0). La última columna son las coordenadas de la
tabla de arriba: es lo único que cambia entre las cuatro poses.

---

## Parte 4A — Comparación de planeadores

```bash
ros2 run fanuc_lrmate200id_taller ros2_planners
```

Tarda aproximadamente un minuto (10 repeticiones por planeador).

| | RRTConnect | RRT* |
|---|---|---|
| Éxitos | **10/10** | 4/10 |
| Tiempo | **0.0139 s** | 2.0011 s |
| Long. articular | 7.744 rad | **3.890 rad** |
| Long. cartesiana | 0.940 m | **0.678 m** |
| Rugosidad | 0.00563 | **0.00298** |

**Planeador elegido: RRTConnect**, por confiabilidad. RRT* produce caminos más
cortos y suaves, pero falla el 60% de los intentos con el obstáculo presente.

---

## Parte 4B — Perfiles cúbico y quíntico

```bash
ros2 run fanuc_lrmate200id_taller ros2_perfiles
```

Abre una ventana con posición, velocidad y aceleración cartesiana.
**Cerrar la ventana para que el nodo termine.**

Restricciones tramo rojo: 0.200 m/s, 0.300 m/s² → T = 1.7321 s
(la restricción activa es la aceleración).

```bash
ros2 run fanuc_lrmate200id_taller ros2_cart4b
```

Genera `traj_4B.npz` en `/tmp/taller_ri/` y abre la gráfica articular.

| | cúbico | quíntico |
|---|---|---|
| v_max cartesiana | 0.1299 m/s | 0.1624 m/s |
| a_max cartesiana | 0.3000 m/s² | 0.2887 m/s² |
| \|q̇\| máx | **0.3509 rad/s** | 0.4384 rad/s |
| \|q̈\| máx | **0.7827 rad/s²** | 0.7971 rad/s² |
| ∫a² dt | **0.5670** | 0.8775 |

**Perfil elegido: quíntico.** El cúbico tiene menores magnitudes, pero presenta
discontinuidad de aceleración en los extremos (jerk no acotado), lo que produce
impacto mecánico justo en el instante de agarre. El quíntico arranca y termina
con aceleración nula.

---

## Parte 4D — Tramo azul

```bash
ros2 run fanuc_lrmate200id_taller ros2_cart4d
```

Restricciones tramo azul: 0.100 m/s, 0.020 m/s² → T = 6.5804 s

Manda la aceleración por amplio margen (6.58 s contra 2.81 s por velocidad).
El mismo desplazamiento de 15 cm dura casi 4× más que en 4B.

---

## Parte 5 — Jacobiano

Requiere que existan `traj_4B.npz` y `traj_4D.npz` en `/tmp/taller_ri/`,
generados por `ros2_cart4b` y `ros2_cart4d`.

```bash
ros2 run fanuc_lrmate200id_taller ros2_jacobian
```

- Jacobiano analítico vs numérico: error 2.3e-7 en las cuatro configuraciones
- Verificación ẋ = J·q̇ sobre 4B: error RMS 1.1e-4 m/s
- v_max = 0.16215 m/s < 0.200 m/s → cumple

En MATLAB (`matlab/Jacobiano.m`), bloque antes de `function T_ij = T_DH(...)`:

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

**Nota:** HOME es una configuración singular. Las columnas 4 y 6 son paralelas
(J4 y J6 comparten eje), por lo que `det(J·Jᵀ) = 0` y el número de condición se
dispara. Conviene comparar también en `pre_pick`, donde el Jacobiano es de
rango completo.

---

## Ciclo completo — animación para el video

```bash
ros2 run fanuc_lrmate200id_taller ros2_ciclo
```

Secuencia: HOME → 4A → 4B → retirada → 4C → 4D → retirada → HOME.
Ocho `error=1` significan ciclo limpio. Dura unos 30 s.

---

## Ejecutables del paquete

| Comando (`ros2 run fanuc_lrmate200id_taller ...`) | Parte | Necesita simulación |
|---|---|---|
| `ros2_scene` | 4 | Sí |
| `ros2_mover <pose>` | — | Sí |
| `ros2_ik` | 3 | Sí |
| `ros2_planners` | 4A | Sí |
| `ros2_perfiles` | 4B | No |
| `ros2_cart4b` | 4B | Sí |
| `ros2_cart4d` | 4D | Sí |
| `ros2_jacobian` | 5 | No |
| `ros2_ciclo` | 4 | Sí |

Poses válidas para `ros2_mover`: `home`, `pre_pick`, `pick`, `pre_place`,
`place`. Sin argumento ejecuta la secuencia completa.

---

## Secuencia completa de verificación

Con `demo.launch.py` corriendo:

```bash
ros2 run fanuc_lrmate200id_taller ros2_scene
ros2 run fanuc_lrmate200id_taller ros2_ik
ros2 run fanuc_lrmate200id_taller ros2_perfiles
ros2 run fanuc_lrmate200id_taller ros2_cart4b
ros2 run fanuc_lrmate200id_taller ros2_cart4d
ros2 run fanuc_lrmate200id_taller ros2_jacobian
ros2 run fanuc_lrmate200id_taller ros2_planners
ros2 run fanuc_lrmate200id_taller ros2_ciclo
```

Los nodos con gráfica esperan a que se cierre la ventana antes de terminar.

---

## Archivos generados

Se guardan en `/tmp/taller_ri/`:

| Archivo | Generado por |
|---|---|
| `perfiles_4B.png` | `ros2_perfiles` |
| `wp_cubico.csv`, `wp_quintico.csv` | `ros2_perfiles` |
| `traj_4B.npz`, `articular_4B.png` | `ros2_cart4b` |
| `traj_4D.npz`, `articular_4D.png` | `ros2_cart4d` |

`/tmp` se borra al reiniciar el equipo. Para conservar resultados:

```bash
mkdir -p ~/ws_fanuc/resultados
cp /tmp/taller_ri/* ~/ws_fanuc/resultados/
```

---

## Problemas frecuentes

| Síntoma | Causa | Solución |
|---|---|---|
| `Package not found` | Terminal sin sourcear | `source ~/ws_fanuc/install/setup.bash` |
| El nodo se queda colgado sin imprimir | Espera un servicio de MoveIt | Levantar `demo.launch.py` |
| `error = 99999` o `-2` al planear | Escena no aplicada | `ros2_scene` |
| `GOAL_STATE_INVALID` en el log | El obstáculo toca al robot en la meta | Revisar posición del poste |
| `FileNotFoundError: traj_4B.npz` | Falta correr `ros2_cart4b` | Ejecutarlo primero |
| El nodo no termina tras la gráfica | `plt.show()` espera | Cerrar la ventana |
| La gráfica no abre, solo guarda | El código quedó en modo `Agg` | Cambiar a `TkAgg` y recompilar |
| Cambios que no surten efecto | Falta recompilar o sourcear | `colcon build` + `source install/setup.bash` |
| RRT* siempre tarda ~12 ms | Falta `ompl_planning.yaml` | Verificar `config/` |
| El robot no se mueve, solo el fantasma | `Loop Animation` activo en RViz | Desactivar en *Planned Path* |
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
