# Taller de Cinemática y Planeación de Movimiento — FANUC LR Mate 200iD

Célula de ensamble simulada en ROS 2 + MoveIt2 + RViz2 que ejecuta un ciclo
completo de *pick-and-place* con evasión de obstáculo y perfiles de velocidad
cúbico y quíntico sobre los tramos de acercamiento fino.

**Universidad EIA — Ingeniería Mecatrónica — Robótica Industrial**
Parcial No. 2 · Equipo ET13 · Robot asignado: FANUC LR Mate 200iD

**Integrantes:** Juan David Botero · Jaime Landinez Ortiz · Carlos Daniel Murillo

---

## Ciclo implementado

| Tramo | Movimiento | Tipo | Método |
|---|---|---|---|
| 4A | HOME → pre_pick | Libre | RRTConnect (evade obstáculo) |
| 4B | pre_pick → pick | Cartesiano recto | Perfil quíntico, tramo rojo |
| 4C | pick → pre_place | Libre | RRTConnect |
| 4D | pre_place → place | Cartesiano recto | Perfil quíntico, tramo azul |

---

## Requisitos

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2
- Python: `numpy`, `matplotlib`
- MATLAB (verificación del modelo DH y del Jacobiano)

```bash
sudo apt install ros-jazzy-moveit python3-tk
pip3 install numpy matplotlib
```

---

## Estructura del repositorio

```
ws_fanuc/
├── src/
│   ├── fanuc_lrmate200id_support/         # URDF/Xacro del robot
│   ├── fanuc_lrmate200id_moveit_config/   # MoveIt config propio
│   │   └── config/
│   │       ├── fanuc_lrmate200id.srdf      # grupos y estados predefinidos
│   │       ├── kinematics.yaml             # solver KDL
│   │       ├── joint_limits.yaml
│   │       └── ompl_planning.yaml          # planeadores declarados
│   └── fanuc_lrmate200id_taller/          # nodos del taller
├── matlab/                                 # verificación DH y Jacobiano
├── GUIA_EJECUCION.md                       # guía paso a paso
└── README.md
```

> El paquete `fanuc_lrmate200id_moveit_config` fue construido desde cero con el
> MoveIt Setup Assistant a partir del URDF, según exige el enunciado. No
> proviene de ningún `moveit_config` preexistente.

---

## Compilación

```bash
git clone https://github.com/juandavidbc1-collab/FANUC_ID200_PR_JDB_JLO_CDM.git ws_fanuc
cd ws_fanuc
colcon build
source install/setup.bash
```

---

## Ejecución rápida

**Terminal 1** — simulación (dejar abierta):

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fanuc_lrmate200id_moveit_config demo.launch.py
```

**Terminal 2** — escena y ciclo completo:

```bash
cd ~/ws_fanuc
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run fanuc_lrmate200id_taller ros2_scene    # escena de colisión
ros2 run fanuc_lrmate200id_taller ros2_ciclo    # ciclo 4A → 4B → 4C → 4D
```

La escena de colisión reside en la memoria de `move_group` y se pierde al
reiniciar la simulación. Si aparecen fallos de planeación, volver a ejecutar
`ros2_scene` es el primer paso.

Guía detallada paso a paso en [`GUIA_EJECUCION.md`](GUIA_EJECUCION.md).

---

## Nodos del paquete

Todos se ejecutan como `ros2 run fanuc_lrmate200id_taller <nodo>`.

| Nodo | Parte | Descripción | Necesita simulación |
|---|---|---|---|
| `ros2_scene` | 4 | Publica mesas, pieza y obstáculo | Sí |
| `ros2_mover <pose>` | — | Mueve el robot a una pose nombrada | Sí |
| `ros2_ik` | 3 | IK de las cuatro poses con el solver de MoveIt2 | Sí |
| `ros2_planners` | 4A | Compara RRTConnect vs RRT* en 10 repeticiones | Sí |
| `ros2_perfiles` | 4B | Perfiles cúbico y quíntico, waypoints | No |
| `ros2_cart4b` | 4B | Trayectoria cartesiana + reparametrización temporal | Sí |
| `ros2_cart4d` | 4D | Ídem para el tramo azul | Sí |
| `ros2_jacobian` | 5 | Jacobiano analítico y verificación ẋ = J·q̇ | No |
| `ros2_ciclo` | 4 | Ejecuta la animación completa en RViz2 | Sí |

Poses válidas para `ros2_mover`: `home`, `pre_pick`, `pick`, `pre_place`,
`place`.

**Dependencia:** `ros2_jacobian` requiere los archivos `.npz` generados por
`ros2_cart4b` y `ros2_cart4d`.

Los resultados (gráficas, trayectorias, CSV) se escriben en `/tmp/taller_ri/`.

---

## Resultados por parte

### Parte 2 — Transformación homogénea en HOME

```bash
ros2 run fanuc_lrmate200id_taller ros2_mover home
ros2 run tf2_ros tf2_echo base_link flange
```

Resultado: `Translation: [0.465, 0.000, 0.695]`, rotación identidad.
Verificación en MATLAB con el modelo DH: error del orden de 1e-16.

### Parte 3 — Cinemática inversa

```bash
ros2 run fanuc_lrmate200id_taller ros2_ik
```

Poses definidas (efector apuntando hacia abajo, giro de 180° sobre Y):

| Pose | x | y | z |
|---|---|---|---|
| pre_pick | 0.40 | −0.25 | 0.43 |
| pick | 0.40 | −0.25 | 0.28 |
| pre_place | 0.40 | 0.30 | 0.43 |
| place | 0.40 | 0.30 | 0.28 |

Los ángulos obtenidos se verifican contra el modelo DH en MATLAB;
la matriz de orientación esperada es `diag(−1, 1, −1)`.

### Parte 4A — Comparación de planeadores

```bash
ros2 run fanuc_lrmate200id_taller ros2_planners
```

| Métrica | RRTConnect | RRT* |
|---|---|---|
| Tasa de éxito | **10/10** | 4/10 |
| Tiempo de planeación | **0.0139 ± 0.0013 s** | 2.0011 ± 0.0002 s |
| Longitud articular | 7.744 rad | **3.890 rad** |
| Longitud cartesiana | 0.940 m | **0.678 m** |
| Rugosidad (Σ Δ²q) | 0.00563 | **0.00298** |

**Planeador seleccionado: RRTConnect.** RRT* produce caminos un 50% más
cortos y un 47% más suaves, pero solo resuelve el 40% de los intentos con el
obstáculo presente. En una célula de producción, una tasa de éxito de 4/10 es
inaceptable por buena que sea la solución cuando aparece.

La diferencia se explica por la estructura de cada algoritmo: RRTConnect crece
dos árboles (desde el inicio y desde la meta) y los conecta, lo que le permite
atravesar pasajes estrechos; RRT* crece un solo árbol y dedica el presupuesto
restante a reconectar nodos, de modo que ante un obstáculo severo invierte su
tiempo optimizando un árbol que nunca alcanzó la meta.

### Parte 4B — Perfiles de interpolación

```bash
ros2 run fanuc_lrmate200id_taller ros2_perfiles   # perfiles teóricos
ros2 run fanuc_lrmate200id_taller ros2_cart4b     # trayectoria articular
```

Restricciones del tramo rojo: 0.200 m/s, 0.300 m/s² → **T = 1.7321 s**
(la restricción activa es la aceleración: √(6L/a) = 1.73 s frente a
1.5L/v = 1.13 s).

| Métrica | Cúbico | Quíntico |
|---|---|---|
| v_max cartesiana | 0.1299 m/s | 0.1624 m/s |
| a_max cartesiana | 0.3000 m/s² | 0.2887 m/s² |
| \|q̇\| máx | **0.3509 rad/s** | 0.4384 rad/s |
| \|q̈\| máx | **0.7827 rad/s²** | 0.7971 rad/s² |
| ∫a² dt articular | **0.5670** | 0.8775 |

**Perfil seleccionado: quíntico.** El cúbico presenta menores magnitudes en
todas las métricas, pero su aceleración salta de 0.41 a 0.78 rad/s² en los
primeros 0.06 s y vuelve a dispararse al final del tramo: esa discontinuidad
implica jerk no acotado justo en el instante de contacto con la pieza. El
quíntico impone aceleración nula en ambos extremos y concentra el esfuerzo en
el tramo medio, donde no hay contacto. En un acercamiento de agarre, el
criterio relevante no es minimizar el esfuerzo total sino el impacto mecánico.

### Parte 4D — Tramo azul

```bash
ros2 run fanuc_lrmate200id_taller ros2_cart4d
```

Restricciones: 0.100 m/s, 0.020 m/s² → **T = 6.5804 s**

El mismo desplazamiento de 15 cm dura casi cuatro veces más que en 4B, y la
aceleración domina por amplio margen (6.58 s frente a 2.81 s por velocidad).

### Parte 5 — Jacobiano

```bash
ros2 run fanuc_lrmate200id_taller ros2_jacobian
```

El Jacobiano geométrico se construye columna a columna como
`J[:,i] = [zᵢ × (pₑ − pᵢ) ; zᵢ]`.

**Validación cruzada:** el Jacobiano analítico se compara contra uno calculado
por diferencias finitas sobre la cinemática directa del URDF. Error máximo de
**2.3e-7** en las cuatro configuraciones evaluadas (HOME, pre_pick, pick,
place), consistente con el error de truncamiento del método numérico.

**Verificación de velocidades sobre 4B:** multiplicando el Jacobiano por las
velocidades articulares en cada instante se reproduce el perfil quíntico
impuesto, con error RMS de **1.1e-4 m/s**. La velocidad máxima del extremo es
**0.16215 m/s**, por debajo del límite de 0.200 m/s del tramo rojo.

**Singularidad de HOME:** en la configuración de reposo las columnas 4 y 6 del
Jacobiano son paralelas (J4 y J6 comparten eje de giro), por lo que el rango
cae de 6 a 5, `det(J·Jᵀ) = 0` y el número de condición se dispara. Es una
singularidad de muñeca, no un error de cálculo.

En MATLAB, las columnas 3 a 6 se multiplican por −1 respecto al modelo de ROS,
porque la tabla DH adoptada define `θᵢ = −qᵢ` para esos ejes. Es una diferencia
de convención: ambos modelos describen el mismo robot.

---

## Modelo Denavit–Hartenberg

Convención de Craig (DH modificado), obtenida del URDF:

| i | aᵢ₋₁ | αᵢ₋₁ | dᵢ | θᵢ |
|---|---|---|---|---|
| 1 | 0 | 0° | 0.330 | q₁ |
| 2 | 0.050 | −90° | 0 | q₂ − 90° |
| 3 | 0.330 | 0° | 0 | −q₃ |
| 4 | 0.035 | −90° | 0.335 | −q₄ |
| 5 | 0 | +90° | 0 | −q₅ |
| 6 | 0 | −90° | 0.080 | −q₆ |

El marco 6 de DH no coincide con el frame `flange` del URDF; se corrige con una
matriz constante:

```
T_corr = [0 0 1 0; 0 −1 0 0; 1 0 0 0; 0 0 0 1]
```

Validado contra la cinemática directa del URDF en 200 configuraciones
aleatorias con error máximo de 3.3e-16.

Los scripts de verificación están en `matlab/`:
`MatrixTDH.m` (Partes 2 y 3), `Jacobiano.m` (Parte 5), `PickPlace.m`.

---

## Notas de implementación

**Servicio `/compute_cartesian_path`.** En esta versión de MoveIt el parámetro
`max_step` no densifica la trayectoria: devuelve un punto por waypoint
independientemente de su valor (verificado entre 0.050 y 0.001 m). Los tramos
finos se resuelven por tanto generando los waypoints en Python según el perfil
y resolviendo el IK de cada uno, usando el punto anterior como semilla para
garantizar continuidad articular.

**Configuración de OMPL.** El `MoveItConfigsBuilder` carga los planeadores por
defecto del paquete `moveit_planners_ompl`, bajo los cuales RRT* termina en la
primera solución (~12 ms) sin optimizar. Fue necesario declarar
`ompl_planning.yaml` explícitamente con `simplify_solutions: false` para que
RRT* consumiera su presupuesto de tiempo y la comparación fuera significativa.

---

## Solución de problemas

| Síntoma | Causa probable | Solución |
|---|---|---|
| `Package not found` | Terminal sin sourcear | `source ~/ws_fanuc/install/setup.bash` |
| El nodo se cuelga sin imprimir | Espera un servicio de MoveIt | Levantar `demo.launch.py` |
| `error = 99999` / `-2` al planear | Escena no aplicada | `ros2_scene` |
| `GOAL_STATE_INVALID` en el log | El obstáculo toca al robot en la meta | Revisar posición del poste |
| `FileNotFoundError: traj_4B.npz` | `ros2_cart4b` no se ha ejecutado | Ejecutarlo primero |
| El nodo no termina tras la gráfica | `plt.show()` bloquea | Cerrar la ventana |
| Cambios que no surten efecto | Falta recompilar | `colcon build` + `source` |
| RRT* siempre tarda ~12 ms | Falta `ompl_planning.yaml` | Verificar `config/` |
| El robot no se mueve, solo el fantasma | `Loop Animation` activo en RViz | Desactivar en *Planned Path* |

**Códigos de MoveIt:** `1` = éxito · `−1` falla general · `−2` planeación
fallida · `−10` estado inicial en colisión · `−12` meta en colisión ·
`−31` sin solución de IK.
