# Taller ROS2/MoveIt2 — FANUC LR Mate 200iD

Universidad EIA — Robótica Industrial — Parcial
Equipo ET13 — Robot asignado: FANUC LR Mate 200iD
Intregrantes:Juan David Botero-Jaime Landinez Ortiz-Carlos Daniel Murillo
## Contenido

- `src/fanuc_lrmate200id_support/` — paquete de descripción (URDF/Xacro)
- `src/fanuc_lrmate200id_moveit_config/` — configuración de MoveIt2 construida
  desde cero con el Setup Assistant a partir del URDF
- `scripts/` — scripts de Python para cinemática, planeación y perfiles

## Requisitos

- Ubuntu 24.04, ROS 2 Jazzy
- MoveIt2, numpy, matplotlib

## Compilación

```bash
cd ~/ws_fanuc
colcon build
source install/setup.bash
```

## Ejecución

Terminal 1 — simulación:

```bash
ros2 launch fanuc_lrmate200id_moveit_config demo.launch.py
```

Terminal 2 — escena y ciclo:

```bash
source ~/ws_fanuc/install/setup.bash
python3 scripts/scene.py      # objetos de colisión (mesas, pieza, obstáculo)
python3 scripts/ciclo.py      # ciclo completo 4A -> 4B -> 4C -> 4D
```

## Scripts por parte del taller

| Parte | Script | Qué hace |
|---|---|---|
| 3 | `ik4.py` | IK de pre_pick, pick, pre_place y place con el solver de MoveIt2 |
| 4A | `scene.py` | Publica la escena de colisión |
| 4A | `cmp2.py` | Compara RRTConnect vs RRT* (tiempo, longitud, suavidad, éxito) |
| 4B | `perfiles.py` | Perfiles cúbico y quíntico, gráficas y waypoints |
| 4B | `cart2.py` | Trayectoria cartesiana con reparametrización temporal |
| 4D | `cart4d.py` | Ídem para el tramo azul (0.100 m/s, 0.020 m/s²) |
| 5 | `jacobiano.py` | Jacobiano analítico y verificación ẋ = J·q̇ |
| — | `ciclo.py` | Ejecuta el ciclo completo en RViz2 |

Las Partes 2, 3 y 5 incluyen además verificación en MATLAB con el modelo DH
(convención de Craig); ver `matlab/`.

## Resultados principales

**Planeador (4A).** RRTConnect: 10/10 éxitos, 0.0139 s, 7.744 rad.
RRT*: 4/10 éxitos, 2.001 s, 3.890 rad. Se eligió RRTConnect por confiabilidad:
RRT* produce caminos más cortos y suaves pero falla en el 60% de los intentos
con el obstáculo presente.

**Perfil (4B).** Se eligió el quíntico. Aunque el cúbico tiene menores
magnitudes de velocidad y aceleración, presenta discontinuidad de aceleración
en los extremos (jerk no acotado), lo que produce impacto mecánico justo en el
instante de agarre. El quíntico arranca y termina con aceleración nula.

**Jacobiano (5).** El Jacobiano analítico coincide con la verificación numérica
por diferencias finitas con error de 2.3e-7. La velocidad del extremo calculada
como ẋ = J·q̇ reproduce el perfil quíntico con error RMS de 1.1e-4 m/s, con
v_max = 0.162 m/s bajo el límite de 0.200 m/s del tramo rojo.

**Nota.** La configuración HOME es singular: las columnas 4 y 6 del Jacobiano
son paralelas (J4 y J6 comparten eje), por lo que det(J·Jᵀ) = 0.
MDEOF
