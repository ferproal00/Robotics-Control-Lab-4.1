# Robotics-Control-Challenge-4.1

Toda la data y resultados puede observarse en el siguiente link de drive:
https://drive.google.com/drive/folders/1Vdt2-x5febMTuPuAhb9r6yd3BAxPGrBG?usp=sharing

Video demostrativo:
https://www.youtube.com/watch?v=FFFMIsnSLWI

## Comparación de Controladores PD y CTC-like en el xArm Lite 6

**Materia:** TE3001B - Fundamentación de Robótica  
**Profesor:** Nezih Nieto Gutiérrez  
**Institución:** Tecnológico de Monterrey  
**Equipo:** 2 - REPO  
**Fecha:** 9 deMarzo 2026  

---

# Descripción del Proyecto

Este proyecto evalúa el desempeño de dos estrategias de control aplicadas al robot xArm Lite 6 ejecutando una tarea de manipulación consistente en transferir tornillos desde un feeder hacia cuatro posiciones de inserción organizadas en forma de cruz.

Se compararon dos controladores:

- Controlador *PD cartesiano* (base obtenida de la actividad Robotics Control Lab 4.2)
- Controlador *CTC*

Ambos controladores se evaluaron bajo condiciones experimentales idénticas, tanto con perturbaciones como sin perturbaciones.

---

# Tarea Experimental

El robot ejecuta un ciclo repetitivo de manipulación compuesto por:

1. Ir a posición segura central  
2. Ir al feeder de tornillos  
3. Descender para tomar el tornillo  
4. Regresar al centro  
5. Ir a posición de inserción  
6. Descender para insertar tornillo  
7. Retiro vertical  
8. Regresar al centro  

Este ciclo se repite para cuatro posiciones de inserción organizadas en forma de cruz cartesiana.

La trayectoria se define en espacio cartesiano y se ejecuta utilizando cinemática inversa diferencial mediante MoveIt Servo.

---

# Controladores Implementados

## Controlador PD

El controlador PD cartesiano calcula la velocidad deseada del efector final a partir del error de posición.

Ley de control:
v = Kp * e + Kd * e_dot

Características:

- Zona muerta (*deadband*)
- Saturación de velocidad
- Ganancias obtenidas en la actividad anterior

Parámetros utilizados:

kp = [7.0, 7.0, 7.0]
kd = [0.3, 0.3, 0.3]
max_speed = 0.18
deadband = 0.003


---

## Controlador CTC

El controlador CTC está inspirado en el Computed Torque Control, pero adaptado a la interfaz de control por velocidad disponible en el robot. En lugar de calcular torques, el controlador calcula aceleraciones cartesianas deseadas, que posteriormente se integran para generar comandos de velocidad.

Forma general:
a = Kp * e + Kd * e_dot


Esta aceleración se escala mediante una **masa virtual** y posteriormente se integra para generar el comando de velocidad.

Parámetros utilizados:
kp = [22.0, 18.0, 30.0]
kd = [8.0, 6.5, 11.0]
virtual_mass = [0.22, 0.20, 0.16]
max_speed = 0.23
max_accel = 2.20
deadband = 0.003


---

# Perturbaciones

Se aplicó ruido gaussiano a la velocidad cartesiana comandada.

Modelo de perturbación:
v_pert = v_cmd + n(t)

Las perturbaciones se activan *únicamente durante la fase de inserción del tornillo*, representando perturbaciones realistas que pueden ocurrir en procesos de ensamblaje.

---

# Experimentos Realizados

Siguiendo los requisitos del reto experimental, se realizaron exactamente cuatro pruebas:

| Prueba | Controlador | Perturbación |
|------|-----------|-------------|
| 1 | CTC | No |
| 2 | PD | No |
| 3 | CTC | Sí |
| 4 | PD | Sí |

Todas las pruebas utilizan la misma trayectoria y generador de referencias.

---

# Resumen de Resultados

| Controlador | Perturbación | RMSE XYZ |
|-------------|--------------|-----------|
| CTC | No | 0.043 m |
| PD | No | 0.049 m |
| CTC | Sí | 0.032 m |
| PD | Sí | 0.034 m |

Ambos controladores logran ejecutar correctamente la tarea.

El controlador CTC mostró mayor robustez frente a perturbaciones y un comportamiento dinámico más suave, mientras que el controlador PD ofrece un desempeño comparable con una estructura más simple.

---

# Cómo Ejecutar el Proyecto

## 1 - Preparar el workspace

```bash
cd ~/xarm_ws
source install/setup.bash
```

## 2 - Lanzar MoveIt Servo

```bash
ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py robot_ip:=192.168.1.154
```

## 3 - Ejecutar trayectoria
```bash
python3 ~/xarm_ws/src/lite6_task_ik/scripts/screw_cross_trajectory.py
```

## 4 - Ejecutar controlador
PD
```bash
ros2 run xarm_perturbations position_controller --ros-args \
-p target_topic:=/desired_position \
-p output_topic:=/controller/delta_twist_cmds \
-p kp:="[7.0,7.0,7.0]" \
-p kd:="[0.3,0.3,0.3]" \
-p max_speed:=0.18 \
-p deadband:=0.003
```

CTC
```bash
python3 ~/xarm_ws/src/lite6_task_ik/scripts/ctc_like_controller.py --ros-args
-p target_topic:=/desired_position
-p output_topic:=/controller/delta_twist_cmds
-p kp:="[22.0,18.0,30.0]"
-p kd:="[8.0,6.5,11.0]"
-p virtual_mass:="[0.22,0.20,0.16]"
-p max_speed:=0.23
-p max_accel:=2.20
-p deadband:=0.003
```

## 5 - Ejecutar perturbación gaussiana
```bash
python3 src/lite6_task_ik/scripts/conditional_perturbation_injector.py --ros-args \
  -p input_topic:=/controller/delta_twist_cmds \
  -p output_topic:=/servo_server/delta_twist_cmds \
  -p enable_topic:=/perturbation_enable \
  -p mode:=gaussian \
  -p gauss_std_linear:=0.03 \
  -p gauss_axis:=x \
  -p base_linear:="[0.0, 0.0, 0.0]" \
  -p seed:=12345
```

## 6 - Registrar datos con rosbag
Para registar
```bash
ros2 bag record -o ~/xarm_ws/bags/pd_withpert \
/desired_position \
/actual_position \
/position_error \
/commanded_speed \
/controller/delta_twist_cmds \
/servo_server/delta_twist_cmds \
/joint_states \
/perturbation_enable \
/task_phase
```
Para crear .csv's
```bash
python3 ~/xarm_ws/src/lite6_task_ik/scripts/export_rosbag_csv.py \
~/xarm_ws/bags/pd_withpert \
~/xarm_ws/bags_csv/pd_withpert
```

---
# Procesamiento de datos
Los datos después se analizan en *MATLAB* para obtener las gráficas de errores, velocidad, trayectoria, comparaciones, valroes RMSE y diagramas de fase de los joints.

Ejecutar:
```bash
analyze_xarm_trial.m
compare_two_trials.m
phase.m
```

Este script genera:

- Gráficas de seguimiento de trayectoria
- Cálculo de RMSE
- Análisis de velocidades comandadas
- Trayectorias articulares
- Diagramas de fase
- Trayectoria 3D del efector final

---

# Requisitos

- Ubuntu 22.04 (puede ser cualquier versión)
- ROS2 Humble (cambia según versión de Ubuntu)
- MoveIt 2
- xArm Lite 6
- Python 3
- MATLAB (opcional para análisis)

---

# Autores

Manuel Ferro Sánchez  
Zacbe Ortega Obregón  
Fabricio Banda Hernández  
Alexandro Kurt Cárdenas Pérez  
Fernando Proal Sifuentes
