# Celda Robótica de Panadería Automatizada

Simulación en CoppeliaSim (API remota ZMQ) de una celda robótica que detecta
galletas de distintas formas sobre una cinta transportadora circular, las
clasifica por visión artificial y las desvía con un brazo robótico de 4 GDL
hacia el destino correcto. El brazo fue modelado en CoppeliaSim a partir de
un diseño mecánico open source (EEZYbotARM MK2, https://www.instructables.com/EEZYbotARM-Mk2-3D-Printed-Robot/) hecho en OnShape.

## Requisitos

```bash
pip install coppeliasim-zmqremoteapi-client opencv-python numpy
```

- CoppeliaSim 4.x abierto, con la escena del proyecto cargada.
- El servidor de la API remota ZMQ viene incluido en CoppeliaSim 4.x (no
  requiere add-ons extra).
- Este proyecto fue implementado en Linux Ubuntu 24.04.

## Estructura del proyecto

| Archivo | Función |
|---|---|
| `config.py` | Constantes: geometría, límites, calibración. |
| `kinematics.py` | Cinemática inversa/directa del brazo. |
| `trajectory.py` | Planificación de trayectorias suaves (S-curve). |
| `grasp.py` | Agarre adaptativo con sensor de fuerza. |
| `routine.py` | Carga y ejecución de rutinas (waypoints). |
| `sim_interface.py` | Comunicación con CoppeliaSim. |
| `bakery.py` | Pipeline completo: visión + clasificación + control del brazo. |
| `main.py` | Ejecuta una rutina puntual (JSON) sobre el brazo. |
| `make_routine.py` | Genera archivos JSON de rutinas (waypoints). |
| `routines/` | Rutinas guardadas en formato JSON. |

## 1. Correr la simulación completa (`bakery.py`)

Este es el modo de operación principal: detecta, clasifica y empuja las
galletas de forma autónoma y continua.

1. Abrir la escena en CoppeliaSim (no hace falta darle Play; el script
   arranca la simulación por su cuenta).
2. Ejecutar:

```bash
python3 bakery.py
```

El script se conecta, inicia la simulación, y lanza en paralelo la
generación de galletas, la limpieza de objetos caídos, el control de
velocidad de la cinta y el bucle de detección/empuje del brazo.
Para detener, `Ctrl+C` o cerrar el proceso.

> En caso de hacer cambios sobre la escena `panaderia_ebmk_final.ttt`:
> Antes de la primera corrida, revisar los valores marcados `TUNE` en
> `bakery.py` (rango de color HSV, puntos de calibración de cámara, umbrales
> de clasificación de forma) según tu escena. Los otros hiperparámetros a cambiar se encuentran en `config.py`.
> Para cambiar las posiciones de origen y target de empujes (si se quisiera mover los puentes por ejemplo), dentro de la escena de coppeliaSim, internamente en el modelo del brazo robótico
> hay unos Dummies dedicados a eso, que el código internamente lee y sigue. 

## 2. Crear una rutina (`make_routine.py`)

Una rutina es una secuencia de waypoints (posición x, y, z + estado del
gripper) guardada en un JSON. `make_routine.py` genera estos archivos sin
necesidad de escribir el JSON a mano.

1. Abrir `make_routine.py` y editar el bloque `if __name__ == "__main__":`
   con las coordenadas deseadas, por ejemplo para un pick-and-place:

```python
pick_and_place = gen.pick_and_place_waypoints(
    home_x=0.12, home_y=0.0, home_z=0.155,
    safe_z=0.17,
    object_x=0.20, object_y=0.10, object_z=0.156,
    target_x=0.20, target_y=-0.10, target_z=0.156,
    loop_reps=2,          # 0 = una sola pasada; >0 = repite ida y vuelta
)
gen.save_waypoints_to_json(pick_and_place, "pick_and_place_final.json")
```

   También existe `push_object_waypoints(...)` para rutinas simples de
   empuje (usadas por `bakery.py`), con la opción `homeback=True` para
   volver al punto de origen tras empujar.

2. Ejecutar el generador:

```bash
python3 make_routine.py
```

Esto crea el archivo JSON (por ejemplo `pick_and_place_final.json`) con las
coordenadas ya calculadas, listo para usar con `main.py`.

## 3. Correr una rutina (`main.py`)

Con CoppeliaSim abierto (la escena cargada, no hace falta iniciar la
simulación manualmente):

```bash
python3 main.py routines/pick_and_place_final.json
```

Si no se indica un archivo, `main.py` usa por defecto
`routines/pick_and_place_final.json`. El script se conecta al simulador,
inicia la simulación y ejecuta la rutina completa a través del pipeline de
cinemática + trayectoria jerk-limited.

## Flujo típico de trabajo

1. Editar `make_routine.py` con las coordenadas deseadas → generar el JSON.
2. Probar la rutina de forma aislada con `main.py archivo.json`.
3. Si la rutina es de empuje (push) y se ajusta bien, integrarla o
   verificarla dentro del pipeline automático de `bakery.py`.
