# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project context

Simulador 3D de rover marciano usado como vehículo pedagógico de un curso
privado 1-a-1 de IA y robótica (metodología **vibecoding**): backend FastAPI
+ frontend Three.js, controlable por API REST. El objetivo final del curso es
llegar a un LLM que maneje el rover de forma autónoma (percibe vía
telemetría/cámara, decide, actúa vía `/api/control`).

## Commands

```bash
pip install -r requirements.txt   # fastapi, uvicorn, pillow, requests
python server.py                  # arranca en http://127.0.0.1:8000
python server.py --seed 7         # mismo simulador, otro mapa (rocas/relieve distintos)
python server.py --port 8001      # otro puerto
python client_test.py             # script de ejemplo: mueve el rover vía API y guarda una foto
```

No hay linter, test runner ni build step configurados en este repo — es un
MVP de curso, no un proyecto con CI.

Documentación interactiva de la API (Swagger, autogenerada por FastAPI) con
el servidor corriendo: `http://127.0.0.1:8000/docs`.

## Architecture

**`server.py`** (todo el backend, un solo archivo) tiene tres capas
independientes que conviene entender por separado:

1. **`World`** — terreno estático generado una sola vez a partir de
   `WORLD_SEED`: heightmap por ruido de valor (fBm, `_value_noise`/
   `_base_relief`), cráteres (`generate_craters`, perfil radial en
   `_crater_profile`) y rocas (`generate_rocks`, con eyecta agrupada
   alrededor de cráteres). Las rocas se indexan en una rejilla espacial
   (`World._cells`, celdas de 8 m) para que la detección de colisiones no
   recorra las ~110 rocas en cada paso de física — usar `rocks_near()`, no
   iterar `WORLD.rocks` directo, en cualquier código de física nuevo.
2. **`Rover`** — estado mutable + física, protegido por `threading.Lock()`
   porque FastAPI atiende requests concurrentes. `_move()` barre el
   desplazamiento en pasos de `PHYSICS_STEP` (5 cm) y corta en el primer
   `_hazard_at()` que detecte (`"edge"` | `"rock"`), así el rover
   frena justo antes del obstáculo en vez de atravesarlo. Las pendientes no
   bloquean: tras cada paso (y tras cada giro) se calcula la actitud con
   `_attitude_at()` y, si supera `TIP_PITCH_DEG`/`TIP_ROLL_DEG`, el rover queda
   `overturned` y rechaza comandos hasta `/api/reset`. El sensor
   `obstacle_ahead()` es un cono frontal (`SENSOR_FOV_DEG`/`SENSOR_RANGE`)
   independiente de la física de colisión — sirve para que un cliente decida
   *antes* de chocar.
3. **`CameraBuffer`** — si el navegador está abierto, sube frames reales
   renderizados por Three.js a `POST /api/camera/frame` (cada 250 ms); si no
   hay navegador o el frame expiró (`FRAME_MAX_AGE`), `GET /api/camera` cae a
   `render_synthetic_camera()` (proyección pinhole simplificada con Pillow),
   así los scripts de visión artificial siempre reciben algo aunque nadie
   tenga la pestaña abierta.

Convención de coordenadas compartida por todo el backend: plano XY en
metros, origen en el centro del terreno, `heading` en grados con 0° = eje +X
creciendo antihorario (matemática estándar, **no** como una brújula).

**`index.html`** es el frontend Three.js en un solo archivo: construye el
terreno desde `GET /api/world` (mismo heightmap que usa la física del
backend, para que lo que se ve y lo que choca coincidan), hace polling de
`GET /api/telemetry` para mover el rover 3D, y renderiza la HazCam a un
`WebGLRenderTarget` que sube como frame real a `/api/camera/frame`. Tiene
tres modos (`body.mode-globe|mission|rover`): intro con el globo de Marte,
centro de control orbital (waypoints, piloto básico en el navegador,
`window.mission`) y vista del rover con minimapa. La ruta y los marcadores
viven en capas de Three.js que la HazCam no ve, para no ensuciar la imagen
que usa la IA. `heightAt()`/`attitudeAt()` replican exactamente
`World.height_at`/`Rover._attitude_at`: si se cambia uno, cambiar el otro. El
mapeo de coordenadas backend→Three.js es `(x, alturaZ, -y)` — invertir el
eje Y es la parte que más confunde al portar coordenadas entre los dos
lados. Además de `"three"`, el importmap trae `"three/addons/"` (mismo CDN,
`examples/jsm/`) para `OrbitControls` (cámara del centro de control) y
`Line2`/`LineMaterial`/`LineGeometry` (rutas y rastro con grosor real, no
`LineBasicMaterial`).

Consola del navegador: `window.mission` expone `add(x, y)`, `start()`,
`pause()`, `clear()`, `reset()`, `status`, `waypoints` y `view(modo)` — útil
para depurar el piloto básico o, más adelante, para que la IA de Nico lo
reemplace por su propia lógica de decisión.

`client_test.py` es a la vez ejemplo de uso de la API y "solución de
referencia" de los ejercicios de vibecoding del curso (bucle que avanza
hasta detectar obstáculo).

## Course-specific conventions

- `.handoff/` (gitignorado) guarda notas de continuidad entre sesiones de
  Claude Code preparando cada clase — **leer el handoff más reciente ahí
  antes de preparar o continuar una clase**. No es contenido para el repo
  público del alumno.
- `presentaciones/` (gitignorado) tiene el material de slides del curso —
  vive fuera de git a propósito.
- Cambios grandes al simulador (agregar endpoints, campos de telemetría,
  etc.) normalmente ocurren *durante* una clase como ejercicio de vibecoding
  con el alumno — priorizar cambios chicos y verificables sobre refactors
  grandes.
- `.claude/settings.json` (este sí va al repo público, a diferencia de
  `.handoff/`) tiene un hook `Stop` que bloquea el cierre de la sesión si
  hubo cambios en el proyecto pero ningún `.handoff/*.md` se tocó desde que
  arrancó — fuerza a dejar la bitácora de continuidad al día. Se apoya en un
  hook `SessionStart` que escribe una marca de tiempo
  (`.claude/.session_start_marker`, gitignorado). Si el hook pide handoff al
  terminar, escribirlo antes de cerrar — no saltearlo.
