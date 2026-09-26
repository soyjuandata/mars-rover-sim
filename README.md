# Simulador 3D de Rover Marciano (MVP)

Simulador mínimo para un curso introductorio de IA y robótica: backend en
Python (FastAPI) con física básica + frontend Three.js en un solo `index.html`.

## Arranque en 2 minutos

```bash
pip install -r requirements.txt
python server.py
```

Abre <http://127.0.0.1:8000> en el navegador (o <http://127.0.0.1:8000/#mision>
para saltar la intro en órbita).

- **Centro de control**: vista orbital de la zona. `✏ Trazar ruta` + clic en el
  terreno agrega waypoints (el último es el destino **B**); `▶ Iniciar` lanza un
  piloto básico que va en línea recta de punto a punto y se detiene si choca.
  Cada tramo se colorea según su riesgo (verde libre, naranja pendiente, rojo
  roca o vuelco). `⚠ Capa de peligro` resalta las pendientes.
- **Vista rover**: cámara de seguimiento + minimapa con la ruta y el rastro
  (clic en el minimapa = nuevo waypoint).
- Teclas: `W A S D` / flechas mover (pausa el piloto), `Espacio` parar,
  `Tab` centro de control ↔ rover, `C` HazCam.
- Desde la consola del navegador: `mission.add(x, y)`, `mission.start()`,
  `mission.pause()`, `mission.clear()`, `mission.status`.

En otra terminal, prueba el control por script:

```bash
python client_test.py
```

## API

| Método | Ruta                 | Descripción |
|--------|----------------------|-------------|
| POST   | `/api/control`       | `{"action": "forward" \| "backward" \| "turn_left" \| "turn_right" \| "stop", "value": float}`. `value` = metros (avance) o grados (giro). |
| GET    | `/api/telemetry`     | `{"x", "y", "z", "heading", "pitch", "roll", "speed", "battery", "obstacle_ahead", "obstacle_distance", "obstacle_type", "blocked", "blocked_by", "overturned", "odometer", "timestamp"}` |
| GET    | `/api/camera`        | Imagen de la HazCam en base64 (`?format=raw` devuelve los bytes de la imagen). |
| GET    | `/api/world`         | Relieve (`heightmap`), cráteres, rocas y límites (`max_slope_deg`, `tip_pitch_deg`, `tip_roll_deg`). |
| POST   | `/api/reset`         | Rover al origen con batería llena. |

Documentación interactiva (Swagger): <http://127.0.0.1:8000/docs>

## Notas de diseño

- **Coordenadas**: plano XY en metros, `heading` en grados con 0° = +X y
  sentido antihorario. El frontend traduce a Three.js (`x, z, -y`).
- **Física**: los comandos se aplican de forma inmediata en pasos de 5 cm con
  barrido de colisión; el rover se detiene justo antes de una roca o del borde
  (`blocked: true`, `blocked_by: "rock" | "edge"`). La batería se descarga por
  metro, por metro de subida y por grado.
- **Vuelco**: las pendientes no bloquean. Si el cabeceo supera 35° o el alabeo
  30° (al avanzar o al girar en una ladera), el rover vuelca: `overturned: true`,
  `status: "overturned"` y no acepta más comandos hasta `/api/reset`.
- **Sensor**: `obstacle_ahead` es un cono frontal de 60° y 4 m de alcance;
  `obstacle_type` es `"rock"`, `"edge"` o `"slope"` (pendiente > 25°, riesgo de vuelco).
- **Cámara**: el navegador sube su render real de la HazCam cada 250 ms. Si no
  hay navegador abierto, `/api/camera` devuelve una imagen sintética (Pillow)
  con las rocas proyectadas, así los scripts de visión siempre funcionan.
- **Mapa**: semilla fija (`WORLD_SEED` en `server.py`); cámbiala para otro terreno.
