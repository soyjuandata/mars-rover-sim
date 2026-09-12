# Simulador 3D de Rover Marciano (MVP)

Simulador mínimo para un curso introductorio de IA y robótica: backend en
Python (FastAPI) con física básica + frontend Three.js en un solo `index.html`.

## Arranque en 2 minutos

```bash
pip install -r requirements.txt
python server.py
```

Abre <http://127.0.0.1:8000> en el navegador. Controla el rover con
`W A S D` / flechas, `Espacio` para parar y `C` para alternar a la HazCam.

En otra terminal, prueba el control por script:

```bash
python client_test.py
```

## API

| Método | Ruta                 | Descripción |
|--------|----------------------|-------------|
| POST   | `/api/control`       | `{"action": "forward" \| "backward" \| "turn_left" \| "turn_right" \| "stop", "value": float}`. `value` = metros (avance) o grados (giro). |
| GET    | `/api/telemetry`     | `{"x", "y", "heading", "speed", "battery", "obstacle_ahead", "obstacle_distance", "blocked", "odometer", "timestamp"}` |
| GET    | `/api/camera`        | Imagen de la HazCam en base64 (`?format=raw` devuelve los bytes de la imagen). |
| GET    | `/api/world`         | Rocas y tamaño del terreno. |
| POST   | `/api/reset`         | Rover al origen con batería llena. |

Documentación interactiva (Swagger): <http://127.0.0.1:8000/docs>

## Notas de diseño

- **Coordenadas**: plano XY en metros, `heading` en grados con 0° = +X y
  sentido antihorario. El frontend traduce a Three.js (`x, 0, -y`).
- **Física**: los comandos se aplican de forma inmediata en pasos de 5 cm con
  barrido de colisión; el rover se detiene justo antes de una roca
  (`blocked: true`). La batería se descarga por metro y por grado.
- **Sensor**: `obstacle_ahead` es un cono frontal de 60° y 4 m de alcance.
- **Cámara**: el navegador sube su render real de la HazCam cada 250 ms. Si no
  hay navegador abierto, `/api/camera` devuelve una imagen sintética (Pillow)
  con las rocas proyectadas, así los scripts de visión siempre funcionan.
- **Mapa**: semilla fija (`WORLD_SEED` en `server.py`); cámbiala para otro terreno.
