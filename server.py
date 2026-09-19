"""
server.py — Backend del simulador 3D de rover marciano (MVP).

Responsabilidades:
  1. Mantener el estado del rover (posición, rumbo, batería...) con una
     física básica de "pasos discretos" y detección de colisiones contra rocas.
  2. Exponer una API REST para controlarlo desde cualquier script externo:
        POST /api/control      -> ejecuta un comando de movimiento
        GET  /api/telemetry    -> estado actual del rover
        GET  /api/camera       -> imagen de la cámara frontal (HazCam) en base64
        GET  /api/world        -> rocas y tamaño del terreno (lo usa el frontend)
        POST /api/reset        -> reinicia la simulación
        POST /api/camera/frame -> el navegador sube aquí el último frame real
  3. Servir el frontend (index.html) como archivo estático.

Ejecución:  python server.py   ->  http://127.0.0.1:8000

Convenciones de coordenadas (backend):
  - Plano XY en metros, origen en el centro del terreno.
  - heading en grados, 0° = eje +X, crece en sentido antihorario (convención
    matemática). El frontend convierte esto a coordenadas de Three.js.
"""

from __future__ import annotations

import base64
import io
import json
import math
import random
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal, Optional

import uvicorn
from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field

# --------------------------------------------------------------------------- #
#  Configuración de la simulación
# --------------------------------------------------------------------------- #
BASE_DIR = Path(__file__).resolve().parent

TERRAIN_SIZE = 60.0          # Lado del terreno cuadrado (m)
HALF = TERRAIN_SIZE / 2
NUM_ROCKS = 8                # Entre 5 y 10 obstáculos
WORLD_SEED = 42              # Semilla fija -> mismo mapa en cada arranque

ROVER_RADIUS = 1.0           # Radio de colisión del rover (m)
PHYSICS_STEP = 0.05          # Resolución del barrido de colisión (m)
MAX_MOVE = 20.0              # Máximo avance por comando (m)
MAX_TURN = 360.0             # Máximo giro por comando (grados)

SENSOR_RANGE = 4.0           # Alcance del sensor de obstáculos (m)
SENSOR_FOV_DEG = 60.0        # Apertura del cono del sensor (grados)

BATTERY_PER_METER = 0.5      # % consumido por metro recorrido
BATTERY_PER_DEGREE = 0.02    # % consumido por grado girado
SPEED_TIMEOUT = 1.0          # s sin comandos tras los cuales speed = 0

CAM_W, CAM_H = 320, 240      # Resolución de la HazCam sintética
CAM_FOV_DEG = 70.0
CAM_HEIGHT = 1.6             # Altura de la cámara sobre el suelo (m)
FRAME_MAX_AGE = 2.0          # Edad máxima del frame del navegador (s)


# --------------------------------------------------------------------------- #
#  Mundo: rocas
# --------------------------------------------------------------------------- #
@dataclass
class Rock:
    id: int
    x: float
    y: float
    radius: float   # Radio de colisión (m)
    height: float   # Altura visual (m)


def generate_rocks(n: int = NUM_ROCKS, seed: int = WORLD_SEED) -> list[Rock]:
    """Genera rocas pseudoaleatorias reproducibles, sin solaparse ni
    invadir la zona de arranque del rover (radio 6 m alrededor del origen)."""
    rng = random.Random(seed)
    rocks: list[Rock] = []
    while len(rocks) < n:
        x = rng.uniform(-HALF + 4, HALF - 4)
        y = rng.uniform(-HALF + 4, HALF - 4)
        r = rng.uniform(0.6, 1.6)
        if math.hypot(x, y) < 6:
            continue
        if any(math.hypot(x - o.x, y - o.y) < r + o.radius + 2.5 for o in rocks):
            continue
        rocks.append(Rock(len(rocks), round(x, 2), round(y, 2), round(r, 2),
                          round(r * rng.uniform(0.8, 1.4), 2)))
    return rocks


ROCKS: list[Rock] = generate_rocks()


# --------------------------------------------------------------------------- #
#  Estado y física del rover
# --------------------------------------------------------------------------- #
def normalize_deg(angle: float) -> float:
    """Normaliza un ángulo a [0, 360)."""
    return angle % 360.0


class Rover:
    """Modelo cinemático del rover. Thread-safe mediante un lock, ya que
    FastAPI puede atender peticiones concurrentes."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.reset()

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.heading = 90.0        # Mira hacia +Y ("norte")
        self.speed = 0.0
        self.battery = 100.0
        self.odometer = 0.0
        self.blocked = False
        self._last_cmd_time = time.time()

    # ------------------------------------------------------------------ #
    def _collides(self, x: float, y: float) -> bool:
        """True si el rover en (x, y) toca una roca o sale del terreno."""
        if abs(x) > HALF - ROVER_RADIUS or abs(y) > HALF - ROVER_RADIUS:
            return True
        return any(math.hypot(x - r.x, y - r.y) < r.radius + ROVER_RADIUS for r in ROCKS)

    def _move(self, distance: float) -> tuple[float, bool]:
        """Avanza `distance` metros (negativo = retroceso) barriendo en pasos
        pequeños para detenerse justo antes de una colisión.
        Devuelve (metros realmente recorridos, bloqueado)."""
        rad = math.radians(self.heading)
        dx, dy = math.cos(rad), math.sin(rad)
        sign = 1.0 if distance >= 0 else -1.0
        remaining = min(abs(distance), MAX_MOVE)
        moved = 0.0
        while remaining > 1e-9:
            step = min(PHYSICS_STEP, remaining)
            nx, ny = self.x + dx * step * sign, self.y + dy * step * sign
            if self._collides(nx, ny):
                return moved, True
            self.x, self.y = nx, ny
            moved += step
            remaining -= step
        return moved, False

    # ------------------------------------------------------------------ #
    def apply(self, action: str, value: float) -> dict:
        """Ejecuta un comando y devuelve un resumen del resultado."""
        with self._lock:
            applied = 0.0
            blocked = False

            if self.battery <= 0 and action != "stop":
                self.speed = 0.0
                return self._result(action, 0.0, False, "battery_empty")

            if action in ("forward", "backward"):
                signed = value if action == "forward" else -value
                applied, blocked = self._move(signed)
                self.odometer += applied
                self.battery -= applied * BATTERY_PER_METER
                self.speed = value if not blocked else 0.0
            elif action in ("turn_left", "turn_right"):
                delta = min(value, MAX_TURN)
                self.heading = normalize_deg(
                    self.heading + (delta if action == "turn_left" else -delta))
                applied = delta
                self.battery -= delta * BATTERY_PER_DEGREE
                self.speed = 0.0
            elif action == "stop":
                self.speed = 0.0

            self.battery = max(0.0, self.battery)
            self.blocked = blocked
            self._last_cmd_time = time.time()
            return self._result(action, applied, blocked, "ok")

    def _result(self, action: str, applied: float, blocked: bool, status: str) -> dict:
        return {"ok": status == "ok", "status": status, "action": action,
                "applied": round(applied, 3), "blocked": blocked,
                "telemetry": self._telemetry_unlocked()}

    # ------------------------------------------------------------------ #
    def obstacle_ahead(self) -> tuple[bool, Optional[float]]:
        """Sensor tipo LIDAR simplificado: detecta la roca más cercana dentro
        de un cono frontal de SENSOR_FOV_DEG y SENSOR_RANGE metros.
        Devuelve (hay_obstáculo, distancia_a_su_superficie)."""
        heading_rad = math.radians(self.heading)
        half_fov = math.radians(SENSOR_FOV_DEG / 2)
        nearest: Optional[float] = None
        for r in ROCKS:
            dx, dy = r.x - self.x, r.y - self.y
            dist_surface = math.hypot(dx, dy) - r.radius - ROVER_RADIUS
            rel = math.atan2(dy, dx) - heading_rad
            rel = (rel + math.pi) % (2 * math.pi) - math.pi   # [-pi, pi]
            if abs(rel) <= half_fov and dist_surface <= SENSOR_RANGE:
                nearest = dist_surface if nearest is None else min(nearest, dist_surface)
        # El borde del terreno también cuenta como obstáculo
        fx = self.x + math.cos(heading_rad) * (SENSOR_RANGE + ROVER_RADIUS)
        fy = self.y + math.sin(heading_rad) * (SENSOR_RANGE + ROVER_RADIUS)
        if abs(fx) > HALF or abs(fy) > HALF:
            edge = min(HALF - abs(self.x), HALF - abs(self.y)) - ROVER_RADIUS
            nearest = edge if nearest is None else min(nearest, edge)
        if nearest is None:
            return False, None
        return True, round(max(nearest, 0.0), 2)

    def _telemetry_unlocked(self) -> dict:
        if time.time() - self._last_cmd_time > SPEED_TIMEOUT:
            self.speed = 0.0
        ahead, dist = self.obstacle_ahead()
        return {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "heading": round(self.heading, 2),
            "speed": round(self.speed, 3),
            "battery": round(self.battery, 2),
            "obstacle_ahead": ahead,
            "obstacle_distance": dist,
            "blocked": self.blocked,
            "odometer": round(self.odometer, 3),
            "timestamp": time.time(),
        }

    def telemetry(self) -> dict:
        with self._lock:
            return self._telemetry_unlocked()

    def do_reset(self) -> dict:
        with self._lock:
            self.reset()
            return self._telemetry_unlocked()


rover = Rover()


# --------------------------------------------------------------------------- #
#  Cámara frontal (HazCam)
# --------------------------------------------------------------------------- #
class CameraBuffer:
    """Guarda el último frame real que sube el navegador. Si no hay navegador
    conectado (o el frame es viejo) se genera una imagen sintética con Pillow
    para que los scripts de visión artificial siempre reciban algo útil."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._data: Optional[bytes] = None
        self._mime = "image/jpeg"
        self._time = 0.0

    def push(self, data: bytes, mime: str) -> None:
        with self._lock:
            self._data, self._mime, self._time = data, mime, time.time()

    def latest(self) -> tuple[bytes, str, str]:
        """Devuelve (bytes, mime, origen) donde origen es 'browser' o 'synthetic'."""
        with self._lock:
            if self._data is not None and time.time() - self._time < FRAME_MAX_AGE:
                return self._data, self._mime, "browser"
        return render_synthetic_camera(), "image/png", "synthetic"


def render_synthetic_camera() -> bytes:
    """Proyección pinhole muy simple de las rocas sobre una imagen 2D:
    cielo arriba, suelo abajo y elipses oscuras donde hay rocas.
    Suficiente para practicar segmentación por color / detección de blobs."""
    from PIL import Image, ImageDraw   # Import perezoso: solo si hace falta

    img = Image.new("RGB", (CAM_W, CAM_H), (232, 184, 150))        # Cielo
    draw = ImageDraw.Draw(img)
    horizon = CAM_H // 2
    draw.rectangle([0, horizon, CAM_W, CAM_H], fill=(181, 80, 42))  # Suelo

    focal = (CAM_W / 2) / math.tan(math.radians(CAM_FOV_DEG / 2))
    t = rover.telemetry()
    heading_rad = math.radians(t["heading"])

    visible = []
    for r in ROCKS:
        dx, dy = r.x - t["x"], r.y - t["y"]
        rel = (math.atan2(dy, dx) - heading_rad + math.pi) % (2 * math.pi) - math.pi
        depth = math.hypot(dx, dy) * math.cos(rel)          # Profundidad frontal
        if depth < 0.5 or abs(rel) > math.radians(CAM_FOV_DEG / 2 + 10):
            continue
        visible.append((depth, rel, r))

    # Pintar de lejos a cerca (algoritmo del pintor)
    for depth, rel, r in sorted(visible, key=lambda v: -v[0]):
        sx = CAM_W / 2 - math.tan(rel) * focal
        ground_y = horizon + focal * CAM_HEIGHT / depth
        w = focal * 2 * r.radius / depth
        h = focal * r.height / depth
        shade = int(max(40, 110 - depth * 3))
        draw.ellipse([sx - w / 2, ground_y - h, sx + w / 2, ground_y + w * 0.2],
                     fill=(shade, shade - 10, shade - 15), outline=(30, 25, 20))

    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


camera = CameraBuffer()


# --------------------------------------------------------------------------- #
#  API
# --------------------------------------------------------------------------- #
app = FastAPI(title="Simulador Rover Marciano", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class ControlCommand(BaseModel):
    """Comando de control. `value` = metros para forward/backward,
    grados para turn_left/turn_right; se ignora en stop."""
    action: Literal["forward", "backward", "turn_left", "turn_right", "stop"]
    value: float = Field(default=1.0, ge=0.0, le=360.0)


class CameraFrame(BaseModel):
    """Frame que sube el navegador (data URL o base64 puro)."""
    image_base64: str
    format: Literal["jpeg", "png"] = "jpeg"


@app.get("/")
def index() -> FileResponse:
    """Sirve el frontend Three.js."""
    return FileResponse(BASE_DIR / "index.html")


@app.post("/api/control")
def control(cmd: ControlCommand) -> dict:
    """Ejecuta un comando de movimiento y devuelve el resultado + telemetría."""
    return rover.apply(cmd.action, cmd.value)


@app.get("/api/telemetry")
def telemetry() -> dict:
    """Estado actual del rover."""
    return rover.telemetry()


@app.get("/api/world")
def world() -> dict:
    """Descripción estática del mundo (la usa el frontend para dibujar)."""
    return {
        "terrain_size": TERRAIN_SIZE,
        "rover_radius": ROVER_RADIUS,
        "sensor_range": SENSOR_RANGE,
        "sensor_fov_deg": SENSOR_FOV_DEG,
        "rocks": [asdict(r) for r in ROCKS],
    }


@app.post("/api/reset")
def reset() -> dict:
    """Devuelve el rover al origen con batería llena."""
    return rover.do_reset()


@app.post("/api/camera/frame")
def upload_frame(frame: CameraFrame) -> dict:
    """Recibe el frame real renderizado por el navegador."""
    payload = frame.image_base64.split(",", 1)[-1]   # Quita 'data:image/...;base64,'
    try:
        data = base64.b64decode(payload)
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(400, f"base64 inválido: {exc}") from exc
    camera.push(data, f"image/{frame.format}")
    return {"ok": True, "bytes": len(data)}


@app.get("/api/camera")
def get_camera(format: Literal["json", "raw"] = Query("json")) -> Response:
    """Imagen de la cámara frontal.
      - format=json (defecto): {"image_base64", "mime", "source", ...}
      - format=raw: bytes de la imagen directamente (para <img> o PIL)."""
    data, mime, source = camera.latest()
    if format == "raw":
        return Response(content=data, media_type=mime)
    t = rover.telemetry()
    body = {
        "image_base64": base64.b64encode(data).decode("ascii"),
        "mime": mime,
        "source": source,          # 'browser' = render real, 'synthetic' = Pillow
        "width": CAM_W,
        "height": CAM_H,
        "heading": t["heading"],
        "timestamp": t["timestamp"],
    }
    return Response(content=json.dumps(body), media_type="application/json")


# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    print('hola')
    print("Simulador de rover marciano -> http://127.0.0.1:8000")
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="warning")
