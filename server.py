"""
server.py — Backend del simulador 3D de rover marciano.

Responsabilidades:
  1. Generar el mundo: relieve con colinas y cráteres (mapa de alturas) y
     rocas repartidas por un terreno de 200 x 200 m.
  2. Mantener el estado del rover (posición, altura, inclinación, batería...)
     con una física básica de "pasos discretos": choca con rocas y no puede
     subir ni bajar pendientes demasiado empinadas.
  3. Exponer una API REST para controlarlo desde cualquier script externo:
        POST /api/control      -> ejecuta un comando de movimiento
        GET  /api/telemetry    -> estado actual del rover
        GET  /api/camera       -> imagen de la cámara frontal (HazCam) en base64
        GET  /api/world        -> relieve, cráteres y rocas (lo usa el frontend)
        POST /api/reset        -> reinicia la simulación
        POST /api/camera/frame -> el navegador sube aquí el último frame real
  4. Servir el frontend (index.html) como archivo estático.

Ejecución:  python server.py            ->  http://127.0.0.1:8000
            python server.py --seed 7   ->  otro mapa distinto

Convenciones de coordenadas (backend):
  - Plano XY en metros, origen en el centro del terreno. Z = altura.
  - heading en grados, 0° = eje +X, crece en sentido antihorario (convención
    matemática). El frontend convierte esto a coordenadas de Three.js.
"""

from __future__ import annotations

import argparse
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

TERRAIN_SIZE = 200.0         # Lado del terreno cuadrado (m)
HALF = TERRAIN_SIZE / 2
GRID_RES = 1.0               # Separación de la malla de alturas (m)
NUM_ROCKS = 110              # Rocas grandes (obstáculos con colisión)
NUM_CRATERS = 14             # Cráteres de impacto
EDGE_FADE = 12.0             # Franja del borde donde el relieve se aplana (m)
WORLD_SEED = 42              # Semilla -> mismo mapa en cada arranque (--seed N)

ROVER_RADIUS = 1.0           # Radio de colisión del rover (m)
PHYSICS_STEP = 0.05          # Resolución del barrido de colisión (m)
MAX_MOVE = 20.0              # Máximo avance por comando (m)
MAX_TURN = 360.0             # Máximo giro por comando (grados)
MAX_SLOPE_DEG = 25.0         # Pendiente peligrosa: el sensor avisa (no bloquea)
TIP_PITCH_DEG = 35.0         # Cabeceo a partir del cual el rover vuelca
TIP_ROLL_DEG = 30.0          # Alabeo a partir del cual el rover vuelca
WHEELBASE = 2.2              # Distancia entre ruedas delanteras y traseras (m)
TRACK_WIDTH = 1.8            # Distancia entre ruedas izquierda y derecha (m)

SENSOR_RANGE = 4.0           # Alcance del sensor de obstáculos (m)
SENSOR_FOV_DEG = 60.0        # Apertura del cono del sensor (grados)

BATTERY_PER_METER = 0.15     # % consumido por metro recorrido
BATTERY_PER_CLIMB = 1.0      # % extra por metro de desnivel subido
BATTERY_PER_DEGREE = 0.02    # % consumido por grado girado
SPEED_TIMEOUT = 1.0          # s sin comandos tras los cuales speed = 0

CAM_W, CAM_H = 320, 240      # Resolución de la HazCam sintética
CAM_FOV_DEG = 70.0
CAM_HEIGHT = 2.4             # Altura de la cámara sobre el suelo (m)
FRAME_MAX_AGE = 2.0          # Edad máxima del frame del navegador (s)


# --------------------------------------------------------------------------- #
#  Mundo: relieve (mapa de alturas) + cráteres + rocas
# --------------------------------------------------------------------------- #
@dataclass
class Crater:
    x: float
    y: float
    radius: float   # Radio del borde (m)
    depth: float    # Profundidad del fondo respecto al terreno (m)


@dataclass
class Rock:
    id: int
    x: float
    y: float
    radius: float   # Radio de colisión (m)
    height: float   # Altura visual (m)
    z: float = 0.0  # Altura del terreno en la base de la roca (m)


def _hash01(ix: int, iy: int, seed: int) -> float:
    """Pseudoaleatorio determinista en [0, 1) para un punto entero de la red."""
    h = (ix * 374761393 + iy * 668265263 + seed * 2147483647) & 0xFFFFFFFF
    h = ((h ^ (h >> 13)) * 1274126177) & 0xFFFFFFFF
    return ((h ^ (h >> 16)) & 0xFFFFFF) / 0x1000000


def _value_noise(x: float, y: float, seed: int) -> float:
    """Ruido de valor suavizado en [-1, 1]."""
    ix, iy = math.floor(x), math.floor(y)
    fx, fy = x - ix, y - iy
    ux, uy = fx * fx * (3 - 2 * fx), fy * fy * (3 - 2 * fy)
    a, b = _hash01(ix, iy, seed), _hash01(ix + 1, iy, seed)
    c, d = _hash01(ix, iy + 1, seed), _hash01(ix + 1, iy + 1, seed)
    return (a + (b - a) * ux + (c - a) * uy + (a - b - c + d) * ux * uy) * 2 - 1


def _base_relief(x: float, y: float, seed: int) -> float:
    """Colinas suaves: suma de octavas de ruido (fBm)."""
    h = 4.0 * _value_noise(x / 55, y / 55, seed)
    h += 1.2 * _value_noise(x / 18, y / 18, seed + 1)
    h += 0.3 * _value_noise(x / 6, y / 6, seed + 2)
    return h


def _crater_profile(d: float, c: Crater) -> float:
    """Perfil radial de un cráter: cuenco parabólico + borde elevado."""
    t = d / c.radius
    bowl = -c.depth * (1 - t * t) if t < 1 else 0.0
    rim = 0.35 * c.depth * math.exp(-((t - 1) / 0.28) ** 2)
    return bowl + rim


def generate_craters(seed: int) -> list[Crater]:
    rng = random.Random(seed + 100)
    craters: list[Crater] = []
    while len(craters) < NUM_CRATERS:
        r = rng.choice([rng.uniform(4, 8), rng.uniform(8, 14), rng.uniform(14, 22)])
        x = rng.uniform(-HALF + EDGE_FADE + r * 1.5, HALF - EDGE_FADE - r * 1.5)
        y = rng.uniform(-HALF + EDGE_FADE + r * 1.5, HALF - EDGE_FADE - r * 1.5)
        if math.hypot(x, y) < r * 1.5 + 12:          # Zona de arranque despejada
            continue
        if any(math.hypot(x - o.x, y - o.y) < (r + o.radius) * 0.9 for o in craters):
            continue
        # Algunos cráteres son suaves (transitables), otros de paredes empinadas
        depth = r * rng.uniform(0.12, 0.32)
        craters.append(Crater(round(x, 2), round(y, 2), round(r, 2), round(depth, 2)))
    return craters


def generate_heightmap(craters: list[Crater], seed: int) -> list[list[float]]:
    """Malla regular de alturas (fila = y, columna = x). La física y el
    frontend interpolan sobre esta misma malla."""
    n = int(TERRAIN_SIZE / GRID_RES) + 1
    grid: list[list[float]] = []
    for j in range(n):
        y = -HALF + j * GRID_RES
        row = []
        for i in range(n):
            x = -HALF + i * GRID_RES
            h = _base_relief(x, y, seed)
            h *= min(1.0, 0.35 + math.hypot(x, y) / 25)   # Arranque más llano
            for c in craters:
                d = math.hypot(x - c.x, y - c.y)
                if d < c.radius * 2:
                    h += _crater_profile(d, c)
            # Aplanar hacia el borde para empalmar con el paisaje lejano
            edge = min(HALF - abs(x), HALF - abs(y))
            h *= max(0.0, min(1.0, edge / EDGE_FADE))
            row.append(round(h, 3))
        grid.append(row)
    return grid


def generate_rocks(craters: list[Crater], seed: int, n: int = NUM_ROCKS) -> list[Rock]:
    """Rocas pseudoaleatorias reproducibles, sin solaparse ni invadir la zona
    de arranque del rover. Se agrupan más alrededor de los cráteres (eyecta)."""
    rng = random.Random(seed)
    rocks: list[Rock] = []
    tries = 0
    while len(rocks) < n and tries < n * 200:
        tries += 1
        if craters and rng.random() < 0.4:           # Eyecta junto al borde de un cráter
            c = rng.choice(craters)
            ang, dist = rng.uniform(0, 2 * math.pi), c.radius * rng.uniform(1.05, 1.8)
            x, y = c.x + math.cos(ang) * dist, c.y + math.sin(ang) * dist
        else:
            x = rng.uniform(-HALF + 4, HALF - 4)
            y = rng.uniform(-HALF + 4, HALF - 4)
        r = rng.uniform(0.5, 1.2) if rng.random() < 0.75 else rng.uniform(1.2, 2.2)
        if abs(x) > HALF - 4 or abs(y) > HALF - 4 or math.hypot(x, y) < 6:
            continue
        if any(math.hypot(x - o.x, y - o.y) < r + o.radius + 2.5 for o in rocks):
            continue
        rocks.append(Rock(len(rocks), round(x, 2), round(y, 2), round(r, 2),
                          round(r * rng.uniform(0.8, 1.4), 2)))
    return rocks


class World:
    """Todo lo estático del mapa, generado a partir de una semilla."""

    CELL = 8.0   # Tamaño de celda de la rejilla espacial de rocas (m)

    def __init__(self, seed: int = WORLD_SEED) -> None:
        self.seed = seed
        self.craters = generate_craters(seed)
        self.heights = generate_heightmap(self.craters, seed)
        self.rocks = generate_rocks(self.craters, seed)
        for r in self.rocks:
            r.z = round(self.height_at(r.x, r.y), 3)
        # Rejilla espacial para no revisar las 110 rocas en cada paso de física
        self._cells: dict[tuple[int, int], list[Rock]] = {}
        for r in self.rocks:
            self._cells.setdefault(self._cell(r.x, r.y), []).append(r)

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return math.floor(x / self.CELL), math.floor(y / self.CELL)

    def rocks_near(self, x: float, y: float) -> list[Rock]:
        """Rocas a menos de ~8 m de (x, y) (celda propia + 8 vecinas)."""
        cx, cy = self._cell(x, y)
        out: list[Rock] = []
        for i in (cx - 1, cx, cx + 1):
            for j in (cy - 1, cy, cy + 1):
                out.extend(self._cells.get((i, j), ()))
        return out

    def height_at(self, x: float, y: float) -> float:
        """Altura del terreno en (x, y). Interpola dentro del triángulo de la
        malla igual que lo dibuja Three.js, así el rover no flota ni se hunde."""
        n = len(self.heights) - 1
        gx = min(max((x + HALF) / GRID_RES, 0.0), n - 1e-6)
        gy = min(max((y + HALF) / GRID_RES, 0.0), n - 1e-6)
        i, j = int(gx), int(gy)
        fx, fy = gx - i, gy - j
        h = self.heights
        h00, h10, h01, h11 = h[j][i], h[j][i + 1], h[j + 1][i], h[j + 1][i + 1]
        # Cada celda se parte en 2 triángulos por la diagonal (i, j)-(i+1, j+1),
        # igual que la malla que arma index.html
        if fx >= fy:
            return h00 + (h10 - h00) * fx + (h11 - h10) * fy
        return h00 + (h01 - h00) * fy + (h11 - h01) * fx

    def slope_deg(self, x: float, y: float) -> float:
        """Pendiente del terreno en (x, y), en grados."""
        e = 0.5
        gx = (self.height_at(x + e, y) - self.height_at(x - e, y)) / (2 * e)
        gy = (self.height_at(x, y + e) - self.height_at(x, y - e)) / (2 * e)
        return math.degrees(math.atan(math.hypot(gx, gy)))


WORLD = World()


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
        self.overturned = False    # Volcado: no acepta más comandos hasta reset
        self.hazard: Optional[str] = None
        self._last_cmd_time = time.time()

    # ------------------------------------------------------------------ #
    @staticmethod
    def _hazard_at(x: float, y: float) -> Optional[str]:
        """Qué impide al rover estar en (x, y): 'edge', 'rock' o None.
        Las pendientes no bloquean: si son demasiado fuertes el rover vuelca."""
        if abs(x) > HALF - ROVER_RADIUS or abs(y) > HALF - ROVER_RADIUS:
            return "edge"
        if any(math.hypot(x - r.x, y - r.y) < r.radius + ROVER_RADIUS
               for r in WORLD.rocks_near(x, y)):
            return "rock"
        return None

    def _move(self, distance: float) -> tuple[float, float, Optional[str]]:
        """Avanza `distance` metros (negativo = retroceso) barriendo en pasos
        pequeños para detenerse justo antes de una colisión. Si en algún paso
        el cabeceo o el alabeo superan el límite, el rover vuelca ahí.
        Devuelve (metros recorridos, metros subidos, motivo: 'edge' | 'rock' |
        'overturned' | None)."""
        rad = math.radians(self.heading)
        dx, dy = math.cos(rad), math.sin(rad)
        sign = 1.0 if distance >= 0 else -1.0
        remaining = min(abs(distance), MAX_MOVE)
        moved = climbed = 0.0
        while remaining > 1e-9:
            step = min(PHYSICS_STEP, remaining)
            nx, ny = self.x + dx * step * sign, self.y + dy * step * sign
            hazard = self._hazard_at(nx, ny)
            if hazard:
                return moved, climbed, hazard
            climbed += max(0.0, WORLD.height_at(nx, ny) - WORLD.height_at(self.x, self.y))
            self.x, self.y = nx, ny
            moved += step
            remaining -= step
            _, pitch, roll = self._attitude_at(self.x, self.y, self.heading)
            if abs(pitch) > TIP_PITCH_DEG or abs(roll) > TIP_ROLL_DEG:
                self.overturned = True
                return moved, climbed, "overturned"
        return moved, climbed, None

    # ------------------------------------------------------------------ #
    def apply(self, action: str, value: float) -> dict:
        """Ejecuta un comando y devuelve un resumen del resultado."""
        with self._lock:
            applied = 0.0
            hazard: Optional[str] = None

            if self.overturned and action != "stop":
                self.speed = 0.0
                return self._result(action, 0.0, True, "overturned")
            if self.battery <= 0 and action != "stop":
                self.speed = 0.0
                return self._result(action, 0.0, False, "battery_empty")

            if action in ("forward", "backward"):
                signed = value if action == "forward" else -value
                applied, climbed, hazard = self._move(signed)
                self.odometer += applied
                self.battery -= applied * BATTERY_PER_METER + climbed * BATTERY_PER_CLIMB
                self.speed = value if not hazard else 0.0
            elif action in ("turn_left", "turn_right"):
                delta = min(value, MAX_TURN)
                self.heading = normalize_deg(
                    self.heading + (delta if action == "turn_left" else -delta))
                applied = delta
                self.battery -= delta * BATTERY_PER_DEGREE
                self.speed = 0.0
                # Girar de lado en una ladera también puede volcarlo
                _, pitch, roll = self.attitude()
                if abs(pitch) > TIP_PITCH_DEG or abs(roll) > TIP_ROLL_DEG:
                    self.overturned = True
                    hazard = "overturned"
            elif action == "stop":
                self.speed = 0.0

            self.battery = max(0.0, self.battery)
            self.blocked = hazard is not None
            self.hazard = hazard
            self._last_cmd_time = time.time()
            return self._result(action, applied, self.blocked,
                                "overturned" if self.overturned else "ok")

    def _result(self, action: str, applied: float, blocked: bool, status: str) -> dict:
        return {"ok": status == "ok", "status": status, "action": action,
                "applied": round(applied, 3), "blocked": blocked,
                "telemetry": self._telemetry_unlocked()}

    # ------------------------------------------------------------------ #
    def obstacle_ahead(self) -> tuple[bool, Optional[float], Optional[str]]:
        """Sensor tipo LIDAR simplificado: detecta lo más cercano dentro de un
        cono frontal de SENSOR_FOV_DEG y SENSOR_RANGE metros.
        Devuelve (hay_obstáculo, distancia, tipo) con tipo en
        'rock' | 'slope' | 'edge'."""
        heading_rad = math.radians(self.heading)
        half_fov = math.radians(SENSOR_FOV_DEG / 2)
        found: list[tuple[float, str]] = []

        for r in WORLD.rocks_near(self.x, self.y):
            dx, dy = r.x - self.x, r.y - self.y
            dist_surface = math.hypot(dx, dy) - r.radius - ROVER_RADIUS
            rel = math.atan2(dy, dx) - heading_rad
            rel = (rel + math.pi) % (2 * math.pi) - math.pi   # [-pi, pi]
            if abs(rel) <= half_fov and dist_surface <= SENSOR_RANGE:
                found.append((dist_surface, "rock"))

        # Pendientes peligrosas (riesgo de vuelco): 3 rayos dentro del cono
        for off in (-half_fov / 2, 0.0, half_fov / 2):
            ang = heading_rad + off
            d = 0.25
            while d <= SENSOR_RANGE:
                px = self.x + math.cos(ang) * (d + ROVER_RADIUS)
                py = self.y + math.sin(ang) * (d + ROVER_RADIUS)
                if abs(px) < HALF and abs(py) < HALF and WORLD.slope_deg(px, py) > MAX_SLOPE_DEG:
                    found.append((d, "slope"))
                    break
                d += 0.25

        # El borde del terreno también cuenta como obstáculo
        fx = self.x + math.cos(heading_rad) * (SENSOR_RANGE + ROVER_RADIUS)
        fy = self.y + math.sin(heading_rad) * (SENSOR_RANGE + ROVER_RADIUS)
        if abs(fx) > HALF or abs(fy) > HALF:
            found.append((min(HALF - abs(self.x), HALF - abs(self.y)) - ROVER_RADIUS, "edge"))

        if not found:
            return False, None, None
        dist, kind = min(found)
        return True, round(max(dist, 0.0), 2), kind

    def attitude(self) -> tuple[float, float, float]:
        """(altura, cabeceo, alabeo) del rover en su posición actual."""
        return self._attitude_at(self.x, self.y, self.heading)

    @staticmethod
    def _attitude_at(x: float, y: float, heading: float) -> tuple[float, float, float]:
        """(altura, cabeceo, alabeo) según el terreno bajo las ruedas.
        pitch > 0 = morro arriba; roll > 0 = lado izquierdo más alto."""
        rad = math.radians(heading)
        fx, fy = math.cos(rad), math.sin(rad)          # Vector frontal
        lx, ly = -fy, fx                               # Vector izquierdo
        hw, ht = WHEELBASE / 2, TRACK_WIDTH / 2
        h = WORLD.height_at
        front = h(x + fx * hw, y + fy * hw)
        back = h(x - fx * hw, y - fy * hw)
        left = h(x + lx * ht, y + ly * ht)
        right = h(x - lx * ht, y - ly * ht)
        z = (front + back + left + right) / 4
        pitch = math.degrees(math.atan2(front - back, WHEELBASE))
        roll = math.degrees(math.atan2(left - right, TRACK_WIDTH))
        return z, pitch, roll

    def _telemetry_unlocked(self) -> dict:
        if time.time() - self._last_cmd_time > SPEED_TIMEOUT:
            self.speed = 0.0
        ahead, dist, kind = self.obstacle_ahead()
        z, pitch, roll = self.attitude()
        return {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "z": round(z, 3),
            "heading": round(self.heading, 2),
            "pitch": round(pitch, 2),
            "roll": round(roll, 2),
            "speed": round(self.speed, 3),
            "battery": round(self.battery, 2),
            "obstacle_ahead": ahead,
            "obstacle_distance": dist,
            "obstacle_type": kind,
            "blocked": self.blocked,
            "blocked_by": self.hazard,
            "overturned": self.overturned,
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
    cam_z = t["z"] + CAM_HEIGHT

    visible = []
    for r in WORLD.rocks:
        dx, dy = r.x - t["x"], r.y - t["y"]
        rel = (math.atan2(dy, dx) - heading_rad + math.pi) % (2 * math.pi) - math.pi
        depth = math.hypot(dx, dy) * math.cos(rel)          # Profundidad frontal
        if depth < 0.5 or depth > 80 or abs(rel) > math.radians(CAM_FOV_DEG / 2 + 10):
            continue
        visible.append((depth, rel, r))

    # Pintar de lejos a cerca (algoritmo del pintor)
    for depth, rel, r in sorted(visible, key=lambda v: -v[0]):
        sx = CAM_W / 2 - math.tan(rel) * focal
        ground_y = horizon + focal * (cam_z - r.z) / depth
        w = focal * 2 * r.radius / depth
        h = focal * r.height / depth
        shade = int(max(40, 110 - depth * 1.5))
        draw.ellipse([sx - w / 2, ground_y - h, sx + w / 2, ground_y + w * 0.2],
                     fill=(shade, shade - 10, shade - 15), outline=(30, 25, 20))

    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


camera = CameraBuffer()


# --------------------------------------------------------------------------- #
#  API
# --------------------------------------------------------------------------- #
app = FastAPI(title="Simulador Rover Marciano", version="0.2.0")
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
    """Descripción estática del mundo (la usa el frontend para dibujar).
    `heightmap.data` es una lista plana fila a fila: la fila j corresponde a
    y = -size/2 + j * resolution y la columna i a x = -size/2 + i * resolution."""
    return {
        "seed": WORLD.seed,
        "terrain_size": TERRAIN_SIZE,
        "rover_radius": ROVER_RADIUS,
        "sensor_range": SENSOR_RANGE,
        "sensor_fov_deg": SENSOR_FOV_DEG,
        "max_slope_deg": MAX_SLOPE_DEG,
        "tip_pitch_deg": TIP_PITCH_DEG,
        "tip_roll_deg": TIP_ROLL_DEG,
        "rocks": [asdict(r) for r in WORLD.rocks],
        "craters": [asdict(c) for c in WORLD.craters],
        "heightmap": {
            "resolution": GRID_RES,
            "size": len(WORLD.heights),
            "data": [h for row in WORLD.heights for h in row],
        },
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
    parser = argparse.ArgumentParser(description="Simulador de rover marciano")
    parser.add_argument("--seed", type=int, default=WORLD_SEED,
                        help="semilla del mapa (cada número genera un terreno distinto)")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    if args.seed != WORLD.seed:
        WORLD = World(args.seed)
    print(f"Simulador de rover marciano (mapa #{WORLD.seed}) -> http://127.0.0.1:{args.port}")
    uvicorn.run(app, host="127.0.0.1", port=args.port, log_level="warning")
