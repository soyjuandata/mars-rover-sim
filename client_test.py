"""
client_test.py — Ejemplo mínimo de control del rover desde Python.

Requiere el servidor en marcha:  python server.py
Uso:                             python client_test.py

El bucle avanza de a 1 m hasta que el sensor frontal detecte un obstáculo,
momento en el que ordena 'stop'. Al final guarda una foto de la HazCam.
"""
import base64, time, requests

API = "http://127.0.0.1:8000/api"

requests.post(f"{API}/reset")                                              # Rover al origen
requests.post(f"{API}/control", json={"action": "turn_left", "value": 45})   # Girar 45° a la izquierda
for _ in range(30):
    requests.post(f"{API}/control", json={"action": "forward", "value": 1.0})  # Avanzar 1 m
    t = requests.get(f"{API}/telemetry").json()                              # Leer telemetría
    print(f"x={t['x']:6.2f}  y={t['y']:6.2f}  rumbo={t['heading']:5.1f}°  bat={t['battery']:5.1f}%  obstáculo={t['obstacle_ahead']}")
    if t["obstacle_ahead"]:
        requests.post(f"{API}/control", json={"action": "stop"}); print("Obstáculo detectado, rover detenido."); break
    time.sleep(0.3)                                                          # Pausa para verlo en el navegador

# Extra: capturar lo que ve la cámara frontal y guardarlo como imagen
cam = requests.get(f"{API}/camera").json()
ext = "jpg" if "jpeg" in cam["mime"] else "png"
open(f"hazcam.{ext}", "wb").write(base64.b64decode(cam["image_base64"]))
print(f"Foto guardada en hazcam.{ext} (origen: {cam['source']})")
print("terminamos")
