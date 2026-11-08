# ESP32 Ultrasonic Radar

A local browser dashboard for an ESP32-WROOM-32, HC-SR04 ultrasonic sensor, and one SG90 radar-scan servo.

## Included features

- Physical scan over a 160° front sector (servo positions 10°–170°).
- Frontend start/stop controls.
- Fixed scan durations from 5 seconds to 5 minutes, plus continuous mode.
- Live angle/bearing, distance, XY telemetry, and software track detection.
- Click-to-select a displayed track. Selection is visual only and is not sent to any motor.
- Simulator mode for testing the dashboard without the ESP32.

The two additional SG90 servos are not used by this version. The firmware does not accept target angles or target-aiming commands. Mount the radar servo so its 90° position points straight ahead; the dashboard then reports -80° on the left and +80° on the right.

## Wiring

| Part | Connection |
|---|---|
| HC-SR04 VCC | ESP32/VIN 5V |
| HC-SR04 GND | GND |
| HC-SR04 TRIG | GPIO 25 |
| HC-SR04 ECHO | GPIO 26 through a voltage divider |
| Radar SG90 signal | GPIO 27 |
| Radar SG90 red | External regulated 5V |
| Radar SG90 brown/black | External supply GND |

**Important:** HC-SR04 ECHO is a 5V signal. Do not connect it directly to ESP32 GPIO26. Use a resistor divider (for example, 1 kΩ between ECHO and GPIO26, plus 2 kΩ between GPIO26 and GND) so the ESP32 sees approximately 3.3V.

Power the servo from a separate regulated 5V supply, not the ESP32 3.3V pin. Connect the external supply ground to ESP32 GND. Do not power the servo from the ESP32 board's 5V pin if the board is USB-powered and the supply cannot handle the servo's current spikes. For one SG90, allow at least around 1A of 5V supply capacity; use a supply with more headroom if adding other loads.

## 1. Upload the ESP32 firmware

1. Open this project in VS Code with PlatformIO.
2. Check `upload_port = COM3` in `platformio.ini`; change it if Windows assigns another COM port.
3. Upload `src/main.cpp` to the ESP32.
4. Close PlatformIO Serial Monitor before starting the Python backend. A serial port normally cannot be shared by both programs at once.

The firmware starts idle and parks the radar servo at its centre. The dashboard starts the scan when you press **Start scan**.

## 2. Install backend dependencies

Use Python 3.10 or newer if possible.

```powershell
cd radar_project
py -m venv .venv
.\.venv\Scripts\Activate.ps1
py -m pip install -r requirements.txt
```

## 3. Run with the physical ESP32

In PowerShell, from the project root:

```powershell
$env:RADAR_SOURCE = "serial"
$env:ARDUINO_PORT = "COM3"
py backend/app.py
```

If PlatformIO uploaded on a different port, use that port instead of `COM3`. The ESP32's serial monitor must be closed while Flask is running.

## 4. Open the frontend

Open `index.html` in your browser. The frontend uses `http://127.0.0.1:5000` by default.

If you serve the frontend from another host or port, the backend currently allows cross-origin requests for local development. You can override the API address before loading the page by setting `window.RADAR_API_BASE` in a small script before `assets/js/app.js` is loaded.

## 5. Simulator mode

To test without hardware:

```powershell
$env:RADAR_SOURCE = "sim"
py backend/app.py
```

The simulator produces a 360° virtual sweep. Physical hardware scans only the front 160° sector.

## API

- `GET /api/health` — backend health.
- `GET /api/radar/latest` — latest telemetry and scan state.
- `GET /api/radar/stream` — Server-Sent Events stream.
- `POST /api/radar/control` — `{"action":"start","duration_s":10}` or `{"action":"stop"}`. Use `duration_s: 0` for continuous scanning.
- `POST /api/lock` — dashboard-only selected track state; does not control hardware.

## Troubleshooting

- **No serial port:** check Device Manager under Ports (COM & LPT), install the USB-to-UART driver matching the board's USB bridge (often CP210x or CH340), and set `ARDUINO_PORT`.
- **Serial port is busy:** close PlatformIO Serial Monitor and any other terminal using the ESP32.
- **Dashboard connects but there are no measurements:** choose `RADAR_SOURCE=serial`, verify the COM port, power the ESP32, and press **Start scan**.
- **Servo jitters or ESP32 resets:** use a separate stable 5V servo supply and connect grounds together.
- **Distance always says No echo:** check TRIG/ECHO wiring, the voltage divider, sensor power, and common ground.
