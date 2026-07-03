<div align="center">

# ESP32-CAM

**QR code scanner with live camera feed on ESP32-CAM**

</div>

QR code scanner built on the AI-Thinker ESP32-CAM module. The camera captures 640x480 JPEG frames, processes them through the quirc library for real-time QR detection, and exposes a self-hosted web UI — live MJPEG stream plus decoded QR text — over its own WiFi access point.

## ■ Features

- ❖ **Real-time QR scanning** — JPEG frames decoded to grayscale and run through quirc every 300ms at 640x480
- ❖ **WiFi AP** — standalone WPA2 access point (`ESP32-CAM`, password `12345678`), no router needed
- ❖ **Web UI** — built-in HTML page polling decoded text via a `/qr` JSON endpoint
- ❖ **MJPEG live stream** — separate HTTP server on port 81 (`/stream`) so the feed never blocks `/qr`
- ❖ **2-second hold** — detected QR result persists for 2s to prevent flicker
- ❖ **ESP-IDF native** — built on the esp-idf framework (not Arduino) for full hardware control
- ❖ **Thread-safe** — mutex-guarded QR result buffer for concurrent HTTP access
- ❖ **PSRAM-backed** — RGB frame buffer and quirc structures allocated in PSRAM

## ■ Stack

<div align="center">

| Component | Technology |
|-----------|------------|
| MCU | ESP32 (AI-Thinker ESP32-CAM, PSRAM) |
| Camera | OV2640 @ 640x480 JPEG (esp32-camera) |
| Framework | ESP-IDF 5.5 |
| QR library | quirc |
| Build | PlatformIO / CMake |
| HTTP | esp_http_server (port 80 page+`/qr`, port 81 `/stream`) |

</div>

## ■ How It Works

```
1. Device boots and creates a WPA2 WiFi access point (SSID: ESP32-CAM)
2. OV2640 captures 640x480 JPEG frames every 300ms
3. Frames are converted to grayscale and fed to the quirc QR decoder
4. Decoded QR text is stored in a mutex-guarded PSRAM buffer and held for 2 seconds
5. Port 80 serves the web page and a /qr JSON endpoint for polling decoded text
6. Port 81 streams MJPEG video independently so scanning never blocks the live feed
```

## ■ Usage

```bash
# Build and upload
pio run -t upload

# Monitor
pio device monitor
```

1. Power on the module
2. Connect to WiFi `ESP32-CAM` (password: `12345678`)
3. Open `http://192.168.4.1` in a browser — the page shows the live feed (port 81) and decoded text
4. Point the camera at a QR code

## ■ License

MIT © [pluttan](https://github.com/pluttan)
