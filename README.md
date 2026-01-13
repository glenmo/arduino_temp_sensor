# Arduino Temp Sensor (Modbus + FTP)

This repository contains an Arduino sketch for reading temperature and humidity
from an XY-MD02 sensor over Modbus RTU (RS485) using an Arduino UNO R4 WiFi, then
logging 10-minute averages to an FTP server as CSV.

## What it does
- Reads temperature and humidity from Modbus input registers.
- Samples every 2 seconds and computes 10-minute averages.
- Syncs time via NTP and timestamps each upload.
- Appends `YYYY-MM-DD,HH:MM:SS,avg_temp,avg_hum,sample_count` to `/data/sensor_data.csv`.

## Hardware
- Arduino UNO R4 WiFi
- RS485 shield (DE/RE control on D2)
- XY-MD02 temperature/humidity sensor

## Quick start
1) Install libraries: `ModbusMaster` (Library Manager).
2) Update placeholders in `temp_sensor_modbus.ino`:
   - `WIFI_SSID`, `WIFI_PASSWORD`
   - `FTP_SERVER`, `FTP_USER`, `FTP_PASSWORD`
3) Compile and upload:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi .
arduino-cli upload -p /dev/cu.usbmodem* --fqbn arduino:renesas_uno:unor4wifi .
arduino-cli monitor -p /dev/cu.usbmodem* -c baudrate=115200
```

## Configuration
Key settings live near the top of `temp_sensor_modbus.ino`:
- `READ_INTERVAL` and `UPLOAD_INTERVAL`
- `TIMEZONE_OFFSET`
- Modbus settings (`SLAVE_ID`, `BAUD_RATE`, register addresses)

See `CLAUDE.md` for wiring details and additional notes.
