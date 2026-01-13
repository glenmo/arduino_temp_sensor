# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Arduino project for reading temperature and humidity from an XY-MD02 sensor via Modbus RTU protocol using an Arduino UNO R4 WiFi with RS485 Shield.

## Hardware Configuration

- **Board**: Arduino UNO R4 WiFi
- **Shield**: RS485 Shield
- **Sensor**: XY-MD02 Temperature & Humidity Sensor
- **Protocol**: Modbus RTU over RS485

Key hardware connections:
- RS485 uses Serial1 (pins D0/D1 for RX/TX)
- DE/RE control pin on D2 for RS485 transceiver direction control
- Sensor default address: 1, baud rate: 9600

## Development Commands

**Compile and upload to board:**
```bash
# Use Arduino IDE or Arduino CLI
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi .
arduino-cli upload -p /dev/cu.usbmodem* --fqbn arduino:renesas_uno:unor4wifi .
```

**Monitor serial output:**
```bash
arduino-cli monitor -p /dev/cu.usbmodem* -c baudrate=115200
```

## Dependencies

- **ModbusMaster** by Doc Walker (install via Arduino Library Manager)
- **WiFiS3** (built-in with Arduino UNO R4 WiFi)
- **WiFiUdp** (built-in with Arduino UNO R4 WiFi)

## Architecture

This is a single-file Arduino sketch with straightforward architecture:

- **Modbus Communication**: Uses ModbusMaster library to communicate with XY-MD02 sensor over RS485
- **RS485 Direction Control**: `preTransmission()` and `postTransmission()` callbacks manage the DE/RE pin (D2) to switch between transmit mode (HIGH) and receive mode (LOW)
- **Register Reading**: Temperature (register 0x0001) and humidity (register 0x0002) are read sequentially as input registers (function code 0x04) with values in 0.1 unit increments
- **WiFi Connectivity**: Connects to WiFi network on startup and maintains connection throughout operation
- **Data Sampling**: Reads sensor every 2 seconds and accumulates samples
- **Data Averaging**: Calculates 10-minute averages (approximately 300 samples) of temperature and humidity
- **NTP Time Sync**: Synchronizes with NTP server on startup and hourly to maintain accurate time
- **FTP Upload**: Implements custom FTP client using passive mode to append CSV data to `/data/sensor_data.csv` on FTP server (192.168.55.93)
- **CSV Format**: Each line contains: `YYYY-MM-DD,HH:MM:SS,avg_temperature,avg_humidity,sample_count`
- **Serial Debugging**: Primary Serial (115200 baud) outputs sensor readings and FTP status, Serial1 handles Modbus RTU communication at 9600 baud

The main loop uses millis()-based timing to read sensors every 2 seconds and upload averages every 10 minutes without blocking delays.

## Configuration

WiFi and FTP credentials are hardcoded in the sketch; replace placeholders before deployment:
- WiFi SSID: `YOUR_WIFI_SSID`
- WiFi Password: `YOUR_WIFI_PASSWORD`
- FTP Server: `FTP_SERVER_IP_OR_HOSTNAME`
- FTP User: `FTP_USERNAME`
- FTP Password: `FTP_PASSWORD`
- Target File: `/data/sensor_data.csv`

To change sampling/upload frequency:
- Modify `READ_INTERVAL` for sampling rate (default: 2000ms = 2 seconds)
- Modify `UPLOAD_INTERVAL` for upload rate (default: 600000ms = 10 minutes)
- Modify `TIMEZONE_OFFSET` for local timezone (default: 0 = UTC)
