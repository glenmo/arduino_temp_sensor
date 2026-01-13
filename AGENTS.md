# Repository Guidelines

## Project Structure & Module Organization
- `temp_sensor_modbus.ino` is the single Arduino sketch containing all logic.
- `CLAUDE.md` documents hardware wiring, dependencies, and operational details.
- No separate test or asset directories are present.

## Build, Test, and Development Commands
Use Arduino CLI or the Arduino IDE to build and upload:

```bash
# Compile the sketch for Arduino UNO R4 WiFi
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi .

# Upload to a connected board
arduino-cli upload -p /dev/cu.usbmodem* --fqbn arduino:renesas_uno:unor4wifi .

# Monitor serial output at 115200 baud
arduino-cli monitor -p /dev/cu.usbmodem* -c baudrate=115200
```

## Coding Style & Naming Conventions
- Language: Arduino C++ in a single `.ino` file.
- Indentation: 2 spaces, match existing formatting.
- Constants: `UPPER_SNAKE_CASE` via `#define` (e.g., `READ_INTERVAL`).
- Functions: `lowerCamelCase` (e.g., `readXYMD02Sensor`).
- Keep additions near related sections (WiFi config, NTP, Modbus, FTP).

## Testing Guidelines
- No automated tests or test framework are configured.
- Validate changes by uploading to hardware and verifying serial output.
- Manual checks: sensor readings every 2 seconds, FTP upload every 10 minutes.

## Commit & Pull Request Guidelines
- This repository is not a Git repo in the current workspace; no commit history is available.
- If you initialize Git, use concise, imperative commit messages (e.g., "Add FTP retry timeout").
- For PRs, include hardware context, wiring changes, and sample serial logs.

## Security & Configuration Tips
- WiFi and FTP credentials are hardcoded in `temp_sensor_modbus.ino`; rotate or move to secrets before sharing.
- Update `TIMEZONE_OFFSET`, `READ_INTERVAL`, and `UPLOAD_INTERVAL` for local deployment needs.
