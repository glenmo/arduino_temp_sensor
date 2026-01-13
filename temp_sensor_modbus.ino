/*
 * XY-MD02 Temperature & Humidity Sensor Reader with FTP Data Logging
 * Arduino UNO R4 WiFi with DFRobot RS485 Shield V1.0
 *
 * Reads temperature and humidity from XY-MD02 sensor via Modbus RTU every 2 seconds
 * Uploads 10-minute average to FTP server with date/time stamps
 *
 * Hardware Connections:
 * - RS485 Shield A terminal -> XY-MD02 A+
 * - RS485 Shield B terminal -> XY-MD02 B-
 * - RS485 Shield TX/RX -> Arduino D0(RX)/D1(TX) (Serial1)
 * - RS485 Shield DE/RE control -> Arduino D2
 * - XY-MD02 VCC -> 5-24V DC
 * - XY-MD02 GND -> GND
 *
 * Libraries Required:
 * - ModbusMaster by Doc Walker
 * - WiFiS3 (built-in for Arduino UNO R4 WiFi)
 * - WiFiUdp (built-in for Arduino UNO R4 WiFi)
 *
 * FTP Configuration:
 * - Server: desky.local:21
 * - Username: glen
 * - File: /data/sensor_data.csv
 */

#include <ModbusMaster.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>

// WiFi Configuration
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASSWORD  "YOUR_WIFI_PASSWORD"

// FTP Configuration
#define FTP_SERVER     "FTP_SERVER_IP_OR_HOSTNAME"
#define FTP_PORT       21
#define FTP_USER       "FTP_USERNAME"
#define FTP_PASSWORD   "FTP_PASSWORD"
#define FTP_FILE_PATH  "/data/sensor_data.csv"

// NTP Configuration
#define NTP_SERVER     "time.nist.gov"
#define NTP_PORT       123
#define TIMEZONE_OFFSET 11  // UTC offset in hours (adjust for your timezone)

// RS485 Shield Configuration
#define DE_RE_PIN          2       // Driver/Receiver Enable pin (D2)

// XY-MD02 Sensor Configuration
#define SLAVE_ID           1       // Default Modbus slave address
#define BAUD_RATE          9600    // Communication baud rate
#define TEMP_REGISTER      0x0001  // Temperature input register address
#define HUMIDITY_REGISTER  0x0002  // Humidity input register address

// Timing Configuration
#define READ_INTERVAL      2000    // Read sensor every 2 seconds (ms)
#define UPLOAD_INTERVAL    600000  // Upload average every 10 minutes (ms)

// Data Structure
struct SensorReading {
  unsigned long epochTime;  // Unix timestamp
  float temperature;
  float humidity;
};

// Global Variables
ModbusMaster node;
WiFiClient client;
WiFiUDP udp;
unsigned long lastReadTime = 0;
unsigned long lastUploadTime = 0;
bool wifiConnected = false;
unsigned long epochTime = 0;
unsigned long lastNTPSync = 0;
unsigned long ntpSyncInterval = 3600000;  // Sync every hour
unsigned long lastUploadEpoch = 0;

// Averaging variables
float tempSum = 0.0;
float humSum = 0.0;
int sampleCount = 0;

// RS485 transceiver control callbacks
void preTransmission() {
  digitalWrite(DE_RE_PIN, HIGH);  // Enable transmit mode
}

void postTransmission() {
  digitalWrite(DE_RE_PIN, LOW);   // Enable receive mode
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  delay(1000);
  Serial.println("=================================");
  Serial.println("XY-MD02 Sensor with FTP Logging");
  Serial.println("Arduino UNO R4 WiFi");
  Serial.println("=================================");
  Serial.println();

  // Connect to WiFi
  connectWiFi();

  // Sync time with NTP server
  syncNTPTime();

  // Configure DE/RE control pin as output
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Start in receive mode

  // Initialize Modbus RTU communication on Serial1
  Serial1.begin(BAUD_RATE);

  // Initialize Modbus node
  node.begin(SLAVE_ID, Serial1);

  // Set callbacks for RS485 direction control
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("Modbus RTU initialized successfully");
  Serial.println("Using INPUT REGISTERS (Function Code 0x04)");
  Serial.println("Starting sensor readings...");
  Serial.println();

  delay(2000); // Allow time for sensor to stabilize
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();
  }

  // Periodically sync NTP time
  if (millis() - lastNTPSync > ntpSyncInterval) {
    syncNTPTime();
  }

  // Check if it's time to read the sensor
  if (millis() - lastReadTime >= READ_INTERVAL) {
    lastReadTime = millis();

    // Read temperature and humidity from XY-MD02
    float temperature = 0.0;
    float humidity = 0.0;

    if (readXYMD02Sensor(temperature, humidity)) {
      // Accumulate samples
      tempSum += temperature;
      humSum += humidity;
      sampleCount++;

      // Display current reading
      Serial.print("Sample #");
      Serial.print(sampleCount);
      Serial.print(" - Temp: ");
      Serial.print(temperature, 1);
      Serial.print(" °C, Hum: ");
      Serial.print(humidity, 1);
      Serial.println(" %RH");
    } else {
      Serial.println("ERROR: Failed to read sensor data");
    }
  }

  // Check if it's time to upload average
  if (millis() - lastUploadTime >= UPLOAD_INTERVAL) {
    lastUploadTime = millis();

    if (sampleCount > 0) {
      // Calculate averages
      float avgTemp = tempSum / sampleCount;
      float avgHum = humSum / sampleCount;
      unsigned long currentEpoch = getCurrentEpochTime();

      Serial.println();
      Serial.print("Uploading average of ");
      Serial.print(sampleCount);
      Serial.print(" samples - ");
      printDateTime(currentEpoch);
      Serial.print(" - Avg Temp: ");
      Serial.print(avgTemp, 1);
      Serial.print(" °C, Avg Hum: ");
      Serial.print(avgHum, 1);
      Serial.println(" %RH");

      // Upload average
      if (uploadAverageToFTP(currentEpoch, avgTemp, avgHum, sampleCount)) {
        Serial.println("Upload successful!");
        lastUploadEpoch = currentEpoch;
        Serial.print("Last upload timestamp: ");
        printDateTime(lastUploadEpoch);
        Serial.println();
      } else {
        Serial.println("Upload failed!");
      }
      Serial.println();

      // Reset accumulators
      tempSum = 0.0;
      humSum = 0.0;
      sampleCount = 0;
    } else {
      Serial.println("No samples collected, skipping upload");
    }
  }
}

void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
  } else {
    Serial.println();
    Serial.println("WiFi connection failed!");
    wifiConnected = false;
  }
  Serial.println();
}

void syncNTPTime() {
  Serial.println("Synchronizing time with NTP server...");

  udp.begin(NTP_PORT);

  byte packetBuffer[48];
  memset(packetBuffer, 0, 48);
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;            // Stratum
  packetBuffer[2] = 6;            // Polling Interval
  packetBuffer[3] = 0xEC;         // Peer Clock Precision
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  udp.beginPacket(NTP_SERVER, NTP_PORT);
  udp.write(packetBuffer, 48);
  udp.endPacket();

  delay(1000);

  if (udp.parsePacket()) {
    udp.read(packetBuffer, 48);

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    const unsigned long seventyYears = 2208988800UL;
    epochTime = secsSince1900 - seventyYears + (TIMEZONE_OFFSET * 3600);
    lastNTPSync = millis();

    Serial.print("Time synchronized: ");
    printDateTime(epochTime);
    Serial.println();
  } else {
    Serial.println("Failed to sync time with NTP server");
  }

  udp.stop();
  Serial.println();
}

unsigned long getCurrentEpochTime() {
  unsigned long elapsedSeconds = (millis() - lastNTPSync) / 1000;
  return epochTime + elapsedSeconds;
}

void printDateTime(unsigned long epoch) {
  int year, month, day, hour, minute, second;
  epochToDateTime(epoch, year, month, day, hour, minute, second);

  char buffer[20];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
  Serial.print(buffer);
}

void epochToDateTime(unsigned long epoch, int &year, int &month, int &day, int &hour, int &minute, int &second) {
  second = epoch % 60;
  epoch /= 60;
  minute = epoch % 60;
  epoch /= 60;
  hour = epoch % 24;
  epoch /= 24;

  int daysSince1970 = epoch;
  year = 1970;

  while (true) {
    int daysInYear = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) ? 366 : 365;
    if (daysSince1970 >= daysInYear) {
      daysSince1970 -= daysInYear;
      year++;
    } else {
      break;
    }
  }

  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) {
    daysInMonth[1] = 29;
  }

  month = 1;
  for (int i = 0; i < 12; i++) {
    if (daysSince1970 >= daysInMonth[i]) {
      daysSince1970 -= daysInMonth[i];
      month++;
    } else {
      break;
    }
  }

  day = daysSince1970 + 1;
}

bool readXYMD02Sensor(float &temperature, float &humidity) {
  uint8_t result;
  uint16_t tempRaw = 0;
  uint16_t humidityRaw = 0;

  // Read temperature (input register 1)
  result = node.readInputRegisters(TEMP_REGISTER, 1);
  if (result == node.ku8MBSuccess) {
    tempRaw = node.getResponseBuffer(0);
    temperature = tempRaw / 10.0;
  } else {
    return false;
  }

  delay(100); // Small delay between reads

  // Read humidity (input register 2)
  result = node.readInputRegisters(HUMIDITY_REGISTER, 1);
  if (result == node.ku8MBSuccess) {
    humidityRaw = node.getResponseBuffer(0);
    humidity = humidityRaw / 10.0;
  } else {
    return false;
  }

  return true;
}

bool uploadAverageToFTP(unsigned long epochTime, float avgTemp, float avgHum, int numSamples) {
  if (!wifiConnected) {
    Serial.println("WiFi not connected. Skipping upload.");
    return false;
  }

  // Connect to FTP server
  Serial.print("Connecting to FTP server: ");
  Serial.println(FTP_SERVER);

  if (!client.connect(FTP_SERVER, FTP_PORT)) {
    Serial.println("FTP connection failed");
    return false;
  }

  // Wait for welcome message
  if (!waitForResponse("220")) {
    client.stop();
    return false;
  }

  // Send USER command
  client.print("USER ");
  client.println(FTP_USER);
  if (!waitForResponse("331")) {
    client.stop();
    return false;
  }

  // Send PASS command
  client.print("PASS ");
  client.println(FTP_PASSWORD);
  if (!waitForResponse("230")) {
    client.stop();
    return false;
  }

  // Set to ASCII mode
  client.println("TYPE A");
  if (!waitForResponse("200")) {
    client.stop();
    return false;
  }

  // Enter passive mode
  client.println("PASV");
  String pasvResponse = "";
  if (!waitForResponseWithData("227", pasvResponse)) {
    client.stop();
    return false;
  }

  // Parse PASV response to get data connection info
  int dataPort = parsePasvResponse(pasvResponse);
  if (dataPort == -1) {
    Serial.println("Failed to parse PASV response");
    client.stop();
    return false;
  }

  // Create data connection
  WiFiClient dataClient;
  if (!dataClient.connect(FTP_SERVER, dataPort)) {
    Serial.println("Data connection failed");
    client.stop();
    return false;
  }

  // Send APPE command (append to file)
  client.print("APPE ");
  client.println(FTP_FILE_PATH);
  if (!waitForResponse("150")) {
    dataClient.stop();
    client.stop();
    return false;
  }

  // Send CSV data with date and time
  int year, month, day, hour, minute, second;
  epochToDateTime(epochTime, year, month, day, hour, minute, second);

  // Format: YYYY-MM-DD,HH:MM:SS,avg_temperature,avg_humidity,sample_count
  char dateStr[11];
  char timeStr[9];
  sprintf(dateStr, "%04d-%02d-%02d", year, month, day);
  sprintf(timeStr, "%02d:%02d:%02d", hour, minute, second);

  dataClient.print(dateStr);
  dataClient.print(",");
  dataClient.print(timeStr);
  dataClient.print(",");
  dataClient.print(avgTemp, 1);
  dataClient.print(",");
  dataClient.print(avgHum, 1);
  dataClient.print(",");
  dataClient.println(numSamples);

  // Close data connection
  dataClient.stop();

  // Wait for transfer complete
  if (!waitForResponse("226")) {
    client.stop();
    return false;
  }

  // Send QUIT
  client.println("QUIT");
  waitForResponse("221");

  client.stop();
  return true;
}

bool waitForResponse(const char* expectedCode) {
  String response = "";
  return waitForResponseWithData(expectedCode, response);
}

bool waitForResponseWithData(const char* expectedCode, String &response) {
  unsigned long timeout = millis() + 10000; // 10 second timeout

  while (millis() < timeout) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.print("FTP: ");
      Serial.println(line);
      response = line;

      if (line.startsWith(expectedCode)) {
        return true;
      }

      // Check for error codes
      if (line.startsWith("4") || line.startsWith("5")) {
        return false;
      }
    }
  }

  Serial.println("FTP timeout");
  return false;
}

int parsePasvResponse(String response) {
  // Response format: 227 Entering Passive Mode (h1,h2,h3,h4,p1,p2)
  int start = response.indexOf('(');
  int end = response.indexOf(')');

  if (start == -1 || end == -1) {
    return -1;
  }

  String data = response.substring(start + 1, end);

  // Parse the last two numbers (p1, p2)
  int lastComma = data.lastIndexOf(',');
  int secondLastComma = data.lastIndexOf(',', lastComma - 1);

  int p1 = data.substring(secondLastComma + 1, lastComma).toInt();
  int p2 = data.substring(lastComma + 1).toInt();

  int port = (p1 * 256) + p2;

  Serial.print("Data port: ");
  Serial.println(port);

  return port;
}
