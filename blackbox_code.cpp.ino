#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include "esp_camera.h"
#include <time.h>

// ----------- WiFi -----------
const char* ssid = "ESP32_CAR";
const char* password = "12345678";
WebServer server(80);

// ----------- Motor Pins -----------
int IN1 = 26;
int IN2 = 27;
int IN3 = 14;
int IN4 = 12;

// ----------- Buzzer -----------
#define BUZZER 23

// ----------- Sensors -----------
Adafruit_MPU6050 mpu;
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define TRIG 5
#define ECHO 18
#define MQ3_PIN 34

// ----------- GPS & GSM -----------
HardwareSerial gpsSerial(1);
HardwareSerial gsmSerial(2);
TinyGPSPlus gps;

// ----------- SD Card -----------
#define SD_CS 13
#define LOG_FILE "/log.txt"
#define MAX_DAYS 7
File logFile;

// ----------- Thresholds -----------
#define TEMP_THRESHOLD 30
#define DIST_THRESHOLD 10
#define TILT_THRESHOLD 45
#define ALCOHOL_THRESHOLD 2000

// ----------- CAMERA SETUP -----------
void initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // ⚠️ Change pins based on your ESP32-S3 board
  config.pin_d0 = 9;
  config.pin_d1 = 8;
  config.pin_d2 = 7;
  config.pin_d3 = 6;
  config.pin_d4 = 5;
  config.pin_d5 = 4;
  config.pin_d6 = 3;
  config.pin_d7 = 2;
  config.pin_xclk = 10;
  config.pin_pclk = 11;
  config.pin_vsync = 12;
  config.pin_href = 13;
  config.pin_sccb_sda = 14;
  config.pin_sccb_scl = 15;
  config.pin_pwdn = -1;
  config.pin_reset = -1;

  esp_camera_init(&config);
}

// ----------- CAPTURE IMAGE -----------
void capturePhoto() {
  camera_fb_t * fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera failed");
    return;
  }

  String path = "/img_" + String(millis()) + ".jpg";

  File file = SD.open(path, FILE_WRITE);
  if (file) {
    file.write(fb->buf, fb->len);
    file.close();
    Serial.println("Saved: " + path);
  }

  esp_camera_fb_return(fb);
}

// ----------- TIME STAMP -----------
String getTimeStamp() {
  return String(millis() / 1000); // seconds
}

// ----------- CLEAN OLD LOGS (7 DAYS) -----------
void cleanOldLogs() {
  File file = SD.open(LOG_FILE);
  if (!file) return;

  File temp = SD.open("/temp.txt", FILE_WRITE);

  while (file.available()) {
    String line = file.readStringUntil('\n');

    int idx = line.indexOf(',');
    long logTime = line.substring(0, idx).toInt();

    if ((millis()/1000 - logTime) <= (MAX_DAYS * 86400)) {
      temp.println(line);
    }
  }

  file.close();
  temp.close();

  SD.remove(LOG_FILE);
  SD.rename("/temp.txt", LOG_FILE);
}

// ----------- SAVE LOG -----------
void saveLog(String data) {
  File file = SD.open(LOG_FILE, FILE_APPEND);
  if (file) {
    file.println(getTimeStamp() + "," + data);
    file.close();
  }
}

// ----------- DISTANCE -----------
float getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.034 / 2;
}

// ----------- GPS LOCATION -----------
String getLocation() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isValid()) {
    return String(gps.location.lat(), 6) + "," +
           String(gps.location.lng(), 6);
  }
  return "No GPS";
}

// ----------- SMS -----------
void sendSMS(String msg) {
  gsmSerial.println("AT+CMGF=1");
  delay(1000);
  gsmSerial.println("AT+CMGS=\"+91XXXXXXXXXX\"");
  delay(1000);
  gsmSerial.print(msg);
  gsmSerial.write(26);
  delay(3000);
}

// ----------- SETUP -----------
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(BUZZER, OUTPUT);

  dht.begin();
  mpu.begin();

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  gsmSerial.begin(9600, SERIAL_8N1, 25, 33);

  WiFi.softAP(ssid, password);
  server.begin();

  SD.begin(SD_CS);

  initCamera(); // 📷 CAMERA INIT
}

// ----------- LOOP -----------
void loop() {

  float temp = dht.readTemperature();
  float dist = getDistance();
  int alcohol = analogRead(MQ3_PIN);

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float accel = sqrt(a.acceleration.x*a.acceleration.x +
                     a.acceleration.y*a.acceleration.y +
                     a.acceleration.z*a.acceleration.z);

  float tilt = atan2(a.acceleration.x,
                     sqrt(a.acceleration.y*a.acceleration.y +
                          a.acceleration.z*a.acceleration.z)) * 180 / PI;

  bool alert = false;

  if (temp > TEMP_THRESHOLD ||
      dist < DIST_THRESHOLD ||
      abs(tilt) > TILT_THRESHOLD ||
      alcohol > ALCOHOL_THRESHOLD ||
      accel > 3) {
    alert = true;
  }

  // ----------- BUZZER -----------
  digitalWrite(BUZZER, alert ? HIGH : LOW);

  // ----------- CAMERA TRIGGER -----------
  if (alert) {
    capturePhoto();  // 📷 TAKE PHOTO
  }

  // ----------- SAVE DATA -----------
  String data = "Temp:" + String(temp) +
                ",Dist:" + String(dist) +
                ",Alcohol:" + String(alcohol) +
                ",Accel:" + String(accel) +
                ",Tilt:" + String(tilt) +
                ",GPS:" + getLocation();

  saveLog(data);

  // ----------- CLEAN OLD DATA -----------
  cleanOldLogs();

  // ----------- SMS -----------
  if (alert) {
    sendSMS("ACCIDENT! https://maps.google.com/?q=" + getLocation());
    delay(10000);
  }

  delay(1000);
}
