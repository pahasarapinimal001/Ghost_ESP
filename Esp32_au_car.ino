/*
 ===================================================================================================
 === ESP32-S3 AI TRACKING CAR - FULLY UPGRADED (v5.31 - S3, Pan/Tilt, RGB, Clean Serial) ===
 ===================================================================================================
 ==                                                                                               ==
 == VERSION GOAL (අරමුණ):                                                                          ==
 == ESP32-S3 N16R8 බෝඩ් එක සඳහා Pinout නිවැරදි කිරීම, Pan/Tilt පාලනය, RGB LED පාලනය සහ              ==
 == පැහැදිලි Serial Monitor ප්‍රතිදානයක් එක් කිරීම.                                                 ==
 ===================================================================================================

 CORE FEATURES (මූලිකාංග):
 - All previous features retained.
 - NEW: Pan/Tilt camera control from Web UI.
 - NEW: Onboard RGB LED (Neopixel) control for status (Boot, WiFi, Error, Mode).
 - NEW: Clear, user-friendly Serial Monitor output with error reporting.
 - FIXED: ESP32-S3-N16R8 board camera pin definitions.
 - FIXED: All pin conflicts (Motors, Sensors, Camera).
 - CHANGED: Removed verbose debug logs from Serial Monitor.
 - CHANGED: Renamed 'onboard_brightness' to 'external_brightness' for clarity.

 FINAL WIRING CONFIGURATION (අවසාන රැහැන් සැකසුම - ESP32-S3):
 --------------------------------------------------
  L298N ENA -> GPIO 21  | L298N ENB -> GPIO 7
  L298N IN1 -> GPIO 19  | L298N IN3 -> GPIO 15
  L298N IN2 -> GPIO 20  | L298N IN4 -> GPIO 6
  PAN SERVO  -> GPIO 35 (තිරස්)
  TILT SERVO -> GPIO 36 (සිරස්)
  SPEAKER (+) -> GPIO 46
  EXTERNAL FLASH LED -> GPIO 4 (ඔබ විසින් එක් කළ බාහිර LED)
  HC-SR04 Trig-> GPIO 3
  HC-SR04 Echo-> GPIO 41
  IR LEFT SENSOR -> GPIO 16
  IR RIGHT SENSOR -> GPIO 17
  ONBOARD RGB LED -> GPIO 45 (ස්වයංක්‍රීයව පාලනය වේ)
 --------------------------------------------------
  *සියලුම GND පින් පොදු ground එකකට සම්බන්ධ කළ යුතුය.
  *HC-SR04 සහ IR Sensors VCC පින් එක +5V වලට සම්බන්ධ කරන්න.
*/

// ========================================================================
// SECTION 1: LIBRARIES
// ========================================================================
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp32-hal-ledc.h>
#include "esp_http_server.h"
#include "esp_camera.h"
#include "img_converters.h"
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h> // <-- NEW: RGB LED සඳහා

// ========================================================================
// SECTION 2: CONFIGURATION & PIN DEFINITIONS (CORRECTED FOR S3)
// ========================================================================

const char* ssid = "📡Pehesara's ESP32 AI Thinking🤖 Car";
const char* password = "8d8d1425";
const char* apssid = "esp32-cam-robot-pro";
const char* appassword = "1280";

// --- Onboard RGB LED (Neopixel) ---
#define NEOPIXEL_PIN 45
Adafruit_NeoPixel rgbLed(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --- Motor & Servo Pins ---
int pinPanServo = 35;   // Pan (Horizontal) Servo Pin
int pinTiltServo = 36;  // Tilt (Vertical) Servo Pin
int servoPanAngle = 90;
int servoTiltAngle = 90;
int speedR = 255;
int speedL = 255;
float decelerate = 0.6;
const int ENA_PIN = 21;  const int IN1_PIN = 19; const int IN2_PIN = 20;
const int ENB_PIN = 7;   const int IN3_PIN = 15; const int IN4_PIN = 6;

// --- Peripheral Pins ---
const int SPEAKER_PIN = 46;
const int FLASH_PIN = 4;      // This is now for your EXTERNAL LED

// --- Sensor Pins ---
const int TRIG_PIN = 3;
const int ECHO_PIN = 41;
const int IR_LEFT_PIN = 16;
const int IR_RIGHT_PIN = 17;

// --- PWM Channels ---
const int ENA_CHANNEL = 5; const int ENB_CHANNEL = 7;
const int FLASH_CHANNEL = 4; // For external LED
const int SPEAKER_CHANNEL = 0;
const int PWM_FREQ = 2000; const int PWM_RESOLUTION = 8;

// --- Global Variables ---
int flashMode = 0;
int externalLedBrightness = 255; // Renamed from 'onboardLedBrightness'
unsigned long patternStartTime = 0;
int patternStep = 0;

int currentMode = 0; // 0: Manual, 1: Patrol, 2: AI, 3: Ultrasonic, 4: Line Following, 5: AI Brain Mode
const int OBSTACLE_DISTANCE_CM = 20;
unsigned long lastMoveTime = 0;

bool lineIsLost = false;
unsigned long lineLostTimestamp = 0;
const int LINE_SEARCH_TIMEOUT = 5000;

// ========================================================================
// SECTION 2.5: GLOBAL DECLARATIONS
// ========================================================================

Servo servoPan;
Servo servoTilt;

void startCameraServer();
typedef struct { httpd_req_t *req; size_t len; } jpg_chunking_t;
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
httpd_handle_t stream_httpd = NULL; httpd_handle_t camera_httpd = NULL;

// ========================================================================
// SECTION 3: CAMERA PINS (CORRECTED FOR ESP32-S3 N16R8)
// ========================================================================

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     1
#define SIOD_GPIO_NUM     8
#define SIOC_GPIO_NUM     9

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       13
#define Y4_GPIO_NUM       10
#define Y3_GPIO_NUM       47
#define Y2_GPIO_NUM       18

#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     39
#define PCLK_GPIO_NUM     40


// ========================================================================
// SECTION 3.5: HARDWARE ABSTRACTION & TONE FUNCTIONS
// ========================================================================

void stopMotors() {
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
  ledcWrite(ENA_CHANNEL, 0); ledcWrite(ENB_CHANNEL, 0);
}
void moveForward(int rightSpeed, int leftSpeed) {
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
  ledcWrite(ENA_CHANNEL, rightSpeed); ledcWrite(ENB_CHANNEL, leftSpeed);
}
void moveBackward(int rightSpeed, int leftSpeed) {
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, HIGH);
  ledcWrite(ENA_CHANNEL, rightSpeed); ledcWrite(ENB_CHANNEL, leftSpeed);
}
void turnRight(int rightSpeed, int leftSpeed) {
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
  ledcWrite(ENA_CHANNEL, rightSpeed); ledcWrite(ENB_CHANNEL, leftSpeed);
}
void turnLeft(int rightSpeed, int leftSpeed) {
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, HIGH);
  ledcWrite(ENA_CHANNEL, rightSpeed); ledcWrite(ENB_CHANNEL, leftSpeed);
}

void playTone(int freq, int duration) {
    if (freq > 0) {
      ledcWriteTone(SPEAKER_CHANNEL, freq);
      // External LED එක ශබ්දයට අනුව දැල්වීම
      int brightness = map(freq, 200, 2000, 50, 255);
      ledcWrite(FLASH_CHANNEL, brightness);
    }
    delay(duration);
    ledcWriteTone(SPEAKER_CHANNEL, 0);
    // ශබ්දය නැවත්වූ පසු, External LED එක එහි පෙර සැකසුම වෙත ගෙන ඒම
    ledcWrite(FLASH_CHANNEL, (flashMode == 1) ? externalLedBrightness : 0);
}

void playWelcomeTone() { for(int i=200; i<=800; i+=100) { playTone(i, 50); } }
void playSuccessTone() { playTone(1000, 100); playTone(1500, 150); }
void playFailTone() { playTone(500, 200); playTone(300, 300); }

// ========================================================================
// SECTION 3.6: CORE LOGIC FUNCTIONS (UPDATED FOR Pan/Tilt)
// ========================================================================

long getDistance() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  return duration * 0.034 / 2;
}

void playObstacleSound() { playTone(800, 100); delay(50); playTone(600, 150); }

void avoidObstacle() {
  stopMotors(); playObstacleSound();
  moveBackward(speedR, speedL); delay(400);
  stopMotors(); delay(200);
  
  // Center Tilt, Scan with Pan
  servoTilt.write(90); 
  servoPan.write(0); delay(500);
  long leftDistance = getDistance(); delay(100);
  servoPan.write(180); delay(800);
  long rightDistance = getDistance(); delay(100);
  servoPan.write(90); delay(500);
  
  if (leftDistance > rightDistance) { turnLeft(speedR, speedL); delay(600);
  } else { turnRight(speedR, speedL); delay(600); }
  stopMotors();
}

void playLineLostSound() {
  playTone(700, 100); delay(50); playTone(700, 100);
}

void playLineFoundSound() {
  playTone(900, 100); delay(50); playTone(1200, 150);
}

void executeLineFollowingLogic() {
  unsigned long currentTime = millis();
  bool leftSeesLine = digitalRead(IR_LEFT_PIN) == LOW;
  bool rightSeesLine = digitalRead(IR_RIGHT_PIN) == LOW;

  if (leftSeesLine && rightSeesLine) {
    if (lineIsLost) { playLineFoundSound(); lineIsLost = false; }
    moveForward(speedR, speedL);
  } else if (leftSeesLine && !rightSeesLine) {
    if (lineIsLost) { playLineFoundSound(); lineIsLost = false; }
    turnLeft(speedR * 0.8, speedL * 0.8);
  } else if (!leftSeesLine && rightSeesLine) {
    if (lineIsLost) { playLineFoundSound(); lineIsLost = false; }
    turnRight(speedR * 0.8, speedL * 0.8);
  } else {
    if (!lineIsLost) {
      stopMotors();
      playLineLostSound();
      lineIsLost = true;
      lineLostTimestamp = currentTime;
    } else {
      if (currentTime - lineLostTimestamp > LINE_SEARCH_TIMEOUT) {
        stopMotors();
      } else {
        turnRight(speedR, speedL);
      }
    }
  }
}

// --- NEW RGB LED STATUS FUNCTION ---
void updateRgbLedStatus() {
  static int lastMode = -1;
  if (currentMode == lastMode) return; // Mode එක වෙනස් වුනොත් පමණක් LED වර්ණය වෙනස් කරන්න

  switch (currentMode) {
    case 0: // Manual
      rgbLed.setPixelColor(0, rgbLed.Color(255, 255, 255)); // White
      break;
    case 3: // Ultrasonic
      rgbLed.setPixelColor(0, rgbLed.Color(0, 150, 255)); // Light Blue
      break;
    case 4: // Line Following
      rgbLed.setPixelColor(0, rgbLed.Color(255, 0, 255)); // Magenta/Purple
      break;
    case 5: // AI Brain
      rgbLed.setPixelColor(0, rgbLed.Color(255, 100, 0)); // Orange
      break;
    case 1: // Patrol
    case 2: // AI Tracking
      rgbLed.setPixelColor(0, rgbLed.Color(0, 255, 0)); // Green (Ready)
      break;
    default:
      rgbLed.setPixelColor(0, rgbLed.Color(255, 255, 255)); // White
      break;
  }
  rgbLed.show();
  lastMode = currentMode;
}

// ========================================================================
// SECTION 4: INITIALIZATION (SETUP FUNCTION - UPDATED FOR S3)
// ========================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  
  Serial.begin(115200);
  Serial.println("\n\n--- ESP32-S3 AI ROBOT BOOTING UP ---");

  // --- Initialize RGB LED ---
  rgbLed.begin();
  rgbLed.setBrightness(40); // 0-255 (අධික දීප්තිය අඩු කිරීමට)
  rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 255)); // Booting = Blue
  rgbLed.show();
  Serial.println("[INFO] RGB LED Initialized. (Booting Blue)");

  // --- Initialize Peripherals ---
  ledcAttachPin(FLASH_PIN, FLASH_CHANNEL); ledcSetup(FLASH_CHANNEL, 5000, 8);
  ledcAttachPin(SPEAKER_PIN, SPEAKER_CHANNEL);
  Serial.println("[INFO] External LED and Speaker Initialized.");

  servoPan.attach(pinPanServo);
  servoTilt.attach(pinTiltServo);
  Serial.println("[INFO] Pan/Tilt Servos Initialized.");
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_LEFT_PIN, INPUT);
  pinMode(IR_RIGHT_PIN, INPUT);
  Serial.println("[INFO] All Sensors Initialized.");

  playWelcomeTone(); // Play sound
  servoPan.write(servoPanAngle);   // Center Pan
  servoTilt.write(servoTiltAngle); // Center Tilt

  // --- Camera Config ---
  Serial.print("[INFO] Initializing Camera...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM; config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    Serial.print(" (PSRAM OK) ");
    config.frame_size = FRAMESIZE_UXGA; config.jpeg_quality = 10; config.fb_count = 2;
  } else {
    Serial.print(" (No PSRAM) ");
    config.frame_size = FRAMESIZE_SVGA; config.jpeg_quality = 12; config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { 
    // --- FATAL ERROR ---
    Serial.printf("\n!!! [FATAL ERROR] CAMERA INIT FAILED !!! Error Code: 0x%x\n", err);
    Serial.println("Check camera connection, pins, and board model.");
    rgbLed.setPixelColor(0, rgbLed.Color(255, 0, 0)); // Solid Red
    rgbLed.show();
    Serial.println("Restarting in 5 seconds...");
    delay(5000);
    ESP.restart(); 
  }
  Serial.println("\n[SUCCESS] Camera Initialized.");
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA); // Set to small size for streaming

  // --- Motor Driver Config ---
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  ledcSetup(ENA_CHANNEL, PWM_FREQ, PWM_RESOLUTION); ledcSetup(ENB_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA_PIN, ENA_CHANNEL); ledcAttachPin(ENB_PIN, ENB_CHANNEL);
  stopMotors();
  Serial.println("[INFO] Motor Driver Initialized.");

  // --- WiFi Config ---
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  Serial.print("[INFO] Connecting to WiFi...");
  long int StartTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - StartTime < 10000)) { Serial.print("."); delay(500); }

  if (WiFi.status() == WL_CONNECTED) {
    // --- WiFi Success ---
    playSuccessTone();
    rgbLed.setPixelColor(0, rgbLed.Color(0, 255, 0)); // Green
    rgbLed.show();
    WiFi.softAP((WiFi.localIP().toString() + "_" + (String)apssid).c_str(), appassword);
    Serial.println("\n[SUCCESS] WiFi Connected.");
  } else {
    // --- WiFi Fail ---
    playFailTone();
    rgbLed.setPixelColor(0, rgbLed.Color(150, 150, 0)); // Yellow
    rgbLed.show();
    Serial.println("\n[WARNING] WiFi Connection Failed. Starting in AP Mode only.");
    WiFi.softAP(apssid, appassword);
  }

  // --- Start Web Server ---
  startCameraServer();
  Serial.println("[INFO] Web Server Started.");

  // --- READY ---
  Serial.println("------------------------------------------");
  Serial.println(">>>         ROBOT IS READY!          <<<");
  Serial.println("------------------------------------------");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(">>> Connect to: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.print(">>> Connect to AP: '");
    Serial.print(apssid);
    Serial.print("'");
    Serial.print("\n>>> Then use IP: http://");
    Serial.println(WiFi.softAPIP());
  }
  Serial.println("------------------------------------------");
  
  // Set default mode color (Manual = White)
  updateRgbLedStatus();
}

// ========================================================================
// SECTION 5: MAIN LOOP (STATE MACHINE)
// ========================================================================
void loop() {
  
  updateRgbLedStatus(); // Update RGB LED based on current mode
  
  unsigned long currentTime = millis();

  switch (currentMode) {
    case 0: // Manual Mode
      // All control is via Web UI
      break;

    case 3: // Ultrasonic Mode
      if (currentTime - lastMoveTime > 100) {
        servoPan.write(90); servoTilt.write(90); // Center head
        long distance = getDistance();
        if (distance > 0 && distance < OBSTACLE_DISTANCE_CM) {
          avoidObstacle();
        } else {
          moveForward(speedR, speedL);
        }
        lastMoveTime = currentTime;
      }
      break;

    case 4: // Line Following Mode
      if (currentTime - lastMoveTime > 50) {
        servoPan.write(90); servoTilt.write(90); // Center head
        long distance = getDistance();
        if (distance > 0 && distance < OBSTACLE_DISTANCE_CM - 5) {
            avoidObstacle();
            lineIsLost = true;
            lineLostTimestamp = currentTime;
            break;
        }
        executeLineFollowingLogic();
        lastMoveTime = currentTime;
      }
      break;

    case 5: // AI Brain Mode
      if (currentTime - lastMoveTime > 100) {
          servoPan.write(90); servoTilt.write(90); // Center head

          // Priority 1: Obstacle Avoidance
          long distance = getDistance();
          if (distance > 0 && distance < OBSTACLE_DISTANCE_CM) {
              avoidObstacle();
              lastMoveTime = currentTime;
              break;
          }

          // Priority 2: Line Following
          bool leftSeesLine = digitalRead(IR_LEFT_PIN) == LOW;
          bool rightSeesLine = digitalRead(IR_RIGHT_PIN) == LOW;
          if(leftSeesLine || rightSeesLine) {
              executeLineFollowingLogic();
          } else {
              // Priority 3: Default Action - Explore
              lineIsLost = true; 
              moveForward(speedR, speedL);
          }
          lastMoveTime = currentTime;
      }
      break;
  }

  // External Flash (LED) patterns handler
  switch(flashMode){
    case 0: ledcWrite(FLASH_CHANNEL, 0); break;
    case 1: ledcWrite(FLASH_CHANNEL, externalLedBrightness); break;
    case 2: if (currentTime - patternStartTime > 100) { patternStartTime = currentTime; ledcWrite(FLASH_CHANNEL, (patternStep++ % 2 == 0) ? externalLedBrightness : 0); } break;
    case 3: if (currentTime - patternStartTime > 500) { patternStartTime = currentTime; ledcWrite(FLASH_CHANNEL, (patternStep++ % 2 == 0) ? externalLedBrightness : 0); } break;
    case 4: {
        int total_duration = 4500;
        unsigned long time_in_pattern = (currentTime - patternStartTime) % total_duration;
        int light = 0;
        if ((time_in_pattern > 0 && time_in_pattern <= 200) || (time_in_pattern > 300 && time_in_pattern <= 500) || (time_in_pattern > 600 && time_in_pattern <= 800) || (time_in_pattern > 1500 && time_in_pattern <= 1900) || (time_in_pattern > 2000 && time_in_pattern <= 2400) || (time_in_pattern > 2500 && time_in_pattern <= 2900) || (time_in_pattern > 3600 && time_in_pattern <= 3800) || (time_in_pattern > 3900 && time_in_pattern <= 4100) || (time_in_pattern > 4200 && time_in_pattern <= 4400)) {
            light = externalLedBrightness;
          }
        ledcWrite(FLASH_CHANNEL, light);
      }
      break;
    case 5: {
        float pulse_duration = 2000.0;
        float val = (sin((currentTime - patternStartTime) * 2 * PI / pulse_duration) + 1.0) / 2.0;
        ledcWrite(FLASH_CHANNEL, (int)(val * externalLedBrightness));
      }
      break;
    case 6: if (currentTime - patternStartTime > 40) { patternStartTime = currentTime; ledcWrite(FLASH_CHANNEL, (patternStep++ % 2 == 0) ? externalLedBrightness : 0); } break;
  }
}

// ========================================================================
// SECTION 6: WEB PAGE (HTML, CSS, JAVASCRIPT) - v5.31
// ========================================================================
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>ESP32 ROBO-CONTROLLER v5.31</title>
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700&family=Roboto:wght@400;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --primary-color: #00ffff; --secondary-color: #ff00ff;
            --surface-color: rgba(20, 20, 40, 0.75); --font-color: #e0e0e0; --danger-color: #ff2a6d;
            --glow-shadow: 0 0 5px var(--primary-color), 0 0 10px var(--primary-color), 0 0 20px var(--primary-color);
            --rainbow-gradient: linear-gradient(90deg, #ff00ff, #00ffff, #00ff00, #ff00ff);
            --text-stroke: -1px -1px 0 #000, 1px -1px 0 #000, -1px 1px 0 #000, 1px 1px 0 #000;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        @keyframes fadeInSlideUp { from { opacity: 0; transform: translateY(20px); } to { opacity: 1; transform: translateY(0); } }
        @keyframes live-blink { 0%, 49% { opacity: 1; } 50%, 100% { opacity: 0; } }
        @keyframes text-reveal { from { background-position: 0% 50%; } to { background-position: 100% 50%; } }

        body {
            font-family: 'Roboto', sans-serif; 
            color: var(--font-color);
            display: flex; flex-direction: column; align-items: center; min-height: 100vh; padding: 10px;
            background-color: #0a0a14;
        }
        .title-font { font-family: 'Orbitron', sans-serif; }
        
        .main-container { 
            width: 100%; 
            max-width: 1400px;
            display: flex;
            flex-direction: column;
            gap: 20px;
            animation: fadeInSlideUp 0.8s ease-in-out;
        }
        
        .left-column, .right-column {
            display: flex;
            flex-direction: column;
            gap: 20px;
        }
        
        @media (min-width: 992px) {
            .main-container {
                display: grid;
                grid-template-columns: 1fr minmax(400px, 450px);
                align-items: stretch;
            }
        }

        .right-column > .card, 
        .left-column > .card:last-child {
            flex-grow: 1;
            display: flex;
            flex-direction: column;
        }
        
        .left-column > .card:last-child {
            justify-content: center;
        }

        .card {
            backdrop-filter: blur(10px); -webkit-backdrop-filter: blur(10px);
            position: relative; 
            background-color: var(--surface-color);
            border-radius: 15px; padding: 20px; 
            box-shadow: 0 0 15px rgba(0, 255, 255, 0.1), 0 0 10px rgba(0,0,0,0.5);
            border: 2px solid var(--primary-color);
        }
        header { width: 100%; text-align: center; margin-bottom: 20px; }
        header h1 {
            background: var(--rainbow-gradient); background-size: 200%; -webkit-background-clip: text; background-clip: text;
            color: transparent; 
            animation: text-reveal 4s linear infinite; 
            font-size: clamp(1.8rem, 5vw, 2.2rem);
            text-shadow: 0 0 8px rgba(0,0,0,0.5);
        }
        button, .btn { transition: transform 0.2s ease, box-shadow 0.2s ease; cursor: pointer; }
        button:hover, .btn:hover { transform: translateY(-3px); box-shadow: var(--glow-shadow); }

        #stream-container { 
            position: relative; width: 100%; margin: auto;
            border-radius: 12px; overflow: hidden; background-color: #000;
            aspect-ratio: 4 / 3;
        }
        #stream, #canvas { display: block; width: 100%; height: 100%; object-fit: contain; }
        #canvas { position: absolute; top: 0; left: 0; z-index: 5; }
        .overlay-text { position: absolute; font-size: 0.9em; padding: 5px 8px; background-color: rgba(0, 0, 0, 0.6); border-radius: 5px; color: white; z-index: 10; text-shadow: var(--text-stroke);}
        #current-time { top: 10px; left: 10px; }
        #live-indicator { top: 10px; right: 10px; background-color: rgba(255, 0, 0, 0.8); }
        #live-indicator span { animation: live-blink 1.5s infinite; }
        #current-date { bottom: 10px; left: 10px; }
        #logo-display { position: absolute; bottom: 10px; left: 10px; max-height: 40px; width: auto; background-color: transparent; padding: 0; border-radius: 50%; z-index: 11;}
        footer { margin-top: 30px; padding: 20px; text-align: center; width:100%; max-width: 600px; display: flex; flex-direction: column; align-items: center; gap: 15px; }
        .footer-link, .youtube-link { color: var(--primary-color); text-decoration: none; transition: all 0.3s; display: inline-flex; align-items: center; gap: 8px; text-shadow: var(--text-stroke); }
        .footer-link:hover, .youtube-link:hover { color: var(--secondary-color); }
        
        .youtube-link img {
            height: 36px; width: 36px;
            border-radius: 50%; object-fit: cover;
            border: 2px solid var(--primary-color);
            background-color: #fff;
        }
        h3 { color: var(--primary-color); margin-bottom: 20px; border-bottom: 1px solid var(--primary-color); padding-bottom: 8px; width:100%; text-align:center; text-shadow: var(--text-stroke); }
        .ip-config { display: flex; flex-direction: column; align-items: center; gap: 10px; }
        
        .ip-input-group { display: flex; gap: 10px; width: 100%; max-width: 400px; margin: 0 auto;}
        .ip-input-group input { flex-grow: 1; min-width: 0; }
        #setip { flex-shrink: 0; padding: 10px 20px; font-size: 1em; }

        input[type="text"], button, select, input[type="button"] { 
            font-family: 'Roboto', sans-serif; border-radius: 8px; border: 1px solid var(--primary-color); 
            background-color: #0d0221; color: var(--font-color); padding: 10px;
        }
        
        select { -webkit-appearance: none; -moz-appearance: none; appearance: none; background-image: url('data:image/svg+xml;charset=US-ASCII,%3Csvg%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20width%3D%22292.4%22%20height%3D%22292.4%22%3E%3Cpath%20fill%3D%22%2300FFFF%22%20d%3D%22M287%2069.4a17.6%2017.6%200%200%200-13-5.4H18.4c-5%200-9.3%201.8-13%205.4A17.6%2017.6%200%200%200%200%2082.2c0%205%201.8%209.3%205.4%2013l128%20127.9c3.6%203.6%207.8%205.4%2013%205.4s9.4-1.8%2013-5.4L287%2095c3.5-3.5%205.4-7.8%205.4-13%200-5-1.9-9.4-5.4-13z%22%2F%3E%3C%2Fsvg%3E'); background-repeat: no-repeat; background-position: right 1rem center; background-size: .8em; }
        
        .drive-controls { 
            display: flex; 
            justify-content: center; 
            align-items: center; 
            gap: 30px;
            flex-wrap: wrap;
        }
        .dpad { display: grid; grid-template-areas: "up-left up up-right" "left stop right" "down-left down down-right"; gap: 10px; width: 240px; height: 240px; }
        .dpad-btn { font-size: 2.5rem; border-radius: 15px; display:flex; justify-content:center; align-items:center; background-color:var(--surface-color); border:1px solid var(--primary-color); color:var(--primary-color); }
        #btn-up { grid-area: up; } #btn-down { grid-area: down; } #btn-left { grid-area: left; } #btn-right { grid-area: right; }
        #btn-up-left { grid-area: up-left; } #btn-up-right { grid-area: up-right; }
        #btn-down-left { grid-area: down-left; } #btn-down-right { grid-area: down-right; }
        #btn-stop { grid-area: stop; background-color: var(--danger-color); border-color: var(--danger-color); color:white; font-size:1.1rem; border-radius:50%; width:70px; height:70px; margin:auto; }
        
        .pantilt-controls {
            display: grid;
            grid-template-areas:
                ". tilt-up ."
                "pan-left center pan-right"
                ". tilt-down .";
            gap: 10px;
            width: 240px;
            height: 240px;
        }
        #btn-tilt-up { grid-area: tilt-up; }
        #btn-pan-left { grid-area: pan-left; }
        #btn-pantilt-center { 
            grid-area: center; 
            background-color: var(--danger-color); 
            border-color: var(--danger-color); 
            color:white; 
            font-size:1.1rem; 
            border-radius:50%; 
            width:70px; 
            height:70px; 
            margin:auto;
        }
        #btn-pan-right { grid-area: pan-right; }
        #btn-tilt-down { grid-area: tilt-down; }

        .right-column > div { margin-bottom: 20px; }
        .right-column > div:last-child { margin-bottom: 0; }

        .utility-buttons, .settings-dropdown { display: flex; gap: 10px; }
        
        .utility-buttons > div, .settings-dropdown > div {
            flex: 1;
            display: flex;
            flex-direction: column;
            gap: 5px;
        }
        
        .setting-group {
            display: grid;
            grid-template-columns: 1fr auto;
            gap: 10px;
            align-items: center;
            width: 100%;
            margin-bottom: 18px; 
        }
        .setting-group > label { text-align: left; }

        .utility-buttons label, .setting-group label, .range-slider-group label, .settings-dropdown label { text-shadow: var(--text-stroke); }
        .switch { position: relative; display: inline-block; width: 60px; height: 34px; flex-shrink: 0; }
        .switch input { display: none; }
        .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #333; transition: .4s; border-radius: 34px; border: 1px solid var(--primary-color); }
        .slider:before { position: absolute; content: ""; height: 26px; width: 26px; left: 4px; bottom: 3px; background-color: white; transition: .4s; border-radius: 50%; }
        input:checked + .slider { background-color: var(--primary-color); }
        input:checked + .slider:before { transform: translateX(26px); }
        
        .range-slider-group { 
            width: 100%; 
            text-align: center;
            margin-bottom: 18px; 
        }

        .adv-settings > div:last-child, .mode-controls > div:last-child, .settings-card > div:last-child {
            margin-bottom: 0;
        }
        
        .key-setting { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
        .key-setting label { flex-basis: 60%; }
        .key-setting input { flex-basis: 35%; text-align: center; text-transform: uppercase; }


        input[type=range] { -webkit-appearance: none; width: 100%; background: transparent; margin: 10px 0; }
        input[type=range]::-webkit-slider-runnable-track { height: 8px; border-radius: 5px; border: 1px solid var(--primary-color); }
        input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; height: 24px; width: 24px; border-radius: 50%; background: var(--primary-color); margin-top: -9px; }
        
        #disconnected-message { display: block; text-align: center; color: var(--danger-color); font-family: 'Orbitron', sans-serif; text-shadow: var(--text-stroke); }
        #disconnected-message img { display: block; margin: 0 auto 10px auto; max-height: 80px; border-radius: 50%; }
    </style>
</head>
<body>
    <header><h1 class="title-font">🤖 ESP32 CYBERPUNK CONTROLLER 🤖</h1></header>

    <div class="main-container">
        
        <div class="left-column">
            <div class="video-section card">
                <div id="stream-container">
                    <img id="stream" src="" style="display:none;">
                    <canvas id="canvas" style="display:none;"></canvas>
                    <div id="connection-overlays" style="display:none;">
                        <div id="current-time" class="overlay-text title-font"></div>
                        <div id="live-indicator" class="overlay-text title-font"><span>🔴</span> LIVE</div>
                        <div id="current-date" class="overlay-text"></div>
                    </div>
                    <img id="logo-display" src="https://yt3.googleusercontent.com/ZOxiSNqdKxOeqp8LBZeTcmNd04G5K2xmpqvdgica1oifFGaBI9FVqD2YodaS2lsNWK9XdAPCiw=s160-c-k-c0x00ffffff-no-rj" alt="Logo">
                    <div id="disconnected-message">
                        <img src="https://yt3.googleusercontent.com/ZOxiSNqdKxOeqp8LBZeTcmNd04G5K2xmpqvdgica1oifFGaBI9FVqD2YodaS2lsNWK9XdAPCiw=s160-c-k-c0x00ffffff-no-rj" alt="Logo">
                        <span>Not Connected</span>
                    </div>
                </div>
                <div id="result" class="title-font">Enter IP and Connect to start...</div>
            </div>

            <div class="card">
                   <div class="drive-controls">
                       <div class="dpad">
                           <button id="btn-up" class="dpad-btn btn">⬆️</button>
                           <button id="btn-down" class="dpad-btn btn">⬇️</button>
                           <button id="btn-left" class="dpad-btn btn">⬅️</button>
                           <button id="btn-right" class="dpad-btn btn">➡️</button>
                           <button id="btn-up-left" class="dpad-btn btn">↖️</button>
                           <button id="btn-up-right" class="dpad-btn btn">↗️</button>
                           <button id="btn-down-left" class="dpad-btn btn">↙️</button>
                           <button id="btn-down-right" class="dpad-btn btn">↘️</button>
                           <button id="btn-stop" class="dpad-btn btn">STOP</button>
                       </div>
                       
                       <div class="pantilt-controls">
                           <button id="btn-tilt-up" class="dpad-btn btn">⬆️</button>
                           <button id="btn-pan-left" class="dpad-btn btn">⬅️</button>
                           <button id="btn-pantilt-center" class="dpad-btn btn">CENTER</button>
                           <button id="btn-pan-right" class="dpad-btn btn">➡️</button>
                           <button id="btn-tilt-down" class="dpad-btn btn">⬇️</button>
                       </div>
                   </div>
            </div>
        </div>

        <div class="right-column">
            <div class="card">
                <div class="ip-config">
                    <h3 class="title-font">📡 CONNECTION</h3>
                    <div class="ip-input-group">
                        <input type="text" id="ip" placeholder="Enter ESP32 IP Address">
                        <button id="setip" class="title-font">Connect</button>
                    </div>
                </div>
                 <hr style="border-color: var(--primary-color); margin: 20px 0;">
                <div class="utility-controls">
                    <h3 class="title-font">⚡️ UTILITIES</h3>
                    <div class="utility-buttons">
                        <div>
                            <label for="flash_mode">💡 External LED</label>
                            <select id="flash_mode" class="default-action">
                                <option value="0">Off</option>
                                <option value="1">On</option>
                                <option value="2">Fast Blink</option>
                                <option value="3">Slow Blink</option>
                                <option value="4">SOS</option>
                                <option value="5">Pulse</option>
                                <option value="6">Strobe</option>
                            </select>
                        </div>
                        <div>
                            <label for="sound_tone">🔊 Sound Tone</label>
                            <select id="sound_tone" class="default-action">
                                <option value="0" selected>Off</option>
                                <option value="1">Nokia</option>
                                <option value="2">Samsung</option>
                                <option value="3">Mario</option>
                                <option value="4">Ascending</option>
                                <option value="5">Alarm</option>
                                <option value="6">High Beep</option>
                                <option value="7">Dual Tone</option>
                            </select>
                        </div>
                    </div>
                </div>
            </div>
            <div class="card settings-card">
                 <h3 class="title-font">⚙️ SETTINGS</h3>
                 <div class="setting-group">
                      <label>Keyboard Controls</label>
                      <label class="switch"><input id="keyboardToggle" type="checkbox"><span class="slider"></span></label>
                 </div>
                 <div id="key-mappings" style="display:none;">
                      <hr style="border-color: var(--primary-color); margin: 10px 0 20px 0;">
                      <div class="key-setting"><label>Forward:</label> <input type="button" class="key-input" id="key-forward" value="W"></div>
                      <div class="key-setting"><label>Backward:</label> <input type="button" class="key-input" id="key-backward" value="S"></div>
                      <div class="key-setting"><label>Left:</label> <input type="button" class="key-input" id="key-left" value="A"></div>
                      <div class="key-setting"><label>Right:</label> <input type="button" class="key-input" id="key-right" value="D"></div>
                      <div class="key-setting"><label>Stop:</label> <input type="button" class="key-input" id="key-stop" value="SPACE"></div>
                 </div>
            </div>
            <div class="card">
                 <div class="mode-controls">
                    <h3 class="title-font">🚀 MODES</h3>
                    <div class="setting-group"><label>🛰️ Patrol Mode</label><label class="switch"><input id="patrolMode" type="checkbox"><span class="slider"></span></label></div>
                    <div class="setting-group"><label>🧠 AI Tracking</label><label class="switch"><input id="detectState" type="checkbox"><span class="slider"></span></label></div>
                    <div class="setting-group"><label>🥏 Ultrasonic Mode</label><label class="switch"><input id="ultrasonicMode" type="checkbox"><span class="slider"></span></label></div>
                    <div class="setting-group"><label>⛳ Line Following</label><label class="switch"><input id="lineFollowingMode" type="checkbox"><span class="slider"></span></label></div>
                    <div class="setting-group"><label>🤖 AI Brain Mode</label><label class="switch"><input id="aiBrainMode" type="checkbox"><span class="slider"></span></label></div>
                 </div>
            </div>
            <div class="card">
                 <div class="adv-settings">
                    <h3 class="title-font">🔧 ADVANCED SETTINGS</h3>
                    <div class="setting-group">
                        <label for="object">Track Object:</label>
                        <select id="object" class="default-action" style="width: 180px; font-size: 0.9em; padding: 5px;">
                            <option value="person" selected="selected">person</option><option value="bicycle">bicycle</option><option value="car">car</option><option value="cat">cat</option><option value="dog">dog</option><option value="bottle">bottle</option><option value="cup">cup</option>
                        </select>
                    </div>
                    <div class="range-slider-group"><label>Score Limit: <span id="scoreVal">0.5</span></label><input type="range" id="score" min="0.1" max="0.9" value="0.5" step="0.1"></div>
                    <div class="setting-group"><label>Motor Control (AI)</label><label class="switch"><input id="motorState" type="checkbox" checked><span class="slider"></span></label></div>
                    <div class="setting-group"><label>Servo Control (AI)</label><label class="switch"><input id="servoState" type="checkbox" checked><span class="slider"></span></label></div>
                    <div class="setting-group"><label>Auto Search When Lost</label><label class="switch"><input id="autodetect" type="checkbox"><span class="slider"></span></label></div>
                    <div class="setting-group"><label>🎯 AI Mode (Overlay)</label><label class="switch"><input id="aiMode" type="checkbox"><span class="slider"></span></label></div>
                    <div class="range-slider-group"><label>🔆 Ext. LED Brightness: <span id="external_brightnessVal">255</span></label><input type="range" id="external_brightness" min="0" max="255" value="255" class="default-action"></div>
                    <div class="range-slider-group"><label>Speed Right: <span id="speedRVal">255</span></label><input type="range" id="speedR" min="0" max="255" value="255" class="default-action"></div>
                    <div class="range-slider-group"><label>Speed Left: <span id="speedLVal">255</span></label><input type="range" id="speedL" min="0" max="255" value="255" class="default-action"></div>
                 </div>
            </div>
        </div>

    </div>

    <footer>
        <div class="copyright">Created by Mr. Pehesara Pinimal</div>
        <a href="https://dailygenixpro.blogspot.com/?m=1" target="_blank" class="footer-link">Need More infomation chek My Official Website</a>
        <a href="https://www.youtube.com/@meowmories.stories-tv" target="_blank" class="youtube-link">
            <img src="https://cdn.pixabay.com/photo/2023/01/23/10/31/subscribe-7738307_1280.png" alt="Subscribe Button">
            <span>Subscribe Now</span>
        </a>
    </footer>
    
    <script src="https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@1.3.1/dist/tf.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.1.0"></script>
    <script>
    document.addEventListener('DOMContentLoaded', () => {
        var baseHost;
        const result = document.getElementById('result');
        const setipBtn = document.getElementById('setip');
        const streamImg = document.getElementById('stream');
        const canvas = document.getElementById('canvas');
        const disconnectedMsg = document.getElementById('disconnected-message');
        const connectionOverlays = document.getElementById('connection-overlays');
        
        const href = location.href;
        if (href.indexOf("?") !== -1) { document.getElementById("ip").value = location.search.split("?")[1].replace(/http:\/\//g, "");
        } else if (href.indexOf("http") !== -1) { document.getElementById("ip").value = location.host; }

        function updateConfig(el) {
            if (!baseHost) return;
            let value;
            switch (el.type) { case 'checkbox': value = el.checked ? 1 : 0; break; case 'range': case 'select-one': value = el.value; break; default: return; }
            fetch(`${baseHost}/control?var=${el.id}&val=${value}`);
            if (el.id === 'sound_tone' && el.value !== '0') { setTimeout(() => { el.selectedIndex = 0; }, 500); }
        }
        
        // UPDATED list (removed 'servo', 'onboard_brightness' -> 'external_brightness')
        ['speedR', 'speedL', 'score', 'external_brightness'].forEach(id => {
            const slider = document.getElementById(id); const valDisplay = document.getElementById(`${id}Val`);
            if(slider && valDisplay) slider.oninput = () => valDisplay.textContent = slider.value;
        });

        function updateDateTime() {
            const now = new Date();
            const timeOpts = { hour: '2-digit', minute: '2-digit', hour12: false };
            const dateOpts = { year: 'numeric', month: '2-digit', day: '2-digit' };
            document.getElementById('current-time').textContent = now.toLocaleTimeString('sv-SE', timeOpts);
            document.getElementById('current-date').textContent = now.toLocaleDateString('sv-SE', dateOpts).replace(/-/g, '/');
        }
        
        function start() {
            baseHost = 'http://' + document.getElementById("ip").value;
            result.innerHTML = "Connecting..."; 
            setipBtn.innerHTML = "Connecting...";
            setipBtn.disabled = true;

            fetch(`${baseHost}/status`)
            .then(response => { if (response.ok) return response.json(); throw new Error('Network error.'); })
            .then(state => { // SUCCESS
                document.querySelectorAll('.default-action').forEach(el => {
                    if(el.type === 'checkbox') el.checked = !!state[el.id]; 
                    else if (el.tagName === 'SELECT' && el.id !== 'sound_tone') el.value = state[el.id];
                    else if (el.type === 'range') {
                        el.value = state[el.id];
                        const valDisplay = document.getElementById(`${el.id}Val`); 
                        if(valDisplay) valDisplay.textContent = state[el.id];
                    }
                });

                result.style.display = 'none';
                disconnectedMsg.style.display = 'none';
                
                streamImg.src = `${baseHost}:81/stream`;
                streamImg.style.display = 'block';

                connectionOverlays.style.display = 'block';
                setInterval(updateDateTime, 1000); 
                updateDateTime();
                
                setipBtn.innerHTML = "Connected";
            })
            .catch(error => { // FAILURE
                result.style.display = 'block';
                result.innerHTML = "❌ Connection Failed. Check IP & Retry.";
                streamImg.style.display = 'none';
                streamImg.src = '';
                connectionOverlays.style.display = 'none';
                disconnectedMsg.style.display = 'block';
                setipBtn.innerHTML = "Connect";
                setipBtn.disabled = false;
            });
        }
        setipBtn.onclick = start;
        document.querySelectorAll('.default-action').forEach(el => el.onchange = () => updateConfig(el));

        // Mode Switches
        const patrolSwitch = document.getElementById('patrolMode');
        const aiModeSwitch = document.getElementById('aiMode');
        const detectState = document.getElementById('detectState');
        const ultrasonicSwitch = document.getElementById('ultrasonicMode');
        const lineFollowingSwitch = document.getElementById('lineFollowingMode');
        const aiBrainSwitch = document.getElementById('aiBrainMode');
        let patrolInterval;

        function disableOtherModes(currentModeId) {
            const modes = {
                patrolMode: patrolSwitch, 
                detectState: detectState, 
                ultrasonicMode: ultrasonicSwitch, 
                lineFollowingMode: lineFollowingSwitch,
                aiBrainMode: aiBrainSwitch
            };
            for (const modeId in modes) {
                if (modeId !== currentModeId) {
                    modes[modeId].checked = false;
                }
            }
            clearInterval(patrolInterval);
        }

        patrolSwitch.onchange = () => {
            disableOtherModes('patrolMode');
            fetch(`${baseHost}/control?var=mode&val=${patrolSwitch.checked ? 1 : 0}`);
        };
        aiModeSwitch.onchange = () => {
            if (aiModeSwitch.checked) { detectImage(); }
        };
        detectState.onchange = () => {
            disableOtherModes('detectState');
            if (detectState.checked) {
                fetch(`${baseHost}/control?var=mode&val=2`);
                detectImage(); 
            } else {
                fetch(`${baseHost}/control?var=mode&val=0`);
            }
        };
        ultrasonicSwitch.onchange = () => {
            disableOtherModes('ultrasonicMode');
            fetch(`${baseHost}/control?var=mode&val=${ultrasonicSwitch.checked ? 3 : 0}`);
        };
        lineFollowingSwitch.onchange = () => {
            disableOtherModes('lineFollowingMode');
            fetch(`${baseHost}/control?var=mode&val=${lineFollowingSwitch.checked ? 4 : 0}`);
        };
        aiBrainSwitch.onchange = () => {
            disableOtherModes('aiBrainMode');
            fetch(`${baseHost}/control?var=mode&val=${aiBrainSwitch.checked ? 5 : 0}`);
        };

        // --- JAVASCRIPT FOR BUTTONS ---
        window.car = function(query) { if (!baseHost) return; fetch(`${baseHost}${query}`); }

        // Drive D-Pad Listeners
        const driveButtons = {
            'btn-up': 1, 'btn-down': 5, 'btn-left': 2, 'btn-right': 4, 'btn-stop': 3,
            'btn-up-left': 6, 'btn-up-right': 7, 'btn-down-left': 8, 'btn-down-right': 9
        };
        for (const [id, val] of Object.entries(driveButtons)) {
            const btn = document.getElementById(id);
            if (btn) {
                const action = () => car(`/control?var=car&val=${val}`);
                btn.addEventListener('mousedown', action);
                btn.addEventListener('touchstart', action, { passive: true });
                if (id !== 'btn-stop') {
                    const stopAction = () => car('/control?var=car&val=3');
                    btn.addEventListener('mouseup', stopAction);
                    btn.addEventListener('touchend', stopAction);
                    btn.addEventListener('mouseleave', stopAction);
                }
            }
        }

        // Pan/Tilt D-Pad Listeners
        const panTiltButtons = {
            'btn-tilt-up': 1, 'btn-tilt-down': 2, 'btn-pan-left': 3, 'btn-pan-right': 4, 'btn-pantilt-center': 5
        };
        for (const [id, val] of Object.entries(panTiltButtons)) {
            const btn = document.getElementById(id);
            if (btn) {
                btn.addEventListener('click', () => car(`/control?var=pantilt&val=${val}`));
            }
        }

        // Keyboard Controls
        const keyboardToggle = document.getElementById('keyboardToggle');
        const keyMappingsDiv = document.getElementById('key-mappings');
        let keyboardEnabled = false;
        let keyMap = { forward: 'w', backward: 's', left: 'a', right: 'd', stop: ' ' };
        const keyInputs = document.querySelectorAll('.key-input');
        
        keyboardToggle.onchange = () => {
            keyboardEnabled = keyboardToggle.checked;
            keyMappingsDiv.style.display = keyboardEnabled ? 'block' : 'none';
        };

        keyInputs.forEach(input => {
            input.onclick = () => {
                input.value = "PRESS KEY";
                input.onkeydown = (e) => {
                    e.preventDefault();
                    let newKey = e.key === ' ' ? 'SPACE' : e.key.toUpperCase();
                    input.value = newKey;
                    const action = input.id.split('-')[1];
                    keyMap[action] = e.key.toLowerCase();
                    input.blur(); 
                    input.onkeydown = null; 
                };
            };
        });

        const pressedKeys = new Set();
        document.addEventListener('keydown', (e) => {
            if (!keyboardEnabled || document.activeElement.tagName === 'INPUT') return;
            const key = e.key.toLowerCase();
            if (pressedKeys.has(key)) return; 
            
            e.preventDefault();
            pressedKeys.add(key);

            if (key === keyMap.forward) car('/control?var=car&val=1');
            else if (key === keyMap.backward) car('/control?var=car&val=5');
            else if (key === keyMap.left) car('/control?var=car&val=2');
            else if (key === keyMap.right) car('/control?var=car&val=4');
            else if (key === keyMap.stop) car('/control?var=car&val=3');
        });

        document.addEventListener('keyup', (e) => {
            if (!keyboardEnabled) return;
            const key = e.key.toLowerCase();
            pressedKeys.delete(key);
            
            if (Object.values(keyMap).includes(key) && key !== keyMap.stop) {
                car('/control?var=car&val=3');
            }
        });

        // AI/ML Model and Functions
        let model;
        result.innerHTML = "🧠 AI Model loading...";
        cocoSsd.load().then(cocoSsd_Model => { model = cocoSsd_Model; result.innerHTML = "✅ AI Model Loaded. Ready to Connect."; });

        function stringToColor(str) {
            let hash = 0;
            for (let i = 0; i < str.length; i++) { hash = str.charCodeAt(i) + ((hash << 5) - hash); }
            const hue = hash % 360;
            return `hsl(${hue}, 90%, 55%)`;
        }

        function detectImage() {
            if (!detectState.checked && !aiModeSwitch.checked) {
                canvas.getContext("2d").clearRect(0, 0, canvas.width, canvas.height); 
                return; 
            }
            
            const context = canvas.getContext("2d");
            const aiView = document.getElementById('stream');
            const scoreSlider = document.getElementById('score');
            
            canvas.width = aiView.clientWidth; canvas.height = aiView.clientHeight;
            context.drawImage(aiView, 0, 0, canvas.width, canvas.height);

            model.detect(canvas).then(Predictions => {
                if (aiModeSwitch.checked || detectState.checked) { 
                    for (let p of Predictions) {
                        if (p.score >= scoreSlider.value) {
                            const color = stringToColor(p.class);
                            context.strokeStyle = color;
                            context.fillStyle = color;
                            context.lineWidth = 3;
                            context.font = '16px Orbitron';
                            
                            context.strokeRect(p.bbox[0], p.bbox[1], p.bbox[2], p.bbox[3]);
                            const label = `${p.class} (${Math.round(p.score * 100)}%)`;
                            const textY = p.bbox[1] > 20 ? p.bbox[1] - 7 : p.bbox[1] + 18;
                            context.fillText(label, p.bbox[0] + 5, textY);
                        }
                    }
                }
                requestAnimationFrame(detectImage);
            });
        }
    });
    </script>
</body>
</html>
)rawliteral";


// ========================================================================
// SECTION 7: WEB SERVER HANDLERS (UPDATED for Pan/Tilt, Ext. LED)
// ========================================================================

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){ j->len = 0; }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){ return 0; }
    j->len += len;
    return len;
}
static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[ERROR] Camera capture failed"); httpd_resp_send_500(req); return ESP_FAIL; }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if(fb->format == PIXFORMAT_JPEG){
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
    }
    esp_camera_fb_return(fb);
    return res;
}
static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){ return res; }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    while(true){
        fb = esp_camera_fb_get();
        if (!fb) { Serial.println("[ERROR] Camera capture failed"); res = ESP_FAIL;
        } else {
          if(fb->format != PIXFORMAT_JPEG){
              bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
              esp_camera_fb_return(fb); fb = NULL;
              if(!jpeg_converted){ Serial.println("[ERROR] JPEG compression failed"); res = ESP_FAIL; }
          } else { _jpg_buf_len = fb->len; _jpg_buf = fb->buf; }
        }
        if(res == ESP_OK){ res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)); }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){ res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len); }
        if(fb){ esp_camera_fb_return(fb); fb = NULL; _jpg_buf = NULL;
        } else if(_jpg_buf){ free(_jpg_buf); _jpg_buf = NULL; }
        if(res != ESP_OK){ break; }
    }
    return res;
}
// --- END OF STREAM HANDLERS ---


static esp_err_t cmd_handler(httpd_req_t *req) {
  char* buf; size_t buf_len;
  char variable[128] = {0,}; char value[128] = {0,};
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      httpd_query_key_value(buf, "var", variable, sizeof(variable));
      httpd_query_key_value(buf, "val", value, sizeof(value));
    }
    free(buf);
  } else { httpd_resp_send_404(req); return ESP_FAIL; }

  int val = atoi(value);
  sensor_t * s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize")) res = s->set_framesize(s, (framesize_t)val);
  else if (!strcmp(variable, "quality")) res = s->set_quality(s, val);
  else if (!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
  else if (!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
  else if (!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
  else if (!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
  else if (!strcmp(variable, "flash_mode")) { flashMode = val; patternStep = 0; patternStartTime = millis(); }
  
  // Renamed to 'external_brightness'
  else if (!strcmp(variable, "external_brightness")) { if (val > 255) val = 255; else if (val < 0) val = 0; externalLedBrightness = val; }
  
  else if (!strcmp(variable, "speedL")) { if (val > 255) val = 255; else if (val < 0) val = 0; speedL = val; }
  else if (!strcmp(variable, "speedR")) { if (val > 255) val = 255; else if (val < 0) val = 0; speedR = val; }
  else if (!strcmp(variable, "decelerate")) { decelerate = String(value).toFloat(); }
  
  // --- NEW PAN/TILT LOGIC ---
  else if (!strcmp(variable, "pantilt")) {
   const int servoStep = 10; 
   if (currentMode == 0) { // Only in Manual Mode
     switch (val) {
       case 1: // Tilt Up
         servoTiltAngle -= servoStep;
         if(servoTiltAngle < 0) servoTiltAngle = 0;
         servoTilt.write(servoTiltAngle);
         break;
       case 2: // Tilt Down
         servoTiltAngle += servoStep;
         if(servoTiltAngle > 180) servoTiltAngle = 180;
         servoTilt.write(servoTiltAngle);
         break;
       case 3: // Pan Left
         servoPanAngle += servoStep;
         if(servoPanAngle > 180) servoPanAngle = 180;
         servoPan.write(servoPanAngle);
         break;
       case 4: // Pan Right
         servoPanAngle -= servoStep;
         if(servoPanAngle < 0) servoPanAngle = 0;
         servoPan.write(servoPanAngle);
         break;
       case 5: // Center
         servoPanAngle = 90;
         servoTiltAngle = 90;
         servoPan.write(servoPanAngle);
         servoTilt.write(servoTiltAngle);
         break;
     }
   }
  }

  else if (!strcmp(variable, "mode")) {
    currentMode = val;
    if(currentMode != 4) { lineIsLost = false; }
    if(currentMode == 0) { stopMotors(); }
    // Serial.printf("Mode changed to: %d\n", currentMode); // Optional: for debugging
  }
  else if (!strcmp(variable, "sound_tone")) {
    switch (val) {
      case 1: playTone(1047, 100); playTone(1175, 100); break;
      case 2: playTone(1319, 100); playTone(1397, 100); break;
      case 3: playTone(660,100); playTone(0,50); playTone(660,100); break;
    }
  }
  else if (!strcmp(variable, "car")) {
    if (currentMode == 0) {
        switch (val) {
          case 1: moveForward(speedR, speedL); break;
          case 2: turnLeft(speedR, speedL); break;
          case 3: stopMotors(); break;
          case 4: turnRight(speedR, speedL); break;
          case 5: moveBackward(speedR, speedL); break;
          case 6: moveForward(speedR, speedL * decelerate); break;
          case 7: moveForward(speedR * decelerate, speedL); break;
          case 8: moveBackward(speedR, speedL * decelerate); break;
          case 9: moveBackward(speedR * decelerate, speedL); break;
        }
    }
  } else { res = -1; }

  if (res) { return httpd_resp_send_500(req); }
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

static esp_err_t status_handler(httpd_req_t *req) {
    static char json_response[1024];
    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';
    p+=sprintf(p, "\"flash_mode\":%d,", flashMode);
    
    // Renamed to 'external_brightness'
    p+=sprintf(p, "\"external_brightness\":%d,", externalLedBrightness);
    
    p+=sprintf(p, "\"speedL\":%d,", speedL);
    p+=sprintf(p, "\"speedR\":%d,", speedR); p+=sprintf(p, "\"decelerate\":%.1f,", decelerate);
    p+=sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p+=sprintf(p, "\"quality\":%u,", s->status.quality); p+=sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p+=sprintf(p, "\"contrast\":%d,", s->status.contrast); p+=sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p+=sprintf(p, "\"vflip\":%u", s->status.vflip);
    *p++ = '}'; *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

// ========================================================================
// SECTION 8: START CAMERA SERVER FUNCTION
// ========================================================================
void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
  httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = NULL };
  httpd_uri_t cmd_uri = { .uri = "/control", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL };
  httpd_uri_t capture_uri = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler, .user_ctx = NULL };
  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
  } else {
    Serial.println("[FATAL ERROR] Main HTTPD Server Start Failed!");
  }
  
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  } else {
    Serial.println("[FATAL ERROR] Stream HTTPD Server Start Failed!");
  }
}
