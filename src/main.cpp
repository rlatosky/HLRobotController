//=======================================================================
// HLRobotController Firmware v1.0
//
// This sketch is the firmware on the HL RobotController.
// The device consists of a NodeMCU ESP8266 WiFi enabled microcontroller,
// two L298N dual channel DC motor controllers, one PCA9865 16 channel PWM,
// and a couple of DC-DC buck converters.
//
// This provides a minimal web server for controlling the motors via
// web browser. When initially powered, it will try and connect to
// WiFi network using saved credentials. If that fails, it will create
// its own access point called "HLRobotController" that can be used
// to enter the credentials of the local WiFi system which it will then
// save.
//
// Once connected to the local WiFi network, the web server can be used
// for controlling the motors via web browser.
//
// The device can also be controlled from python using the WebSockets
// interface. An example python script is included. (See comments in
// the platformio.ini file for details).
//
// This firmware includes an OTA (Over-The_Air) updater (port 8266) that
// can be used to update the firmware over WiFi.
//
// This code is maintained at:
//
//  https://github.com/faustus123/HLRobotController
//=======================================================================

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <LittleFS.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>

// Initialize the PCA9685 using the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
bool PWM_FOUND = false;
#define PWM_FREQ 50 // in Hz

// 16chan PWM channel assignments
#define SERVO0 0
#define SERVO1 1
#define SERVO2 2
#define SERVO3 3
#define MOTOR0A 4
#define MOTOR0B 5
#define MOTOR1A 6
#define MOTOR1B 7
#define MOTOR2A 8
#define MOTOR2B 9
#define MOTOR3A 10
#define MOTOR3B 11
#define SERVO4 12
#define SERVO5 13
#define SERVO6 14
#define SERVO7 15

// Current motor settings (-1 to +1)
//String MOTOR_NAMES[] = {"M0", "M1", "M2", "M3"};
enum MotorNames
{
  M0,
  M1,
  M2,
  M3
};

bool MOTOR_ENABLE[4]; // C++ sets these all to false be default
float MOTOR_SET[5];

// Current servo settings
//String SERVO_NAMES[] = {"S0", "S1", "S2", "S3", "S4", "S5", "S6", "S7"};
enum ServoNames
{
  S0,
  S1,
  S2,
  S3,
  S4,
  S5,
  S6,
  S7
};
bool SERVO_ENABLE[8]; // C++ sets these all to false be default
float SERVO_SET[8];

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledactive = false;

//==========================================================================================================================

//-----------------------------------------
// escapeJsonString
//-----------------------------------------
String escapeJsonString(const String &input)
{
  String output;
  for (char c : input)
  {
    switch (c)
    {
    case '\"':
      output += "\\\"";
      break; // Escape double quote
    case '\\':
      output += "\\\\";
      break; // Escape backslash
    case '\n':
      output += "\\n";
      break; // Escape newlines
    // Add more cases here if you need to escape other characters
    default:
      output += c;
    }
  }
  return output;
}

//-----------------------------------------
// ERASE_AND_RESTART
//
// This will erase the WiFi connection info (and all
// other configuration info) from the ESP and then
// restart it. It is useful for debugging.
//
// This gets called when the ERASE_AND_RESTART.html
// URL is requested from the webserver.
//-----------------------------------------
void ERASE_AND_RESTART()
{
  // Erase all config parameters from ESP8266
  // ESP.eraseConfig();
  // ESP.restart();
  nvs_flash_init();
  nvs_flash_erase();
  nvs_flash_init();
}

//==========================================================================================================================

//-----------------------------------------
// SetMotor
//-----------------------------------------
void SetMotor(int idx, float speed)
{
  if (idx < 0 || idx > 3)
  {
    Serial.print("Bad Motor number: ");
    Serial.println(idx);
    return;
  }

  if (speed < -1.0)
    speed = -1.0;
  if (speed > +1.0)
    speed = +1.0;

  MOTOR_SET[idx] = speed;
  int motorIN1, motorIN2;
  switch (idx)
  {
  case 0:
    motorIN1 = MOTOR0A;
    motorIN2 = MOTOR0B;
    break;
  case 1:
    motorIN1 = MOTOR1A;
    motorIN2 = MOTOR1B;
    break;
  case 2:
    motorIN1 = MOTOR2A;
    motorIN2 = MOTOR2B;
    break;
  case 3:
    motorIN1 = MOTOR3A;
    motorIN2 = MOTOR3B;
    break;
  }

  int pwmValue = abs(speed) * 4095; // Map speed to PWM value (0-4095)
  if (!MOTOR_ENABLE[idx])
    pwmValue = 0.0;

  if (!PWM_FOUND)
    return;

  if (speed > 0)
  {
    // Forward
    pwm.setPWM(motorIN1, 0, pwmValue);
    pwm.setPWM(motorIN2, 0, 0); // Or use 4095 for braking
  }
  else if (speed < 0)
  {
    // Backward
    pwm.setPWM(motorIN1, 0, 0); // Or use 4095 for braking
    pwm.setPWM(motorIN2, 0, pwmValue);
  }
  else
  {
    // Stop or Brake
    pwm.setPWM(motorIN1, 0, 0);
    pwm.setPWM(motorIN2, 0, 0); // Or set both to 4095 for hard braking
  }
}

//-----------------------------------------
// SetServo
//-----------------------------------------
void SetServo(int idx, float pos)
{
  if (idx < 0 || idx > 7)
  {
    Serial.print("Bad Servo number: ");
    Serial.println(idx);
    return;
  }

  if (pos < 0.0)
    pos = 0.0;
  if (pos > 1.0)
    pos = 1.0;

  SERVO_SET[idx] = pos;
  int chan;
  switch (idx)
  {
  case 0:
    chan = SERVO0;
    break;
  case 1:
    chan = SERVO1;
    break;
  case 2:
    chan = SERVO2;
    break;
  case 3:
    chan = SERVO3;
    break;
  case 4:
    chan = SERVO4;
    break;
  case 5:
    chan = SERVO5;
    break;
  case 6:
    chan = SERVO6;
    break;
  case 7:
    chan = SERVO7;
    break;
  }

  // Standard servos have a typical pulse width range of 1ms-2ms
  // corresponding to the two limits of motion. To try and accomodate
  // servors with a wider range, we map pos to a pulse width value
  // ranging from 0.75ms (pos=0) to 1.25ms (pos=1).
  const float ticks_per_ms = PWM_FREQ * 4095.0 / 1000.0;
  const float pulse_min_ms = 0.75;
  const float pulse_max_ms = 2.25;
  const float pulse_width_ms = pulse_min_ms + pos * (pulse_max_ms - pulse_min_ms);

  int pwmValue = ticks_per_ms * pulse_width_ms; // convert to pulse off time in PWM ticks
  if (pwmValue > 4095)
    pwmValue = 4095;

  if (!PWM_FOUND)
    return;

  if (SERVO_ENABLE[idx])
  {
    pwm.setPWM(chan, 0, pwmValue);
  }
  else
  {
    pwm.setPWM(chan, 1, 0); // setting ON to value larger than OFF effectively disables servo
  }
}
//==========================================================================================================================

//-----------------------------------------
// splitStringByWhitespace
//-----------------------------------------
std::vector<std::string> splitStringByWhitespace(const std::string &str)
{
  std::istringstream iss(str);
  std::vector<std::string> tokens;
  std::string token;

  while (std::getline(iss, token, ' '))
  {
    tokens.push_back(token);
  }

  return tokens;
}

//-----------------------------------------
// onSerialMessage
//
// Handle any serial message that comes through
//-----------------------------------------
void onSerialMessage(void)
{
  if (Serial.available() > 0)
  {
    String data = Serial.readStringUntil('\n'); // Read what R-pi sends
    data.trim();
    auto tokens = splitStringByWhitespace(data.c_str());
    std::string cmd = tokens.empty() ? "none" : tokens[0];
    if (cmd == "none")
    {
      //Serial.println("Empty command received.");

      // enable/disable
    }
    else if (cmd == "enable" && tokens.size() == 3)
    {                                          // token size =3 since Set M1 X-num
      int idx = atoi(&(tokens[1].c_str()[1])); // TODO: check that tokens[1].length()==1
      bool enable = atoi(tokens[2].c_str());
      if (tokens[1].at(0) == 'M')
      {
        //Serial.println("Enabling Motor " + idx);
        if (idx >= 0 && idx <= 3)
        {
          MOTOR_ENABLE[idx] = enable;
          SetMotor(idx, MOTOR_SET[idx]);
        }
        else
        {
          Serial.println("Bad Motor number in: " + data);
        }
      }
      if (tokens[1].at(0) == 'S')
      {
        //Serial.println("Enabling Servo " + idx);
        if (idx >= 0 && idx <= 7)
        {
          SERVO_ENABLE[idx] = enable;
          SetServo(idx, SERVO_SET[idx]);
        }
        else
        {
          Serial.println("Bad Servo number in: " + data);
        }
      }

      // Motor/servo increment/decrement
    }
    else if (cmd == "incr" && tokens.size() == 3)
    {
      int idx = atoi(&(tokens[1].c_str()[1])); // TODO: check that tokens[1].length()==1
      float delta = atof(tokens[2].c_str());
      if (tokens[1].at(0) == 'M')
      {
        //Serial.println("Incrementing Motor.");
        if (idx >= 0 && idx <= 3)
        {
          SetMotor(idx, MOTOR_SET[idx] + delta);
        }
        else
        {
          Serial.println("Bad Motor number in: " + data);
        }
      }
      if (tokens[1].at(0) == 'S')
      {
        //Serial.println("Incrementing Servo " + idx);
        if (idx >= 0 && idx <= 7)
        {
          SetServo(idx, SERVO_SET[idx] + delta);
        }
        else
        {
          Serial.println("Bad Servo number in: " + data);
        }
      }

      // Motor/servo set
    }
    else if (cmd == "set" && tokens.size() == 3)
    {
      int idx = atoi(&(tokens[1].c_str()[1])); // TODO: check that tokens[1].length()==1
      float val = atof(tokens[2].c_str());
      if (tokens[1].at(0) == 'M')
      {
        //Serial.println("Setting Motor " + idx);
        if (idx >= 0 && idx <= 3)
        {
          SetMotor(idx, val);
        }
        else
        {
          Serial.println("Bad Motor number in: " + data);
        }
      }
      if (tokens[1].at(0) == 'S')
      {
        //Serial.println("Setting Servo " + idx);
        if (idx >= 0 && idx <= 7)
        {
          SetServo(idx, val);
        }
        else
        {
          Serial.println("Bad Servo number in: " + data);
        }
      }

      // Unknown or bad command
    }
    else
    {
      Serial.println("Bad command: " + data);
    }
  }
}

//----------------------------------------------
// handleDisplay
//
// Update display
//----------------------------------------------
void handleDisplay(void)
{
  if (!oledactive)
    return;

  static auto last_time = millis();
  if ((millis() - last_time) < 1000)
    return;

  display.clearDisplay();

  display.setTextColor(WHITE);
  display.setTextSize(0);

  int y = 0;
  display.setCursor(0, y);
  String ipstr = String("Ready to receive");
  display.print(ipstr);

  y += 10;
  display.setCursor(0, y);
  String serinput = String("Serial Input");
  display.print(serinput);

  // TODO: Connect input voltage to NodeMCU analog input pin with 1:10 voltage divider.
  // y += 10;
  // display.setCursor(0,y);
  // String vstr = String("   Battery: " + String(vals.vA0, 2) + String("V"));
  // display.print(vstr);

  display.display();
  last_time = millis();
}

//-----------------------------------------
// setup
//-----------------------------------------
void setup()
{
  // Start serial monitor
  // Serial.begin(74880); // the ESP8266 boot loader uses this. We can set it to something else here, but the first few messages will be garbled.
  Serial.begin(115200); // the ESP32 boot loader uses this. We can set it to something else here, but the first few messages will be garbled.
  Serial.println();

  // Initialize OLED 128x64 pixel display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
  }
  else
  {
    Serial.println(F("SSD1306 found"));
    oledactive = true;
    display.display();
  }

  // Initialize LittleFS filesystem
  if (LittleFS.begin())
  {
    Serial.println("LittleFS set up");
  }
  else
  {
    Serial.println("LittleFS FAILED to initialize!");
  } // TODO: check for error

  // Start PCS9685 PWM driver and set base frequency
  if (pwm.begin())
  {
    Serial.println("PCA9685 PWM module initialized");
    // pwm.setPWMFreq(1000); // Set frequency to 1 kHz
    pwm.setPWMFreq(50); // Set frequency to 50 kHz (typical servo frequency)
    PWM_FOUND = true;
  }
  else
  {
    Serial.println("FAILED: PCA9685 PWM module initialization");
  }
}

//-----------------------------------------
// loop
//-----------------------------------------
void loop()
{
  //timeClient.update();   // keeps system clock updated

  onSerialMessage();

  handleDisplay(); // Handle updating display
}
