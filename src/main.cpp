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
#include <WiFiManager.h>
#include <NTPClient.h>
// #include <ESP8266WiFi.h>
// #include <ESP8266WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <WebServer.h>
#include <ArduinoWebsockets.h>
#include <EEPROM.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <ArduinoOTA.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <LittleFS.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>



// Name of Access Point (AP) to create when unable to connect to local WiFi
const char *APNAME = "HLMotorController";

// Setup our webserver which just serves up the static pages (with embedded javascript)
// ESP8266WebServer server(80);
// ESP8266WebServer *server = nullptr;
WebServer *server = nullptr;

// Setup our websockets server for persistent connections
using namespace websockets;
WebsocketsServer webSocketServer;                 // main server for Websockets
std::vector<WebsocketsClient> webSocketClients;   // holds each Websockets connection

// For getting and keeping current time (see https://github.com/arduino-libraries/NTPClient)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60*60*1000); // 0 is for adjusting timezone (0=UTC). Only update once/hr)

unsigned long html_page_requests = 0;

// Initialize the PCA9685 using the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
bool PWM_FOUND = false;
#define PWM_FREQ  50  // in Hz

// 16chan PWM channel assignments
#define SERVO0   0 
#define SERVO1   1 
#define SERVO2   2 
#define SERVO3   3 
#define MOTOR0A  4
#define MOTOR0B  5
#define MOTOR1A  6
#define MOTOR1B  7
#define MOTOR2A  8
#define MOTOR2B  9
#define MOTOR3A 10
#define MOTOR3B 11
#define SERVO4  12
#define SERVO5  13 
#define SERVO6  14 
#define SERVO7  15

// Current motor settings (-1 to +1)
String MOTOR_NAMES[] = {"M0", "M1", "M2", "M3"};
bool MOTOR_ENABLE[4]; // C++ sets these all to false be default
float MOTOR_SET[5];

// Current servo settings
String SERVO_NAMES[] = {"S0", "S1", "S2", "S3", "S4", "S5", "S6", "S7"};
bool SERVO_ENABLE[8]; // C++ sets these all to false be default
float SERVO_SET[8];


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledactive = false;

//==========================================================================================================================


//-----------------------------------------
// getStatusHTML
//-----------------------------------------
String getStatusHTML(void)
{
    String html = R"(
      <table style='border-collapse: collapse; border: 2px solid black; text-align: center;'>
        <tr style='background-color: green;'>
          <th style='color: white; border: 1px solid black;'>motor</th>
          <th style='color: white; border: 1px solid black;'>enabled</th>
          <th style='color: white; border: 1px solid black;'>setting</th>
        </tr>
    )";
    String types[] = {"M0", "M1", "M2", "M3", "S0", "S1", "S2", "S3", "S4", "S5", "S6", "S7"};

    for(int i = 0; i < 4; i++) {
        html += "<tr>";
        html += "<td style='border: 1px solid black;'>" + types[i] + "</td>";
        html += "<td style='border: 1px solid black;'>" + (String)(MOTOR_ENABLE[i] ? "Y" : "N") + "</td>";
        html += "<td style='border: 1px solid black;'>" + String(MOTOR_SET[i]) + "</td>";
        html += "</tr>";
    }

    for(int i = 0; i < 8; i++) {
        html += "<tr>";
        html += "<td style='border: 1px solid black;'>" + types[i+4] + "</td>";
        html += "<td style='border: 1px solid black;'>" + (String)(SERVO_ENABLE[i] ? "Y" : "N") + "</td>";
        html += "<td style='border: 1px solid black;'>" + String(SERVO_SET[i]) + "</td>";
        html += "</tr>";
    }

    html += "</table>";
    return html;

}

//-----------------------------------------
// WebServerSendFile
//
// This is called whenever a file is requested from the web server
// that it has not been explicitly told how to handle. This will look
// for a file of the given name in the onboard LittleFS filesystem
// and, if found, return it.
//-----------------------------------------
void WebServerSendFile() {
  String path = server->uri(); // Get the URI of the request
  String contentType = "text/plain"; // Default content type

  if( path == "/" ) path = "index.html";

  if (path.endsWith(".html")) contentType = "text/html";
  if (path.endsWith(".png" )) contentType = "image/png";
  if (path.endsWith(".jpg" )) contentType = "image/jpeg";
  if (path.endsWith(".py"  )) contentType = "text/x-python";

  File file = LittleFS.open(path, "r");
  if (!file) {
    server->send(404, "text/plain", "File not found");
    return;
  }

  server->streamFile(file, contentType);
  file.close();
}

//-----------------------------------------
// escapeJsonString
//-----------------------------------------
String escapeJsonString(const String& input) {
    String output;
    for (char c : input) {
        switch (c) {
            case '\"': output += "\\\""; break; // Escape double quote
            case '\\': output += "\\\\"; break; // Escape backslash
            case '\n': output += "\\n"; break;  // Escape newlines
            // Add more cases here if you need to escape other characters
            default: output += c;
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
void ERASE_AND_RESTART(){
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
  if( idx<0 || idx>3 ){
    Serial.print("Bad Motor number: ");
    Serial.println(idx);
    return;
  }

  if( speed< -1.0 ) speed = -1.0;
  if( speed> +1.0 ) speed = +1.0;
  
  MOTOR_SET[idx] = speed;
  int motorIN1, motorIN2;
  switch(idx){
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
  if( ! MOTOR_ENABLE[idx] ) pwmValue = 0.0;

  if( ! PWM_FOUND ) return; 

  if (speed > 0) {
    // Forward
    pwm.setPWM(motorIN1, 0, pwmValue);
    pwm.setPWM(motorIN2, 0, 0); // Or use 4095 for braking
  } else if (speed < 0) {
    // Backward
    pwm.setPWM(motorIN1, 0, 0); // Or use 4095 for braking
    pwm.setPWM(motorIN2, 0, pwmValue);
  } else {
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
  if( idx<0 || idx>7 ){
    Serial.print("Bad Servo number: ");
    Serial.println(idx);
    return;
  }

  if( pos< 0.0 ) pos = 0.0;
  if( pos> 1.0 ) pos = 1.0;

  SERVO_SET[idx] = pos;
  int chan;
  switch(idx){
    case 0: chan =SERVO0; break;
    case 1: chan =SERVO1; break;
    case 2: chan =SERVO2; break;
    case 3: chan =SERVO3; break;
    case 4: chan =SERVO4; break;
    case 5: chan =SERVO5; break;
    case 6: chan =SERVO6; break;
    case 7: chan =SERVO7; break;
  }

  // Standard servos have a typical pulse width range of 1ms-2ms
  // corresponding to the two limits of motion. To try and accomodate
  // servors with a wider range, we map pos to a pulse width value
  // ranging from 0.75ms (pos=0) to 1.25ms (pos=1).
  const float ticks_per_ms = PWM_FREQ*4095.0/1000.0;
  const float pulse_min_ms = 0.75;
  const float pulse_max_ms = 2.25;
  const float pulse_width_ms = pulse_min_ms + pos * (pulse_max_ms - pulse_min_ms);

  int pwmValue = ticks_per_ms*pulse_width_ms; // convert to pulse off time in PWM ticks
  if( pwmValue>4095 ) pwmValue = 4095;

  if( ! PWM_FOUND ) return; 

  if( SERVO_ENABLE[idx] ) {
    pwm.setPWM(chan, 0, pwmValue);
  }else{
    pwm.setPWM(chan, 1, 0); // setting ON to value larger than OFF effectively disables servo
  }
}
//==========================================================================================================================

//-----------------------------------------
// splitStringByWhitespace
//-----------------------------------------
std::vector<std::string> splitStringByWhitespace(const std::string &str) {
    std::istringstream iss(str);
    std::vector<std::string> tokens;
    std::string token;

    while (iss >> token) { tokens.push_back(token); }

    return tokens;
}

//-----------------------------------------
// onMessage
//
// This is the callback that handles incoming messages from WebSockets.
//-----------------------------------------
void onMessage(WebsocketsClient& client, WebsocketsMessage message) {
  String data = message.data();
  Serial.print("WS message received: ");
  Serial.println(data);

  auto tokens = splitStringByWhitespace(data.c_str());
  std::string cmd = tokens.empty() ? "none":tokens[0];
  if( cmd=="none"){
    Serial.println("Empty command received.");

  // enable/disable
  }else if( cmd=="enable" && tokens.size()==3){
    int idx = atoi(&(tokens[1].c_str()[1])); // TODO: check that tokens[1].length()==1
    bool enable = atoi(tokens[2].c_str());
    if( tokens[1][0] == 'M' ){
      if( idx>=0 && idx<=3 ){MOTOR_ENABLE[idx]=enable; SetMotor(idx, MOTOR_SET[idx]);}else{Serial.println("Bad Motor number in: " + data);}
    }
    if( tokens[1][0] == 'S' ){
      if( idx>=0 && idx<=7 ){SERVO_ENABLE[idx]=enable; SetServo(idx, SERVO_SET[idx]);}else{Serial.println("Bad Servo number in: " + data);}
    }

  // Motor/servo increment/decrement
  }else if( cmd=="incr" && tokens.size()==3){
    int idx = atoi(&(tokens[1].c_str()[1])); // TODO: check that tokens[1].length()==1
    float delta = atof(tokens[2].c_str());
    if( tokens[1][0] == 'M' ){
      if( idx>=0 && idx<=3 ){SetMotor(idx, MOTOR_SET[idx]+delta);}else{Serial.println("Bad Motor number in: " + data);}
    }
    if( tokens[1][0] == 'S' ){
      if( idx>=0 && idx<=7 ){SetServo(idx, SERVO_SET[idx]+delta);}else{Serial.println("Bad Servo number in: " + data);}
    }

  // Motor/servo set
  }else if( cmd=="set" && tokens.size()==3){
    int idx = atoi(&(tokens[1].c_str()[1])); // TODO: check that tokens[1].length()==1
    float val = atof(tokens[2].c_str());
    if( tokens[1][0] == 'M' ){
      if( idx>=0 && idx<=3 ){SetMotor(idx, val);}else{Serial.println("Bad Motor number in: " + data);}
    }
    if( tokens[1][0] == 'S' ){
      if( idx>=0 && idx<=7 ){SetServo(idx, val);}else{Serial.println("Bad Servo number in: " + data);}
    }
  
  // Unknown or bad command
  }else{
    Serial.println("Bad command: " + data);
  }
}

//-----------------------------------------
// handleWebSockets
//
// This is called from loop() to handle new WebSockets connections,
// poll exisiting connections, and remove dead connections.
//-----------------------------------------
void handleWebSockets() {
  // Accept new WebSocket clients
  if (webSocketServer.poll()) {
    Serial.println("Accepting connection");
    WebsocketsClient client = webSocketServer.accept();
    if (client.available()) {
      Serial.println("Client available");
      client.onMessage([&client](WebsocketsMessage msg){ onMessage(client, msg);});
      webSocketClients.push_back(std::move(client));
    }
  }

  // Handle messages from all connected clients
  for (auto& client : webSocketClients) {
    client.poll(); // will automtically call onMessage() if message available
  }

  // Clean up disconnected clients
  webSocketClients.erase(
    std::remove_if(webSocketClients.begin(), webSocketClients.end(),
                   [](WebsocketsClient& c) {
                      if(!c.available()){
                        Serial.print("Removing WS client. Close reason: ");
                        Serial.println(c.getCloseReason());
                        return true;
                      }else{
                        return false;
                      }
                    }),
    webSocketClients.end());
  
  // Send status periodically to all connected clients
  static auto last_send = millis();
  if( millis()-last_send > 1000 ){

    Serial.println("Sending status ...");
    auto status_html = getStatusHTML();
    String statusMessage = "{\"type\": \"status\", \"content\": \"" + escapeJsonString(status_html) + "\"}";
    for(auto &client : webSocketClients){
      client.send(statusMessage);
    }
    last_send = millis();
  }
}

//----------------------------------------------
// handleDisplay
//
// Update display
//----------------------------------------------
void handleDisplay(void)
{
  if (! oledactive ) return;

  static auto last_time = millis();
  if( (millis() - last_time) < 1000 ) return;

  display.clearDisplay();
 
  display.setTextColor(WHITE);
  display.setTextSize(0);

  int y = 0;
  display.setCursor(0,y);
  String ipstr = String("IP: ") + WiFi.localIP().toString();
  display.print(ipstr);

  y += 10;
  display.setCursor(0,y);
  String macstr = String("MAC:") + WiFi.macAddress();
  display.print(macstr);

  // TODO: Connect input voltage to NodeMCU analog input pin with 1:10 voltage divider.
  // y += 10;
  // display.setCursor(0,y);
  // String vstr = String("   Battery: " + String(vals.vA0, 2) + String("V"));
  // display.print(vstr);

  display.display();
  last_time = millis();
}

//----------------------------------------------
// CreateWebServer
//
// Called to create the webserver. If all goes well and we
// connect to WiFi, then this will be called with the 
// standard port=80. If the WiFiManager falls back to creating
// its own AP, then it will listen on port 80 to serve up
// the configuration page and this will be called with port
// = 8080. This will allow the device to still function in
// AP mode.
//----------------------------------------------
void CreateWebServer (int port=80) {
  if( server ) delete server;
  // server = new ESP8266WebServer(port);
  server = new WebServer(port);

  auto reload_home = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"0;url=/\" /></head><body></body></html>";
  server->on("/ERASE_AND_RESTART.html", HTTP_GET, [reload_home]() { server->send(200, "text/html", reload_home); ERASE_AND_RESTART();});
  server->onNotFound(WebServerSendFile); // handle all file requests such as images and downloads
  server->begin();
}

//----------------------------------------------
// WiFiAPCallback
//
// Called iff WiFi is unable to connect and has
// to fall back to creating an AP for configuration
//----------------------------------------------
void WiFiAPCallback (WiFiManager *myWiFiManager) {
  Serial.println("WiFi connection falling back to AP for configuration.");
  CreateWebServer(8080);
  // Start websocket server which will handle persistent connections from javascript
  webSocketServer.listen(81);
  Serial.print("Websockets Server Status: ");
  Serial.println(webSocketServer.available());

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(0);
  display.setCursor(0, 0); display.print("WiFi config AP:");
  display.setCursor(0,10); display.print(APNAME);
  display.setCursor(0,25); display.print("Controls Web server:");
  display.setCursor(0,35); display.print("192.168.4.1:8080");
  display.display();
}

//----------------------------------------------
// WiFiAPLoopCallback
//
// Called iff WiFi is unable to connect and has
// fallen back to creating an AP for configuration.
// This is called from within the inner loop of the
// configPortal to give us a chance to service our
// own web server. This allows the robot to be controlled
// while using the AP configuration portal.
//
// NOTE: THE WiFiManager CODE WAS MODIFIED TO ALLOW
// THIS! IT IS NOT PART OF THE OFFICIAL WiFiManager
// SOURCE!
//----------------------------------------------
void WiFiAPLoopCallback (WiFiManager *myWiFiManager) {
  server->handleClient(); // Handle HTTP requests
  handleWebSockets();    // Handle WebSocket connections
  // handleDisplay();       // Handle updating display
}

//==========================================================================================================================

//-----------------------------------------
// setup
//-----------------------------------------
void setup() {
// Start serial monitor
  // Serial.begin(74880); // the ESP8266 boot loader uses this. We can set it to something else here, but the first few messages will be garbled.
  Serial.begin(115200); // the ESP32 boot loader uses this. We can set it to something else here, but the first few messages will be garbled.
  Serial.println();

  // Initialize OLED 128x64 pixel display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
  }else{
    Serial.println(F("SSD1306 found"));
    oledactive = true;
    display.display();
  }

  // Initialize LittleFS filesystem
  LittleFS.begin(); // TODO: check for error

  // Start PCS9685 PWM driver and set base frequency
  if( pwm.begin() ){
    Serial.println("PCA9685 PWM module initialized");
    // pwm.setPWMFreq(1000); // Set frequency to 1 kHz
    pwm.setPWMFreq(50); // Set frequency to 50 kHz (typical servo frequency)
    PWM_FOUND = true;
  }else{
    Serial.println("FAILED: PCA9685 PWM module initialization");
  }

  // Connect to WiFi
  // This will use cached credentials to try and connect. If unsuccessful,
  // it will automatically setup an AP where the credentials can be set
  // via web browser.
  WiFiManager wm;
  wm.setAPCallback(WiFiAPCallback);
  wm.setAPLoopCallback(WiFiAPLoopCallback);
  bool res = wm.autoConnect(APNAME);
  // Serial.println(F("Disconnecting and forgetting WiFi ..."));
  

  // If we failed to connect to WiFi, restart the device so we can try again.
  if(!res) {
    Serial.println("Failed to connect");
    ESP.restart();
  }else{
    Serial.println(" connected");
    // Connect to ntp Time Server
    Serial.printf("Connecting to ntp server to get current time ...\n");
    timeClient.begin();
    timeClient.update(); // n.b. this does not block and is not required to succeed
  }

  // If we get here, we're connected to WiFi
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());  // Print the local IP address

  // Start websocket server which will handle persistent connections from javascript
  webSocketServer.listen(81);
  Serial.print("Websockets Server Status: ");
  Serial.println(webSocketServer.available());

  //...............................................................................
  // Setup to allow Over The Air (OTA) updates to the firmware. This allows the
  // program to be updated via WiFi without having to connect via USB.
  // NOTE: To use this, select the approriate IP address for the serial port in
  // the Arduino IDE. This will allow uploading the new firmware but
  // IT WILL NOT ALLOW SERIAL MONITORING OVER THE WiFi!!!

  // Set password for firmware updates
  ArduinoOTA.setPassword("314159");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  //...............................................................................
  
  // Start HTTP Server for Web Page
  CreateWebServer();
  // server.on("/", HTTP_GET, []() { server.send(200, "text/html", getHomePageHTML());});
  // auto reload_home = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"0;url=/\" /></head><body></body></html>";
  // server.on("/ERASE_AND_RESTART.html", HTTP_GET, [reload_home]() { server.send(200, "text/html", reload_home); ERASE_AND_RESTART();});
  // server.onNotFound(WebServerSendFile); // handle all file requests such as images and downloads
  // server.begin();
  
}

//-----------------------------------------
// loop
//-----------------------------------------
void loop() {
  ArduinoOTA.handle();   // allows firmware upgrade over WiFi
  timeClient.update();   // keeps system clock updated

  server->handleClient(); // Handle HTTP requests
  handleWebSockets();    // Handle WebSocket connections

  handleDisplay();       // Handle updating display
}

