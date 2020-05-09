#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>


// EEPROM
#define EEPROM_SIZE              1
#define EEPROM_ADDR              0

// WiFi SSID and Password
#define WiFi_SSID                "SSID"
#define WiFi_Pass                "PASS"
#define watchdogWiFi             30000

// Static IP
IPAddress local_IP(192, 168, 1, 105);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// Asynchronous Web Server
AsyncWebServer server(8080);

// Homekit States
int targetPosition;  // Set By EEPROM in Setup
int currentPosition; // Set By EEPROM in Setup
int currentState;    // Set By EEPROM in Setup

// Timing States
unsigned long startTime   = 0;
unsigned long currentTime = 0;

// Delays
const int DELTAS[]    = {5000000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000};
const int DELTA_C     = 250000;

// Commands
const int UP_CODE[]   = {5000, 580, 580, 260, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 580, 260, 580, 260, 260, 580, 580, 260, 580, 260, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 260, 580, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 580, 260, 260, 580, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260};
const int STOP_CODE[] = {5000, 580, 580, 260, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 580, 260, 580, 260, 260, 580, 580, 260, 580, 260, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 260, 580, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 580, 260, 580, 260, 260, 580, 260, 580, 260, 580, 260, 580, 580, 260, 260, 580, 580, 260, 580, 260, 580, 260};
const int DOWN_CODE[] = {5000, 580, 580, 260, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 580, 260, 580, 260, 260, 580, 580, 260, 580, 260, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 260, 580, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 260, 580, 260, 580, 580, 260, 580, 260, 580, 260, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 260, 580, 580, 260, 260, 580, 260, 580, 260, 580, 260, 580, 580, 260, 580, 260, 260, 580, 260, 580, 580, 260, 260, 580, 580, 260, 260, 580, 580, 260, 580, 260, 580, 260};

// Transmit Configuration
#define TRANSMIT_PIN             15          // Digital Pin 15
#define TRANSMIT_REG             0x8000      // Register Pin 15

// Code Configuration
#define REPEAT_COMMAND           2           // Number of times to transmit
#define COMMAND_SIZE             132         // 65*2 + 1*2
#define RADIO_SILENCE            5000


// Transmit Function
void transmit(const int command[]) {
  // LED On
  digitalWrite(LED_BUILTIN, LOW);

  // Disable Interrupts
  noInterrupts();
  
  // Repeat Command
  for (int i = 0; i < REPEAT_COMMAND; i++)
    {
      // Transmit AGC and Code
      for (int j = 0; j < COMMAND_SIZE; j += 2)
        {
          // High
          GPOS = TRANSMIT_REG;
          delayMicroseconds(command[j]);
    
          // Low
          GPOC = TRANSMIT_REG;
          delayMicroseconds(command[j+1] - 40);      
        }
    
      // Transmit Radio Silence
      delayMicroseconds(RADIO_SILENCE);
    }

  // Enable Interrupts
  interrupts();

  // LED Off
  digitalWrite(LED_BUILTIN, HIGH);
}


// Set Request
void handleSet(AsyncWebServerRequest *request){
  // Handle Query
  if (request->hasParam("target_position"))
    {
      // Get Target
      int target = request->getParam("target_position")->value().toInt();

      // Constrain Target
      target = min(target, 100);
      target = max(0, target);

      // Set Target Position
      targetPosition = target;

      // HTTP Reply
      request->send(204);
    }

  else if (request->hasParam("reset"))
    {
      // HTTP Reply
      request->send(204);

      // Reset
      reset();      
    }

  else
    {
      // HTTP Reply
      request->send(400);
    }
}


// Get Request
void handleGet(AsyncWebServerRequest *request){
  // Handle Query
  if (request->hasParam("current_position"))
    {
      // HTTP Reply
      request->send(200, "text/plain", String(currentPosition));
    }

  else if (request->hasParam("current_state"))
    {
      // HTTP Reply
      request->send(200, "text/plain", String(currentState));
    }

  else if (request->hasParam("target_position"))
    {
      // HTTP Reply
      request->send(200, "text/plain", String(targetPosition));
    }

  else if (request->hasParam("rssi"))
    {
      // HTTP Reply
      request->send(200, "text/plain", String(WiFi.RSSI()));
    }

  else if (request->hasParam("uptime"))
    {
      // Get Uptime in Seconds
      unsigned long upSeconds = millis() / 1000;

      // Construct Time Delta
      unsigned long upMinutes = upSeconds / 60;
      unsigned long upHours = upMinutes / 60;
      unsigned long upDays = upHours / 24;

      // Format String
      String strUpTime = 
        String(upDays) + String(" Days, ") +
        String(upHours % 24) + String(" Hours, ") +
        String(upMinutes % 60) + String(" Minutes, ") +
        String(upSeconds % 60) + String(" Seconds") +
        " (" + String(upSeconds) + " Seconds)";

      // HTTP Reply
      request->send(200, "text/plain", strUpTime);
    }

  else
    {
      // HTTP Reply
      request->send(400);
    }
}


// Not Found
void handleNotFound(AsyncWebServerRequest *request) {
  // HTTP Reply
  request->send(404, "text/plain", "Not found");
}


// Reset
void reset(){
  // Signal Reset with LED on for 5 seconds
  digitalWrite(LED_BUILTIN, LOW);
  delay(5000);

  // Reset ESP8266
  ESP.reset();
}


// Setup
void setup(){
  // Start EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Read States from EEPROM
  targetPosition = EEPROM.read(EEPROM_ADDR);  // Stored in EEPROM
  currentPosition = EEPROM.read(EEPROM_ADDR); // Stored in EEPROM
  currentState = 2;                           // Start Stopped

  // Constrain Positions in Case of EEPROM Failure (Default 0xFF)
  targetPosition = min(targetPosition, 100);
  targetPosition = max(0, targetPosition);
  currentPosition = min(currentPosition, 100);
  currentPosition = max(0, currentPosition);

  // Setup LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup RF GPIO
  pinMode(TRANSMIT_PIN, OUTPUT);

  // Set Transmitter LOW
  GPOC = TRANSMIT_REG;

  // Connect to Wifi
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(WiFi_SSID, WiFi_Pass);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  // Wait for Connection
  unsigned long startWiFiTime = millis();
  while (WiFi.status() != WL_CONNECTED)
    {
      // WiFi Watchdog
      unsigned long currentWiFiTime = millis();
      if (currentWiFiTime - startWiFiTime >= watchdogWiFi)
        {
          // Reset
          reset();
        }

      // Delay
      delay(500);
  
      // Toggle LED
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

  // Connected
  if (WiFi.status() == WL_CONNECTED) 
    {
      // Turn LED Off
      digitalWrite(LED_BUILTIN, HIGH);

      // Flash LED
      for (int i = 0; i < 10; i++) 
        {
          // Delay
          delay(100);
    
          // Toggle LED
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
      // Turn LED Off
      digitalWrite(LED_BUILTIN, HIGH);
    }

  // HTTP API Handles
  server.on("/set", HTTP_POST, handleSet);
  server.on("/get", HTTP_GET, handleGet);

  // HTTP Not Found
  server.onNotFound(handleNotFound);

  // Start Server
  server.begin();
}


// Loop
void loop(){
  // Yield
  yield();
  
  // Handle Go Up
  if ((currentPosition < targetPosition) && (currentState != 1))
    {
      // Commit '100' to EEPROM in Case of Reset
      EEPROM.write(EEPROM_ADDR, 100);
      EEPROM.commit();
      
      // Transmit Up
      transmit(UP_CODE);

      // Extra Delay For Direction Change
      if (currentState != 2)
        {
          delayMicroseconds(DELTA_C);
        }

      // Set Time
      startTime = micros();

      // Set State
      currentState = 1;
    }

  // Handle Go Down
  else if ((currentPosition > targetPosition) && (currentState != 0))
    {
      // Commit '0' to EEPROM in Case of Reset
      EEPROM.write(EEPROM_ADDR, 0);
      EEPROM.commit();

      // Transmit Down
      transmit(DOWN_CODE);

      // Extra Delay For Direction Change
      if (currentState != 2)
        {
          delayMicroseconds(DELTA_C);
        }

      // Set Time
      startTime = micros();

      // Set State
      currentState = 0;
    }

  // Update Current Position
  else if (currentState != 2)
    {
      // Current Time
      currentTime = micros();
      unsigned long delta = currentTime - startTime;

      // Update Position
      if (currentState == 1) // Opening
        {
          if (delta >= DELTAS[currentPosition])
          {
            currentPosition += 1;
            startTime = currentTime;
          }
        }
      else // Closing
        {
          if (delta >= DELTAS[currentPosition - 1])
          {
            currentPosition -= 1;
            startTime = currentTime;
          }
        }

      // Stop
      if (currentPosition == targetPosition)
        {
          if (targetPosition != 100 && targetPosition != 0)
          {
            // Transmit Stop
            transmit(STOP_CODE);
            transmit(STOP_CODE);
          }

          // Set State
          currentState = 2;

          // Commit currentPosition to EEPROM in Case of Reset
          EEPROM.write(EEPROM_ADDR, currentPosition);
          EEPROM.commit();
        }
    }

  // Sleep
  else
    {
      // Check if WiFi Connected
      if (WiFi.status() != WL_CONNECTED)
        {
          // Reset if WiFi Disconnect
          reset();
        }
    }
}
