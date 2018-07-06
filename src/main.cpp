#define DEBUG 1
#define FW_VERSION 1

#define BUTTON 19

#define SEALEVELPRESSURE_HPA 1013.25

// ---- used for JSN-SR04T-2.0
#define DISTANCE_SENSOR_PRESENT 0
#define TRIGPIN 25
#define ECHOPIN 26

// --- config reset levels
#define RESET_NONE 0
#define RESET_WIFI 1
#define RESET_ALL 2

extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}

#include <Arduino.h>
#include <Update.h>
#include <Wire.h>
#include <WiFi.h>
//#include <FS.h>
#include <ArduinoJson.h>
#include <Configuration.hpp>
#include <AsyncMqttClient.h>
#include <ESP32WebServer.h> // https://github.com/nhatuan84/esp32-webserver
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

WiFiClient client;
AsyncMqttClient mqttClient;
DynamicJsonBuffer jsonBuffer(512);
ESP32WebServer server(80);
Adafruit_BME280 bme;

byte sensorPresentBME280 = 0x00; // -- will hold the address of the BME280 if found.

Configuration conf = Configuration(String{"/config.json"});
TimerHandle_t mqttReconnectTimer;
TimerHandle_t sensorTimer;
TimerHandle_t pixelTimer;

volatile int r = 0;
volatile int g = 0;
volatile int b = 0;
volatile int tgtR = 0;
volatile int tgtG = 0;
volatile int tgtB = 0;
volatile int rPerStep = 0;
volatile int gPerStep = 0;
volatile int bPerStep = 0;

// Variables to validate
// response from OTA Server
int contentLength       = 0;
bool isValidContentType = false;
String mqttBasePath     = "dietrix/v1/";

// -- Prototypes
String jsonCreate();
void execOTA();
void connectToWifi();

void resetConfiguration(int lvl = RESET_NONE) {
    switch (lvl) {
        case RESET_ALL:
            Serial.println("Reset all Configuration and restart.");
            conf.reset();
            break;
        case RESET_WIFI:
            Serial.println("Reset Wifi Configuration.");
            conf.set("SSID","");
            conf.set("PSWD","");
            break;
        default:
            break;
    }
    conf.save();
    conf.dump(); // --- Debugging output
    esp_restart();
}

int getDistance() {
  digitalWrite(TRIGPIN, LOW); // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH); // Send a 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW); // Send pin low again
  int distance = pulseIn(ECHOPIN, HIGH,26000); // Read in times pulse
  distance= distance/58;
  Serial.print(distance);
  Serial.println("   cm");                    
  // Hint: Wait 50mS before next ranging  
  return distance;
}

void initDistanceSensor() {
  pinMode(ECHOPIN, INPUT_PULLUP);
  pinMode(TRIGPIN, OUTPUT);
  digitalWrite(TRIGPIN,LOW);
  Serial.println("Distance Sensor initialized.");
}

void i2cScan() {
  Wire.begin();
  byte error, address;
 
  Serial.println("Scanning...");
 
  for(address = 1; address < 127; address++ ) {
    // Use return value of Write.endTransmisstion to check for ACK
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0) {

      switch (address) {
        case 0x76:
        case 0x77:
          Serial.printf("BME280 found at 0x%x\n",address);
          sensorPresentBME280 = address;
          break;
        default:
          Serial.printf("unknown I2C device found at 0x%x\n",address);
      }

    }     
  }
}

// --- Triggered when SoftAP is ready. Should start the webserver.
void startConfigServer() {
  server.on("/config", HTTP_GET, [](){
    Serial.println("Request Received.");
    bool noCfgChg = true;
    // Handle configuration settings
    if (server.hasArg("sid")) {
        conf.set("SSID",server.arg("sid"));
        noCfgChg = false;
    } 
    if (server.hasArg("pswd")) {
        conf.set("PSWD",server.arg("pswd"));
        noCfgChg = false;
    }
    if (server.hasArg("name")) {
        conf.set("NAME",server.arg("name"));
        noCfgChg = false;
    } 
    if (server.hasArg("mqtt_host")) {
        conf.set("MQTT_HOST",server.arg("mqtt_host"));
        noCfgChg = false;
    } 
    if (server.hasArg("mqtt_port")) {
        conf.set("MQTT_PORT",server.arg("mqtt_port"));
        noCfgChg = false;
    } 

    if (noCfgChg) 
        server.send(406, "text/plain", "No known Parameters.");
    else {    
        conf.save();
        conf.dump();
        server.send(200, "text/plain", "Parameter set.");
    }
  });
  server.onNotFound([](){
    server.send(404, "text/plain", "this page is not known");
  });
  //server.serveStatic("/files",SPIFFS,"/","max-age=86400");
  server.begin();
  Serial.println("HTTP server started");
}

void connectToMqtt() {
  Serial.println(conf.get("NAME") + " is connecting to MQTT " + conf.get("MQTT_SRV"));
  mqttClient.connect();
}

void sendSensorDiscover(String sensorName, char* UoM, char* icon) {
  String mqttDiscoverPrefix      = "homeassistant";
  String mqttDiscoverTopic       = mqttDiscoverPrefix + "/sensor/" + conf.get("NAME") + "/"; 
  String mqttSensorDiscoverTopic = mqttDiscoverTopic + conf.get("NAME") + "_" + sensorName + "/config";

  String stateTopic = String{ conf.get("NAME") + "/sensor/" + conf.get("NAME") + "_" + sensorName + "/state"};
  String availTopic = String{ conf.get("NAME") + "/status"};
  String buf;
  String str = sensorName;
  str[0] = toupper(str[0]);
  String friendlyName = conf.get("NAME") + " " + str;

  JsonObject& root = jsonBuffer.createObject();
  root["unit_of_measurement"] = UoM;
  root["expire_after"]        = 225;
  root["icon"]                = icon; // -- for Reference: https://materialdesignicons.com/
  root["name"]                = friendlyName;
  root["state_topic"]         = stateTopic.c_str();
  root["availability_topic"]  = availTopic.c_str();
  root.printTo(buf);
  Serial.printf("sending DISCOVER to %s\n",mqttSensorDiscoverTopic.c_str());
  root.printTo(Serial);
  Serial.println();
  mqttClient.publish(mqttSensorDiscoverTopic.c_str(), 1, true, buf.c_str());
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  // --- Send DISCOVER messages
  if (sensorPresentBME280 > 0) {
    sendSensorDiscover("temperature", "°C", "mdi:thermometer");
    sendSensorDiscover("pressure", "hPa", "mdi:terrain");
    sendSensorDiscover("humidity", "%", "mdi:opacity");
  }

  if (DISTANCE_SENSOR_PRESENT) 
    sendSensorDiscover("distance", "cm", "mdi:ruler");

  // --- Publish status change
  uint16_t packetIdPub1 = mqttClient.publish(String{ conf.get("NAME") + "/status"}.c_str(), 1, true, "online");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  //setPixelSteps(0,0,1);

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

uint16_t sendSensorData(String sensorName, char* sensorValue) {
  // sensor02/sensor/sensor02_temp/state  
  String stateTopic = String{ conf.get("NAME") + "/sensor/" + conf.get("NAME") + "_" + sensorName + "/state"};
  Serial.printf("sending DATA to %s (%s)\n",stateTopic.c_str(), sensorValue);
  return mqttClient.publish(stateTopic.c_str(), 1, true, sensorValue);
}

uint16_t sendSensorData(String sensorName, float sensorValue) {
    char str_float[10];
    snprintf(str_float,10,"%.2f",sensorValue);
    return sendSensorData(sensorName,str_float);
}


String jsonCreate() {
    JsonObject& root = jsonBuffer.createObject();

    JsonObject& device = root.createNestedObject("device");
    device["name"] = conf.get("NAME");
    device["ver"]  = FW_VERSION;
    device["up"]   = millis()/1000;
    JsonArray& groups = device.createNestedArray("grps");
    // -- ToDo: add all groups configured

    JsonObject& wifi = root.createNestedObject("wifi");
    wifi["ssid"] = conf.get("SSID");
    //wifi["pswd"] = ""; // --- only setter supported ;-)
    wifi["ip"]   = WiFi.localIP().toString();

    JsonObject& ota = root.createNestedObject("ota");
    ota["host"] = conf.get("OTA_HOST");
    ota["port"] = conf.get("OTA_PORT");
    ota["path"] = conf.get("OTA_PATH");
    ota["hash"] = conf.get("OTA_HASH");

    String buf;
    root.printTo(buf);
    return buf;
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    bool ota_needed = false;
    Serial.println("Publish received.");
    Serial.print("  topic: ");
    Serial.println(topic);
    Serial.print("  qos: ");
    Serial.println(properties.qos);
    Serial.print("  dup: ");
    Serial.println(properties.dup);
    Serial.print("  retain: ");
    Serial.println(properties.retain);
    Serial.print("  len: ");
    Serial.println(len);
    Serial.print("  index: ");
    Serial.println(index);
    Serial.print("  total: ");
    Serial.println(total);

    String pl = String(payload);
    if (pl.length() > 2) {
      JsonObject& root = jsonBuffer.parseObject(pl);
      if (root.containsKey("wifi")) {
        // -- ssid
        JsonObject& wifi = root["wifi"];
        if (wifi.containsKey("ssid"))
          conf.set("SSID",wifi["ssid"]);
        // -- pswd
        if (wifi.containsKey("pswd"))
          conf.set("PSWD",wifi["pswd"]);
      }
      if (root.containsKey("ota")) {
        JsonObject& ota = root["ota"];
        // -- host
        if (ota.containsKey("host"))
          conf.set("OTA_HOST",ota["host"]);
        // -- port
        if (ota.containsKey("port"))
          conf.set("OTA_PORT",ota["port"]);
        // -- path
        if (ota.containsKey("path"))
          conf.set("OTA_PATH",ota["path"]);
        // -- hash
        if (ota.containsKey("hash"))
          conf.set("OTA_HASH",ota["hash"]);
      }
      if (root.containsKey("device")) {
        JsonObject& device = root["device"];
        // -- name
        if (device.containsKey("name"))
          conf.set("NAME",device["name"]);
        // -- ver 
        if (device.containsKey("ver")) {
          if (device["ver"] > FW_VERSION)
            ota_needed = true;
        }
        // -- int
        if (device.containsKey("int"))
          conf.set("INTERVAL",device["int"]);
        // -- grps
        // To be done....
      }
      conf.save();
      // -- keep "control" last to make sure settings are applied before.
      if (root.containsKey("control")) {
        JsonObject& control = root["control"];
        // -- reboot
        if (control.containsKey("reboot"))
          esp_restart();
        // -- ota
        if (control.containsKey("ota") or ota_needed)
          execOTA();
        // -- reconnect
      }
      

    }
}

void onMqttPublish(uint16_t packetId) {
  Serial.printf("Publish acknowledged. (packID: %i)\n",packetId);
}

// Utility to extract header value from headers
String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}

// OTA Logic 
void execOTA() {
  String host = conf.get("OTA_HOST");
  int port    = atoi(conf.get("OTA_PORT").c_str());
  String bin  = conf.get("OTA_PATH");

  Serial.println("Connecting to: " + String(host));
  // Connect to Server
  if (client.connect(host.c_str(), port)) {
    // Connection Succeed.
    // Fecthing the bin
    Serial.println("Fetching Bin: " + String(bin));

    // Get the contents of the bin file
    client.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    // Check what is being sent
    //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
    //                 "Host: " + host + "\r\n" +
    //                 "Cache-Control: no-cache\r\n" +
    //                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("Client Timeout !");
        client.stop();
        return;
      }
    }
    // Once the response is available,
    // check stuff

    /*
       Response Structure
        HTTP/1.1 200 OK
        x-amz-id-2: NVKxnU1aIQMmpGKhSwpCBh8y2JPbak18QLIfE+OiUDOos+7UftZKjtCFqrwsGOZRN5Zee0jpTd0=
        x-amz-request-id: 2D56B47560B764EC
        Date: Wed, 14 Jun 2017 03:33:59 GMT
        Last-Modified: Fri, 02 Jun 2017 14:50:11 GMT
        ETag: "d2afebbaaebc38cd669ce36727152af9"
        Accept-Ranges: bytes
        Content-Type: application/octet-stream
        Content-Length: 357280
        Server: AmazonS3
                                   
        {{BIN FILE CONTENTS}}
    */
    while (client.available()) {
      // read line till /n
      String line = client.readStringUntil('\n');
      // remove space, to check if the line is end of headers
      line.trim();

      // if the the line is empty,
      // this is end of headers
      // break the while and feed the
      // remaining `client` to the
      // Update.writeStream();
      if (!line.length()) {
        //headers ended
        break; // and get the OTA started
      }

      // Check if the HTTP Response is 200
      // else break and Exit Update
      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
          Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
          break;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: ")) {
        contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } else {
    // Connect to Server failed
    // May be try?
    // Probably a choppy network?
    Serial.println("Connection to " + String(host) + " failed. Please check your setup");
    // retry??
    // execOTA();
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin) {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("Written : " + String(written) + " successfully");
      } else {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
        // retry??
        // execOTA();
      }

      if (Update.end()) {
        Serial.println("OTA done!");
        if (Update.isFinished()) {
          Serial.println("Update successfully completed. Rebooting.");
          ESP.restart();
        } else {
          Serial.println("Update not finished? Something went wrong!");
        }
      } else {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    } else {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      Serial.println("Not enough space to begin OTA");
      client.flush();
    }
  } else {
    Serial.println("There was no content in the response");
    client.flush();
  }
}

void WiFiEvent(WiFiEvent_t event) {
    switch(event) {
        case SYSTEM_EVENT_WIFI_READY:           /**< ESP32 WiFi ready */
        case SYSTEM_EVENT_SCAN_DONE:            /**< ESP32 finish scanning AP */
        case SYSTEM_EVENT_STA_START:            /**< ESP32 station start */
        case SYSTEM_EVENT_STA_STOP:             /**< ESP32 station stop */
        case SYSTEM_EVENT_STA_CONNECTED:        /**< ESP32 station connected to AP */
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:  /**< the auth mode of AP connected by ESP32 station changed */
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:   /**< ESP32 station wps succeeds in enrollee mode */
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:    /**< ESP32 station wps fails in enrollee mode */
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:   /**< ESP32 station wps timeout in enrollee mode */
        case SYSTEM_EVENT_STA_WPS_ER_PIN:       /**< ESP32 station wps pin code in enrollee mode */
        case SYSTEM_EVENT_AP_PROBEREQRECVED:    /**< Receive probe request packet in soft-AP interface */
        case SYSTEM_EVENT_GOT_IP6:              /**< ESP32 station or ap or ethernet interface v6IP addr is preferred */
        case SYSTEM_EVENT_ETH_START:            /**< ESP32 ethernet start */
        case SYSTEM_EVENT_ETH_STOP:             /**< ESP32 ethernet stop */
        case SYSTEM_EVENT_ETH_CONNECTED:        /**< ESP32 ethernet phy link up */
        case SYSTEM_EVENT_AP_STOP:              /**< ESP32 soft-AP stop */
        case SYSTEM_EVENT_ETH_DISCONNECTED:     /**< ESP32 ethernet phy link down */
            Serial.printf("[WiFi-event] event: %d\n", event);
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:      /**< a station connected to ESP32 soft-AP */
            Serial.println("Client connected to SoftAP.");
            startConfigServer();
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:   /**< a station disconnected from ESP32 soft-AP */
            server.stop();
            Serial.println("Client disconnected from SoftAP.");
            break;
        case SYSTEM_EVENT_AP_START:             /**< ESP32 soft-AP start */
            Serial.println("Starting SoftAP...");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:           /**< ESP32 ethernet got IP from connected AP */
        case SYSTEM_EVENT_STA_GOT_IP:           /**< ESP32 station got IP from connected AP */
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
            connectToMqtt();
            break;
        case SYSTEM_EVENT_STA_LOST_IP:          /**< ESP32 station lost IP and the IP is reset to 0 */
        case SYSTEM_EVENT_STA_DISCONNECTED:     /**< ESP32 station disconnected from AP */
            Serial.println("WiFi lost connection");
            xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
            break;
    }
}

int bttnDown = 0;
void handleBTTN(){
    uint bDuration = 0;
    if ((bttnDown == 0) && (digitalRead(BUTTON) == LOW)) {
        bttnDown = millis();
    } else if (bttnDown > 0) {
        bDuration = millis() - bttnDown;
        bttnDown = 0;
    }

    if (bDuration > 0) {
        Serial.printf("Button 1 had been down %i millis.\r\n",bDuration);
        if (bDuration > 10) {
          if (bDuration > 5000) {
            SPIFFS.format();
          } else if (bDuration > 3000) {
            resetConfiguration(RESET_WIFI);
          }
        }
    }

}

bool readBME280plausibility(float& temp, float& pres, float& humi) {
  temp = bme.readTemperature();
  if ((temp < -40) || (temp > 85)) {
    Serial.printf("Non-Plausible value for Temp (%.2f)!",temp);
    return false;
  }

  pres = bme.readPressure();
  if ((pres < 30000) || (pres > 110000)) {
    Serial.printf("Non-Plausible value for Pressure (%.2f)!",pres);
    return false;
  }

  humi = bme.readHumidity();
  if ((humi < 0) || (humi > 100)) {
    Serial.printf("Non-Plausible value for Humidity (%.2f)!",humi);
    return false;
  }

  return true;
}

void initBME280() {
  if (! bme.begin(sensorPresentBME280)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  } else {
    xTimerStart(sensorTimer, 0);
  }
}

void printValues() {
  if (sensorPresentBME280 > 0) {
    // Error Message after a while (all Sensors!):
    // [E][esp32-hal-i2c.c:161] i2cWrite(): Busy Timeout! Addr: 77
    float temp, pres, humi;
    if (readBME280plausibility(temp,pres,humi)) {
      sendSensorData("temperature",temp);
      sendSensorData("pressure",pres / 100.0F);
      sendSensorData("humidity",humi);
    } else {
      Serial.println("Re-Initializing BME280 to avoid I2C bug.");
      xTimerStop(sensorTimer, 0);
      initBME280();
    }
  }
  if (DISTANCE_SENSOR_PRESENT) {
    getDistance();
  }
}

void setup() {
    Serial.begin(115200);

    //pixelTimer = xTimerCreate("pixelTimer", pdMS_TO_TICKS(100), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(runPixelSteps));
    //setPixelSteps(0,0,12); // --- RGB, lower numbers make the light flash faster. Try Values between 1 and 25.
    
    // --- Basic Hardware setup: attach button Interupts
    pinMode(BUTTON,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON), handleBTTN, CHANGE);

   WiFi.onEvent(WiFiEvent);

    if (! SPIFFS.begin(true)) {
      Serial.println("Could not start SPIFFS !");
      //setPixelSteps(1,0,0);
      while (1) {}
    }

    if (! (conf.load() &&
          (conf.keyExists("SSID")) &&
          (conf.keyExists("PSWD")) &&
          (conf.keyExists("NAME"))) ) {
            WiFi.mode(WIFI_MODE_AP);
            WiFi.softAP("ssp_setup");
    } else {
      conf.dump();
      WiFi.mode(WIFI_MODE_STA);
      WiFi.begin(conf.get("SSID").c_str(), conf.get("PSWD").c_str());

      if (conf.keyExists("MQTT_HOST") &&
          conf.keyExists("MQTT_PORT")) {
        mqttClient.onConnect(onMqttConnect);
        mqttClient.onDisconnect(onMqttDisconnect);
        mqttClient.onSubscribe(onMqttSubscribe);
        mqttClient.onUnsubscribe(onMqttUnsubscribe);
        mqttClient.onMessage(onMqttMessage);
        mqttClient.onPublish(onMqttPublish);
        mqttClient.setServer(conf.get("MQTT_HOST").c_str(), atoi(conf.get("MQTT_PORT").c_str()));

        // -- Configuration complete, now start scanning for attached sensors.
        i2cScan();

        if (sensorPresentBME280 > 0) {
          sensorTimer = xTimerCreate("sensorTimer", pdMS_TO_TICKS(30000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(printValues));
          initBME280();
        }

        if (DISTANCE_SENSOR_PRESENT) {
          initDistanceSensor();
        }

        // --- setup done, connecting to MQTT...
        mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
      } else {
        Serial.println("MQTT setup incomplete. Strating config server.");
        WiFi.mode(WIFI_MODE_AP);
        WiFi.softAP("ssp_setup");
      }
    }
}

void loop() {
    // put your main code here, to run repeatedly:
    server.handleClient(); // handle config Server events
}