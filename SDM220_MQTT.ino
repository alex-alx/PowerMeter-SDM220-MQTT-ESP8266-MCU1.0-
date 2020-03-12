/*
    ************************************************************
    Modified by AlexAlx 2020
    ************************************************************
     
    ____ original *** esp-sdm-mqtt - patrik.mayer@codm.de _____

    SDMXXX (SDM120 / SDM220 / SDM630) series power meters from eastron over modbus
    using an esp8266 and an 3.3V RS485 tranceiver.

    This uses the SDM library by reaper7 https://github.com/reaper7/SDM_Energy_Meter
    mqtt client from https://github.com/knolleary/pubsubclient/
    Tasker from https://github.com/sticilface/Tasker

*/

// MQTT_MAX_PACKET_SIZE : Maximum packet size
#undef  MQTT_MAX_PACKET_SIZE // un-define max packet size
#define MQTT_MAX_PACKET_SIZE 512  // fix for MQTT client dropping messages over 128B

#include <Tasker.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <SDM.h>
#include <SoftwareSerial.h> 
#include <ArduinoJson.h>


//--------- Configuration
// WiFi
const char* ssid = "***WIFI***";                    // Wifi SSID
const char* password = "***PASS***";       // Wifi password
const char* mqttServer = "192.168.xx.xx";     // MQTT server
const char* mqttUser = "***admin***";           // MQTT userid
const char* mqttPass = "***PASS****";        // MQTT password
const char* mqttClientName = "***NAME***";     // MQTT client ID
const char* mqttTopicPrefix = "***NAME***/";   // MQTT root topic for the device, keep / at the end

//MQTT delay read every 60s
int measureDelay = 5000; 

// ************  Modbus RS485 settings ************
const int rxPin = 12;
const int txPin = 13;
const int derePin = 4;
const long baud = 9600;

// internal vars
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Tasker tasker;

SoftwareSerial swSerSDM;                                            //config SoftwareSerial
SDM sdm(swSerSDM, baud, derePin, SWSERIAL_8N1, rxPin, txPin);       //config SDM

// ********* MQTT parameters ***********
char mqttTopicStatus[64];
char mqttTopicIp[64];
char mqttTopicStat[64];
char mqqtBuff [512];

long lastReconnectAttempt = 0; //For the non blocking mqtt reconnect (in millis)

void setup() {
  Serial.begin(115200);                                                         //initialize serial
  Serial.println("Starting...");

  sdm.begin();

  //put in mqtt prefix
  sprintf(mqttTopicStatus, "%sstatus", mqttTopicPrefix);
  sprintf(mqttTopicIp, "%sip", mqttTopicPrefix);
  sprintf(mqttTopicStat, "%sSTATUS8", mqttTopicPrefix);

  setup_wifi();
  mqttClient.setServer(mqttServer, 1883);

  tasker.setInterval(meassureSDM, measureDelay);

  //----------- OTA -----------------------
  MDNS.update();
  ArduinoOTA.setHostname(mqttClientName);

 //Password authentication
  ArduinoOTA.setPassword((const char *)"simaticpanel");
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    delay(1000);
    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

}

void loop() {

  //handle mqtt connection, non-blocking
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (MqttReconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  mqttClient.loop();

  tasker.loop();

  //handle OTA
  ArduinoOTA.handle();

}

//void meassureSDM(int)
void meassureSDM() {
  float v = sdm.readVal(SDM220T_VOLTAGE);                                       //read voltage [V]
  float c = sdm.readVal(SDM220T_CURRENT);                                       //read current [A]
  float p = sdm.readVal(SDM220T_POWER);                                         //read power [W]
  float f = sdm.readVal(SDM220T_FREQUENCY);                                     //read frequency [Hz}
  float pf = sdm.readVal(SDM220T_POWER_FACTOR);                                 //read Power Factor []
  float pa = sdm.readVal(SDM220T_PHASE_ANGLE);                                  //read Phase Angle [Deg]
  float aap = sdm.readVal(SDM220T_ACTIVE_APPARENT_POWER);                       //read Active Apparent Power [VA]
  float rap = sdm.readVal(SDM220T_REACTIVE_APPARENT_POWER);                     //read Reactive Apparent Power [VAR]
  float iae = sdm.readVal(SDM220T_IMPORT_ACTIVE_ENERGY);                        //read Import Active Energy [Wh]
  float eae = sdm.readVal(SDM220T_EXPORT_ACTIVE_ENERGY);                        //read Export Active Energy [Wh]
  float ire = sdm.readVal(SDM220T_IMPORT_REACTIVE_ENERGY);                      //read Import Active Energy [VARh]
  float ere = sdm.readVal(SDM220T_EXPORT_REACTIVE_ENERGY);                      //read Export Active Energy [VARh]
  float tae = sdm.readVal(SDM220T_TOTAL_ACTIVE_ENERGY);                         //read Export Active Energy [Wh]
  float tre = sdm.readVal(SDM220T_TOTAL_REACTIVE_ENERGY);                       //read Export Active Energy [VARh]  

//*********  JSon packaging ********************
  StaticJsonDocument <512> JSONbuffer;
  JSONbuffer["Voltage"] = v;
  JSONbuffer["Current"] =  c;
  JSONbuffer["Power"] =  p;
  JSONbuffer["Freq"] =  f;
  JSONbuffer["PowerFactor"] =  pf;
  JSONbuffer["PhaseAngle"] =  pa;
  JSONbuffer["ActApparentPwr"] =  aap;
  JSONbuffer["RctApparentPwr"] =  rap; 
  JSONbuffer["ImpActEnergy"] =  iae;  
  JSONbuffer["ExpActEnergy"] =  eae; 
  JSONbuffer["ImpRctEnergy"] =  ire;  
  JSONbuffer["ExpRctEnergy"] =  ere;
  JSONbuffer["TotActEnergy"] =  tae;
  JSONbuffer["TotRctEnergy"] =  tre;
 
  serializeJson(JSONbuffer, mqqtBuff);
  
  mqttClient.publish(mqttTopicStat, mqqtBuff );

//******** Serial monitor of values *********
 Serial.println( mqqtBuff );
 Serial.println();

}

void setup_wifi() {

  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA); //disable AP mode, only station
  WiFi.hostname(mqttClientName);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


bool MqttReconnect() {

  if (!mqttClient.connected()) {

    Serial.print("Attempting MQTT connection...");

    // Attempt to connect with last will retained
    if (mqttClient.connect(mqttClientName, mqttUser, mqttPass, mqttTopicStatus, 1, true, "offline")) {

      Serial.println("connected");

      // Once connected, publish an announcement...
      char curIp[16];
      sprintf(curIp, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);

      mqttClient.publish(mqttTopicStatus, "online", true);
      mqttClient.publish(mqttTopicIp, curIp, true);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
    }
  }
   return mqttClient.connected();
}

//Example tasmota
//stat/SHP6_BD1/STATUS8 = {"StatusSNS":{"Time":"2020-02-21T22:41:29","ENERGY":{"TotalStartTime":"2020-02-11T14:29:03","Total":26.381,"Yesterday":1.234,"Today":1.310,"Power":187,"ApparentPower":
