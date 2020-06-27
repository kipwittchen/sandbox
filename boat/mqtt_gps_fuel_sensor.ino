#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>

#ifndef STASSID
#define STASSID "ENTTER_SSID" //
#define STAPSK  "ENTER_PASSWORD"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;

/*
   TinyGPS++ (TinyGPSPlus)
*/
static const int RXPin = D1; //RX on D1 connects to TX on GPS module
static const int TXPin = D2; //TX on D1 connects to RX on GPS module
static const uint32_t GPSBaud = 9600;

#define mqtt_server "192.168.x.x" //enter your MQTT server IP address
#define mqtt_user "username"
#define mqtt_password "password"

#define gph_topic "boat/sensor/gph"
#define gallonstotal_topic "boat/sensor/gallonstotal"
#define mph_topic "boat/sensor/mph"
#define mpg_topic "boat/sensor/mpg"
#define satellites_topic "boat/sensor/satellites"

WiFiClient espClient;
PubSubClient client(espClient);

//fuel flow sensor
int pinflow = D5; 
volatile unsigned int pulses = 0;
unsigned int pulse = 0;
unsigned long totalgallonpulses = 0;
const int pulses_per_gallon = 8224; //math works out to 8224.156 pulse per gallon based on 0.46mL per pulse
float gallonsperhour;
float milespergallon;

// For stats that happen every 5 seconds
unsigned long last = 0UL;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  //setup mqtt
  client.setServer(mqtt_server, 1883);
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");


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
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  //fuel flow sesor
  pinMode(pinflow, INPUT); 
  attachInterrupt(digitalPinToInterrupt(pinflow), count_pulse, RISING); 
  
  delay(1000); //delay 1 second to allow gps to start
  //Begin GPS
  ss.begin(GPSBaud);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 0.5 seconds before retrying
      delay(500);
    }
  }
}

//interrupt pulse counter for fuel flow
void count_pulse() 
{ 
  pulses++; 
} 

void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  // Dispatch incoming characters
  while (ss.available() > 0)
    gps.encode(ss.read());
    
  //check for a time desired refresh
  if (millis() - last > 5000){
    last = millis();
    //do fuel stuffs
    pulse = pulses;
    //pulse = random(17,160); //comment this line when hooked up to fuel meter
    pulses = 0;
    
    //5 second update
    gallonsperhour = (((float)pulse/(float)pulses_per_gallon)/5.00) * 3600.00;
    Serial.print("GPH: ");
    Serial.println(gallonsperhour);
    client.publish(gph_topic, String(gallonsperhour).c_str(), true);
    //add to total
    totalgallonpulses += pulse;
    Serial.print("Total Gallons: ");
    Serial.println((float)totalgallonpulses/(float)pulses_per_gallon);
    client.publish(gallonstotal_topic, String((float)totalgallonpulses/(float)pulses_per_gallon).c_str(), true);
    //now stuff with GPS
    if (gps.charsProcessed() > 9){
      client.publish(satellites_topic, String(gps.satellites.value()).c_str(), true);
      client.publish(mph_topic, String(gps.speed.mph()).c_str(), true);
      if (gallonsperhour != 0) {
        client.publish(mpg_topic, String((float)gps.speed.mph()/gallonsperhour).c_str(), true);
      }
    }
  }
}
