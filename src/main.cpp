#include <wifi_config.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
//#include <Adafruit_NeoPixel.h>

//#include "colorconversion.h"


#include "FastLED.h"

#define SERIAL_VERBOSE
#define SENSOR_UPDATE_RATE 1000
#define MQTT_RECONNECT_RATE 5000

#define PIXEL_PIN    1    // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 61

#define HOSTNAME "HK-RGBLight-"



const char* mqtt_server = "beaglebone";
long lastReconnect = 0;
long now = 0;
long lastSensorUpdate = 0;

WiFiClient espClient;
PubSubClient client(espClient);

bool setupDone = false;

bool lampOn = false;
float hue = 0.0;
float saturation = 0.0;
float brightness = 0.0;

//Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
CRGB leds[PIXEL_COUNT];

// // Fill the dots one after the other with a color
// void colorWipe(uint32_t c, uint8_t wait) {
//   for(uint16_t i=0; i<strip.numPixels(); i++) {
//     strip.setPixelColor(i, c);
//     strip.show();
//     delay(wait);
//   }
// }

// void setAllPixels(uint32_t c)
// {
//   for(uint16_t i=0; i<strip.numPixels(); i++) {
//     strip.setPixelColor(i, c);
//     strip.show();
//   }

// }

void callback(char* topic, byte* payload, unsigned int length) ;

// Convert Hue/Saturation/Brightness values to a packed 32-bit RBG color.
// hue must be a float value between 0 and 360
// saturation must be a float value between 0 and 1
// brightness must be a float value between 0 and 1
CRGB HSVColor(float h, float s, float v) {

 h = constrain(h, 0, 360);
  s = constrain(s, 0, 1);
  v = constrain(v, 0, 1);

  int i, b, p, q, t;
  float f;

  h /= 60.0;  // sector 0 to 5
  i = floor( h );
  f = h - i;  // factorial part of h

  b = v * 255;
  p = v * ( 1 - s ) * 255;
  q = v * ( 1 - s * f ) * 255;
  t = v * ( 1 - s * ( 1 - f ) ) * 255;

  switch( i ) {
    case 0:
      return CRGB(b, t, p);
    case 1:
      return CRGB(q, b, p);
    case 2:
      return CRGB(p, b, t);
    case 3:
      return CRGB(p, q, b);
    case 4:
      return CRGB(t, p, b);
    default:
      return CRGB(b, p, q);
  }
}



bool reconnect() {

  if(WiFi.status() == WL_CONNECTED) {

    if(!setupDone) {
      ArduinoOTA.setHostname((const char *)WiFi.hostname().c_str());
      ArduinoOTA.begin();
      client.setServer(mqtt_server, 1883);
      client.setCallback(callback);

      setupDone = true;
    }

    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect((const char *)WiFi.hostname().c_str())) {
      Serial.println("connected");

      // ... and resubscribe
      // client.subscribe("fairylight");
      client.subscribe("light/#");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
    }

  }

}


void callback(char* topic, byte* payload, unsigned int length) {


  Serial.println(" ");
  Serial.println(" ");
  Serial.print("Received topic:\t");
  Serial.println(topic);
  Serial.print("Payload:\t");
  Serial.print((char *)payload);
  Serial.print("\tSize: ");
  Serial.println(length);
  Serial.println("--------");

  String value;
  for (int i = 0; i < length; i++) {
    value += (char)payload[i];
  }

  char *lightString = strtok(topic, "/");
  char *lampName = strtok(NULL, "/");
  char *key = strtok(NULL, "/");


  Serial.print("Parsed main topic:\t");
  Serial.println(lightString);
  Serial.print("Parsed lamp name:\t");
  Serial.println(lampName);
  Serial.print("Parsed key:\t");
  Serial.println(key);
  Serial.print("Parsed value:\t");
  Serial.println(value.c_str());


  if((strcmp(lightString, "light") != 0) ||
  (strcmp(lampName, "living") != 0))   {
    Serial.println("Wrong topic, skipping message");
    return;
  }

  if(strcmp(key, "on") == 0 ) {
    if(value.equals(String("true"))) {
      lampOn = true;
    }
    else {
      Serial.println(value.c_str());
      lampOn = false;
    }
  }
  else if(strcmp(key, "hue") == 0) {
      hue = value.toFloat();
  }
  else if(strcmp(key, "saturation") == 0) {
      saturation = value.toFloat();
  }
  else if(strcmp(key, "brightness") == 0) {
      brightness = value.toFloat();
  }


  if(lampOn) {
    CRGB color = HSVColor(hue, saturation/100.0, brightness/100.0);

    FastLED.showColor(color);

    Serial.print("Setting color to hue: ");
    Serial.print(hue);
    Serial.print(" sat: ");
    Serial.print(saturation/100.0);
    Serial.print(" brightness: ");
    Serial.println(brightness/100.0);
    Serial.print("R: ");
    Serial.print(color.red);
    Serial.print("\tG: ");
    Serial.print(color.green);
    Serial.print("\tB: ");
    Serial.println(color.blue);
  }
  else {
    FastLED.showColor(CRGB::Black);
    Serial.println("lamp off");
  }

}



void setup()
{

  Serial.begin(115200);
  //Internal LED
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  delay(100);

  Serial.println("Setup started");


  //strip.begin();
  //colorWipe(strip.Color(255,255,255), 10);
  FastLED.addLeds<NEOPIXEL, PIXEL_PIN>(leds, PIXEL_COUNT);


  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wpa_key);

  Serial.println("Setup finished");


}



void loop()
{
  now = millis();
  ArduinoOTA.handle();


  if (!client.connected()) {
    digitalWrite(BUILTIN_LED, LOW);
    if((now -lastReconnect) > MQTT_RECONNECT_RATE) {
      reconnect();
      lastReconnect = now;
    }
  }
  else {
    digitalWrite(BUILTIN_LED, HIGH);
    client.loop();

    // if(now - lastSensorUpdate > SENSOR_UPDATE_RATE) {
    //
    //   lastSensorUpdate = now;
    //
    //
    //
    // }

  }
  delay(10);



}
