#include <ota.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
//#include <Adafruit_NeoPixel.h>

//#include "colorconversion.h"

#include "FastLED.h"


#define SENSOR_UPDATE_RATE 1000
#define MQTT_RECONNECT_RATE 5000

#define PIXEL_PIN    D1    // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 61





const char* mqtt_server = "beaglebone";
long lastReconnect = 0;
long now = 0;
long lastSensorUpdate = 0;

WiFiClient espClient;
PubSubClient client(espClient);

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

    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect((const char *)WiFi.hostname().c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("hello", (const char *)WiFi.hostname().c_str());
      // ... and resubscribe
      client.subscribe("fairylight");
      //client.subscribe("elro");
      return true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      return false;
    }

  }

}


void callback(char* topic, byte* payload, unsigned int length) {

  String sTopic = topic;

  if(sTopic.equals("fairylight")) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject((char *)payload);
    if (!root.success())
    {
      Serial.println("parseObject() failed");
      return;
    }
    
    float hue = root["hue"];
    float sat = ((double)root["saturation"]);
    float bright = ((double)root["brightness"]);
    bool lightOn = root["powerOn"];


    if(lightOn) {
      CRGB color = HSVColor(hue, sat/100, bright/100);

      FastLED.showColor(color); 
    }
    else {
      FastLED.showColor(CRGB::Black); 
    }


    
    

  }



}



void setup()
{
  Serial.begin(115200);
  //Internal LED
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  delay(100);
  
  setupOTA();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //strip.begin();
  //colorWipe(strip.Color(255,255,255), 10);
  FastLED.addLeds<NEOPIXEL, PIXEL_PIN>(leds, PIXEL_COUNT);


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

    if(now - lastSensorUpdate > SENSOR_UPDATE_RATE) {

      lastSensorUpdate = now;



    }



  }
  delay(10);

  

}
