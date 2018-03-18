#define ARDUINO_ESP32_DEV
#include <PubSubClient.h>
#include <Arduino.h>

#include <ESPBASE.h>


WiFiServer TelnetServer(23);  // Optional in case you want to use telnet as Monitor
WiFiClient Telnet;            // Optional in case you want to use telnet as Monitor

ESPBASE Esp;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];


#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
//uint8_t g_phyFuns;
#ifdef __cplusplus
}
#endif

//int LED_BUILTIN = 2;
int LED_PIN = 5;
int LED_PIN_R = 32;
int LED_PIN_G = 33;
int LED_PIN_B = 12;
int LED_PIN_W = 13;
int freq = 10000;
int ledChannel = 0;
int ledChannel_r = 0;
int ledChannel_g = 1;
int ledChannel_b = 2;
int ledChannel_w = 3;
int resolution = 8;
int dutyCycle = 0;
int flag = 0;
uint8_t temp_farenheit;
float temp_celsius;

int reconnect_lastrun=0;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  delay(1000); // give me time to bring up serial monitor 
  
   Esp.initialize();
   if(Esp.WIFI_connected)
   {
      digitalWrite(LED_BUILTIN, HIGH);
   }
   // put your setup code here, to run once:

   //mqtt
   Serial.println("MQTT server: ");
   Serial.println(config.mqtt_server);


   TelnetServer.begin();            // Optional in case you want to use telnet as Monitor
   TelnetServer.setNoDelay(true);   // Optional in case you want to use telnet as Monitor

   ledcSetup(ledChannel_r, freq, resolution);
   ledcSetup(ledChannel_g, freq, resolution);
   ledcSetup(ledChannel_b, freq, resolution);
   ledcSetup(ledChannel_w, freq, resolution);
   ledcAttachPin(LED_PIN_R, ledChannel_r);
   ledcAttachPin(LED_PIN_G, ledChannel_g);
   ledcAttachPin(LED_PIN_B, ledChannel_b);
   ledcAttachPin(LED_PIN_W, ledChannel_w);

   client.setServer(config.mqtt_server.c_str(), atoi(config.mqtt_port.c_str()));
   client.setCallback(callback);

}

void reconnect() {
  String s;
  
  // Loop until we're reconnected
  //while (!client.connected()) {

  if(millis()-reconnect_lastrun < 5000)
  {
    return ;
  }
  digitalWrite(LED_BUILTIN, LOW);
  
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-"+config.mqtt_prefix+"-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    //if (client.connect(clientId.c_str())) {
    if (client.connect(clientId.c_str(),config.mqtt_username.c_str(),config.mqtt_password.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      s = "ESP32_"+config.mqtt_prefix+"_led_r";
      client.subscribe(s.c_str());
      s = "ESP32_"+config.mqtt_prefix+"_led_g";
      client.subscribe(s.c_str());
      s = "ESP32_"+config.mqtt_prefix+"_led_b";
      client.subscribe(s.c_str());
      s = "ESP32_"+config.mqtt_prefix+"_led_w";
      client.subscribe(s.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }

  //}
  digitalWrite(LED_BUILTIN, HIGH);
  reconnect_lastrun = millis();
}

void callback(char* topic, byte* payload, unsigned int length) {
  String str_topic, str_payload;
  int brightness=0;
  //string str_brightness;
  int i;
  char c[1];
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  str_payload = "\0\0\0\0\0\0\0\0";
  for (i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    //str_brightness[i]=(
    str_payload+=(char)payload[i];
  }
  Serial.println();
  Serial.print("Length: [");
  Serial.print(length);
  Serial.println("] ");
  str_topic = "\0\0\0\0\0\0\0\0";
  str_topic = topic;
  //str_payload = "\0\0\0\0\0\0\0\0";
  //str_payload = (char*)payload;
  Serial.println("str_payload: ");
    Serial.println(str_payload);
   
  
  //c='\0';
  //if ((strcmp (topic,"ESP32_led_r") == 0) || (strcmp (topic,"ESP32_led_g") == 0) || (strcmp (topic,"ESP32_led_b") == 0)|| (strcmp (topic,"ESP32_led_w") == 0))
  if((str_topic.endsWith(config.mqtt_prefix+"_led_r"))||(str_topic.endsWith(config.mqtt_prefix+"_led_g"))||(str_topic.endsWith((config.mqtt_prefix+"_led_b")))||(str_topic.endsWith(config.mqtt_prefix+"_led_w")))
  {
    
    /*for(i=0; i<length; i++)
    {
      //c="\0";
      c[0]=(char)payload[i];
       Serial.println(c[0]);
       Serial.println(atoi(c));
       Serial.println(i);
      brightness=brightness+atoi(c)*pow(10,(length-(i+1)));
    }*/
    brightness = atoi(str_payload.c_str());
    Serial.println("Brightness: ");
    Serial.println(brightness);

    //if(strcmp (topic,"ESP32_led_r") == 0)
    if(str_topic.endsWith("led_r"))
    {
      Serial.println("Channel: R");
      ledcWrite(ledChannel_r, brightness);
    }
    //if(strcmp (topic,"ESP32_led_g") == 0)
    if(str_topic.endsWith("led_g"))
    {
      Serial.println("Channel: G");
      ledcWrite(ledChannel_g, brightness);
    }
    //if(strcmp (topic,"ESP32_led_b") == 0)
    if(str_topic.endsWith("led_b"))
    {
      Serial.println("Channel: B");
      ledcWrite(ledChannel_b, brightness);
    }
    //if(strcmp (topic,"ESP32_led_w") == 0)
    if(str_topic.endsWith("led_w"))
    {
      Serial.println("Channel: W");
      ledcWrite(ledChannel_w, brightness);
    }
  }

  for (i = 0; i < length; i++) {
    payload[i]=0;
    //str_brightness[i]=(
  }
}



void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  temp_farenheit = temprature_sens_read();
  temp_celsius = ( temp_farenheit - 32 ) / 1.8;
  //ledcWrite(ledChannel, dutyCycle);
  //delay(500);
  
  
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(3000);                       // wait for a second
  //digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  //delay(1000);                       // wait for a second
  // put your main code here, to run repeatedly:
  //Serial.println(millis());

  // OTA request handling
  //ArduinoOTA.handle();

  //  WebServer requests handling
  server.handleClient();

   //  feed de DOG :)
  customWatchdog = millis();

 // activate telnet service to act as output console

  if (TelnetServer.hasClient()){            // Optional in case you want to use telnet as Monitor
    if (!Telnet || !Telnet.connected()){
      if(Telnet) Telnet.stop();
      Telnet = TelnetServer.available();
    } else {
      TelnetServer.available().stop();
    }
  }


    //**** Normal Skecth code here ...


}
