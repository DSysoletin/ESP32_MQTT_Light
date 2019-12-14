#define ARDUINO_ESP32_DEV
#include <PubSubClient.h>
#include <Arduino.h>

#include <ESPBASE.h>
#include <SHT1x.h>

#define min(a,b) ((a)<(b)?(a):(b))


#define dataPin 23
#define clockPin 19
SHT1x sht1x(dataPin, clockPin);

#define CO2_sensor_pin 34
#define PWM_MAX 255
#define PWM_MIN 0


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
/*int LED_PIN = 22;
uint8_t LED_PIN_R = 5;
uint8_t LED_PIN_G = 17;
uint8_t LED_PIN_B = 16;
uint8_t LED_PIN_W = 18;*/
int LED_PIN = 5;
int LED_PIN_R = 32;
int LED_PIN_G = 33;
int LED_PIN_B = 12;
int LED_PIN_W = 13;

int freq = 1000;
//int ledChannel = 0;
int ledChannel_r = 1;
int ledChannel_g = 2;
int ledChannel_b = 3;
int ledChannel_w = 4;
int resolution = 8;
//int dutyCycle = 0;
int flag = 0;
uint8_t temp_farenheit;
float temp_celsius;

//Actual and desired LED brightness values
static int led_r_sp=0, led_r_pv=0, led_g_sp=0, led_g_pv=0, led_b_sp=0, led_b_pv=0, led_w_sp=0, led_w_pv=0;

//sht10 data
float sht10_temp=0;
float sht10_hum=0;

int reconnect_lastrun=0;
bool ota_active=false;

TaskHandle_t TaskA;

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


   //TelnetServer.begin();            // Optional in case you want to use telnet as Monitor
   //TelnetServer.setNoDelay(true);   // Optional in case you want to use telnet as Monitor
   ledcAttachPin(LED_PIN_R, ledChannel_r);
   ledcAttachPin(LED_PIN_G, ledChannel_g);
   ledcAttachPin(LED_PIN_B, ledChannel_b);
   ledcAttachPin(LED_PIN_W, ledChannel_w);

   ledcSetup(ledChannel_r, freq, resolution);
   ledcSetup(ledChannel_g, freq, resolution);
   ledcSetup(ledChannel_b, freq, resolution);
   ledcSetup(ledChannel_w, freq, resolution);


   client.setServer(config.mqtt_server.c_str(), atoi(config.mqtt_port.c_str()));
   client.setCallback(callback);

   led_w_pv=30;

   
   //OTA stuff
   ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
      ota_active = true;
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
   ArduinoOTA.begin();

   //SHT10 loop on core 0
   xTaskCreatePinnedToCore(
   sht10_loop,                  /* pvTaskCode */
   "Workload1",            /* pcName */
   1000,                   /* usStackDepth */
   NULL,                   /* pvParameters */
   1,                      /* uxPriority */
   &TaskA,                 /* pxCreatedTask */
   0);                     /* xCoreID */
 
}

void reconnect() {
  String s;
  static int cont_fails=0;
  
  // Loop until we're reconnected
  //while (!client.connected()) {
  customWatchdog = millis();
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
      cont_fails=0;
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      s = "ESP32_"+config.mqtt_prefix+"_led_r";
      Serial.println(s);
      client.subscribe(s.c_str());
      s = "ESP32_"+config.mqtt_prefix+"_led_g";
      Serial.println(s);
      client.subscribe(s.c_str());
      s = "ESP32_"+config.mqtt_prefix+"_led_b";
      Serial.println(s);
      client.subscribe(s.c_str());
      s = "ESP32_"+config.mqtt_prefix+"_led_w";
      Serial.println(s);
      client.subscribe(s.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      cont_fails++;
      if(cont_fails>120)
      {
        ESP.restart();
      }
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
    
    brightness = atoi(str_payload.c_str());
    Serial.println("Brightness: ");
    Serial.println(brightness);

    if(str_topic.endsWith("led_r"))
    {
      Serial.println("Channel: R");
      led_r_sp=brightness;
    }

    if(str_topic.endsWith("led_g"))
    {
      Serial.println("Channel: G");
      led_g_sp=brightness;
    }

    if(str_topic.endsWith("led_b"))
    {
      Serial.println("Channel: B");
      led_b_sp=brightness;
    }
    
    if(str_topic.endsWith("led_w"))
    {
      Serial.println("Channel: W");
      led_w_sp=brightness;
    }
  }

  for (i = 0; i < length; i++) {
    payload[i]=0;
  }
}

void sht10_loop(void * parameter)
{
  while(1)
  {
    sht10_temp = sht1x.readTemperatureC();
    sht10_hum = sht1x.readHumidity();
    delay(5000);
  }
}

void process_sht10()
{
  float tempC=0; 
  float humidity=0;
  int ret=0;
  String s;
  String payload;
  
  char payload_chars[32];
  int payload_len=0;

    tempC=sht10_temp;
    humidity=sht10_hum;
    s = config.mqtt_prefix+"_tempSHT10";
    Serial.println("Temp SHT10: ");
    Serial.println(s);
    Serial.println(tempC);
    Serial.println("");
    payload=String(tempC);
    payload.toCharArray(payload_chars,10);
    payload_len=payload.length();

    client.publish(s.c_str(),payload.c_str(),payload_len);
    //ret=client.publish("wtr_sensor1_tempSHT10",payload.c_str());
    //client.publish(s.c_str(),payload_chars,payload_len);
    //client.publish("wtr_sensor1_tempSHT10",payload_chars,payload_len);
    Serial.print("Ret: ");
    Serial.println(ret);
    //client.loop();

    //customWatchdog = millis();
    
    s = config.mqtt_prefix+"_humiditySHT10";
    Serial.println("Humidity: ");
    Serial.println(s);
    Serial.println(humidity);
    Serial.println("");
    payload=String(humidity);
    payload.toCharArray(payload_chars,10);
    payload_len=payload.length();
    
    ret=client.publish(s.c_str(),payload_chars,payload_len);
    Serial.print("Ret: ");
    Serial.println(ret);
   
}

void process_co2_sensor()
{
  float co2_value=0;
  String s;
  String payload;
  int ret=0;
  
  char payload_chars[10];
  int payload_len=0;

  co2_value=analogRead(CO2_sensor_pin);

  s = config.mqtt_prefix+"_co2_value";
    Serial.println("co2_value: ");
    Serial.println(s);
    Serial.println(co2_value);
    Serial.println("");
    payload=String(co2_value);
    payload.toCharArray(payload_chars,10);
    payload_len=payload.length();
    
    ret=client.publish(s.c_str(),payload_chars,payload_len);
    Serial.print("Ret: ");
    Serial.println(ret);
}

void publish_brightness_data(String channel, int brightness)
{
  String s;
  String payload;
  char payload_chars[10];
  int payload_len=0;
 
  s = "ESP32_"+config.mqtt_prefix+channel;
  payload=String(brightness);
  payload.toCharArray(payload_chars,10);
  payload_len=payload.length();
    Serial.println("topic: ");
    Serial.println(s);
    Serial.println(brightness);
    Serial.println("");
    
  client.publish(s.c_str(),payload_chars,payload_len);
}

void loop() {
  static int last_millis,last_millis_timer2;
  //if(!ota_active)
  //{
    if(Esp.WIFI_connected)
    {
       if (!client.connected()) {
          reconnect();
        }
        client.loop();
    }
    
    //Every 60 sec.
    if(millis()-last_millis>60000)
    {
      temp_farenheit = temprature_sens_read();
      temp_celsius = ( temp_farenheit - 32 ) / 1.8;
      Serial.println(temp_celsius);
      process_sht10();
      process_co2_sensor();
      last_millis=millis();

      //Publish data about actual LED brightness
      publish_brightness_data("_led_r_pv",led_r_pv);
      publish_brightness_data("_led_g_pv",led_g_pv);
      publish_brightness_data("_led_b_pv",led_b_pv);
      publish_brightness_data("_led_w_pv",led_w_pv);
    }

    //every 0.25 sec
    if(millis()-last_millis_timer2>250)
    {
      last_millis_timer2=millis();
      //Slowly equalize SPs and PVs
      if(led_r_sp!=led_r_pv)
      {
        if((led_r_sp>led_r_pv)&&(led_r_pv<PWM_MAX))
        {
          led_r_pv++;
          ledcWrite(ledChannel_r, led_r_pv);
        }
        if((led_r_sp<led_r_pv)&&(led_r_pv>PWM_MIN))
        {
          led_r_pv--;
          ledcWrite(ledChannel_r, led_r_pv);
        }
      }

      if(led_g_sp!=led_g_pv)
      {
        if((led_g_sp>led_g_pv)&&(led_g_pv<PWM_MAX))
        {
          led_g_pv++;
          ledcWrite(ledChannel_g, led_g_pv);
        }
        if((led_g_sp<led_g_pv)&&(led_g_pv>PWM_MIN))
        {
          led_g_pv--;
          ledcWrite(ledChannel_g, led_g_pv);
        }
      }

      if(led_b_sp!=led_b_pv)
      {
        if((led_b_sp>led_b_pv)&&(led_b_pv<PWM_MAX))
        {
          led_b_pv++;
          ledcWrite(ledChannel_b, led_b_pv);
        }
        if((led_b_sp<led_b_pv)&&(led_b_pv>PWM_MIN))
        {
          led_b_pv--;
          ledcWrite(ledChannel_b, led_b_pv);
        }
      }
      if(led_w_sp!=led_w_pv)
      {
        if((led_w_sp>led_w_pv)&&(led_w_pv<PWM_MAX))
        {
          led_w_pv++;
          ledcWrite(ledChannel_w, led_w_pv);
        }
        if((led_w_sp<led_w_pv)&&(led_w_pv>PWM_MIN))
        {
          led_w_pv--;
          ledcWrite(ledChannel_w, led_w_pv);
        }
      }
    }
    
    
  
    //  WebServer requests handling
    server.handleClient();
  //}
  // OTA request handling
  ArduinoOTA.handle();

   //  feed de DOG :)
  //customWatchdog = millis();

 
}
