# ESP32_MQTT_Light
Make your ESP32 work as RGBW light controller, remote controlled with MQTT protocol!

You'll need a few libraries, and one of them - ESPBASE, I've modified. While my changes are not in master of ESPBASE,you can apply patch with my changes manually.

How to use this firmware:
-Build it with Arduino IDE,
-Load it to your ESP32,
-When you'll power up your ESP32, it will create WiFi network. Connect there (try IP's 192.168.1.100 or 192.168.4.1), and enter your WiFi and MQTT parameters, and reboot board,
-After booting with configuration entered, it will try to connect to your WiFi and MQTT server you specified,
-After connecting to MQTT server, it will subscribe to topics PREFIX_r, PREFIX_g, PREFIX_b, PREFIX_w for red, green, blue and white channel brightness setpoints. If will wait for integers 0-255 for each channel. Once new setpoint is published at MQTT topic, it will set brightness at corresponding channel.

If you have board like ESP32-WEMOS, which have LED, that LED will be turned on when firmware begins to boot, and it will turn off when board is connected to WiFi. While firmware is booted and working, LED will turn on if MQTT connection broken, and there is reconnect attempt. Once connected, LED will turn off.

If you have any questions - please, contact me.
