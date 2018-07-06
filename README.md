# home_sensor

This platform.io project is aiming at building a foundation for Home Automation Sensors based on the ESP32 chip.
While inspired by merlinschumachers' Basecamp, it deviated from it a while ago. I am still using his "Configuration" Library though.

Featurelist (implemented):
* MQTT support
* JSON messages
* Pull-based OTA approach
* Automatic SoftAP mode, providing REST-style API for configuration (no UI!)
* Automatical detection of attached sensors (I2C)
* Supports HomeAssistant, including Auto Discover Messages on MQTT.
* Support HW-triggered configuration reset by button or plain wire bridge.

Roadmap (implementation pending):
* The MQTT-Trigger for an OTA or Configuration changes is currently broken, due to the move towards HomeAssistant.
* PCB and Housing

The main focus right now is on the BME280 Sensor, but there are more on my wishlist:
* Flow Sensor to measure water pumped out of a cistern
* Water-Resistant Ultrasonic distance sensor (JSN-SR04T-2.0). Idea is to measure what's left in the cistern.
