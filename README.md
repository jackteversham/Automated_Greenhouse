# Automated_Greenhouse

In this project, different environmental sensors are monitored in real-time so as to maintain optimal conditions in a greenhouse.

All the recorded information should be easily accessible using Blynk. Blynk is a popular Internet of Things (IoT) platform that connects devices to the cloud, allows for easy app design and management of deployed products. This allows the user to access the conditions in the greenhouse using remote monitoring.

Interfacing with the Raspberry Pi required the use of multiple libraries. The WiringPi library was used to streamline the use of the Pi’s on-board I2C interface. The mcp3004 library was used to interface with the MCP3008 ADC chip using a serial SPI connection.

![alt text](<workflow.png>)
