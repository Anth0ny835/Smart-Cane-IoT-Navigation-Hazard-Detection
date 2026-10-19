# Prototype hardware and setup

The module selection and pin map below are transcribed from the group's university report. **Hardware wiring, pin assignment, and the setup diagram were Sam's contributions.** Verify the physical connections, power protection and the code before powering the board; this document does not certify the circuit.

![System setup diagram](images/hardware-setup.jpg)

## Major components

- Raspberry Pi Pico W microcontroller
- HC-SR04 ultrasonic distance sensor
- VL53L0X ToF distance sensor
- Water/moisture sensor module
- MPU6050 accelerometer/gyroscope (GY-521)
- QMC5883L compass (GY-273)
- NEO-6M GPS receiver
- DFPlayer Mini with microSD and mini speaker
- Passive buzzer, LED with resistor, vibration motor and MOSFET driver
- Five push buttons
- 18650 cell, battery holder, TP4056 charging module, 5 V boost converter and switch
- Breadboards, jumper wires and resistors

## Raspberry Pi Pico W pin assignments

| Function | Pico pin(s) |
| --- | --- |
| Ultrasonic TRIG / ECHO | GP12 / GP13 (ECHO level shifting required) |
| Water digital output | GP15 |
| Buzzer | GP10 |
| Vibration driver trigger | GP8 |
| LED | GP9 |
| Buttons 1–5 | GP2, GP3, GP6, GP7, GP11 |
| DFPlayer UART | GP0 TX, GP1 RX |
| GPS UART | GP4 TX, GP5 RX |
| Shared I2C SDA / SCL (ToF, MPU6050, compass) | GP20 / GP21 |

## Power and testing

The report specifies a 3.7 V 18650 cell, TP4056 charger and a DC-DC boost converter for the 5 V rail. The Pico 3.3 V output supplies 3.3 V sensor modules. **Confirm voltage and ground compatibility**, especially the 5 V ultrasonic ECHO signal and the battery charging/protection arrangement, rather than assuming the diagram is safe to reproduce unmodified.

For prototype tests, check each individual module, confirm audio/vibration outputs, perform baseline calibration, test navigation with valid GPS reception and evaluate alerts in a controlled environment. The project remains a research/educational prototype.

## Photo gallery

See the [repository README](../README.md#prototype-gallery) for photos of the actual prototype.
