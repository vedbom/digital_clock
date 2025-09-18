# Digital Clock

## Objective
The objective of this project is to build a digital clock using an Arduino microcontroller to learn more about the Arduino's peripherals and some common sensors utilized in embedded applications.
The features that will be built into the clock are as follows:

- Real Time Clock (RTC) to save the date and time in case of power disruption to the Arduino
- 7-segment display with 4 digits to show the hour and minutes
- Photoresistor to turn off the display when the lights in the room are turned off
- Pushbuttons and potentiometer to set the time and alarms
- Ability to set 2 seperate alarms 
- Temperature sensors to measure and display the temperature in degrees
- Buzzer to generate an audible sound when the alarm is triggered
- Ultrasonic distance sensor to turn off the alarm only when an object is within a certain distance (10 cm), forcing the user to get out of bed

The embedded systems peripherals and concepts that will be used in this project are GPIO's (7-segment display and pushbuttons), ADC's (potentiometer and photoresistor), PWM (buzzer), 
external interrupts (pushbuttons), I2C (RTC), and the 1-Wire interface (temperature sensor). 

Here is a demo of the breadboard prototype working: [demo link](https://drive.google.com/file/d/1FlTgrgJDMJIxUTOzj_wMEAkOvxhbOC5B/view?usp=drive_link) 

## Components
- Arduino Uno
- 4 x 7-segment display
- 4 x 2N3904 NPN BJT transistor
- 1 x SN74HC595N shift register
- 2 x 3mm LED
- 1 x DS3231 real time clock
- 1 x HC-SR04 ultrasonic distance sensor
- 1 x photoresistor
- 1 x potentiometer
- 2 x DS18B20 temperature sensor
- 2 x pushbuttons
- 1 x buzzer

## Construction
![Schematic of Arduino](images/Digital%20Clock%20Arduino%20Schematic.PNG)
![Schematic of 7 segment display](images/Digital%20Clock%207%20Segment%20Display%20Schematic.PNG)
