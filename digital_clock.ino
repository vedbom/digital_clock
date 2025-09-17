#include <OneWire.h>
#include <RTClib.h>
#include <Wire.h>

#define MAX_SAMPLES 10

const int dig0_pin = 10;
const int dig1_pin = 11;
const int dig2_pin = 12;
const int dig3_pin = 13;

// the byte in the shift register corresponds to (DP, G, F, E, D, C, B, A) from MSB to LSB
const int latch_pin = 8;
const int clock_pin = 9;
const int data_pin = 7;

const int colon_pin = 6;

// Analog input pins can be used as digital output or digital input
const int analog_button_pin = 17;

// trigger input and echo output of the HC-SR04 ultrasonic distance sensor
const int hcsr04_trig_pin = 15;
const int hcsr04_echo_pin = 14;

// light dependent resistor pin
const int ldr_pin = 16;

// push buttons are connected to the following external interrupt pins
const int int0_pin = 2;                        // connected to button0
const int int1_pin = 3;                        // connected to button1

// DS18B20 temperature sensors are connected to this pin
const int temp_pin = 4;

// piezo pin
const int piezo_pin = 5;

int debounce_delay = 50;                     // buttons must hold their new state for this amount of time (in milliseconds) before the state transition is finalized
volatile bool button0_flag = false;
volatile unsigned int button0_timer = 0;
int button0_state = HIGH;

volatile bool button1_flag = false;
volatile unsigned int button1_timer = 0;
int button1_state = HIGH;

// variables for displaying the clock
int len = 4;
// 0 = time, 1 = date, 2 = temp
int disp_mode = 0;
int digits[4] = {4, 2, 0, 2};
int digitToByte[17] = {63, 6, 91, 79, 102, 109, 125, 7, 127, 111, 119, 127, 57, 63, 121, 113, 64};
int mltplx_period = 500;                       // the multiplexing frequency of the 7-segment display (ie. the amount of time each digit is turned on in microseconds)
unsigned int mltplx_prev_time = 0;                            
int curr_digit = 0;
int en_decimal = -1;
unsigned int read_RTC_time = 0;
const int read_RTC_period = 500;
bool on_period = true;
unsigned int fast_blink_time = 0;
const int fast_blink_period = 250;
unsigned int ping_time = 0;
const int ping_period = 100;
unsigned int dist_sense_time = 0;
const int dist_sense_period = 250;
bool colon_blink_on = false;
bool colon_state = LOW;

bool alarm_disp_blink_on = false;
bool alarm_disp_state = LOW;
int alarm1_hour = 0;
int alarm1_min = 0;
int alarm2_hour = 0;
int alarm2_min = 0;
int time_hour = 0;
int time_min = 0;
unsigned int slow_blink_time = 0;
const int slow_blink_period = 500;

bool alarm_on = false;
unsigned int alarm_time = 0;
const int alarm_period = 30000;

bool turn_off_disp = false;

byte temp_addr[8];

RTC_DS3231 rtc;
OneWire OneWireBus(temp_pin);

void ISR_int0() {
  if (!button0_flag) {
    button0_flag = true;
    button0_timer = int(millis());
  }
}

void ISR_int1() {
  if (!button1_flag) {
    button1_flag = true;
    button1_timer = int(millis());
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(dig0_pin, OUTPUT);
  pinMode(dig1_pin, OUTPUT);
  pinMode(dig2_pin, OUTPUT);
  pinMode(dig3_pin, OUTPUT);
  pinMode(latch_pin, OUTPUT);
  pinMode(clock_pin, OUTPUT);
  pinMode(data_pin, OUTPUT);
  pinMode(colon_pin, OUTPUT);
  pinMode(int0_pin, INPUT);
  pinMode(int1_pin, INPUT);
  pinMode(hcsr04_trig_pin, OUTPUT);
  pinMode(hcsr04_echo_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(int0_pin), ISR_int0, FALLING);
  attachInterrupt(digitalPinToInterrupt(int1_pin), ISR_int1, FALLING);

  mltplx_prev_time = micros();
  read_RTC_time = millis();
  fast_blink_time = millis();
  slow_blink_time = millis();
  alarm_time = millis();
  dist_sense_time = millis();
  
  Serial.begin(9600);

  // read and store the addresses of the DS18B20 temperature sensors
  OneWireBus.reset();
  OneWireBus.search(temp_addr);
  Serial.println("Found temperature sensor with serial code: ");
  for(int i = 0; i < 8; ++i) {
    Serial.print(temp_addr[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  if (!rtc.begin()) {
    Serial.println("Cound'nt detect RTC");
    Serial.flush();
    while(1) delay(10);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {
  // put your main code here, to run repeatedly:
  DateTime now = rtc.now();

  // button0 uses an external interrupt so the button0_flag is set in the interrupt service routine
  if (button0_flag) {
    if (int(millis()) - button0_timer > debounce_delay && digitalRead(int0_pin) == LOW) {
      button0_state = LOW;
      button0_flag = false;
      switch(disp_mode) {
        case 0:         // time
        {
          disp_mode = 1;
          break;
        };
        case 1:         // date  
        {
          disp_mode = 2;
          break;
        };
        case 2:         // temp
        {
          disp_mode = 3;
          break;
        };
        case 3:         // display alarm 1
        {
          disp_mode = 4;
          break;
        };
        case 4:         // display alarm 2
        {
          disp_mode = 0;
          //disp_mode = 11;
          break;
        };
        case 5:         // set time hour
        {
          disp_mode = 0;
          rtc.adjust(DateTime(now.year(), now.month(), now.day(), time_hour, time_min, 0));
          break;
        };
        case 6:         // set time minutes
        {
          disp_mode = 0;
          rtc.adjust(DateTime(now.year(), now.month(), now.day(), time_hour, time_min, 0));
          break;
        };
        case 7:         // set alarm 1 hour
        {
          disp_mode = 3;
          break;
        };
        case 8:         // set alarm 1 minutes
        {
          disp_mode = 3;
          break;
        };
        case 9:         // set alarm 2 hour
        {
          disp_mode = 4;
          break;
        };
        case 10:        // set alarm 2 minutes
        {
          disp_mode = 4;
          break;
        };
        case 11:        // trigger alarm
        {
          disp_mode = 0;
          alarm_time = int(millis());
          alarm_on = false;
          turn_off_disp = false;
          noTone(piezo_pin);
          break;
        }
        default:
        {
          break;
        };
      }
      Serial.println("button0 is pressed!");
    }
  }

  // button1 uses an external interrupt so the button1_flag is set in the interrupt service routine
  if (button1_flag) {
    if (int(millis()) - button1_timer > debounce_delay && digitalRead(int1_pin) == LOW) {
      button1_state = LOW;
      button1_flag = false;
      switch(disp_mode) {
        case 0:         // time
        {
          disp_mode = 5;
          break;
        };
        case 1:         // date  
        {
          break;
        };
        case 2:         // temp
        {
          break;
        };
        case 3:         // display alarm 1
        {
          disp_mode = 7;
          break;
        };
        case 4:         // display alarm 2
        {
          disp_mode = 9;
          break;
        };
        case 5:         // set time hour
        {
          disp_mode = 6;
          break;
        };
        case 6:         // set time minutes
        {
          disp_mode = 5;
          break;
        };
        case 7:         // set alarm 1 hour
        {
          disp_mode = 8;
          break;
        };
        case 8:         // set alarm 1 minutes
        {
          disp_mode = 7;
          break;
        };
        case 9:         // set alarm 2 hour
        {
          disp_mode = 10;
          break;
        };
        case 10:        // set alarm 2 minutes
        {
          disp_mode = 9;
          break;
        };
        case 11:        // trigger alarm
        {
          disp_mode = 0;
          alarm_time = int(millis());
          alarm_on = false;
          turn_off_disp = false;
          noTone(piezo_pin);
          break;
        }
        default:
        {
          break;
        };
      }
      Serial.println("button1 is pressed!");
    }
  }

  //Serial.print("LDR read: ");
  //Serial.println(analogRead(ldr_pin));
  if ((analogRead(ldr_pin) < 300) && !alarm_on) {
    turn_off_disp = true;
    colon_state = LOW;
  }
  else {
    turn_off_disp = false;
    colon_state = HIGH;
  }

  // if the colon_blink_on is HIGH, blink the colon
  if (colon_blink_on) {
    if (int(millis()) - fast_blink_time > fast_blink_period) {
      fast_blink_time = int(millis());
      colon_state = !colon_state;
    }
  }
  digitalWrite(colon_pin, colon_state);

  if (alarm_on) {
    tone(piezo_pin, 5000);
    if (int(millis()) - dist_sense_time > dist_sense_period) {
      dist_sense_time = int(millis());
      // send a 10us pulse to the trigger pin of the HCSR04 sensor
      digitalWrite(hcsr04_trig_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(hcsr04_trig_pin, HIGH);
      delayMicroseconds(5);
      digitalWrite(hcsr04_trig_pin, LOW);

      // read the pulse from the echo pin
      // it represents the time required for the sound signal to be transmitted, bounce off the obstable and return to the receiver
      unsigned long dist_meas = pulseIn(hcsr04_echo_pin, HIGH);
      float dist_meas_cm = dist_meas / 58.8;
      // assuming the speed of sound is 340 m/s, the distance between the sensor and the obstacle is:
      // distance = (pulse width in seconds) * (340 m/s) / 2 (distance is in meters)
      // distance = (pulse width in micro seconds) / 58.8 (distance is in cm)
      Serial.print("dist: ");
      Serial.print(dist_meas_cm);
      Serial.println();

      if (dist_meas_cm < 10.0) {
        disp_mode = 0;
        alarm_time = int(millis());
        alarm_on = false;
        turn_off_disp = false;
        noTone(piezo_pin);
      }
    }
    if (int(millis()) - alarm_time > alarm_period) {
      alarm_time = int(millis());
      disp_mode = 0;
      alarm_on = false;
      turn_off_disp = false;
      noTone(piezo_pin);
    }
  }

  // switch back and forth from the alarm time and the alarm code
  if (alarm_disp_blink_on || alarm_on) {
    if (int(millis()) - slow_blink_time > slow_blink_period) {
      slow_blink_time = int(millis());
      if (alarm_disp_blink_on) {
        alarm_disp_state = !alarm_disp_state;
        if(alarm_disp_state) {
          if (disp_mode == 3) {
            digits[0] = 1;
            digits[1] = 10;
            digits[2] = 0;
            digits[3] = 0;
          }
          else if (disp_mode == 4) {
            digits[0] = 2;
            digits[1] = 10;
            digits[2] = 0;
            digits[3] = 0;
          }
          else {

          }
        }
        else {
          if (disp_mode == 3) {
            get_digits(100*alarm1_hour + alarm1_min, digits, len);
          }
          else if (disp_mode == 4) {
            get_digits(100*alarm2_hour + alarm2_min, digits, len);
          }
          else {
            
          }
        }
      }
    }
  }

  if (int(millis()) - read_RTC_time > read_RTC_period) {
    // Serial.println(analogRead(ldr_pin));
    read_RTC_time = int(millis());
    int concat_num = 0;
    int pot_read = 0;
    en_decimal = -1;
    alarm_disp_blink_on = false;
    switch(disp_mode) {
      // time
      case 0:
      // must add brackets within the cases of a switch statement for it to work properly!!!
      {
        Serial.println("time");
        colon_state = HIGH;
        colon_blink_on = false;
        concat_num = 100*now.hour() + now.minute();
        get_digits(concat_num, digits, len);
        bool alarm1_is_on = (alarm1_hour == now.hour()) && (alarm1_min == now.minute()) && (now.second() > 0 && now.second() < 3);
        bool alarm2_is_on = (alarm2_hour == now.hour()) && (alarm2_min == now.minute()) && (now.second() > 0 && now.second() < 3);
        Serial.print(now.hour());
        Serial.print(":");
        Serial.print(now.minute());
        Serial.print(":");
        Serial.print(now.second());
        Serial.print(" ");
        Serial.println(!alarm_on && (alarm1_is_on || alarm2_is_on));
        if (!alarm_on && (alarm1_is_on || alarm2_is_on)) {
          disp_mode = 11;
          alarm_time = int(millis());
        }
        break;
      };
      // date
      case 1:
      {
        Serial.println("date");
        colon_state = LOW;
        colon_blink_on = false;
        concat_num = 100*now.month() + now.day();
        get_digits(concat_num, digits, len);
        break;
      };
      // temp
      case 2:
      {
        Serial.println("temp");
        colon_state = LOW;
        colon_blink_on = false;
        float temp = read_temp(OneWireBus, temp_addr);
        // check if the temperature is within the displayable range
        if (temp >= -9.9 && temp <= 99.9) {
          //Serial.println(temp);
          if (temp < 0.0) {
            // there are only 2 significant digits
            concat_num = int(10*abs(temp));
            get_digits(10*concat_num, digits, len);
            en_decimal = 2;
            digits[3] = 16;
          }
          else {
            // there are 3 significant digits
            concat_num = int(100*abs(temp));
            if (concat_num < 1000) {
              get_digits(10*concat_num, digits, len);
              en_decimal = 3;
            }
            else {
              get_digits(concat_num, digits, len);
              en_decimal = 2;
            }
          }
          digits[0] = 12;
        }
        else {
          // otherwise, show an error message
          digits[0] = 15;
          digits[1] = 15;
          digits[2] = 15;
          digits[3] = 15;
        }
        break;
      };
      case 3:         // display alarm 1
      {
        Serial.println("display alarm 1");
        colon_state = HIGH;
        colon_blink_on = false;
        alarm_disp_blink_on = true;
        break;
      };
      case 4:         // display alarm 2
      {
        Serial.println("display alarm 2");
        colon_state = HIGH;
        colon_blink_on = false;
        alarm_disp_blink_on = true;
        break;
      };
      case 5:         // set time hour
      {
        Serial.println("set time hour");
        colon_state = HIGH;
        colon_blink_on = true;
        time_hour = map(analogRead(analog_button_pin), 0, 1023, 0, 24);
        concat_num = 100*time_hour + time_min;
        get_digits(concat_num, digits, len);
        break;
      };
      case 6:         // set time minutes
      {
        Serial.println("set time minutes");
        colon_state = HIGH;
        colon_blink_on = true;
        time_min = map(analogRead(analog_button_pin), 0, 1023, 0, 60);
        concat_num = 100*time_hour + time_min;
        get_digits(concat_num, digits, len);
        break;
      };
      case 7:         // set alarm 1 hour
      {
        Serial.println("set alarm 1 hour");
        colon_state = HIGH;
        colon_blink_on = true;
        alarm1_hour = map(analogRead(analog_button_pin), 0, 1023, 0, 24);
        concat_num = 100*alarm1_hour + alarm1_min;
        get_digits(concat_num, digits, len);
        break;
      };
      case 8:         // set alarm 1 minutes
      {
        Serial.println("set alarm 1 minutes");
        colon_state = HIGH;
        colon_blink_on = true;
        alarm1_min = map(analogRead(analog_button_pin), 0, 1023, 0, 60);
        concat_num = 100*alarm1_hour + alarm1_min;
        get_digits(concat_num, digits, len);
        break;
      };
      case 9:         // set alarm 2 hour
      {
        Serial.println("set alarm 2 hour");
        colon_state = HIGH;
        colon_blink_on = true;
        alarm2_hour = map(analogRead(analog_button_pin), 0, 1023, 0, 24);
        concat_num = 100*alarm2_hour + alarm2_min;
        get_digits(concat_num, digits, len);
        break;
      };
      case 10:        // set alarm 2 minutes
      {
        Serial.println("set alarm 2 minutes");
        colon_state = HIGH;
        colon_blink_on = true;
        alarm2_min = map(analogRead(analog_button_pin), 0, 1023, 0, 60);
        concat_num = 100*alarm2_hour + alarm2_min;
        get_digits(concat_num, digits, len);
        break;
      };
      case 11:        // trigger alarm
      {
        Serial.println("trigger alarm");
        colon_state = HIGH;
        alarm_on = true;
        concat_num = 100*now.hour() + now.minute();
        get_digits(concat_num, digits, len);
        break;
      };
      default:
      {
        concat_num = 0000;
        break;  
      };
    }
  }

  if (int(micros()) - mltplx_prev_time > mltplx_period) {
    mltplx_prev_time = int(micros());
    if (on_period) {
      (curr_digit >= 3) ? curr_digit = 0 : ++curr_digit;
      if (turn_off_disp) {
        disp_num(0, curr_digit);
      }
      else {
        disp_num(digitToByte[digits[curr_digit]] + (en_decimal == curr_digit ? 128 : 0), curr_digit);
      }
    }
    else {
      disp_num(0, curr_digit);
    }
    on_period = !on_period;
  }
}

// function converts a 4 digit integer into an array of integers containing the individual digits
void get_digits(int num, int *arr, int len) {
  int digit_index = 0;
  int place_val = 1;
  do {
    *(arr + digit_index) = (num / place_val) % 10;
    ++digit_index;
    place_val *= 10;
  }
  while (digit_index < len);
}

void disp_num(int num, int digit) {
  switch (digit) {
    case 0:
      digitalWrite(dig0_pin, HIGH);
      digitalWrite(dig1_pin, LOW);
      digitalWrite(dig2_pin, LOW);
      digitalWrite(dig3_pin, LOW);
      break;
    case 1:
      digitalWrite(dig0_pin, LOW);
      digitalWrite(dig1_pin, HIGH);
      digitalWrite(dig2_pin, LOW);
      digitalWrite(dig3_pin, LOW);
      break;
    case 2:
      digitalWrite(dig0_pin, LOW);
      digitalWrite(dig1_pin, LOW);
      digitalWrite(dig2_pin, HIGH);
      digitalWrite(dig3_pin, LOW);
      break;
    case 3:
      digitalWrite(dig0_pin, LOW);
      digitalWrite(dig1_pin, LOW);
      digitalWrite(dig2_pin, LOW);
      digitalWrite(dig3_pin, HIGH);
      break;
    default:
      digitalWrite(dig0_pin, LOW);
      digitalWrite(dig1_pin, LOW);
      digitalWrite(dig2_pin, LOW);
      digitalWrite(dig3_pin, LOW);
      break;
  }
  digitalWrite(latch_pin, LOW);
  shiftOut(data_pin, clock_pin, MSBFIRST, num);
  digitalWrite(latch_pin, HIGH);
}

float read_temp(OneWire &sensor, byte addr[8]) {
  sensor.reset();
  sensor.select(addr);                  // address the device on the bus with the given serial code
  sensor.write(0x4E);                   // give a command to the slave to write the following 3 bytes into its scratchpad memory
  sensor.write(0x00);                   // TH register
  sensor.write(0x00);                   // TL register
  sensor.write(B01000000);              // configuration register, set the temperature resolution to 11-bit

  sensor.reset();
  sensor.select(addr);
  sensor.write(0x44);                   // give a command to the slave to take a temperature measurement and store it in the register

  sensor.reset();
  sensor.select(addr);
  sensor.write(0xBE);                   // give a command to the slave to transmit the contents of the scratchpad memory

  byte data[9];                         // the scratchpad memory has 9 bytes in total
  for(int i = 0; i < 9; ++i) {
    data[i] = sensor.read();            // read and store the contents of the scratchpad memory byte-by-byte
  }

  int raw = (data[1] << 8) | data[0];   // store the contents of the temperature register into an integer

  byte res = data[4] & B01100000;
  float result = 0.0;
  switch (res) {
    case 96:                            // 12-bit resolution
      result = float((raw >> 4) + float(raw & B00001111)/16);
      break;
    case 64:                            // 11-bit resolution
      result = float((raw >> 4) + float(raw >> 1 & B00000111)/8);
      break;
    case 32:                            // 10-bit resolution
      result = float((raw >> 4) + float(raw >> 2 & B00000011)/4);
      break;
    default:                            // 9-bit resolution
      result = float((raw >> 4) + float(raw >> 3 & B00000001)/2);
      break;
  }

  return result;
}
