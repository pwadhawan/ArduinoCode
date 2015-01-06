/*

Copyright (c) 2012, 2013 RedBearLab

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include <Servo.h>
#include <SPI.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include <services.h>
#include "Boards.h"
#include <Stepper.h>

#define PROTOCOL_MAJOR_VERSION   0 //
#define PROTOCOL_MINOR_VERSION   0 //
#define PROTOCOL_BUGFIX_VERSION  2 // bugfix

#define PIN_CAPABILITY_NONE      0x00
#define PIN_CAPABILITY_DIGITAL   0x01
#define PIN_CAPABILITY_ANALOG    0x02
#define PIN_CAPABILITY_PWM       0x04
#define PIN_CAPABILITY_SERVO     0x08
#define PIN_CAPABILITY_I2C       0x10

// pin modes
//#define INPUT                 0x00 // defined in wiring.h
//#define OUTPUT                0x01 // defined in wiring.h
#define ANALOG                  0x02 // analog pin in analogInput mode
#define PWM                     0x03 // digital pin in PWM output mode
#define SERVO                   0x04 // digital pin in Servo output mode

//SLZ: Number of steps to rotate for opening/closing the lock
#define LOCK_DEGREES 95;
const int stepsPerRevolution = 512;
const float SpeedRPM = 50;

byte pin_mode[TOTAL_PINS];
byte pin_state[TOTAL_PINS];
byte pin_pwm[TOTAL_PINS];
byte pin_servo[TOTAL_PINS];

Servo servos[MAX_SERVOS];

//SmartLivez(SLZ): Pins to be used for Running the motor
int gpio_coil1 = 3;
int gpio_coil2 = 4;
int gpio_coil3 = 5;
int gpio_coil4 = 6;

int             LockControlPin = 2;
int 		delayFactor = 1;  // keep constant rpm even with microstepping
unsigned int 	uSecDelay;
int             rotation_direction; //Direction

byte current_state = LOW; //SLZ: Default state of lock

unsigned int 	lookup[10][4] = {{1,0,0,0}, 
                                 {1,1,0,0}, 
                                 {0,1,0,0}, 
                                 {0,1,1,0}, 
                                 {0,0,1,0},
                                 {0,0,1,1},
                                 {0,0,0,1},
                                 {1,0,0,1},
                                 {1,1,1,1},
                                 {0,0,0,0}};
//SLZ:Motor Instance 
//Stepper motor(stepsPerRevolution, gpio_coil1, gpio_coil2, gpio_coil3, gpio_coil4);



void setup()
{
  Serial.begin(57600);
  Serial.println("BLE Arduino Slave");
  
  /* Default all to digital input */
  for (int pin = 0; pin < TOTAL_PINS; pin++)
  {
    // Set pin to input with internal pull up
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);

    // Save pin mode and state
    pin_mode[pin] = INPUT;
    pin_state[pin] = LOW;
    
  }
    //SLZ: setup for Motor. Save pin mode and state
    pinMode(gpio_coil1, OUTPUT);
    pin_mode[gpio_coil1] = OUTPUT;
    
    pinMode(gpio_coil2, OUTPUT);
    pin_mode[gpio_coil2] = OUTPUT;
    
    pinMode(gpio_coil3, OUTPUT);
    pin_mode[gpio_coil3] = OUTPUT;
    
    pinMode(gpio_coil4, OUTPUT);
    pin_mode[gpio_coil4] = OUTPUT;
    
    setGPIOs(9); //start with driving all pins LOW

    pinMode(LockControlPin, INPUT);
    digitalWrite(LockControlPin, LOW);
    pin_mode[LockControlPin] = OUTPUT;
 
    setSpeed(SpeedRPM);   

  // Default pins set to 9 and 8 for REQN and RDYN
  // Set your REQN and RDYN here before ble_begin() if you need
  //ble_set_pins(3, 2);
  
  // Set your BLE Shield name here, max. length 10
  //ble_set_name("My Name");
  
  // Init. and start BLE library.
  ble_begin();
}



//SLZ: functions to Step the motor by the number of steps

void usleep(int delaytime) { //Delay in useconds. Add sleep functionality later
delay(delaytime/1000);
}

void setSpeed(float rpm) {
	float delayPerSec = (60/rpm)/stepsPerRevolution; // delay per step in seconds
	uSecDelay = (int)(delayPerSec * 1000 * 1000);    // in microseconds
        Serial.print("rpm = ");Serial.print(rpm);Serial.println();
        Serial.print("uSecDelay = ");Serial.print(uSecDelay);Serial.println();

}
void setGPIOs(int index){
    digitalWrite(gpio_coil1, lookup[index][0]);
    digitalWrite(gpio_coil2, lookup[index][1]);
    digitalWrite(gpio_coil3, lookup[index][2]);
    digitalWrite(gpio_coil4, lookup[index][3]);
}

void step(int numberOfSteps){
    Serial.print("Doing ");Serial.print(numberOfSteps);Serial.print(" steps and going to sleep for ");Serial.println(uSecDelay/delayFactor);

    int sleepDelay = uSecDelay/delayFactor;
    setGPIOs(9); //start with driving all pins LOW

	for(int i=0; i<numberOfSteps; i++) {
		if (rotation_direction==0) { //clockwise
			for(int j=0; j<8; j++) {
				setGPIOs(j);
				usleep(sleepDelay);
			}
		} else { // counter-clockwise
			for(int j=7; j>=0; j--) {
				setGPIOs(j);
				usleep(sleepDelay);
			}
		}
	}
        
        setGPIOs(9); //end with driving all pins LOW
	usleep (1); // sleep for 1us
}

void rotate(int degrees_in, int direction_in){
	rotation_direction  = direction_in;
	float degreesPerStep = 360.0f/stepsPerRevolution;
	int numberOfSteps = degrees_in/degreesPerStep;
	Serial.print( "The number of steps is ");Serial.print(numberOfSteps);Serial.println();
	Serial.print( "The delay factor is ");Serial.print(delayFactor);Serial.println();
	step(numberOfSteps*delayFactor);
}

byte lockreaction(byte state)
{
  Serial.print("Inside the lockreaction function call. Current_state =");Serial.print(current_state); Serial.print(state);  Serial.println();
  
  if((current_state==LOW) && (state==HIGH))
    {
       rotate(95, 0);
       current_state = HIGH;
       Serial.println(" Inside the lockreaction function call:LOCK");
       Serial.print(current_state); Serial.print(state);  Serial.println();
       delay(500);
    }
      else if((current_state=HIGH) && (state==LOW))
    {
       rotate(95, 1);
       current_state = LOW;
       Serial.println(" Inside the lockreaction function call:UNLOCK");
       Serial.print(current_state); Serial.print(state);  Serial.println();    
       delay(500);
     }   
  else    
    {
      Serial.println("The Current state of Lock and request state are same. State Value = ");
      Serial.print(state);
      Serial.println();
    }

}

static byte buf_len = 0;

void ble_write_string(byte *bytes, uint8_t len)
{
  if (buf_len + len > 20)
  {
    for (int j = 0; j < 15000; j++)
      ble_do_events();
    
    buf_len = 0;
  }
  for (int j = 0; j < len; j++)
  {
    ble_write(bytes[j]);
    buf_len++;
  }
  if (buf_len == 20)
  {
    for (int j = 0; j < 15000; j++)
      ble_do_events();
    buf_len = 0;
  }  
}

byte reportDigitalInput()
{
  if (!ble_connected())
    return 0;

  static byte pin = 0;
  byte report = 0;
  
  if (!IS_PIN_DIGITAL(pin))
  {
    pin++;
    if (pin >= TOTAL_PINS)
      pin = 0;
    return 0;
  }
  
  if (pin_mode[pin] == INPUT)
  {
      byte current_state = digitalRead(pin);
            
      if (pin_state[pin] != current_state)
      {
        pin_state[pin] = current_state;
        byte buf[] = {'G', pin, INPUT, current_state};
        ble_write_string(buf, 4);
        
        report = 1;
      }
  }
  
  pin++;
  if (pin >= TOTAL_PINS)
    pin = 0;
    
  return report;
}

void reportPinCapability(byte pin)
{
  byte buf[] = {'P', pin, 0x00};
  byte pin_cap = 0;
                    
  if (IS_PIN_DIGITAL(pin))
    pin_cap |= PIN_CAPABILITY_DIGITAL;
            
  if (IS_PIN_ANALOG(pin))
    pin_cap |= PIN_CAPABILITY_ANALOG;

  if (IS_PIN_PWM(pin))
    pin_cap |= PIN_CAPABILITY_PWM;

  if (IS_PIN_SERVO(pin))
    pin_cap |= PIN_CAPABILITY_SERVO;

  buf[2] = pin_cap;
  ble_write_string(buf, 3);
}

void reportPinServoData(byte pin)
{
//  if (IS_PIN_SERVO(pin))
//    servos[PIN_TO_SERVO(pin)].write(value);
//  pin_servo[pin] = value;
  
  byte value = pin_servo[pin];
  byte mode = pin_mode[pin];
  byte buf[] = {'G', pin, mode, value};         
  ble_write_string(buf, 4);
}

byte reportPinAnalogData()
{
  if (!ble_connected())
    return 0;
    
  static byte pin = 0;
  byte report = 0;
  
  if (!IS_PIN_DIGITAL(pin))
  {
    pin++;
    if (pin >= TOTAL_PINS)
      pin = 0;
    return 0;
  }
  
  if (pin_mode[pin] == ANALOG)
  {
    uint16_t value = analogRead(pin);
    byte value_lo = value;
    byte value_hi = value>>8;
    
    byte mode = pin_mode[pin];
    mode = (value_hi << 4) | mode;
    
    byte buf[] = {'G', pin, mode, value_lo};         
    ble_write_string(buf, 4);
  }
  
  pin++;
  if (pin >= TOTAL_PINS)
    pin = 0;
    
  return report;
}

void reportPinDigitalData(byte pin)
{
  byte state = digitalRead(pin);
  byte mode = pin_mode[pin];
  byte buf[] = {'G', pin, mode, state};         
  ble_write_string(buf, 4);
}

void reportPinPWMData(byte pin)
{
  byte value = pin_pwm[pin];
  byte mode = pin_mode[pin];
  byte buf[] = {'G', pin, mode, value};         
  ble_write_string(buf, 4);
}

void sendCustomData(uint8_t *buf, uint8_t len)
{
  uint8_t data[20] = "Z";
  memcpy(&data[1], buf, len);
  ble_write_string(data, len+1);
}

byte queryDone = false;

void loop()
{
  while(ble_available())
  {
    byte cmd;
    cmd = ble_read();
    Serial.write(cmd);
    
    // Parse data here
    switch (cmd)
    {
      case 'V': // query protocol version
        {
          byte buf[] = {'V', 0x00, 0x00, 0x01};
          ble_write_string(buf, 4);
        }
        break;
      
      case 'C': // query board total pin count
        {
          byte buf[2];
          buf[0] = 'C';
          buf[1] = TOTAL_PINS; 
          ble_write_string(buf, 2);
        }        
        break;
      
      case 'M': // query pin mode
        {  
          byte pin = ble_read();
          byte buf[] = {'M', pin, pin_mode[pin]}; // report pin mode
          ble_write_string(buf, 3);
        }  
        break;
      
      case 'S': // set pin mode
        {
          byte pin = ble_read();
          byte mode = ble_read();
          
          if (IS_PIN_SERVO(pin) && mode != SERVO && servos[PIN_TO_SERVO(pin)].attached())
            servos[PIN_TO_SERVO(pin)].detach();
  
          /* ToDo: check the mode is in its capability or not */
          /* assume always ok */
          if (mode != pin_mode[pin])
          {              
            pinMode(pin, mode);
            pin_mode[pin] = mode;
          
            if (mode == OUTPUT)
            {
              digitalWrite(pin, LOW);
              pin_state[pin] = LOW;
            }
            else if (mode == INPUT)
            {
              digitalWrite(pin, HIGH);
              pin_state[pin] = HIGH;
            }
            else if (mode == ANALOG)
            {
              if (IS_PIN_ANALOG(pin)) {
                if (IS_PIN_DIGITAL(pin)) {
                  pinMode(PIN_TO_DIGITAL(pin), LOW);
                }
              }
            }
            else if (mode == PWM)
            {
              if (IS_PIN_PWM(pin))
              {
                pinMode(PIN_TO_PWM(pin), OUTPUT);
                analogWrite(PIN_TO_PWM(pin), 0);
                pin_pwm[pin] = 0;
                pin_mode[pin] = PWM;
              }
            }
            else if (mode == SERVO)
            {
              if (IS_PIN_SERVO(pin))
              {
                pin_servo[pin] = 0;
                pin_mode[pin] = SERVO;
                if (!servos[PIN_TO_SERVO(pin)].attached())
                  servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin));
              }
            }
          }
            
  //        if (mode == ANALOG)
  //          reportPinAnalogData(pin);
          if ( (mode == INPUT) || (mode == OUTPUT) )
            reportPinDigitalData(pin);
          else if (mode == PWM)
            reportPinPWMData(pin);
          else if (mode == SERVO)
            reportPinServoData(pin);
        }
        break;

      case 'G': // query pin data
        {
          byte pin = ble_read();
          reportPinDigitalData(pin);
        }
        break;
        
      case 'T': // set pin digital state
        {
          byte pin = ble_read();
          byte state = ble_read();
          
          digitalWrite(pin, state);
            Serial.print(" Printing the value of the Pin, state: "); Serial.print(pin);Serial.print(" and "); Serial.print(state);Serial.println();
          if (pin==LockControlPin) //SLZ: In case the Person is controlling the Lock pin, call the lock function.
               {
                 Serial.println(" Inside the function call if loop ");
                 lockreaction(state);
               }
          reportPinDigitalData(pin);
        }
        break;
      
      case 'N': // set PWM
        {
          byte pin = ble_read();
          byte value = ble_read();
          
          analogWrite(PIN_TO_PWM(pin), value);
          pin_pwm[pin] = value;
          reportPinPWMData(pin);
        }
        break;
      
      case 'O': // set Servo
        {
          byte pin = ble_read();
          byte value = ble_read();

          if (IS_PIN_SERVO(pin))
            servos[PIN_TO_SERVO(pin)].write(value);
          pin_servo[pin] = value;
          reportPinServoData(pin);
        }
        break;
      
      case 'A': // query all pin status
        for (int pin = 0; pin < TOTAL_PINS; pin++)
        {
          reportPinCapability(pin);
          if ( (pin_mode[pin] == INPUT) || (pin_mode[pin] == OUTPUT) )
            reportPinDigitalData(pin);
          else if (pin_mode[pin] == PWM)
            reportPinPWMData(pin);
          else if (pin_mode[pin] == SERVO)
            reportPinServoData(pin);  
        }
        
        queryDone = true; 
        {
          uint8_t str[] = "ABC";
          sendCustomData(str, 3);
        }
       
        break;
          
      case 'P': // query pin capability
        {
          byte pin = ble_read();
          reportPinCapability(pin);
        }
        break;
        
      case 'Z':
        {
          byte len = ble_read();
          byte buf[len];
          for (int i=0;i<len;i++)
            buf[i] = ble_read();
          Serial.println("->");
          Serial.print("Received: ");
          Serial.print(len);
          Serial.println(" byte(s)");
          Serial.print(" Hex: ");
          for (int i=0;i<len;i++)
            Serial.print(buf[i], HEX);
          Serial.println();
        }
    }

    // send out any outstanding data
    ble_do_events();
    buf_len = 0;
    
    return; // only do this task in this loop
  }

  // process text data
  if (Serial.available())
  {
    byte d = 'Z';
    ble_write(d);

    delay(5);
    while(Serial.available())
    {
      d = Serial.read();
      ble_write(d);
    }
    
    ble_do_events();
    buf_len = 0;
    
    return;    
  }

  // No input data, no commands, process analog data
  if (!ble_connected())
    queryDone = false; // reset query state
    
  if (queryDone) // only report data after the query state
  { 
    byte input_data_pending = reportDigitalInput();  
    if (input_data_pending)
    {
      ble_do_events();
      buf_len = 0;
      
      return; // only do this task in this loop
    }
  
    reportPinAnalogData();
    
    ble_do_events();
    buf_len = 0;
    
    return;  
  }
    
  ble_do_events();
  buf_len = 0;
}

