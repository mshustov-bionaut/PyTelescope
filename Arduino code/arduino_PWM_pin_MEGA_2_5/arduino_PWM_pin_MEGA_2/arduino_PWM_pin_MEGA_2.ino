// Arduino sketch for 8EM Mini setup. Written for ArduinoMega
// Bionaut labs 2023. Shustov

// In 8EM Mini ArduinoMega serves as a USB-GPIO interface, which sends PWM signal to current drivers
// The ArduinoMega's default frequency (490 Hz on pins except 4 and 13) can be changed to 3921.16 Hz 

#include<avr/wdt.h>
// Variables and constants:
int userInput[4];    // raw input from serial buffer, 4 bytes
int analogChanNumbers[4]; //input from Serial - numbers of analog channels to measure voltage
const int analogPinsNames[16] = {A0,A1,A2,A3,A4,A5,A6,A7}; 
byte longByteInput[8];// raw input from serial buffer, 8 bytes
byte byteInput[4];
int startbyte;       // start byte, begin reading input
int i;               // iterator
int mode;            // working mode: 0-nothing, 1-single comand, 2- multiple comand
int ledPin = 13;     // LED on Pin 13 for digital on/off demo
int pinState = LOW;
int chan;
int pwmChan;
int dirChan;
int pwmVal;
int dirVal;
int chanArr[4];
int pinStateArr[4];
int multComandStart;
int multComandTime;
int currStep;
int state;
int comandRecvTime;
int lightRecvTime;
int timerTime;
const int usbBaud = 19200; //Arduino baud rate
int sensorValue;
int num;


void coilsOff(){
  analogWrite(2, 0);
  analogWrite(3, 0);
  analogWrite(4, 0);
  analogWrite(5, 0);
  analogWrite(6, 0);
  analogWrite(7, 0);
  analogWrite(8, 0);
  analogWrite(9, 0);
  analogWrite(10, 0);
  analogWrite(11, 0);
  analogWrite(12, 0);
  analogWrite(13, 0);
}
void lightOff(){
  analogWrite(44, 0);
  analogWrite(45, 0);
  analogWrite(46, 0);
}

void dPinsPrepare(){
  for (i=22;i<38;i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i,LOW);
  }
}

void pololuCoilComand(){
  Serial.readBytes(byteInput,4);  
  // First byte = pwm pin
  pwmChan = byteInput[0];
  // Second byte direction pin
  dirChan = byteInput[1];
  // Third byte = pwm value
  pwmVal = byteInput[2];
  // Fourth byte state
  dirVal = byteInput[3];
  //Check values, if ok - send to the pin
  if (pwmVal>=0 && pwmVal<=255){
    if (dirVal == 0){
      digitalWrite(dirChan,LOW);
    } else {
      digitalWrite(dirChan,HIGH);
    }
    analogWrite(pwmChan, pwmVal);
    mode = 1;
    comandRecvTime = millis();
    analogWrite(13, 255);
  }
}

void BTSCoilCommand3bytes(){
  Serial.readBytes(byteInput,2);
  // First byte = digital channel?
  chan = byteInput[0];
  // Second byte state?
  state = byteInput[1];
  //Check values, if ok - send to the pin
  if (chan>1 && chan<14 && state>=0 && state<=255){
    analogWrite(chan, state);
    mode = 1;
    comandRecvTime = millis();
    analogWrite(13, 255);
  }
}

// Setup
void setup() {
  //Change frequency of PWM pins from 490 Hz (default) to 3921.16 Hz. Did not test
  TCCR1B = TCCR1B & B11111000 | B00000010;  // for PWM frequency of 3921.16 Hz pins D11,12
  TCCR2B = TCCR2B & B11111000 | B00000010;  // for  PWM frequency of 3921.16 Hz pins D9,10

  lightOff();
  coilsOff();
  dPinsPrepare();
  // Open the serial connection
  Serial.begin(usbBaud);
  mode = 0;
  wdt_disable();  // Disable the watchdog and wait for more than 2 seconds 
  delay(3000);  // Done so that the Arduino doesn't keep resetting infinitely in case of wrong configuration 
  wdt_enable(WDTO_8S);  // Enable the hardware watchdog with a timeout of 8 seconds 
}

void loop() 
{ 

  // Wait for serial input (min 3 bytes in buffer)
  if (Serial.available() > 2) {
    // Read the first byte
    startbyte = Serial.read();
    
    // If it is the startbyte (253) for single comand ...
    if (startbyte == 0xfd) {
      BTSCoilCommand3bytes();
    }

    // If it is the startbyte (251) for single channel comand (all othe channels off) ...
    else if (startbyte == 0xfb) {
      // ... then get the next two bytes
      coilsOff();     
      BTSCoilCommand3bytes();
    }

    // If it is the startbyte (250) - all coils Off ...
    else if (startbyte == 0xfa) {
      Serial.readBytes(byteInput,2);
      coilsOff();
    }

    // If it is the startbyte (252) for multiple comand ...
    else if (startbyte == 0xfc) {
      // ... then get 8 bytes: 4 times by 2 bytes
      
      comandRecvTime = millis();
      Serial.readBytes(longByteInput,8);
      // First byte of the couple = digital channel, Second byte of the couple = state
      chanArr[0] = longByteInput[0];
      pinStateArr[0] = longByteInput[1];
      chanArr[1] = longByteInput[2];
      pinStateArr[1] = longByteInput[3];
      chanArr[2] = longByteInput[4];
      pinStateArr[2] = longByteInput[5];
      chanArr[3] = longByteInput[6];
      pinStateArr[3] = longByteInput[7];
      currStep = 0;
      mode = 2;
      multComandStart = 0;
    }

    // if the startbyte = 233 - measure voltage on 4 analog channels and send result to Serial port
    else if (startbyte == 0xe9){
      // read 4 bytes with channels numbers
      Serial.readBytes(byteInput,4);      
      Serial.println("Voltage");
      for (i=0;i<4;i++) {
        int num = byteInput[i];
        sensorValue = analogRead(num);
        Serial.println(sensorValue);
      }
    }

    // if the startbyte = 234 - measure voltage on a single analog channel
    else if (startbyte == 0xea){
      // read 2 bytes, the first is a channel number, the second is dummy
      Serial.readBytes(byteInput,2);
      Serial.println("VoltageSingle");
      num = byteInput[0];
      sensorValue = analogRead(num);
      Serial.println(sensorValue);
    }

    // if the startbyte = 235 - measure voltage on all analog channels 0-15
    else if (startbyte == 0xeb){
      // read 2 bytes, they are dummy, just to keep the dimension of the sent array
      Serial.readBytes(byteInput,2);
      Serial.println("VoltageAll");
      for (num=0;num<16;num++) {
        sensorValue = analogRead(num);
        Serial.println(sensorValue);
      }
    }

    // if the startbyte = 231 - PWM control of pins 44,45,46
    else if (startbyte == 0xe7){
      // read channel number and value
      lightOff();
      Serial.readBytes(byteInput,2);
      // First byte = digital channel?
      chan = byteInput[0];
      // Second byte state?
      state = byteInput[1];
      //Check values, if ok - send to the pin
      if (chan>43 && chan<47 && state>=0 && state<=255){
        analogWrite(chan, state);
        }
      lightRecvTime = millis();
    }

  // if the startbyte = 230 - Light off
    else if (startbyte == 0xe6){
      // read 2 bites just to free the buffer
      Serial.readBytes(byteInput,2);
      lightOff(); 
    }

   // if the startbyte =  223 - Pololu individual control
    else if (startbyte == 0xdf) {
      pololuCoilComand();     
    }

    // If it is the startbyte (221) for single channel Pololu comand (all othe channels off) ...
    else if (startbyte == 0xdd) {
      coilsOff();
      pololuCoilComand();
    }

    // Pololu all off
    else if (startbyte == 0xdc){
      Serial.readBytes(byteInput,4);
      coilsOff();
      dPinsPrepare();
    }
  }

  // if the mode is multiple command and we reached the period of 7ms
  multComandTime = millis()-multComandStart;
  if (mode==2 && multComandTime>7) {
    // check, if the channel number is -1, than reset the channel iterator
    if (chanArr[currStep] == -1){
      currStep = 0;
    }
    else {
      chan = chanArr[currStep];
      state = pinStateArr[currStep];
      //Check values, if ok - send to the pin
      if (chan>1 && chan<14 && state>=0 && state<=255){
        analogWrite(chan, state);
        multComandStart = millis();
      }
      currStep++;
      if (currStep>3){
        currStep = 0;
      }
    }
  }

  //Soft watchdog timer of the Serial connection
  timerTime = millis() - comandRecvTime;
  if (timerTime > 3000){
    coilsOff();
    dPinsPrepare();
    mode = 0;
  }
  timerTime = millis() - lightRecvTime;
  if (timerTime > 600000){
    lightOff();
    mode = 0;
  }
  // Hardware watchdog timer of the Arduino itself - reset
  wdt_reset();  // Reset the hardware watchdog of Arduino 
}
























