/*************************************************************************/
/* Copyright (C) 2015/16 - Daniel Rubio Bonilla                          */
/*                         danielrubiob [at] gmail [dot] com             */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */
/*************************************************************************/

#include <Wire.h>

#define MOTOR_OFF 0
#define MOTOR_MIN 160
#define MOTOR_NOR 200
#define MOTOR_MAX 255

#define FORD LOW
#define BACK HIGH

#define DEVID  0x12

#define SEN_C_VAL 196

enum RMOVE {MSS = 0,
            MFL = 1, MFF = 2, MFR = 3,
            MRL = 4, MRR = 5,
            MBL = 6, MBB = 7, MBR = 8};
byte cMove  = MSS;
byte cPower = MOTOR_OFF;

struct dsen_t {
  byte pin;
  byte value;
};

struct motor_t {
  byte p_power;   // Motor Power pin (PWM)
  byte p_dir;     // Motor Direction pin
  byte p_encoder; // Motor Encoder pin
  byte power;     //
  byte dir;
  byte enc_status;
  unsigned int enc_count;
  unsigned int enc_total_count;
};

struct dsen_t dsens[4];
struct motor_t motors[4];

byte nextCommand = 0x00;

byte _heartbeat = 0;
byte _forceStop = 0;
byte _eevent    = 0;

byte setPower(byte power) {
  if(power == 0) {
    return 0;
  }
  else if(power < MOTOR_MIN) {
    return MOTOR_MIN;
  }
  else {
    return power;
  }
}

void i2cReceiveEvent(int n) {
  byte i;
  byte reg;
  byte val[4];

  if(n<1)
    return;
  reg = Wire.read();

  for(i=0; i<4; i++) {
    val[i] = 0;
  }
  
  for(i=0; i<n-1 && i<4; i++) {
    val[i] = Wire.read();
  }

  switch(reg) {
    case 0x0A:
    setCommand(val[0]);
    break;
    case 0x20:
    goStop();
    break; 
    case 0x21:
    goStraight(val[0], val[1]);
    break;
    case 0x22:
    goTurn(val[0], val[1]);
    break;
    case 0x23:
    goRotate(val[0], val[1]);
    break;

    case 0x2A:
    changePowerAll(val[0]);
    break;

    case 0x2B:
    changePowerAllNZ(val[0]);
    break;
    
    case 0xFB:
    cli();
    _heartbeat = 0;
    sei();
    if(_eevent) {
      Wire.write(0x33);
    }else{
      Wire.write(0x11);
    }
    break;
    case 0xFC:
    _eevent = 0;
    break;
    default:
    break;
  }
}

void i2cRequestEvent() {
  byte data[4];
  
  switch(nextCommand) {
    case 0xFA:
    Wire.write(0xAC);
    break;
    case 0x30:
    Wire.write(dsens[0].value);
    break;
    case 0x31:
    Wire.write(dsens[1].value);
    break;
    case 0x32:
    Wire.write(dsens[2].value);
    break;
    case 0x33:
    Wire.write(dsens[3].value);
    break;
    case 0x35:
    data[0] = dsens[0].value;
    data[1] = dsens[1].value;
    data[2] = dsens[2].value;
    data[3] = dsens[3].value;
    Wire.write(data, 4);
    break;

    
    case 0x40:
    data[0] = motors[0].power;
    data[1] = motors[0].dir;
    Wire.write(data, 2);
    break;
    case 0x41:
    data[0] = motors[1].power;
    data[1] = motors[1].dir;
    Wire.write(data, 2);
    break;
    case 0x42:
    data[0] = motors[2].power;
    data[1] = motors[2].dir;
    Wire.write(data, 2);
    break;
    case 0x43:
    data[0] = motors[3].power;
    data[1] = motors[3].dir;
    Wire.write(data, 2);
    break;
    case 0x44:
    data[0] = cMove;
    data[1] = cPower;
    Wire.write(data, 2);
    break;
    case 0x45:
    data[0] = motors[0].power;
    data[1] = motors[0].dir;
    data[2] = motors[1].power;
    data[3] = motors[1].dir;
    data[4] = motors[2].power;
    data[5] = motors[2].dir;
    data[6] = motors[3].power;
    data[7] = motors[3].dir;
    Wire.write(data, 2);
    break;

    
    default:
    break;
  }
}

void setCommand(byte command) {
  nextCommand = command;
}

void applyMotorValues() {
  byte i;
  for(i=0; i<4; i++) {
    digitalWrite(motors[i].p_dir, motors[i].dir);
    analogWrite(motors[i].p_power, motors[i].power);
  }
}

void setup() {
  byte i;

  // change the resolution to 8 bits
  //analogReadResolution(8);
  
  Wire.begin(DEVID);
  Wire.onReceive(i2cReceiveEvent);
  Wire.onRequest(i2cRequestEvent);

  motors[0].p_power = 5;
  motors[1].p_power = 6;
  motors[2].p_power = 3;
  motors[3].p_power = 11;

  motors[0].p_dir = A0;
  motors[1].p_dir = A1;
  motors[2].p_dir = 13;
  motors[3].p_dir = 7;

  motors[0].p_encoder = 8;
  motors[1].p_encoder = 9;
  motors[2].p_encoder = 10;
  motors[3].p_encoder = 12;

//////////////////////////

  dsens[0].pin = A6;
  dsens[1].pin = A7;
  dsens[2].pin = A2;
  dsens[3].pin = A3;
  
  for(i=0; i<4; i++) {
    dsens[i].value = 0;
    pinMode(dsens[i].pin, INPUT);
  }

  for(i=0; i<4; i++) {  
    pinMode(motors[i].p_power,   OUTPUT);
    pinMode(motors[i].p_dir,     OUTPUT);
    pinMode(motors[i].p_encoder, INPUT);
    motors[i].power = MOTOR_OFF;
    motors[i].dir   = FORD;
    motors[i].enc_status = LOW;
    motors[i].enc_count  = 0;
    motors[i].enc_total_count = 0;
  }

  applyMotorValues();
  
  Serial.begin(9600);

  cli();//stop interrupts
  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set timer count for 1khz increments
  //OCR1A = 1999;// = (16*10^6) / (1000*8) - 1
  OCR1A = 999;// = (8*10^6) / (1000*8) - 1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for 8 prescaler
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
}

//Interrupt at freq of 1kHz
ISR(TIMER1_COMPA_vect) {
  byte _t, i;

  for(i=0; i<4; i++) {
    _t = digitalRead(motors[i].p_encoder);
    if(_t != motors[i].enc_status) {
      motors[i].enc_status = _t;
      if(motors[i].enc_status == LOW) {
        motors[i].enc_count++;
        motors[i].enc_total_count++;
      }
    }
  }
  
  _heartbeat++;
  if(_heartbeat > 250) {
    _forceStop = 1;
  }
}

void changePowerAll(byte power) {
  byte i;
  cPower = setPower(power);
  for(i=0; i<4; i++) {
    motors[i].power = cPower;
    analogWrite(motors[i].p_power, motors[i].power);
  }
}

void changePowerAllNZ(byte power) {
  byte i;
  cPower = setPower(power);
  for(i=0; i<4; i++) {
    if(motors[i].power != MOTOR_OFF) {
      motors[i].power = cPower;
      analogWrite(motors[i].p_power, motors[i].power);
    }
  }
}

void moveFF(byte power) {
  cPower = setPower(power);

  if(dsens[0].value < SEN_C_VAL ||
    dsens[2].value < SEN_C_VAL ) {
      goStop();
      _eevent = 1; 
  } else {
    motors[0].power = cPower;
    motors[0].dir   = FORD;
    motors[1].power = cPower;
    motors[1].dir   = FORD;
    motors[2].power = cPower;
    motors[2].dir   = FORD;
    motors[3].power = cPower;
    motors[3].dir   = FORD;
    cMove = MFF;
    applyMotorValues();
  }
}

void moveBB(byte power) {
  cPower = setPower(power);

  if(dsens[1].value < SEN_C_VAL ||
    dsens[3].value < SEN_C_VAL ) {
      goStop();
      _eevent = 1; 
  } else {
    motors[0].power = cPower;
    motors[0].dir   = BACK;
    motors[1].power = cPower;
    motors[1].dir   = BACK;
    motors[2].power = cPower;
    motors[2].dir   = BACK;
    motors[3].power = cPower;
    motors[3].dir   = BACK;
    cMove = MBB;
    applyMotorValues();
  }
}

void moveRL(byte power) {
  cPower = setPower(power);

  if(dsens[0].value < SEN_C_VAL ||
    dsens[1].value < SEN_C_VAL ||
    dsens[2].value < SEN_C_VAL ||
    dsens[3].value < SEN_C_VAL ) {
      goStop();
      _eevent = 1; 
  } else {
    motors[0].power = cPower;
    motors[0].dir   = FORD;
    motors[1].power = cPower;
    motors[1].dir   = FORD;
    motors[2].power = cPower;
    motors[2].dir   = BACK;
    motors[3].power = cPower;
    motors[3].dir   = BACK;
    cMove = MRL;
    applyMotorValues();
  }
}

void moveRR(byte power) {
  cPower = setPower(power);

  if(dsens[0].value < SEN_C_VAL ||
    dsens[1].value < SEN_C_VAL ||
    dsens[2].value < SEN_C_VAL ||
    dsens[3].value < SEN_C_VAL ) {
      goStop();
      _eevent = 1; 
  } else {
    motors[0].power = cPower;
    motors[0].dir   = BACK;
    motors[1].power = cPower;
    motors[1].dir   = BACK;
    motors[2].power = cPower;
    motors[2].dir   = FORD;
    motors[3].power = cPower;
    motors[3].dir   = FORD;
    cMove = MRR;
    applyMotorValues();
  }
}

void moveFL(byte power) {
  cPower = setPower(power);

  if(dsens[0].value < SEN_C_VAL ||
    dsens[2].value < SEN_C_VAL) {
      goStop();
      _eevent = 1; 
  } else {
    motors[0].power = cPower;
    motors[0].dir   = FORD;
    motors[1].power = cPower;
    motors[1].dir   = FORD;
    motors[2].power = MOTOR_OFF;
    motors[2].dir   = FORD;
    motors[3].power = MOTOR_OFF;
    motors[3].dir   = FORD;
    cMove = MFL;
    applyMotorValues();
  }
}

void moveFR(byte power) {
  cPower = setPower(power);  

  if(dsens[0].value < SEN_C_VAL ||
    dsens[2].value < SEN_C_VAL) {
      goStop();
      _eevent = 1; 
  } else {
    motors[0].power = MOTOR_OFF;
    motors[0].dir   = FORD;
    motors[1].power = MOTOR_OFF;
    motors[1].dir   = FORD;
    motors[2].power = cPower;
    motors[2].dir   = FORD;
    motors[3].power = cPower;
    motors[3].dir   = FORD;
    cMove = MFR;
    applyMotorValues();
  }
}

void moveBL(byte power) {
  cPower = setPower(power);

  if(dsens[1].value < SEN_C_VAL ||
    dsens[3].value < SEN_C_VAL) {
      goStop();
      _eevent = 1; 
  } else {
    motors[0].power = cPower;
    motors[0].dir   = BACK;
    motors[1].power = cPower;
    motors[1].dir   = BACK;
    motors[2].power = MOTOR_OFF;
    motors[2].dir   = BACK;
    motors[3].power = MOTOR_OFF;
    motors[3].dir   = BACK;
    cMove = MBL;
    applyMotorValues();
  }
}

void moveBR(byte power) {
  cPower = setPower(power);  

  if(dsens[1].value < SEN_C_VAL ||
    dsens[3].value < SEN_C_VAL) {
      goStop();
      _eevent = 1; 
  } else {
    motors[0].power = MOTOR_OFF;
    motors[0].dir   = BACK;
    motors[1].power = MOTOR_OFF;
    motors[1].dir   = BACK;
    motors[2].power = cPower;
    motors[2].dir   = BACK;
    motors[3].power = cPower;
    motors[3].dir   = BACK;
    cMove = MBR;
    applyMotorValues();
  }
}

void goStraight(byte dir, byte power) {
  switch(dir) {
    case FORD: //FL
      moveFF(power);
      break;
    case BACK: //FR
      moveBB(power);
      break;
    default:
      goStop();
      return;
      break;
  }
}

void goTurn(byte dir, byte power) {
  switch(dir) {
    case 0: //FL
      moveFL(power);
      break;
    case 1: //FR
      moveFR(power);
      break;
    case 2: //BL
      moveBL(power);
      break;
    case 3: //BR
      moveBR(power);
      break;
    default:
      goStop();
      return;
      break;
  }
}

void goRotate(byte dir, byte power) { 
  switch(dir) {
    case 0: //RL
      moveRL(power);
      break;
    case 1: //RR
      moveRR(power);
      break;
     default:
      goStop();
      return;
      break;
  }
}

void goStop() {
  byte i;
  cPower = MOTOR_OFF;

  motors[0].power = cPower;
  motors[1].power = cPower;
  motors[2].power = cPower;
  motors[3].power = cPower;
  
  cMove = MSS;
  applyMotorValues();
}

void readSensors() {
  byte i;
  for(i=0; i<4; i++) {
    dsens[i].value = analogRead(dsens[i].pin) >> 2;
  }
}
   
void checkSensors() {
  if(dsens[0].value < SEN_C_VAL) { //FR
    switch(cMove) {
      case MFL:
      case MFF:
      case MFR:
      case MRL:
      case MRR:
      goStop();
      _eevent = 1;
      break;
    }
  } else if(dsens[1].value < SEN_C_VAL) { //BR
    switch(cMove) {
      case MBL:
      case MBB:
      case MBR:
      case MRL:
      case MRR:
      goStop();
      _eevent = 1;
      break;
    }
  } else if(dsens[2].value < SEN_C_VAL) { //FL
    switch(cMove) {
      case MFL:
      case MFF:
      case MFR:
      case MRL:
      case MRR:
      goStop();
      _eevent = 1;
      break;
    }
  } else if(dsens[3].value < SEN_C_VAL) { //BL
    switch(cMove) {
      case MBL:
      case MBB:
      case MBR:
      case MRL:
      case MRR:
      goStop();
      _eevent = 1;
      break;
    }
  }
}

void loop() {
  byte _fs;
  goStop();

  while(1) {
    cli();
    _fs = _forceStop;
    sei();
    if(_fs) {
     _forceStop = 0;
     goStop();
    }

    delay(25);
    readSensors();
    checkSensors();
  }
}

