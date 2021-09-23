#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusRtu.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

#define DEBUG   1
#define ID   90
#define ONEWIRE_RESOLUTION   11
#define ONEWIRE_NUMBER   8
#define ONEWIRE_POOLING   1000 //750/ (1 << (12-ONEWIRE_RESOLUTION))
#define ONEWIRE_PINS   2

#define PWM_NUMBER   2
#define RELAY_NUMBER   8
#define ANALOG_NUMBER   4
#define ANALOG_POOLING   500

SoftwareSerial modbusSerial(A4, A5); // RX, TX
 
OneWire oneWire[ONEWIRE_PINS] = {
  OneWire(2),
  OneWire(3)
};
DallasTemperature sensors[ONEWIRE_PINS];
DeviceAddress oneWhireSensorsUnique[ONEWIRE_NUMBER];
int oneWireSensorsCount = 0;
float temperature[ONEWIRE_NUMBER];

byte pwmPins[PWM_NUMBER] = { // Uno 980hz
  5,
  6
};

byte relayPins[RELAY_NUMBER] = { // Uno
  4, 
  7,
  8,
  9,
  10,
  11,
  12,
  13
};

byte analogPins[ANALOG_NUMBER] = { // Uno
  A0, 
  A1,
  A2,
  A3
};

enum {
      mbs_ds1,
      mbs_ds2,
      mbs_ds3,
      mbs_ds4,
      mbs_ds5,
      mbs_ds6,
      mbs_ds7,
      mbs_ds8,
      mbs_pin1,
      mbs_pin2,
      mbs_pin3,
      mbs_pin4,
      mbs_relay,
      mbs_pwm1,
      mbs_pwm2,
      mbs_regs        
};
uint16_t modbusRegs[mbs_regs];


/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus slave(ID,modbusSerial,0); // this is slave @1 and RS-485

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
      {
      byte lowByte = ((p_value >> 0) & 0xFF);
      byte highByte = ((p_value >> 8) & 0xFF);

      EEPROM.write(p_address, lowByte);
      EEPROM.write(p_address + 1, highByte);
      }

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
      {
      byte lowByte = EEPROM.read(p_address);
      byte highByte = EEPROM.read(p_address + 1);

      return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
      }


void setupDS18() {

  for (int pi = 0; pi < ONEWIRE_PINS; pi++) {
    sensors[pi] = DallasTemperature(&oneWire[pi]);
    sensors[pi].begin();

    int countSensors = sensors[pi].getDeviceCount();

    for (int i = 0; i < countSensors; i++) {
      if (oneWireSensorsCount < ONEWIRE_NUMBER) {
        sensors[pi].getAddress(oneWhireSensorsUnique[oneWireSensorsCount], i);
        sensors[pi].setResolution(oneWhireSensorsUnique[oneWireSensorsCount], ONEWIRE_RESOLUTION);
      }

      oneWireSensorsCount++;
    }
  }

  if (DEBUG) {
    for (int i = 0; i < oneWireSensorsCount; i++) {
      Serial.print("Device ");
      Serial.print(i);
      Serial.print(" Address: ");
      printAddress(oneWhireSensorsUnique[i]);
  
      Serial.println();
    }
  
    Serial.print("U sensors: ");
    Serial.println(oneWireSensorsCount);
    
    Serial.println();
  }
}

void setupSlave() {
  pinMode(A4, INPUT);
  pinMode(A5, OUTPUT);
  
  Serial.begin( 9600); // baud-rate at 19200
  //modbusSerial.begin( 9600, SERIAL_8N2); // baud-rate at 19200
  modbusSerial.begin( 9600); // baud-rate at 19200
  slave.start();

  // load registers from eeprom
  for (int i = 0; i < mbs_regs * 2; i+=2) {
    modbusRegs[i / 2] = EEPROMReadInt(i);
  }
}

void setupPwm() {
  for (int i = 0; i < PWM_NUMBER; i++) {
    pinMode(pwmPins[i],OUTPUT);
  }
}

void setupRelay() {
  for (int i = 0; i < RELAY_NUMBER; i++) {
    pinMode(relayPins[i],OUTPUT);
  }
}

 
// функция вывода адреса датчика
void printAddress(DeviceAddress deviceAddress){
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


 
void setup(){
  setupSlave();


  setupDS18();
  setupPwm();
  setupRelay();

  

  delay(1000);
}



void readDS18(unsigned long interval) {
  static unsigned long poolTime = 0;
  static unsigned long convTime = 0;

  unsigned long t = millis();
  if (t - poolTime > interval) {
    byte tidx=0;
    for (int pi = 0; pi < 2; pi++) {

      if (poolTime > 0 && poolTime - convTime > interval) {
        for (int i = 0; i < sensors[pi].getDeviceCount(); i++) {
          if (tidx < ONEWIRE_NUMBER) temperature[tidx++] = sensors[pi].getTempCByIndex(i);
        }

        for (int i = 0; i < oneWireSensorsCount; i++) {
          float ft = (float)temperature[i];
          modbusRegs[mbs_ds1+i]= (uint16_t)((ft + 127.f) * 255);
      
      
          if (DEBUG) {
            Serial.print("DS ");
            Serial.print(i);
            Serial.print(" Temp C: ");
            Serial.print(temperature[i]);
            Serial.print(" REG: ");
            Serial.print(modbusRegs[mbs_ds1+i]);
            Serial.println();
          }
        }

        convTime = t;
      }

      
      sensors[pi].setWaitForConversion(false);  // makes it async
      sensors[pi].requestTemperatures();
      sensors[pi].setWaitForConversion(true);
    }

    

    poolTime = t;
  }
}

void readAnalog(unsigned long interval) {
  static unsigned long poolTime = 0;

  unsigned long t = millis();
  if (t - poolTime > interval) {

    for (int i = 0; i < ANALOG_NUMBER; i++) {
      uint16_t level = analogRead(analogPins[i]);
      
      modbusRegs[mbs_pin1+i] = level;
  
      if (DEBUG) {
        Serial.print("VAL ");
        Serial.print(i);
        Serial.print(" Level: ");
        Serial.print(level);
        Serial.println();
      }
    }
    poolTime = t;
  }
}


 
void loop(){

  readDS18(ONEWIRE_POOLING);
  readAnalog(ANALOG_POOLING);
  
  
  

  uint16_t old_modbusRegs[mbs_regs];
  memcpy(old_modbusRegs, modbusRegs, sizeof(modbusRegs));
  
  

  slave.poll( modbusRegs, mbs_regs );
  int reg_changed = memcmp( (const void *)old_modbusRegs, (const void *)modbusRegs, sizeof(old_modbusRegs));


  

  if ( reg_changed != 0)
  {
    for (int i = 0; i < PWM_NUMBER; i++) {
      uint16_t pwm = modbusRegs[mbs_pwm1 + i];
      byte level = (byte)(pwm / 256);
  
      analogWrite(pwmPins[i], level);
  
      if (DEBUG) {
        Serial.print("PWM ");
        Serial.print(i);
        Serial.print(" Level: ");
        Serial.print(level);
        Serial.print(" REG: ");
        Serial.print(pwm);
        Serial.println();
      }
    }
  
    uint16_t relays = modbusRegs[mbs_relay];
    for (int i = 0; i < RELAY_NUMBER; i++) {
      
      byte state = (byte)bitRead(relays, i);
  
      if (state == 1) digitalWrite(relayPins[i], HIGH);
      else digitalWrite(relayPins[i], LOW);
  
      if (DEBUG) {
        Serial.print("RELAY ");
        Serial.print(i);
        Serial.print(" state: ");
        Serial.print(state);
        Serial.println();
      }
    }
    
    if (DEBUG) {
      Serial.println("STORE TO EEPROM!");
    }

    for (int i = 0; i < mbs_regs; i++) {
      EEPROMWriteInt(i * 2, modbusRegs[i]);
    }
  }

  //delay(200);
}
