#include <Wire.h>
#include <Servo.h>

#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0

#define DEFAULT_I2C_ADDRESS 0x31
// servo pins definition

#define PIN_SERVO1 PB1
#define PIN_SERVO2 PA7
#define PIN_SERVO3 PA6
#define PIN_SERVO4 PA4
#define PIN_SERVO5 PA0
#define PIN_SERVO6 PA1
#define PIN_SERVO7 PA2
#define PIN_SERVO8 PA3

// I2C definition
#define MAX_TRANSMIT_SIZE 8 // maximal number of bytes the master can request

// register bank
#define REG_SIZE32 6   // size of register bank, in 4-byte (32 bit) units
#define RW_REGISTERS 15 // number of RW registers (in bytes)

// R/W registers
#define REG_SERVO1          0
#define REG_SERVO2          2
#define REG_SERVO3          4
#define REG_SERVO4          6
#define REG_SERVO5          8
#define REG_SERVO6          10
#define REG_SERVO7          12
#define REG_SERVO8          14

// Read-only registers
#define REG_FW_VERSION      16
#define REG_WHO_AM_I        18

//  main register bank
volatile int32_t REG32[REG_SIZE32];
// cast as byte array
volatile byte *REGBANK = (byte *)REG32;

volatile uint8_t request_address;

// create aliases for registers
volatile uint16_t *servo_power      = (uint16_t *) &REGBANK[REG_SERVO1]; // 8-element array

// Read-only registers
volatile uint8_t *fw_version        = (uint8_t *) &REGBANK[REG_FW_VERSION]; // 2-element array
volatile uint8_t *who_am_i          = (uint8_t *) &REGBANK[REG_WHO_AM_I];


// flags
volatile uint8_t flag_servo_power = 0;
volatile bool flag_servo_init = false;

Servo servos[8];

void servos_init()
{
  servos[0].attach(PIN_SERVO1, 500, 2500);
  servos[1].attach(PIN_SERVO2, 500, 2500);
  servos[2].attach(PIN_SERVO3, 500, 2500);
  servos[3].attach(PIN_SERVO4, 500, 2500);
  servos[4].attach(PIN_SERVO5, 500, 2500);
  servos[5].attach(PIN_SERVO6, 500, 2500);
  servos[6].attach(PIN_SERVO7, 500, 2500);
  servos[7].attach(PIN_SERVO8, 500, 2500);
}

void ReceiveEvent(int bytes_received) {
  // receive value is stored to set the speed of blinking
  uint8_t reg_address = Wire.read(); // get the register offset, always first byte sent
  //Serial.print(bytes_received); Serial.print(" => "); Serial.print(int(reg_address)); Serial.println(".");
  if ((bytes_received > 1) && (reg_address < RW_REGISTERS)) {
    // this was to write data to register
    for (uint8_t i = 0; i < bytes_received - 1; i++) {
        REGBANK[reg_address + i] = Wire.read();
        //Serial.print(reg_address + i); Serial.print(": "); Serial.println(REGBANK[reg_address + i]);
    }
    // if necessary, set some flags for future processing
    if (reg_address == REG_SERVO1) {
      flag_servo_power = 1;
    } else if (reg_address == REG_SERVO2) {
      flag_servo_power = 2;
    } else if (reg_address == REG_SERVO3) {
      flag_servo_power = 3;
    } else if (reg_address == REG_SERVO4) {
      flag_servo_power = 4;
    } else if (reg_address == REG_SERVO5) {
      flag_servo_power = 5;
    } else if (reg_address == REG_SERVO6) {
      flag_servo_power = 6;
    } else if (reg_address == REG_SERVO7) {
      flag_servo_power = 7;
    } else if (reg_address == REG_SERVO8) {
      flag_servo_power = 8;
    }
  } else {
    // this was a request to read data from registers
    request_address = reg_address; // save for request handler
  }
}

void RequestEvent() 
{
  // put maximum possible number of bytes in the buffer - the master will stop transimssion
  // after reading as many as it needs
  // start at offset requestAddress - the one received from master at last transmission
  Wire.write((char *)REGBANK+request_address, MAX_TRANSMIT_SIZE);
  //Serial.print("Sent bytes starting at offset "); Serial.println(request_address);
}

void setup()
{
  //Serial.setTx(PB6); // for STM32G030
  //Serial.setRx(PB7); // for STM32G030
  //Serial.begin(115200);
  Wire.begin(DEFAULT_I2C_ADDRESS);
  // Note for CH32V003: enable I2C slave functionality in /libraries/Wire/src/utility/twi.h to prevent error message for next two lines
  Wire.onReceive(ReceiveEvent);
  Wire.onRequest(RequestEvent);

  analogWriteResolution(16);  // set PWM resolution to 16 bits, default is 8 bits

  *who_am_i = DEFAULT_I2C_ADDRESS;
  flag_servo_init = false;

  fw_version[0] = FW_VERSION_MINOR;
  fw_version[1] = FW_VERSION_MAJOR;
}

void loop()
{
  if (flag_servo_power) {
    if (!flag_servo_init) {
      servos_init();
      flag_servo_init = true;
    }

    // update servo position
    uint16_t angle = servo_power[flag_servo_power-1];
    servos[flag_servo_power-1].write(angle);
    flag_servo_power = 0;
  }
}