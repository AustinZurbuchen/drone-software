#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // LED pin for arduino is 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float eular[3];
float ypr[3];
float pOffset = -9.40;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// DECLARE ARDUINO PINS
int frontRight = 3;
int backRight = 4;
int backLeft = 5;
int frontLeft = 6;
int flMotorPin = 11;
int frMotorPin = 10;
int blMotorPin = 9;
int brMotorPin = 8;

// INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// INITIAL SETUP
void setup() {
  // join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (115200 chosen bcause it is required for Teapot Demo output)
  Serial.begin(115200);
  while (!Serial);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
//  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  while (Serial.available() && Serial.read());  // empty buffer
//  while (!Serial.available());                  // wait for data
//  while (Serial.available() && Serial.read());  // empty buffer again  

  // load and configure the DMP
  Serial.println(F("Intializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // gyro offsets here
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, not that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(frontRight, OUTPUT);
  pinMode(frontLeft, OUTPUT);
  pinMode(backRight, OUTPUT);
  pinMode(backLeft, OUTPUT);

  pinMode(flMotorPin, OUTPUT);
  pinMode(frMotorPin, OUTPUT);
  pinMode(blMotorPin, OUTPUT);
  pinMode(brMotorPin, OUTPUT);
}

// MAIN PROGRAM LOOP
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // test in between other stuff to see if mpuInterrupt is true, and if so, "break;" 
    // from the while() loop to immediately process the MPU data
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too ineffienct)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  // check for DMP data ready interrupt
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // read more without waiting for interrupt
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // diplay Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      float yaw = ypr[0] * 180/M_PI;
      float pitch = (ypr[1] * 180/M_PI) - pOffset;
      float roll = ypr[2] * 180/M_PI;

      if (pitch >= 10 && abs(pitch) >= abs(roll)) {
        pitchingUp();
      } else if (pitch <= -10 && abs(pitch) >= abs(roll)){
        pitchingDown();
      } else if (roll >= 10 && abs(roll) >= abs(pitch)) {
        rollingRight();
      } else if (roll <= -10 && abs(roll) >= abs(pitch)) {
        rollingLeft();
      } else {
        level();
      }
      Serial.print("yaw\t");
      Serial.print(yaw);
      Serial.print("\tpitch\t");
      Serial.print(pitch);
      Serial.print("\troll\t");
      Serial.println(roll);
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void pitchingUp() {
  Serial.print("Pitching up!!\t");
  digitalWrite(backRight, HIGH);
  digitalWrite(backLeft, HIGH);
  digitalWrite(frontRight, LOW);
  digitalWrite(frontLeft, LOW);
  digitalWrite(flMotorPin, LOW);
  digitalWrite(frMotorPin, LOW);
  digitalWrite(blMotorPin, HIGH);
  digitalWrite(brMotorPin, HIGH);
}

void pitchingDown() {
  Serial.print("Pitching down!!\t");
  digitalWrite(frontRight, HIGH);
  digitalWrite(frontLeft, HIGH);
  digitalWrite(backRight, LOW);
  digitalWrite(backLeft, LOW);
  digitalWrite(flMotorPin, HIGH);
  digitalWrite(frMotorPin, HIGH);
  digitalWrite(blMotorPin, LOW);
  digitalWrite(brMotorPin, LOW);
}

void rollingRight() {
  Serial.print("Rolling right!!\t");
  digitalWrite(frontRight, HIGH);
  digitalWrite(backRight, HIGH);
  digitalWrite(frontLeft, LOW);
  digitalWrite(backLeft, LOW);
  digitalWrite(flMotorPin, LOW);
  digitalWrite(frMotorPin, HIGH);
  digitalWrite(blMotorPin, LOW);
  digitalWrite(brMotorPin, HIGH);
}

void rollingLeft() {
  Serial.print("Rolling left!!\t");
  digitalWrite(frontLeft, HIGH);
  digitalWrite(backLeft, HIGH);
  digitalWrite(frontRight, LOW);
  digitalWrite(backRight,LOW);
  digitalWrite(flMotorPin, HIGH);
  digitalWrite(frMotorPin, LOW);
  digitalWrite(blMotorPin, HIGH);
  digitalWrite(brMotorPin, LOW);
}

void level() {
  Serial.print("Level!!\t");
  digitalWrite(frontLeft, LOW);
  digitalWrite(backLeft, LOW);
  digitalWrite(frontRight, LOW);
  digitalWrite(backRight,LOW);
  digitalWrite(flMotorPin, HIGH);
  digitalWrite(frMotorPin, HIGH);
  digitalWrite(blMotorPin, HIGH);
  digitalWrite(brMotorPin, HIGH);
}
