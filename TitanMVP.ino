
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;        // quaternion container [w, x, y, z]
VectorInt16 aa;      // accel sensor measurements [x, y, z]
VectorInt16 aaReal;  // gravity-free accel sensor measurements [x, y, z]
VectorInt16 aaWorld; // world-frame accel sensor measurements [x, y, z]
VectorFloat gravity; // gravity vector [x, y, z]
float euler[3];      // Euler angle container [psi, theta, phi]
float ypr[3];        // yaw/pitch/roll container and gravity vector [yaw, pitch, roll]

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

void setup() {
  Serial.begin(115200);
  pinMode(8, OUTPUT);

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();

  // MPU6050 calibration
  mpu.setXGyroOffset(58);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(54);
  mpu.setXAccelOffset(-2773);
  mpu.setYAccelOffset(-1424);
  mpu.setZAccelOffset(1250);

  // make sure DMP initialized (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  if (!dmpReady)
    return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // calculations below:
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // code plan
    // we want to calculate when the user is ready to throw a punch
    // using x rotation (roll) we can determine if the users hands are
    // horizontal enough to throw a punch then we calculate acceleration
    // roll data  +- 90 degrees

    int roll = ypr[2] * 180 / M_PI;
    if (roll >= 60) {
      // Serial.println("High guard");
    } else if (roll > 0 && roll <= 60) {
      // Serial.println("Mid guard");
      Serial.println(aaWorld.y);
    } else {
      // Serial.println("Hands are down");
    }
  }
}
