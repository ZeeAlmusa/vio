
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 18 on Arduino mega & most boards


namespace IMU {


  MPU6050 mpu;
  float correct;
  int j = 0;
  bool blinkState = false;
  // MPU control/status vars
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  
  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
 
  
  //my time variables, this is used for the linear compensator of yaw drift
  float new_time = 0;
  float old_time = 0;
  float dt = 0;
  
  void dmpDataReady() {
    mpuInterrupt = true;
  
  
  }

// ================================================================
// ===                         IMU SETUP                        ===
// ================================================================

  
  void IMU_setup() {
  
  
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    // Initialize I2C communications as Slave
    Wire.begin();
    // Function to run when data received from master
    //  Wire.onReceive(receiveEvent);
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
  
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(83);
    mpu.setYGyroOffset(-36);
    mpu.setZGyroOffset(38);
    mpu.setZAccelOffset(1520); // 1688 factory default for my test chip
  
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      // Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
  
      //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      //mpuIntStatus = mpu.getIntStatus();
  
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      //Serial.println(F("DMP ready! Waiting for first interrupt..."));
      //dmpReady = true;
  
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      // Serial.print(F("DMP Initialization failed (code "));
      //Serial.print(devStatus);
      //Serial.println(F(")"));
    }
    fifoCount = mpu.getFIFOCount();
  }
  
// ================================================================
// ===                         IMU UPDATE                       ===
// ================================================================

//This is the main looping function, it calculates yaw, pitch, roll.
//The drift in the yaw is compensated for removing a specific value linearly with time
  void find_orientation() {
    new_time = millis(); //I added this time
    dt = new_time - old_time;
    while (fifoCount < packetSize) {
  
      fifoCount = mpu.getFIFOCount();
  
    }
  
    if (fifoCount == 1024) {
  
      mpu.resetFIFO();
      
      
    }
    else {
  
      if (fifoCount % packetSize != 0) {
  
        mpu.resetFIFO();
  
      }
      else {
  
        while (fifoCount >= packetSize) {
  
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;
  
        }
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
        ypr[0] = ypr[0] * 180 / M_PI;
        ypr[1] = ypr[1] * 180 / M_PI;
        ypr[2] = ypr[2] * 180 / M_PI;
  
        if (j <= 300) {
          correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
          j++;
        }
        // After 300 readings
        else {
          ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees
          ypr[0] = ypr[0] - (-0.005 * dt); //Linear compensation for yaw drift
        }
  
      }
  

    }
  
    old_time = new_time;
  
  }
  
  


// ================================================================
// ===                      GETTER METHODS                      ===
// ================================================================

//Linear acceleration in the x direction [m/s^2]
  float getAx() {
    return (float)aaReal.x / 8192.0 * 9.806;
  }

//Linear acceleration in the y direction [m/s^2]
  float getAy() {
    return (float)aaReal.y / 8192.0 * 9.806;
  }

//Linear acceleration in the z direction [m/s^2]
  float getAz() {
    return (float)aaReal.y / 8192.0 * 9.806;
  }

//Yaw in degrees 
  float get_yaw(){
    return ypr[0];
  }
  
//Pitch in degrees 
   float get_pitch(){
    return ypr[1];
  }

//Roll in degrees 
   float get_roll(){
    return ypr[2];
  }


// ================================================================
// ===                      PRINT METHODS                       ===
// ================================================================
   //print all yaw, pitch, roll in degrees
  void printOrientation() {
  
    Serial.print(ypr[0]);
    Serial.print('/');
    Serial.print(ypr[1]);
    Serial.print('/');
    Serial.println(ypr[2]);
  }
  
  //Print yaw in degrees
  void printYaw() {
    Serial.print(millis());
    Serial.print(" ");
    Serial.println(ypr[0]);
  }
  
  //This returns linear acceleration in m/s^2
  void printLinearAcc() {
    Serial.print((float)aaReal.x / 8192.0 * 9.806);
    Serial.print(' ');
    Serial.print((float)aaReal.y / 8192.0 * 9.806);
    Serial.print(' ');
    Serial.println((float)aaReal.z / 8192.0 * 9.806);
  
  }


} ;
