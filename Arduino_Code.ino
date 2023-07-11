#include <PID_v1.h>




#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;



#define OUTPUT_READABLE_YAWPITCHROLL




#define LED_PIN 13
bool blinkState = false;


bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
const int IN1 = 9;
const int IN2 = 8;
const int ENA = 10;
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];
float PID_error;



uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

//pid control variables declaration
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 50, Ki = 0.8, Kd =80;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial);

  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again


  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {

    mpu.setDMPEnabled(true);


    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();


    dmpReady = true;


    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {

    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }



  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (ENA, OUTPUT);

  //initialize the variables we're linked to
 
  Setpoint = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {



  if (!dmpReady) return;


  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(Output);
#endif


  }


  Input = ypr[1] * 180 / M_PI;

  PID_error = Setpoint - Input ;
  //Next we calculate the error between the setpoint and the real value
  
  // Input = analogRead(PIN_INPUT);
  

  if (PID_error > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    myPID.Compute();
    analogWrite(ENA, Output);
   
  }
  if (PID_error < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    Input=-Input;
     myPID.Compute();
    analogWrite(ENA, Output);
   
  }

  




  //    if(ypr[2]*180/M_PI != 0){
  //
  //        a = 255/30*(ypr[2]*180/M_PI);
  //        if(a<0){
  //          a=-a;
  //        }else{
  //          a=a;
  //        }
  //        if(ypr[2]*180/M_PI > 30)
  //        {
  //          a=255;
  //        }
  //    }
  //
  //         //a=map(ypr[2] * 180/M_PI, -180, 180, 0, 255);
  //         digitalWrite(IN1, LOW);
  //         digitalWrite(IN2, HIGH);
  //         analogWrite(ENA,a);
  //         delay(20);



  //        for(int i=0; i<256;i++)
  //        {
  //           digitalWrite(IN1, LOW);
  //           digitalWrite(IN2, HIGH);
  //           analogWrite(ENA,i);
  //           delay(20);
  //
  //        }



}
