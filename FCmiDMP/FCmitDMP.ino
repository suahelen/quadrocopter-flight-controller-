
#include "SBUS.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Filters.h"
#include "PID_v1.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//filter vars
  FilterTwoPole Y_input_filter, Y_d_filter;
  FilterTwoPole P_input_filter, P_d_filter;
  FilterTwoPole R_input_filter, R_d_filter;



//PID Vars
  double Y_Setpoint, Y_Input, Y_Output;
  double Y_kp = 0.1, Y_ki = 0.1, Y_kd = 0.1;
  double P_Setpoint, P_Input, P_Output;
  double P_kp = 0.1, P_ki = 0.1, P_kd = 0.1;
  double R_Setpoint, R_Input, R_Output;
  double R_kp = 0.1, R_ki = 0.1, R_kd = 0.1;

  #define PID_LOOP_TIME 100

  PID Yaw_PID(   &Y_Input, &Y_Output, &Y_Setpoint, Y_kp, Y_ki, Y_kd, DIRECT, &Y_input_filter, &Y_d_filter);
  PID Pitch_PID( &P_Input, &P_Output, &P_Setpoint, P_kp, P_ki, P_kd, DIRECT, &P_input_filter, &P_d_filter);
  PID Roll_PID(  &R_Input, &R_Output, &R_Setpoint, R_kp, R_ki, R_kd, DIRECT, &R_input_filter, &R_d_filter);
//

//motor vars
  uint8_t mot1 = 20;
  uint8_t mot2 = 21;
  uint8_t mot3 = 22;
  uint8_t mot4 = 23;
  float rc = 2;                                                                // belibig
  float sup = 1.7;                                                               //0-2
  float expo = 1;                                                              //1-2

  volatile float motorout[4];

  #define max_motor  250
//

//Sbus vars
  SBUS x4R(Serial3);
  // channel, fail safe, and lost frames data
  uint16_t channels[16];
  uint8_t failSafe;
  uint16_t lostFrames = 0;

  int arm_switch;
  int chMode;
  float chThrottle;
  int start = 0;

  uint8_t Arm_Flag = 0;
  bool Throttle_High_Flag = 1;
  bool Second_Arm_Switch_Flag = 1;
  uint8_t Arm_disable = 1;
//

//MPU Vars
  MPU6050 mpu;
  #define INTERRUPT_PIN 17

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
  int16_t gx, gy, gz;


  volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  
 void dmpDataReady() {
      mpuInterrupt = true;
 }
//

void mpusetup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(30);
  mpu.setYGyroOffset(-7);
  mpu.setZGyroOffset(-44);
  mpu.setZAccelOffset(1221); // 1688 factory default for my test chip
  mpu.setFullScaleGyroRange(2);  //-> range 1000 deg/s  in 32.8 Lbs/deg/s
  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();

}

void setup() {


  Serial.begin(38400);

  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  analogWriteFrequency(20,2000);
  analogWriteResolution(12);

  mpusetup();
  x4R.begin();

  Yaw_PID.SetMode(AUTOMATIC);
  Pitch_PID.SetMode(AUTOMATIC);
  Roll_PID.SetMode(AUTOMATIC);

  Yaw_PID.SetOutputLimits(-255,255);
  Pitch_PID.SetOutputLimits(-255,255);
  Roll_PID.SetOutputLimits(-255,255);

  Yaw_PID.SetSampleTime(PID_LOOP_TIME);
  Pitch_PID.SetSampleTime(PID_LOOP_TIME);
  Roll_PID.SetSampleTime(PID_LOOP_TIME);

}

void mpu_loop() {

  if (!dmpReady) return;
  if (!mpuInterrupt && fifoCount < packetSize) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }
  mpu.getRotation(&gx, &gy, &gz);
  
}


void receiver_loop(){

  if(x4R.read(&channels[0], &failSafe, &lostFrames)) {
    arm_switch = channels[9];
    chMode = channels[4];
    chThrottle = map(channels[0], 172, 1811, 145, 250);
    
    for (int i=1; i<4; i++){
      if (abs(channels[i]- 1000) < 2) channels[i] = 1000;
    }
  }
}

void PID_loop(){
  if (Arm_Flag == 1) {
    Y_Setpoint = channels[3]-1000;                                         //>von -1 - 1  ,1 < exp < 2
    R_Setpoint = channels[1]-1000;
    P_Setpoint = channels[2]-1000;

    /*
      Y_Setpoint = (pow((((float)channels[3]-1000)/1000),expo)*pow(rc,sup))*150;                                         //>von -1 - 1  ,1 < exp < 2
      R_Setpoint = (pow((((float)channels[1]-1000)/1000),expo)*pow(rc,sup))*150;
      P_Setpoint = (pow((((float)channels[2]-1000)/1000),expo)*pow(rc,sup))*150;
    */
  }
    
  if (Arm_Flag == 0){
    Yaw_PID.lastInput = 0;
    Pitch_PID.lastInput = 0;
    Roll_PID.lastInput = 0;
    Yaw_PID.outputSum = 0;
    Pitch_PID.outputSum = 0;
    Roll_PID.outputSum = 0;
    
    Y_Input = 0; R_Input = 0;P_Input = 0;
    Y_Setpoint= 0;  R_Setpoint = 0; P_Setpoint = 0;
  } 
  Y_Input = -gz/38.4; 
  P_Input = -gy/38.4; 
  R_Input = -gx/38.4 ;

  Yaw_PID.Compute();
  Pitch_PID.Compute();
  Roll_PID.Compute();

  if (abs(Y_Output) < 0.02) Y_Output = 0;
  if (abs(P_Output) < 0.02) P_Output = 0;
  if (abs(R_Output) < 0.02) R_Output = 0;
}

void pwmRoutine() {
  if (Arm_Flag == 1){

    motorout[0] = chThrottle - Y_Output + P_Output - R_Output;//hR
    motorout[1] = chThrottle + Y_Output - P_Output - R_Output; //vR
    motorout[2] = chThrottle + Y_Output + P_Output + R_Output; //hL
    motorout[3] = chThrottle - Y_Output - P_Output + R_Output; //vL

    for (int i=0; i<4; i++){
      if (motorout[i] < 155) motorout[i] = 155;
      if (motorout[i] > max_motor) motorout[i] = max_motor;
      motorout[i] = map(motorout[i], 0, 500, 0, 4095);   
    } 
  }

  if (Arm_Flag == 0){
    for (int i=0; i<4; i++){
      motorout[i] = 125;
      motorout[i] = map(motorout[i], 0, 500, 0, 4095);
    }
  }

  analogWrite(mot1, motorout[0]);
  analogWrite(mot2, motorout[1]);
  analogWrite(mot3, motorout[2]);
  analogWrite(mot4, motorout[3]);
}

uint8_t tryarm(){

  if (arm_switch <= 1800) {
    Arm_Flag = 0;
    Throttle_High_Flag = 0;
    Second_Arm_Switch_Flag = 0;
    Arm_disable = 0;
  }

  if (arm_switch >= 1800 && !Arm_disable){
    
    if (Second_Arm_Switch_Flag == 0){
      if (channels[11] <=1800){
        Second_Arm_Switch_Flag = 1;
        Arm_disable += 1<<0 ;
      }
    }

    if (Throttle_High_Flag == 0){
      if (chThrottle >= 150) {
        Throttle_High_Flag = 1;
        Arm_disable += 1<<1 ;
      }
    }

    if (!Arm_disable) // check all arm flags if ok arm 
    {
      Arm_Flag = 1;
    }
  }
  return Arm_Flag;
}

void loop() {
  static uint32_t last_time;
  
  if (micros()-last_time > 500){
        
    receiver_loop();
    mpu_loop();
    tryarm();
    PID_loop();
    pwmRoutine();
    Serial.println();
    }
}

