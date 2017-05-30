
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


//Quad constants

int m = 0.4;                               //mass in gramms
int c_f =  0;                               //force constant in N/(pulse length)
int c_t =  0;                               //torque constant in Nm/(pulse length)
int c_acc = 0;                              //acceleration constant (multiply with mpu signal)


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID angle gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 0.4;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 10;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 6.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error, yaw_vel;


// start
int start = 0;

// timing
long int loop_timer;
long int loop_length = 4000;
volatile int ISR_timer;
volatile int channel_timer[4];

volatile bool channelbit[4];
long int ESC_timer[4];
long int ESC_loop_timer;


//control vars
volatile int channel_in[4];
int motor_in[4];
bool motor_flag[4] = {0, 0, 0, 0};




// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars

int16_t ax, ay, az;
int16_t gx, gy, gz;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool blinkState = true;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);   
 
    mpu.initialize();
    
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);                                                                                //adapt
    mpu.setYGyroOffset(76);                                                                                 //adapt
    mpu.setZGyroOffset(-85);                                                                                //adapt
    mpu.setZAccelOffset(1788);                                                                              //adapt

    
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }   
    DDRD |= B11110000;   
    loop_timer = micros();

    PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
     
}
   
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    if (!dmpReady){
      //return;
    }

   while (!mpuInterrupt && fifoCount < packetSize) {
  

  
    if (start == 0 && channel_in[2] <= 1050 && channel_in[3] >= 1800) start = 1;
    if (start == 1 && channel_in[2] < 1050 && channel_in[3] < 1450) {
      start = 2;
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
   }
   if (start == 2 && channel_in[2] <= 1050 && channel_in[3] >= 1050) start = 0;
  
   if (start == 2){
     //The PID set point in degrees per second is determined by the roll receiver input.
     //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
     pid_roll_setpoint = 0;
     //We need a little dead band of 16us for better results.
     if(channel_in[1] > 1508)pid_roll_setpoint = (channel_in[1] - 1508)/6.0;
     else if(channel_in[1] < 1492)pid_roll_setpoint = (channel_in[1] - 1492)/6.0;
     else if(channel_in[1] > 1492 && channel_in[1] < 1508) {
       pid_roll_setpoint = asin((cos(ypr[1])/(m * c_acc * aaWorld.y))*(c_f *(motor_in[0] + motor_in[1] + motor_in[2] + motor_in[3])));
     }
  
     //The PID set point in degrees per second is determined by the pitch receiver input.
     //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
     pid_pitch_setpoint = 0;
     //We need a little dead band of 16us for better results.
     if(channel_in[0] > 1508)pid_pitch_setpoint = (channel_in[0] - 1508)/2.0;
     else if(channel_in[0] < 1492)pid_pitch_setpoint = (channel_in[0] - 1492)/2.0;
     else if(channel_in[0] > 1492 && channel_in[0] < 1508) {
       pid_pitch_setpoint = -asin((cos(ypr[2])/(m * c_acc * aaWorld.x))*(c_f *(motor_in[0] + motor_in[1] + motor_in[2] + motor_in[3])));
     }

     //The PID set point in degrees per second is determined by the yaw receiver input.
     //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ). 
     pid_yaw_setpoint = 0;
     //We need a little dead band of 16us for better results.
     if(channel_in[2] > 1050){ //Do not yaw when turning off the motors.
       if(channel_in[3] > 1508)pid_yaw_setpoint = (channel_in[3] - 1508)/6.0;
       else if(channel_in[3] < 1492)pid_yaw_setpoint = (channel_in[3] - 1492)/6.0;
     }
     
     
     calculations();
           
     motor_in[0] = channel_in[2] + pid_output_pitch + pid_output_roll - pid_yaw_setpoint;
     motor_in[1] = channel_in[2] - pid_output_pitch + pid_output_roll + pid_yaw_setpoint;
     motor_in[2] = channel_in[2] - pid_output_pitch - pid_output_roll - pid_yaw_setpoint;
     motor_in[3] = channel_in[2] + pid_output_pitch - pid_output_roll + pid_yaw_setpoint;
 
     if (motor_in[0] < 1000) motor_in[0] = 1000;
     if (motor_in[1] < 1000) motor_in[1] = 1000;
     if (motor_in[2] < 1000) motor_in[2] = 1000;
     if (motor_in[3] < 1000) motor_in[3] = 1000;

     if (motor_in[0] > 2000) motor_in[0] = 2000;
     if (motor_in[1] > 2000) motor_in[1] = 2000;
     if (motor_in[2] > 2000) motor_in[2] = 2000;
     if (motor_in[3] > 2000) motor_in[3] = 2000;

     if (channel_in[2] <= 1050){
       motor_in[0] = 1000;
       motor_in[1] = 1000;
       motor_in[2] = 1000;
       motor_in[3] = 1000;
     }
              
     while (micros() - loop_timer <= 4000);
     loop_length = micros - loop_timer;
     loop_timer = micros();

     PORTD |= B11110000;
     ESC_timer[0] = motor_in[0] + loop_timer;
     ESC_timer[1] = motor_in[1] + loop_timer;
     ESC_timer[2] = motor_in[2] + loop_timer;
     ESC_timer[3] = motor_in[3] + loop_timer;

     while (PORTD >=16){
       ESC_loop_timer = micros();
       if (ESC_loop_timer >= ESC_timer[0])  PORTD &= B11101111;
       if (ESC_loop_timer >= ESC_timer[1])  PORTD &= B11011111;
       if (ESC_loop_timer >= ESC_timer[2])  PORTD &= B10111111;
       if (ESC_loop_timer >= ESC_timer[3])  PORTD &= B01111111;
     }   
   }    
   mpuInterrupt = false;
   mpuIntStatus = mpu.getIntStatus();

   fifoCount = mpu.getFIFOCount();

   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();

   } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
       
 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);                                                   
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   }

 }
    
}
//////////////////////////////////////////////////////
//ISR
/////////////////////////////////////////////////////
ISR(PCINT0_vect){
 ISR_timer = micros();
 if (PINB & B00000001){
  if (channelbit[0] == 0 ){
    channelbit[0] = 1;
    channel_timer[0] = ISR_timer;
  }
 }
 else if(channelbit[0] == 1){
  channelbit[0] = 0;
  channel_in[0] = ISR_timer - channel_timer[0];
 }
///////////////////////////////////////////////////////
 if (PINB & B00000010){
  if (channelbit[1] == 0 ){
    channelbit[1] = 1;
    channel_timer[1] = ISR_timer;
  }
 }
 else if(channelbit[1] == 1){
  channelbit[1] = 0;
  channel_in[1] = ISR_timer - channel_timer[1];
 }
///////////////////////////////////////////////////////
 if (PINB & B00000100){
  if (channelbit[2] == 0 ){
    channelbit[2] = 1;
    channel_timer[2] = ISR_timer;
  }
 }
 else if(channelbit[2] == 1){
  channelbit[2] = 0;
  channel_in[2] = ISR_timer - channel_timer[2];
 }
///////////////////////////////////////////////////////

 if (PINB & B00001000){
  if (channelbit[3] == 0 ){
    channelbit[3] = 1;
    channel_timer[3] = ISR_timer;
  }
 }
 else if(channelbit[3] == 1){
  channelbit[3] = 0;
  channel_in[3] = ISR_timer - channel_timer[3];
 }
}
///////////////////////////////////////////////////////

void calculations() {
    //Roll calculations
  pid_error_temp = ypr[2]*180 - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = ypr[1]*180 - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  yaw_vel = gz*cos(ypr[1])*cos(ypr[2]) + gx*sin(ypr[1]) + gy*sin(ypr[2]);
  pid_error_temp = yaw_vel - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

