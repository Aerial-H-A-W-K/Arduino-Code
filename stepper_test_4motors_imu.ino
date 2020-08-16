#include <PPMReader.h>
#include <Wire.h>
#include <TimerOne.h>

// Stepper motors are labelled as such (top view)
// front
// 1  4
// 2  3

// Sets how fast to reel in or out the bridle lines
//#define PERIOD_MIN 350 // Fastest acceleration (not required)
#define PERIOD_MAX 25 // Slowest acceleration

//Offset to zero position
#define CH1_OFFSET 0
#define CH2_OFFSET 0
#define CH3_OFFSET 0
#define CH4_OFFSET 0

// IMU defintions
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
//#define    GYRO_FULL_SCALE_250_DPS    0x00  
//#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
//#define    GYRO_FULL_SCALE_2000_DPS   0x18
//#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
//#define    ACC_FULL_SCALE_8_G        0x10
//#define    ACC_FULL_SCALE_16_G       0x18

int interruptPin = 2; // Assign the number of pins needed for the number of channels used
int channelAmount = 2; //Two channels for pitch and roll
PPMReader ppm(interruptPin, channelAmount);

// Instantiate variables to keep track of the current position of each of the 4 stepper motors
int stepper1_current_pos = 0;
int stepper2_current_pos = 0;
int stepper3_current_pos = 0;
int stepper4_current_pos = 0;

// Instantiate variables to keep track of the desired position of each of the 4 stepper motors
int stepper1_desired_pos = 0;
int stepper2_desired_pos = 0;
int stepper3_desired_pos = 0;
int stepper4_desired_pos = 0;

// Instantiate variables to keep track of how much each motor needs to rotate to achieve its desired position
int stepper1_diff = 0;
int stepper2_diff = 0;
int stepper3_diff = 0;
int stepper4_diff = 0;

// Sets the period of signal output to motors
int stepper1_period_curr = PERIOD_MAX;
int stepper2_period_curr = PERIOD_MAX;
int stepper3_period_curr = PERIOD_MAX;
int stepper4_period_curr = PERIOD_MAX;

// Put read bytes starting at register (Register) in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

// Arduino initializations
void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(200);
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);

  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS, 28, GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS, 27, ACC_FULL_SCALE_4_G);
}

volatile bool intFlag=false;
void loop() {
  while (intFlag);
  
  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  // Gyroscope
  int16_t ay=-(Buf[0]<<8 | Buf[1]);
  int16_t ax=-(Buf[2]<<8 | Buf[3]);
  //Serial.print (ax,DEC); 
  //Serial.print ("\t");
  //Serial.print (ay,DEC);
  //Serial.print ("\t");
  //Serial.println("");

  // Update desired position
  stepper1_desired_pos = CH1_OFFSET + 0.25*(ax+ay);
  stepper2_desired_pos = CH2_OFFSET + 0.25*(-ax+ay);
  stepper3_desired_pos = CH3_OFFSET + 0.25*(ax-ay);
  stepper4_desired_pos = CH4_OFFSET + 0.25*(-ax-ay);
  
  // Reads the inputs from the radio controller
  for (int channel = 1; channel <= channelAmount; ++channel) {
    unsigned long value = ppm.latestValidChannelValue(channel, 0);
    stepper1_desired_pos += map(value,1000,2000,-1000,1000);
    stepper4_desired_pos -= map(value,1000,2000,-1000,1000);
    if (channel == 1) {
      stepper2_desired_pos -= map(value,1000,2000,-1000,1000);
      stepper3_desired_pos += map(value,1000,2000,-1000,1000);
    }
    else {
      stepper2_desired_pos += map(value,1000,2000,-1000,1000);
      stepper3_desired_pos -= map(value,1000,2000,-1000,1000);
    }
  }
 
  stepper1_diff = stepper1_desired_pos - stepper1_current_pos;
  stepper2_diff = stepper2_desired_pos - stepper2_current_pos;
  stepper3_diff = stepper3_desired_pos - stepper3_current_pos;
  stepper4_diff = stepper4_desired_pos - stepper4_current_pos;

  // Each of the motors has a dead zone, where the motors would not move.
  // This value is set between -200 to 200. Within this range, the desired_pos will be 0.
  // If >200, the motors will turn clockwise. If <-200, the motors will turn counter clockwise.

  // Stepper1
  if (stepper1_diff > 200){
    digitalWrite(10, HIGH); // Direction pin, positive (HIGH) direction. This applies to all the 4 motors.
    stepper1_period_curr = PERIOD_MAX;
    // Pulse to move stepper motor
    digitalWrite(11,HIGH); 
    delayMicroseconds(stepper1_period_curr);
    digitalWrite(11,LOW);
    delayMicroseconds(stepper1_period_curr);
    stepper1_current_pos += digitalRead(10)*2-1; // Updates current position
  }
  if (stepper1_diff < -200){
    digitalWrite(10, LOW); // Direction pin, negative (LOW) direction. This applies to all the 4 motors.
    stepper1_period_curr = PERIOD_MAX;
    // Pulse to move stepper motor
    digitalWrite(11,HIGH); 
    delayMicroseconds(stepper1_period_curr);
    digitalWrite(11,LOW);
    delayMicroseconds(stepper1_period_curr);
    stepper1_current_pos += digitalRead(10)*2-1; // Updates current position
  }

  // Stepper2
  if (stepper2_diff > 200){
    digitalWrite(8, HIGH); 
    stepper2_period_curr = PERIOD_MAX;
    // Pulse to move stepper motor
    digitalWrite(9,HIGH); 
    delayMicroseconds(stepper2_period_curr);
    digitalWrite(9,LOW);
    delayMicroseconds(stepper2_period_curr);
    stepper2_current_pos += digitalRead(8)*2-1; // Updates current position
  }
  if (stepper2_diff < -200){
    digitalWrite(8, LOW);
    stepper2_period_curr = PERIOD_MAX;
    // Pulse to move stepper motor
    digitalWrite(9,HIGH); 
    delayMicroseconds(stepper2_period_curr);
    digitalWrite(9,LOW);
    delayMicroseconds(stepper2_period_curr);
    stepper2_current_pos += digitalRead(8)*2-1; // Updates current position
  }

  // Stepper3
  if (stepper3_diff > 200){
    digitalWrite(6, HIGH);
    stepper3_period_curr = PERIOD_MAX;
    // Pulse to move stepper motor
    digitalWrite(7,HIGH); 
    delayMicroseconds(stepper3_period_curr);
    digitalWrite(7,LOW);
    delayMicroseconds(stepper3_period_curr);
    stepper3_current_pos += digitalRead(6)*2-1; // Updates current position
  }
  if (stepper3_diff < -200){
    digitalWrite(6, LOW);
    stepper3_period_curr = PERIOD_MAX;
    // Pulse to move stepper motor
    digitalWrite(7,HIGH); 
    delayMicroseconds(stepper3_period_curr);
    digitalWrite(7,LOW);
    delayMicroseconds(stepper3_period_curr);
    stepper3_current_pos += digitalRead(6)*2-1; // Updates current position
  }

  // Stepper4
  if (stepper4_diff > 200){
    digitalWrite(4, HIGH);
    stepper4_period_curr = PERIOD_MAX;
    // Pulse to move stepper motor
    digitalWrite(5,HIGH); 
    delayMicroseconds(stepper4_period_curr);
    digitalWrite(5,LOW);
    delayMicroseconds(stepper4_period_curr);
    stepper4_current_pos += digitalRead(4)*2-1; // Updates current position
  }
  if (stepper4_diff < -200){
    digitalWrite(4, LOW);
    stepper4_period_curr = PERIOD_MAX;
    // Pulse to move stepper motor
    digitalWrite(5,HIGH); 
    delayMicroseconds(stepper4_period_curr);
    digitalWrite(5,LOW);
    delayMicroseconds(stepper4_period_curr);
    stepper4_current_pos += digitalRead(4)*2-1; // Updates current position
  }
}
