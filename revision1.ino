
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
// Include header for custom datatypes and function prototypes 
#include "includes.h"

// Include the Arduino PID library
#include <PID_v1.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 mpu;

/* =========================================================================
NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
depends on the MPU-6050's INT pin being connected to the Arduino's
external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
digital I/O pin 2.
* ========================================================================= */



// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL




#define LED_PIN 13 
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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// PID variables
double Setpoint, Input, Output;
unsigned long serialTime_FrontEnd; //this will help us know when to talk with processing
double aggKp=130, aggKi=22.0, aggKd=2.4;
double consKp=55.0, consKi=50.00, consKd=1.5;

//Specify the links and initial tuning parameters for PID
PID myPID(&Input, &Output, &Setpoint,consKp,consKi,consKd, DIRECT);





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
  
        //Initialize PID parameters
        Setpoint = 0.0;	
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(-255.0,255.0);
        myPID.SetSampleTime(20);
        pinMode(rightMotorA, OUTPUT);
        pinMode(rightMotorB , OUTPUT);
        pinMode(leftMotorA ,  OUTPUT);
        pinMode(leftMotorB , OUTPUT);
        // join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();

	// initialize serial communication
	Serial.begin(115200);

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// wait for ready
	//Serial.println(F("\nSend any character to begin DMP programming and demo: "));
	while (Serial.available() && Serial.read()); // empty buffer
	//while (!Serial.available());                 // wait for data
	//while (Serial.available() && Serial.read()); // empty buffer again

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
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
	} 
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

        

        // wait 8 seconds for DMP to stabilize
        Serial.println("Waiting for DMP to stabilize...");
        for(int i=0; i<70; i++)
        {
          Serial.print(i+1);
          Serial.print("..."); 
          Serial.println(mpu.getFIFOCount());
          
          delay(100);
          mpu.resetFIFO();
                    
          blinkState = !blinkState;
	  digitalWrite(LED_PIN, blinkState);
                    
        }
        delay(10);
        // configure LED for output
	pinMode(LED_PIN, OUTPUT);
           
}


long time = millis();
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
	// if programming failed, don't try to do anything
	if (!dmpReady) return;
        
        
//        Serial.println("loop starts, last loop time :");
//        Serial.println(millis() - time);
//        Serial.println(fifoCount);
//        time = millis();
//	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {

                Input = ypr[1] * 180/M_PI;

//                 double gap = abs(Setpoint-Input); //distance away from setpoint
//                  if(gap<1.4)
//                  {  //we're close to setpoint, use conservative tuning parameters
//                    myPID.SetTunings(consKp, consKi, consKd);
//                  }
//                  else
//                  {
//                     //we're far from setpoint, use aggressive tuning parameters
//                     myPID.SetTunings(aggKp, aggKi, aggKd);
//                  }
//  
                if(abs(Input) > KILLZONE)
                {
                  myPID.SetMode(MANUAL);
                  //Serial.println("bang");
                  Output = 0;
                }
                else
                {
                  myPID.SetMode(AUTOMATIC);
                  //Serial.println("BOOYAH");
                  myPID.Compute();
                }
                                  
                               
                if(Output == 0 )
                {
                  
                  driveMotor(leftMotor, still, abs(Output));
                  driveMotor(rightMotor, still, abs(Output));
                }
                if(Output > 0.0)                
                {
                  //Serial.println("backwARD TIME");
                   driveMotor(leftMotor, forward, Output+ motorOffset);
                   driveMotor(rightMotor, forward, Output - motorOffset);
                 }                 
                 if(Output < 0.0)
                 {
                  //Serial.println("FORWARD TIME");
                   driveMotor(leftMotor, backward, -1.0*(Output) + motorOffset );
                  driveMotor(rightMotor, backward, -1.0*(Output) - motorOffset);
                 }
   
                   
                if(millis()>serialTime_FrontEnd)
                {
                  SerialReceive();
                  SerialSend();
                  serialTime_FrontEnd+=200;
                }
                    
    
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
	} 
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//		Serial.print("ypr\t");
//		Serial.print(ypr[0] * 180/M_PI);
//		Serial.print("\t");
//		Serial.print(ypr[1] * 180/M_PI);
//		Serial.print("\t");
//		Serial.println(ypr[2] * 180/M_PI);
#endif


		// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}
}


void driveMotor(Motor motor, Direction dir, double pwm)
{
	int dirPin, pwmPin;
	if(motor == leftMotor)
	{
		dirPin = leftMotorA;
		pwmPin = leftMotorB;
	}
	else if(motor == rightMotor)
	{
		dirPin = rightMotorA;
		pwmPin = rightMotorB;
	}
	else return;

	pwm = round(constrain(pwm,0.0,255.0));

	if(dir == forward)
	{
		digitalWrite(dirPin, LOW);
		analogWrite(pwmPin, (int)pwm);
	}
	else if(dir == backward)
	{
		digitalWrite(dirPin, HIGH);
		analogWrite(pwmPin, 255-(int)pwm);
	} 
        else if(dir == still)
        {
          digitalWrite(dirPin,LOW);
          analogWrite(pwmPin, 0);
        }
}

