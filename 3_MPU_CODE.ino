/*
▪ * Team Id:  eYRC#3575
▪ * Author List: AMAN KUMAR YADAV , SATYA PRAKASH MALLICK AND SAMIKSHA SAHU 
▪ * Filename: 3_MPU_CODE
▪ * Theme: KrishiBalancer(KB)
▪ * Functions: setup() , loop() , GET_ANGLE()
▪ * Global Variables: MY_MPU , CURRENT_ANGLE , CURRENT_YAW , COS_COMPONENT_OF_YAW , SIN_COMPONENT_OF_YAW
*/ 


/*
IN THIS FILE OF CODE WE GET THE PITCH AND YAW ANGLE OF OUR BOT. AS WE ARE USING DMP LIBRARY IN THIS SO IT GIVES YAW ANGLE IN RANGE FROM [-180°  to  +180° ]. SO WHILE ROTATING THE BOT WHEN WE 
ARE GIVING COMMAND LIKE , ROTATE TO 210 DEGREE , THE MPU YAW DOES NOT UNDERSTAND THIS ANGLE AND THE BOT FALLS . TO SOLVE THIS PROBLEM WE HAVE USED ROTATION MATRIX. WE ARE CALCULATING 
THE SIN AND COS COMPONENT OF CURRENT YAW. TO MOVE TO 200 DEGREES WE ARE SETTING TARGET AS COS(200°) AND SIN(200°) THEN IT CALCULATES THE ERROR AND OUR PID SYSTEM WORKS AND MAKE BOT ROTATE 
TO 200°
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 MY_MPU;  // CREATING THE MPU OBJECT 

#define OUTPUT_READABLE_YAWPITCHROLL

volatile float CURRENT_ANGLE  = 0 ;       // THIS IS THE ANGLE(PITCH) GETTING FROM THE MPU 
volatile float CURRENT_YAW    = 0 ;       // THE CURRRENT YAW ANGLE  GETTING FROM THE MPU

volatile float COS_COMPONENT_OF_YAW = 0 ;   // COS COMPONENET OF CURRENT YAW ANGLE 
volatile float SIN_COMPONENT_OF_YAW = 0 ;   // SIN COMPONENT  OF CURRENT YAW ANGLE 

/* MPU CONFIGURATION */
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;
VectorInt16 gyro;                                // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];  

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



/*********************************************************************************************
* Function Name : GET_ANGLE
* Input :  NONE
* Output:  NONE
* Logic :  THIS FUNCTION UPDATES CURRENT ANGLE(PITCH) AND YAW ANGLE
* Example Call : GET_ANGLE()
*********************************************************************************************/
void GET_ANGLE()
{
    if (!dmpReady) return;

    if (MY_MPU.dmpGetCurrentFIFOPacket(fifoBuffer)) {

        MY_MPU.dmpGetQuaternion(&q, fifoBuffer);
        MY_MPU.dmpGetGravity(&gravity, &q);
        MY_MPU.dmpGetYawPitchRoll(ypr, &q, &gravity);
        MY_MPU.dmpGetGyro(&gyro, fifoBuffer);

        CURRENT_ANGLE = ypr[1] * 180.0 / M_PI ;      //  COVERTED IT INTO DEGREES 
        CURRENT_YAW   = ypr[0]  ;  
        COS_COMPONENT_OF_YAW = cos(CURRENT_YAW ) ;   // COS COMPONENT OF CURRENT YAW ANGLE 
        SIN_COMPONENT_OF_YAW = sin(CURRENT_YAW)  ;   // SIN COMPONENT OF CURRENT YAW ANGLE 
       
    }  
}



/*********************************************************************************************
* Function Name : setup
* Input :  NONE
* Output:  NONE
* Logic :  TO INITALIZE ALL THE PARAMETERS ONCE 
* Example Call : setup() - CALLED INTERNALLY 
*********************************************************************************************/
void setup() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);    //  TO INCREASE COMMUNICATION SPEED BETWEEN ARDUINO AND MPU SENSOR 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

Serial.begin(9600);

// MPU INITALIZATION AND CONFIGURATION 
    MY_MPU.initialize();
    MY_MPU.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    MY_MPU.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    MY_MPU.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    MY_MPU.setDLPFMode(MPU6050_DLPF_BW_42);

    devStatus = MY_MPU.dmpInitialize();

// HERE WE CALIBERATED OUR MPU SUCH THAT IT GIVES PITCH 0 DEGREE WHEN OUR BOT IS UPRIGHT 
    MY_MPU.setXAccelOffset(-1580);
    MY_MPU.setYAccelOffset(85);
    MY_MPU.setZAccelOffset(934);

    MY_MPU.setXGyroOffset(97);
    MY_MPU.setYGyroOffset(-15);
    MY_MPU.setZGyroOffset(21);

    if (devStatus == 0) {
        MY_MPU.setDMPEnabled(true);
        dmpReady = true;
        packetSize = MY_MPU.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP init failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        while (1); 
    }
}




/*********************************************************************************************
* Function Name : loop
* Input :  NONE
* Output:  NONE
* Logic :  CALLS GET_ANGLE() REPEATEDLY
* Example Call : loop() - AUTOMATICALLY CALLED INTERNALLY
*********************************************************************************************/
void loop(){    
  
GET_ANGLE() ; 


}