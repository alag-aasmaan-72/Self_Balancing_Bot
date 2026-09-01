/*
▪ * Functions: setup() , loop() , 
▪ * Global Variables: ARM , GRIPPER , state , lastMoveTime , lastMoveTime1 , lastMoveTime2 , lastMoveTime3 , lastMoveTime4
                      lastMoveTime5 , lastMoveTime6 , interval , servoPos_1 , servoPos_2 , servoPos_3 , servoPos_4 , servoPos_5
                      servoPos_6 , x , y , z , p , l , r 
*/ 

#include <Servo.h>  

Servo  ARM;        // ARM - UP AND DOWN MOTION 
Servo  GRIPPER;    // GRIPPER - OPEN AND CLOSE MOTION 




/**********************************************************************************/
//state = 0 → Idle
//state = 1 → Pick up sequence
//state = 2 → Drop down sequence                       
int   state = 0;  
/************************************************************************************/



/***********************************************************************************/
//TIME CONTROL VARIABLES FOR NON-BLOCKING SERVO MOVEMENT
//These store the last time each servo was moved.
//Used with millis() to create smooth motion without delay()
unsigned long lastMoveTime =  0; 
unsigned long lastMoveTime1 = 0;
unsigned long lastMoveTime2 = 0;
unsigned long lastMoveTime3 = 0;
unsigned long lastMoveTime4 = 0;
unsigned long lastMoveTime5 = 0;
unsigned long lastMoveTime6 = 0;
/************************************************************************************/

const unsigned long interval= 5;    // Servo moves every 5 milliseconds(The servo updates its position once every 5 ms.)


int servoPos_1 =  170 ;  // ARM initial UP position (Pick sequence start)
int servoPos_2 =  0;     // GRIPPER initial OPEN position
int servoPos_3 =  145 ;  // ARM DOWN position before lifting block
int servoPos_4 = 170;    // ARM UP position (Drop sequence start)
int servoPos_5 = 180 ;   // GRIPPER CLOSED position (Holding block)
int servoPos_6 =  145;   // ARM DOWN position before releasing block




// PICK UP SEQUENCE CONTROL FLAGS
int x = 1;  
// FLAG to control ARM downward motion during PICK sequence
// x = 1 ---> ARM moves down
// x = 0 ---> ARM reached lower limit, stop ARM 

int y = 1;  
// FLAG to control GRIPPER closing during PICK sequence
// y = 0 ---> GRIPPER starts closing
// y = 1 ---> GRIPPER fully closed, stop GRIPPER 

int z = 1;  
// FLAG to control ARM upward motion after grabbing block
// z = 0 ---> ARM lifts block
// z = 1 ---> ARM reached upper limit, stop ARM || PICK sequence completed



// DROP DOWN SEQUENCE CONTROL FLAGS
int p = 1;  
// FLAG to control ARM downward motion during DROP sequence
// p = 1 ---> ARM moves down
// p = 0 ---> ARM reached lower limit, stop ARM 

int l = 1;  
// FLAG to control GRIPPER opening during DROP sequence
// l = 0 ---> GRIPPER starts opening
// l = 1 ---> GRIPPER fully opened, stop GRIPPER 

int r = 1;  
// FLAG to control ARM upward motion after releasing block
// r = 0 ---> ARM moves up
// r = 1 ---> ARM reached upper limit, stop ARM || DROP sequence completed  



/*********************************************************************************************
* Function Name : setup
* Input :  NONE
* Output:  NONE
* Logic :  TO INITALIZE ALL THE PARAMETERS ONCE 
* Example Call : setup() - CALLED INTERNALLY 
*********************************************************************************************/
void setup(){
    ARM.attach(10);
    GRIPPER.attach(11);

    ARM.write(170);
    GRIPPER.write(0);
}



/*********************************************************************************************
* Function Name : loop
* Input :  NONE
* Output:  NONE
* Example Call : loop() - AUTOMATICALLY CALLED INTERNALLY
*********************************************************************************************/
void loop(){


/****************************************************************************/

//PICK UP SEQUENCE
//state = 1 triggers pick operation

  if (state == 1) {   
    
  unsigned long currentTime  = millis();   // Get current system time


// ARM MOVES DOWN TO REACH THE BLOCK

  if ((currentTime - lastMoveTime >= interval) &&  (x==1)) {
    lastMoveTime = currentTime;
    servoPos_1 = servoPos_1 - 1;  // Move ARM down slowly

    if (servoPos_1 <= 145) {     // Stop when lower limit reached
      servoPos_1 = 145 ;
      x = 0 ;         // Stop ARM down motion
      y = 0 ;         // Start GRIPPER closing
    }
    ARM.write(servoPos_1) ; 
  }
/*******************************************************************************/




/******************************************************************************/
//GRIPPER CLOSES TO GRAB THE BLOCK 

  if(y==0)  {

    unsigned long currentTime1 = millis();
    if ((currentTime1 - lastMoveTime1 >= interval) ) {
    lastMoveTime1 = currentTime1;

    servoPos_2 = servoPos_2 + 1;  // Close GRIPPER slowly

    if (servoPos_2 >= 180) {       // Fully closed (stop the gripper )
      servoPos_2 = 180 ;

      y=1 ;            // Stop gripper closing mechanism
      z=0 ;            // Start ARM lifting
     
    }
    GRIPPER.write(servoPos_2);   
  }
  }
/*******************************************************************************/




/******************************************************************************/
//ARM OF THE MANIPULATOR GOES UP WITH THE BLOCK IN THE GRIPPER 

    if(z==0)  {

    unsigned long currentTime2 = millis();
    if ((currentTime2 - lastMoveTime2 >= interval) ) {
    lastMoveTime2 = currentTime2;

    servoPos_3 = servoPos_3 + 1;    // Move ARM up with the block enclosed in gripper 

    if (servoPos_3 >= 170 ) {        // Fully up position reached
      servoPos_3 = 170;

      z=1 ;      // Stop lifting the arm 

      state = 0 ;   // MOVE TO IDEAL STATE || THE BLOCK HAS BEEN PICKED UP
      // Reset timers for next operation
      lastMoveTime =  0  ;
      lastMoveTime2 = 0  ;
      lastMoveTime3 = 0  ;
    }
    ARM.write(servoPos_3);
  }
  }
  }
/*********************************************************************************/





/*******************************************************************************/
//DROP DOWN SEQUENCE
//state = 2 triggers drop operation

  if (state == 2) {   



//ARM OF THE MANIPULATOR GOES DOWN WITH THE BLOCK IN THE GRIPPER 

  unsigned long currentTime3  = millis();
  if ((currentTime3 - lastMoveTime3 >= interval) &&  (p==1)) {
    lastMoveTime3 = currentTime3;

    servoPos_4 = servoPos_4 - 1 ;   // Move ARM down slowly

    if (servoPos_4 <= 145 ) {       // LOWER LIMIT OF ARM IS REACHED 
      servoPos_4 = 145;

      p = 0 ;    // Stop ARM motion
      l = 0 ;    // Start GRIPPER opening 
    }
    ARM.write(servoPos_4) ; 
  }
/**********************************************************************************/






/******************************************************************************/
//GRIPPER OPENS UP TO RELEASE THE BLOCK  

  if(l==0)  {

    unsigned long currentTime4 = millis();
    if ((currentTime4 - lastMoveTime4 >= interval) ) {
    lastMoveTime4 = currentTime4;

    servoPos_5 = servoPos_5 - 1 ;  // Open GRIPPER slowly

    if (servoPos_5 <= 0) {  
      servoPos_5 =0 ;

      l=1 ;      // Stop GRIPPER opening
      r=0 ;      // Start ARM lifting

      servoPos_6 = 145 ; 

    }
    GRIPPER.write(servoPos_5);
  }
  }
/********************************************************************************/





/********************************************************************************/
//ARM OF THE MANIPULATOR GOES UP AFTER RELEASING THE BLOCK

    if(r==0)  {

    unsigned long currentTime5 = millis();
    if ((currentTime5 - lastMoveTime5 >= interval) ) {
    lastMoveTime5 = currentTime5;

    servoPos_6 = servoPos_6 + 1;       // Move ARM up             

    if (servoPos_6 >= 170 ) {  
      servoPos_6 = 170;
      r=1 ;   // Stop lifting
      state = 0 ; // THE BLOCK HAS BEEN DROP DOWN SO RETURN TO THE IDEAL STATE

      // Reset flags for next PICK sequence
      x=1 ;  
      p = 1 ; 

      // Reset timers
      lastMoveTime3 = 0  ;
      lastMoveTime4 = 0  ;
      lastMoveTime5 = 0  ;

      // Reset servo positions to initial values
      servoPos_1 =  170;
      servoPos_2 =  0 ;
      servoPos_3 =  145;
      servoPos_4 =  170;
      servoPos_5 =  180;
      
    }
    ARM.write(servoPos_6);       
  }
  }
  }
/*******************************************************************************************/

}
