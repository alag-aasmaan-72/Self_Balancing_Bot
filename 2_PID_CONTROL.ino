/*
▪ * Team Id:  eYRC#3575
▪ * Author List: AMAN KUMAR YADAV , SATYA PRAKASH MALLICK AND SAMIKSHA SAHU 
▪ * Filename: 2_PID_CONTROL
▪ * Theme: KrishiBalancer (KB)
▪ * Functions: setup() , loop() , task_5_milli_sec() , task_30_milli_sec()
▪ * Global Variables: KD_POSITION , KI_POSITION  , KP_POSITION , K_YAW , REFERENCE_ANGLE , pseudo_REFERENCE_ANGLE , REFERENCE_YAW  ,  T_X ,  T_Y  
                      REFERENCE_POSITION  , REFERENCE_SPEED , angle_buffer , angle_for_pid , new_angle_ready , COUNT , last_Time_loop , interval_of_loop
                      CURRENT_ANGLE , PREV_ANGLE , ERROR_IN_ANGLE , PREV_ERROR_IN_ANGLE , ANGLE_DERIVATIVE , ANGLE_INTEGRAL , ANGLE_CORRECTION_SENT , 
                      MOTOR_2_ENCODER_COUNT , MOTOR_1_ENCODER_COUNT , PREV_MOTOR_2_ENCODER_COUNT , PREV_MOTOR_1_ENCODER_COUNT , CURRENT_SPEED 
                      ERROR_IN_SPEED , PREV_ERROR_IN_SPEED  ,  SPEED_DERIVATIVE , SPEED_INTEGRAL , SPEED_CORRECTION_SENT , CURRENT_POSITION ,  last_Time_loop_2
                      ERROR_IN_POSITION , PREV_ERROR_IN_POSITION , POSITION_DERIVATIVE , POSITION_INTEGRAL , POSITION_CORRECTION_SENT , CURRENT_YAW ,interval_of_loop_2
                      ERROR_IN_YAW , YAW_CORRECTION_SENT , COS_COMPONENT_OF_YAW , SIN_COMPONENT_OF_YAW , FINAL_PWM_SENT_TO_MOTOR_1 , FINAL_PWM_SENT_TO_MOTOR_2
*/ 


/*  PID TUNING */
#define KP_ANGLE  15    // PROPORTIONAL GAIN FOR ANGLE 
#define KD_ANGLE  0.5   // DERIVATIVE GAIN FOR ANGLE 
#define KI_ANGLE  0.6   // INTEGRAL GAIN FOR ANGLE 

#define KP_SPEED  0.4   // PROPORTIONAL GAIN FOR SPEED 
#define KD_SPEED  0     // DERIVATIVE GAIN FOR SPEED  
#define KI_SPEED  0.003 // INTEGRAL GAIN FOR SPEED 

volatile float  KI_POSITION =  0.0006;   // INTEGRAL GAIN FOR POSITION  
volatile float  KP_POSITION =  0.008 ;   // PROPORTIONAL GAIN FOR  POSITION
volatile float  KD_POSITION =  0.0006;   // DERIVATIVE GAIN FOR POSITION

volatile float  K_YAW = 50 ;   // PROPORTIONAL GAIN FOR  YAW 

/*REFERENCE TERMS*/
volatile float  REFERENCE_ANGLE     =  0.2 ;   // TARGET ANGLE OR REFERENCE ANGLE AT WHICH BOT BALANCES 
volatile float  pseudo_REFERENCE_ANGLE = 0.2 ; 
volatile float  REFERENCE_YAW       =  0   ; 
volatile float  T_X                 =  1   ;  // COS COMPONENT OF REFERENCE YAW ANGLE (COS 0 = 1)
volatile float  T_Y                 =  0   ;  // SIN COMPONENT OF REFERENCE YAW ANGLE (SIN 0 = 0)
volatile float  REFERENCE_POSITION  =  0   ;
volatile float  REFERENCE_SPEED     =  0   ;  // WHEN BOT IS AT EQUILIBRIUM ITS SPEED MUST BE 0 


volatile bool       new_angle_ready = false;


unsigned long       last_Time_loop = 0;   
const unsigned long interval_of_loop = 5;  // 5 milli seconds 

unsigned long       last_Time_loop_2 = 0;   
const unsigned long interval_of_loop_2 =30 ;  // 30 milli seconds 


/*ANGLE CONFIGURATION FOR PID */
volatile float CURRENT_ANGLE  = 0 ;           // THE ANGLE WE ARE GETTING FROM MPU THROUGH DMP
volatile float PREV_ANGLE     = 0 ;           // THIS IS THE PREVIOUS ANGLE 
volatile float ERROR_IN_ANGLE = 0 ;           // THIS IS ERROR IN ANGLE       
volatile float PREV_ERROR_IN_ANGLE = 0 ;      // PRREVIOUS ERROR IN ANGLE NEEDED FOR CALCULATING DERIVATIVE 
volatile float ANGLE_DERIVATIVE    = 0 ;      // ANGLE DERIVATIVE 
volatile float ANGLE_INTEGRAL      = 0 ;      // ANGLE INTEGRAL 
int ANGLE_CORRECTION_SENT = 0 ;               // THE PWM SIGNAL SENT MUST BE AN INTEGER 


volatile float angle_buffer = 0;               // TEMPORARY BUFFER to safely store the latest CURRENT_ANGLE
volatile float angle_for_pid = 0;              // FILTERED CURRENT_ANGLE used for PID calculation


/* SPEED CONTROL CONFIGURATION */
volatile long MOTOR_2_ENCODER_COUNT = 0;  
volatile long MOTOR_1_ENCODER_COUNT = 0; 
volatile long PREV_MOTOR_2_ENCODER_COUNT = 0;   // PREVIOUS MOTOR 2 ENCODER COUNT
volatile long PREV_MOTOR_1_ENCODER_COUNT = 0;   // PREVIOUS MOTOR 1 ENCODER COUNT   
volatile float CURRENT_SPEED = 0 ; 
volatile float ERROR_IN_SPEED = 0; 
volatile float PREV_ERROR_IN_SPEED = 0 ; 
volatile float SPEED_DERIVATIVE ;
volatile float SPEED_INTEGRAL ; 
int SPEED_CORRECTION_SENT = 0;                  

  
/* POSITION CONTROL CONFIGURATION */
volatile float CURRENT_POSITION = 0  ; 
volatile float ERROR_IN_POSITION = 0 ; 
volatile float PREV_ERROR_IN_POSITION = 0 ;   // PREVIOUS ERROR IN POSITION
volatile float POSITION_DERIVATIVE ; 
volatile float POSITION_INTEGRAL ; 
int POSITION_CORRECTION_SENT = 0 ; 


/* YAW CONTROL CONFIGURATION */
volatile float CURRENT_YAW    = 0 ;           // THE CURRRENT YAW ANGLE
volatile float ERROR_IN_YAW = 0; 
int YAW_CORRECTION_SENT = 0; 
volatile float COS_COMPONENT_OF_YAW  = 0 ;   // COS COMPONENET OF CURRENT YAW ANGLE 
volatile float SIN_COMPONENT_OF_YAW  = 0 ;   // SIN COMPONENT  OF CURRENT YAW ANGLE 


volatile int FINAL_PWM_SENT_TO_MOTOR_1 = 0  ;  // FINAL PWM SIGNAL(0-255) SENT TO MOTOR_1 
volatile int FINAL_PWM_SENT_TO_MOTOR_2 = 0  ;  // FINAL PWM SIGNAL(0-255) SENT TO MOTOR_2




/*************************************************************************************************
* Function Name: task_5_milli_sec
* Input:  NONE
* Output: NONE
* Logic:  THIS FUNCTION RUNS AT EVERY 5 MILLI SECONDS INTERVAL AND CALCULATES ALL ANGLE AND POSITION ERROR TERMS AND ITS CORRECTION TERMS. ( PID MATH WORK)
* Example Call : task_5_milli_sec
****************************************************************************************************/
void task_5_milli_sec() {

/*
The below piece of code ensures that the controller always has a valid pitch angle, even when no new sensor data is available.
if new angle is availabile use that , if no new angle is availaible use previous angle.
hence Prevents using invalid or garbage pitch angle data
*/
     static float last_angle = 0;

     if (new_angle_ready) {
        angle_for_pid = angle_buffer;    
       
        last_angle = angle_for_pid;
        new_angle_ready = false;
    } 
    else
    {
        angle_for_pid = last_angle; 
    }


/***********************************************
ANGLE CORRECTION OF THE BOT 
************************************************/
ERROR_IN_ANGLE = REFERENCE_ANGLE - angle_for_pid  ; // ERROR IN THE ANGLE       // angle_for_pid = CURRENT_ANGLE 

ANGLE_DERIVATIVE = (ERROR_IN_ANGLE - PREV_ERROR_IN_ANGLE)/ 0.005  ; // DERIVATIVE     

PREV_ERROR_IN_ANGLE = ERROR_IN_ANGLE ; 

ANGLE_INTEGRAL = ANGLE_INTEGRAL + (ERROR_IN_ANGLE ) ;       // INTEGRAL 

ANGLE_INTEGRAL = constrain(ANGLE_INTEGRAL, -200, 200) ;    // PROTECTING FROM OVERSHOOTING ANG LARGE INTEGRAL 

ANGLE_CORRECTION_SENT = KP_ANGLE * ERROR_IN_ANGLE + KD_ANGLE * ANGLE_DERIVATIVE + KI_ANGLE * ANGLE_INTEGRAL  ;  // ANGLE COREECTION (IMPLEMENTING PID)



/*************************************************
POSITION CORRECTION OF THE BOT 
**************************************************/
CURRENT_POSITION = (MOTOR_2_ENCODER_COUNT +   MOTOR_1_ENCODER_COUNT) * 0.5  ; 

ERROR_IN_POSITION =  REFERENCE_POSITION - CURRENT_POSITION ; 

POSITION_DERIVATIVE = ( ERROR_IN_POSITION - PREV_ERROR_IN_POSITION )/0.005 ;      

PREV_ERROR_IN_POSITION = ERROR_IN_POSITION ; 

POSITION_INTEGRAL = POSITION_INTEGRAL + ERROR_IN_POSITION  ; 

POSITION_INTEGRAL = constrain(POSITION_INTEGRAL, -100,100 ) ;    // PROTECTING FROM OVERSHOOTING ANG LARGE INTEGRAL

POSITION_CORRECTION_SENT = (KP_POSITION * ERROR_IN_POSITION)  +  (KD_POSITION *POSITION_DERIVATIVE) + (KI_POSITION * POSITION_INTEGRAL )  ;



/**********************************************************
THE FINAL PWM SIGNAL TO MOTORS FOR BALANCING 
***********************************************************/
FINAL_PWM_SENT_TO_MOTOR_1 = ANGLE_CORRECTION_SENT + SPEED_CORRECTION_SENT  +  YAW_CORRECTION_SENT ;    
FINAL_PWM_SENT_TO_MOTOR_2 = ANGLE_CORRECTION_SENT + SPEED_CORRECTION_SENT  - YAW_CORRECTION_SENT  ;         

}




/*************************************************************************************************
* Function Name: task_30_milli_sec
* Input:  NONE
* Output: NONE
* Logic:  THIS FUNCTION RUNS AT EVERY 30 MILLI SECONDS INTERVAL AND CALCULATES THE SPEED AND YAW ERRORS AND ITS CORRECTION. 
          SPEED AND YAW ARE CALCULATED AT 30 MILLI SECONDS INTERVAL BECAUSE THEY CHANGE MORE SLOWLY AND DO NOT AFFECT IMMEDIATE BALANCE .
          AND ALSO REDUCES COMPUTATIONAL LOAD 
* Example Call : task_30_milli_sec
****************************************************************************************************/
void task_30_milli_sec() {

/************************************************************************
SPEED CORRECTION OF THE BOT 
************************************************************************/
CURRENT_SPEED =  ((MOTOR_2_ENCODER_COUNT - PREV_MOTOR_2_ENCODER_COUNT ) +  ( MOTOR_1_ENCODER_COUNT - PREV_MOTOR_1_ENCODER_COUNT)) * 0.5 ;    // GETTING CURRENT SPEED 

PREV_MOTOR_2_ENCODER_COUNT = MOTOR_2_ENCODER_COUNT ; 

PREV_MOTOR_1_ENCODER_COUNT = MOTOR_1_ENCODER_COUNT ;

ERROR_IN_SPEED = REFERENCE_SPEED - CURRENT_SPEED  ;                    // GETTING ERROR IN SPEED 

SPEED_DERIVATIVE = (ERROR_IN_SPEED -  PREV_ERROR_IN_SPEED) / 0.03 ;    // SPEED DERIVATIVE 

PREV_ERROR_IN_SPEED = ERROR_IN_SPEED ; 

SPEED_INTEGRAL = SPEED_INTEGRAL + ERROR_IN_SPEED ;                     // SPEED INTEGRAL 

SPEED_INTEGRAL = constrain(SPEED_INTEGRAL, -2000, 2000);               //  PROTECTING IT FROM OVERSHOOTING . AND IT WORKED FOR THE BOT 

SPEED_CORRECTION_SENT = (KP_SPEED * ERROR_IN_SPEED)  + (KD_SPEED * SPEED_DERIVATIVE) +  (KI_SPEED * SPEED_INTEGRAL)  ;  


/*******************************************************
YAW CORRECTION OF THE BOT 
*******************************************************/
ERROR_IN_YAW   = COS_COMPONENT_OF_YAW  * T_Y - SIN_COMPONENT_OF_YAW * T_X  ;

YAW_CORRECTION_SENT = K_YAW * ERROR_IN_YAW ; 
}





/*********************************************************************************************
* Function Name : setup
* Input :  NONE
* Output:  NONE
* Logic :  TO INITALIZE ALL THE PARAMETERS ONCE 
* Example Call : setup() - CALLED INTERNALLY 
*********************************************************************************************/
void setup()
{

}


/*********************************************************************************************
* Function Name : loop
* Input :  NONE
* Output:  NONE
* Logic :  CALLS task_5_milli_sec() at every  5 milli seconds interval  
           CALLS task_30_milli_sec() at every 30 milli seconds interval 
* Example Call : loop() - AUTOMATICALLY CALLED INTERNALLY
*********************************************************************************************/
void loop()
{
        noInterrupts();   // WHILE CALCULATING STOP ALL ISR SO THAT IT DO NOT READ CORRUPTED DATA
        angle_buffer = CURRENT_ANGLE;     
        new_angle_ready = true;
        interrupts();

  /*****************************************************/
  //DO TASKS AT EVERY 5 MILLI SECONDS OF INTERVAL
  unsigned long current_Time_of_loop = millis();
  if (current_Time_of_loop - last_Time_loop >= interval_of_loop) {
    last_Time_loop = current_Time_of_loop;
    task_5_milli_sec(); 
  }
  /*****************************************************/


  /*************************************************/
    //DO TASKS AT EVERY 30 MILLI SECONDS OF INTERVAL
  unsigned long current_Time_of_loop_2 = millis();
  if (current_Time_of_loop_2 - last_Time_loop_2 >= interval_of_loop_2) {
    last_Time_loop_2 = current_Time_of_loop_2;
    task_30_milli_sec(); 
  /**************************************************/

  }

}
