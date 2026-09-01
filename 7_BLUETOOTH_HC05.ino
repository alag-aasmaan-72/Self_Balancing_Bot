/*
▪ * Team Id: eYRC#3575
▪ * Author List: AMAN KUMAR YADAV , SATYA PRAKASH MALLICK AND SAMIKSHA SAHU 
▪ * Filename: 7_BLUETOOTH_HC05
▪ * Theme: KrishiBalancer (KB)
▪ * Functions: Bluetooth_Command(char command) , setup , loop 
▪ * Global Variables:  last_time_buzz , interval_buzz ,   E , BEEP_MODE  , State , motion_mode , speed , forward_motion , backward_motion 
*/ 

#define buzz  = A1;  

/*BLUETOOTH COMMAND CONTROL*/
#define PICK_UP              'F'          // PICK UP THE BLOCK 
#define DROP_DOWN            'B'          // DROP THE BLOCK 
#define LEFT_TURN            'L'          // DO LEFT TURN 
#define RIGHT_TURN           'R'          // DO RIGHT TURN 
#define STOP                 'S'          // STOP COMMAND 
#define FORWARD              'T'          // MOVE FORWARD
#define FORWARD_WITH_SPEED   'C'          // MOVE FORWARD WITH HIGH SPEED 
#define BACKWARD             'X'          // MOVE BACKWARD 
#define BEEP                 'A'          // MAKE BEEP SOUND 

// Commands to fine-tune reference angle
#define PSEUDO_REFERENCE_ANGLE_FORWARD      'P'         
#define PSEUDO_REFERENCE_ANGLE_BACKWARD     'M'         

// Commands to set fixed forward/backward tilt
#define SETTING_FORWARD_TILT            'n'        
#define SETTING_BACKWRAD_TILT           'd' 


// Turn OFF position hold PID
#define POS_NO                'b'         // TURN OFF POSITION PID 



unsigned long  last_time_buzz = 0 ; 
const unsigned long interval_buzz = 1000 ;   // BUZZER BEEPS FOR 1 SEC 

int  E  = 0 ; // E → Execution flag to monitor buzzer timing
int  BEEP_MODE = 0 ;  // variable controlling beep and no beep  mode  ||  when BEEP_MODE = 0 (NO BEEP) , WHEN BEEP_MODE = 1  (BEEP ON)

int  state = 0;  // variable controlling pick and drop operations ||  when 0 = IDLE, 1 = PICK_UP, 2 = DROP_DOWN

volatile float speed = 0 ; // // MOVE FORWARD WITH HIGH SPEED 


volatile float forward_motion   =  0 ;  
volatile float backward_motion  =  0 ; 


/*********************************************************************************/
int motion_mode = 0;  
// MOTION MODE CONTROL VARIABLE
// Determines the current movement mode of the robot:

// motion_mode = 0 → Position hold OFF mode (robot balances but does not correct position)
// motion_mode = 1 → Position hold ON mode (robot maintains same position using PID)
// motion_mode = 2 → Forward motion mode (robot moves forward with normal speed)
// motion_mode = 3 → Backward motion mode (robot moves backward)
// motion_mode = 4 → High-speed forward motion mode 
/************************************************************************************/                


/*************************************************************************************************
* Function Name: Bluetooth_Command()
* Input:  char command → Character received from Bluetooth
* Output: NONE
* Logic: THIS FUNCTION RECEIVES THE COMMAND THROUGH BLUETOOTH AND ACT ACCORDINGLY FOR MOVEMENT CONTROL 
* Example Call : Bluetooth_Command()
****************************************************************************************************/

void Bluetooth_Command(char command) {

    if (command == PICK_UP && state == 0) {        
        state = 1;        // WHEN STATE IS 1 THEN PICK UP OF BLOCK WILL HAPPEN 
    }
    else if (command == DROP_DOWN && state == 0) {
        state = 2;        // WHEN STATE IS 2 THEN DROP DOWN OF BLOCK WILL HAPPEN 
    }


    switch(command) {
        case LEFT_TURN:  
            // REDUCE REFERENCE YAW ANGLE BY 10 DEGREES 
            REFERENCE_YAW =  (REFERENCE_YAW  - 10) ; 

            // CONVERT YAW INTO UNIT VECTOR COMPONENTS 
            T_X = cos(REFERENCE_YAW * M_PI/180) ;   // TARGET COS COMPONENET OF REFERENCE YAW 
            T_Y = sin(REFERENCE_YAW * M_PI/180) ;   // TARGET SIN COMPONENET OF REFERENCE YAW 
            break;

        case RIGHT_TURN:  
            // INCREASING REFERENCE YAW ANGLE BY 10 DEGREES 
            REFERENCE_YAW =  REFERENCE_YAW  + 10 ; 

            T_X = cos(REFERENCE_YAW* M_PI/180) ;    // TARGET COS COMPONENET OF REFERENCE YAW 
            T_Y = sin(REFERENCE_YAW* M_PI/180) ;    // TARGET SIN COMPONENET OF REFERENCE YAW 
            break;

        case STOP:  
            /*
            WHEN STOP COMMAND IS GIVEN THEN , REFERENCE_POSITION AND REFERENCE_YAW IS SET OF THAT INSTANT SO BOT  DOES NOT MOVE FORWARD AND BACKWARD , ALSO ROTATION STOPS 
            AND POSITION PID ALSO TURNS ON TO MAKE IT AT ONE PLACE 
            */
            motion_mode = 1 ;    // Enable position hold mode

            REFERENCE_POSITION =(MOTOR_2_ENCODER_COUNT + MOTOR_1_ENCODER_COUNT ) * 0.5  ;   // Store current position as reference position

            T_X = cos(CURRENT_YAW ) ;  // Store current yaw direction
            T_Y = sin(CURRENT_YAW)  ;

             // Enable Position PID
            KP_POSITION =  0.008 ;  
            KD_POSITION =  0.0006 ;  
            KI_POSITION =  0.0006;   
            
             // Enable yaw PID
            K_YAW = 50 ;  

            break;

        case FORWARD :  
            motion_mode = 2 ;   // Forward motion mode

            // TURNING OFF THE POSITION PID. (POSITION HOLD IS OFF , NOW BOT CAN DO FORWARD MOTION)
            KP_POSITION =  0 ;          
            KD_POSITION =  0 ;
            KI_POSITION =  0 ; 
            break;

         case FORWARD_WITH_SPEED :  

            speed = Pseudo_REFERENCE_ANGLE - 1.3 ; // FOR MOVING FORWARD WITH HIGH SPEED WE ARE DECREASING Pseudo_REFERENCE_ANGLE BY 1.3 
            motion_mode = 4 ;         // High speed forward mode

            // TURNING OFF THE POSITION PID. (POSITION HOLD IS OFF , NOW BOT CAN DO FORWARD MOTION)
            KP_POSITION =  0;          
            KD_POSITION =  0 ;
            KI_POSITION =  0 ; 

            break;

        case BACKWARD :  
           motion_mode = 3 ;      // backward motion mode

           // TURNING OFF THE POSITION PID. (POSITION HOLD IS OFF , NOW BOT CAN DO BACKWARD MOTION)
           KP_POSITION =  0 ;
           KD_POSITION =  0 ;
           KI_POSITION =  0 ; 
           break;

        case BEEP : 
           BEEP_MODE = 1 ;  // Activate buzzer
           break ;


        /*
           PSEUDO REFERENCE ANGLE CONCEPT
           Explanation: Even when robot is perfectly vertical, MPU may show small error (-0.5° to +0.5°)
           This command corrects that offset.
        */
        case PSEUDO_REFERENCE_ANGLE_FORWARD : 
          Pseudo_REFERENCE_ANGLE = Pseudo_REFERENCE_ANGLE - 0.1 ; 
           
           break ;  

        case PSEUDO_REFERENCE_ANGLE_BACKWARD : 
          Pseudo_REFERENCE_ANGLE = Pseudo_REFERENCE_ANGLE + 0.1 ; 
           
           break ;  
        /*************************************************************************************************************
        *************************************************************************************************************/


        case SETTING_FORWARD_TILT  : 
          forward_motion = Pseudo_REFERENCE_ANGLE - 0.8 ; 
           
           break ;  

        case SETTING_BACKWARD_TILT  : 
          backward_motion = Pseudo_REFERENCE_ANGLE + 0.8 ; 
           
           break ;  

        case POS_NO : 
           motion_mode = 0 ; // turn off the position hold mode 
           break ;
    }
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
  Serial.begin(9600)  ; 
}

/*********************************************************************************************
* Function Name : loop
* Input :  NONE
* Output:  NONE
* Logic :  CALLS    Bluetooth_Command() REPEATEDLY
* Example Call : loop() - AUTOMATICALLY CALLED INTERNALLY
*********************************************************************************************/
void loop(){

    if (Serial.available()) {
    char command = Serial.read();
    Bluetooth_Command(command);
    
  }

       if(motion_mode == 1 )   //   POSITION HOLD MODE
        {
        /*
        Robot maintains same position
        If pushed forward →
        PID moves robot backward to correct.

        If pushed backward →
        PID moves robot forward to correct.
        */

        REFERENCE_ANGLE = Pseudo_REFERENCE_ANGLE  - POSITION_CORRECTION_SENT * 1.29 - SPEED_CORRECTION_SENT* 0.07 ;  // CONTROLLING IT WITH OFFSET SO IT DOES NOT AFFECT THE PWM GOING TO THE MOTORS 

        }


         else if(motion_mode==0)  //  POSITION HOLD OFF MODE
       {
    
        REFERENCE_ANGLE =  Pseudo_REFERENCE_ANGLE ;           // here reference angle do not adjusts when bot is pushed forwards or backwards            
           
        }

        else if(motion_mode==2) {
          

        REFERENCE_ANGLE = forward_motion ;       // DECREASING TILT ANGLE TO MOVE FORWARD (subtracting 0.8 from pseudo reference angle )                       
           
        }

        else if (motion_mode==3) 

        {
          
        REFERENCE_ANGLE = backward_motion ;      // INCREASING TILT ANGLE TO MOVE BACKWARD (ADDING  0.8 from pseudo reference angle )  
        
        }

        else if (motion_mode == 4)
        {
        REFERENCE_ANGLE = speed ;                 // DECREASING TILT ANGLE TO LARGE AMOUNT TO MOVE FORWARD FASTLY (subtracting 1.4 from pseudo reference angle)
        }

        




        /**************************************************************************************/
        //BUZZER 
        
        // BEEP_MODE → Trigger flag to start buzzer 
        // E → Execution flag to monitor buzzer timing
        if(BEEP_MODE == 1) 
        {
          tone(buzz, 2000);
          BEEP_MODE = 0 ; 
          E = 1 ; 
          last_time_buzz = millis()  ; 
         }

         if(E==1)   
         {
          unsigned long current_time_buzz = millis();
          if (current_time_buzz - last_time_buzz >= interval_buzz) {
          noTone(buzz);
          E = 0 ; 
         }

        }
        /********************************************************************************************/
}