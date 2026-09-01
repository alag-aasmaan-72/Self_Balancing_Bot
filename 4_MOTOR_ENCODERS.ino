/*
▪ * Team Id:  eYRC#3575
▪ * Author List: AMAN KUMAR YADAV , SATYA PRAKASH MALLICK AND SAMIKSHA SAHU 
▪ * Filename: 4_MOTOR_ENCODERS
▪ * Theme: KrishiBalancer (KB)
▪ * Functions: setup , loop , MOTOR_2_ENCODER() , ISR(PCINT2_vect)
▪ * Global Variables: MOTOR_1_ENCODER_CH_A , MOTOR_1_ENCODER_CH_B , MOTOR_2_ENCODER_CH_A , MOTOR_2_ENCODER_CH_B , PREV_STATE_OF_ENCODER_CH_A
*/ 


/* MOTOR ENCODER CONFIGURATION */

#define MOTOR_2_ENCODER_CH_A 2  
#define MOTOR_2_ENCODER_CH_B 3 


volatile long prev_count ;
volatile long current_count ;

volatile long MOTOR_2_ENCODER_COUNT = 0;      
volatile long MOTOR_1_ENCODER_COUNT = 0;     
volatile long change ;    


unsigned long prev_time ;
unsigned long current_time ; 

volatile uint8_t  PREV_STATE_OF_ENCODER_CH_A = 0;   // stores the previous state of MOTOR 1 ENCODER CHANNEL A.(HIGH OR LOW)



/************************************************************************************************
* Function Name: MOTOR_2_ENCODER
* Input :  NONE
* Output:  NONE
* Logic :  THIS FUNCTION UPDATES MOTOR_2_ENCODER_COUNT.
           EVERY TIME WHEN THERE IS RISING EDGE IN PULSE OF MOTOR_2_ENCODER_CH_A , THEN THIS FUNCTON IS CALLED. THIS FUNCTION READS ENCODER CHANNEL B , WHEN CHANNEL B IS HIGH THIS MEANS
           OUR N20 WHEEL IS ROTATING CLOCKWISE AND ENCODER COUNT INCREASES.
           WHEN CHANNEL B IS LOW , THIS MEANS OUR N20 WHEEL IS  ROTATING ANTI CLOCKWISE AND ENCODER COUNT DECREASES. 
* EXAMPLE CALL :  MOTOR_2_ENCODER() 
**************************************************************************************************/
void MOTOR_2_ENCODER() { 
    if (digitalRead(MOTOR_2_ENCODER_CH_B )) {
        MOTOR_2_ENCODER_COUNT++;    // INCREASE MOTOR 2 ENCODER COUNT 
    } else {
        MOTOR_2_ENCODER_COUNT--;    // DECREASE MOTOR 2 ENCODER COUNT 
    }
}





/***************************************************************************************************
* Function Name: ISR()
* Input:  NONE
* Output: NONE
* Logic:  THIS FUNCTION UPDATES MOTOR 1 ENCODER COUNT . HERE WE ARE USING  PIN CHANGE INTERRUPT. IF OUR TYRE IS ROTATIONG CLOCKWISE THEN ENCODER COUNT INCREASES AND WHEN IT IS 
          ROTATED ANTI- CLOCKWISE THE ENCODER COUNT DECREASES OF THE SECOND MOTOR 
* EXAMPLE CALL : HARDWARE CALLS IT AUTOMATICALLY        
***************************************************************************************************/





/*********************************************************************************************
* Function Name : setup
* Input :  NONE
* Output:  NONE
* Logic :  TO INITALIZE ALL THE PARAMETERS ONCE 
* Example Call : setup() - CALLED INTERNALLY 
*********************************************************************************************/
void setup()
{

    Serial.begin(9600) ; 
    /*ENABLING PULL UPS ON BOTH ENCODER CHANNELS FOR CLEAN AND NOISE FREE SIGNALS*/
    pinMode(MOTOR_2_ENCODER_CH_A, INPUT_PULLUP); 
    pinMode(MOTOR_2_ENCODER_CH_B, INPUT_PULLUP);



    PCICR  |= (1 << PCIE2);      // Enable PORTD PCINT
    PCMSK2 |= (1 << PCINT23);    // Enable PCINT for D7


    /*AT EVERY RISING EDGE OF MOTOR_2_ENCODER_CH_A ,  CALL THE FUNCTION --> MOTOR_2_ENCODER()   AND CALCULATE THE  MOTOR_2_ENCODER_COUNT*/
    attachInterrupt(digitalPinToInterrupt(MOTOR_2_ENCODER_CH_A), MOTOR_2_ENCODER, RISING); 

}


void func()

{
 current_count = MOTOR_2_ENCODER_COUNT ; 

 change = current_count - prev_count ; 
prev_count = current_count ; 


}



/*********************************************************************************************
* Function Name : loop
* Input :  NONE
* Output:  NONE
* Example Call : loop() - AUTOMATICALLY CALLED INTERNALLY
*********************************************************************************************/
void loop()
{
  
 

   


  current_time = millis() ; 

  if(current_time - prev_time >= 100) ; 
  {

    func() ;
    Serial.println(change) ; 
  }


}
