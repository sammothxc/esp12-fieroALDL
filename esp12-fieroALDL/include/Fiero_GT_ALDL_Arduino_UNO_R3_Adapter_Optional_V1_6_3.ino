/*******************************************************************************

    Filename: Fiero_GT_ALDL_Arduino_UNO_R3_Adapter_Optional_V1_6_0.ino
      Author: Paul Romsky
     Company: Xyaxis - Romsky Consultation Consortium
    Document: 008-00012-901
        Date: 18 JUL 2024
     Project: Fiero GT ALDL Monitor
   Processor: Arduino UNO R3
 Description: This is the microcontroller code to monitor and emulate the
              Fiero GT ALDL 160 Baud Comm Link and the Service Engine Soon (SES)
              signal - if the Fiero is modified to include the SES signal on 
              the Assembly Line Diagnostic Link (ALDL) connector. This code is 
              to be installed on the Fiero GT ALDL Adapter V1.6.0 that interfaces 
              to the Fiero GT ALDL Monitor V1.6.0 - a Graphical User Interface (GUI)
              designed for this adapter.  The adapter sends interpreted signals 
              from the Fiero to the GUI installed on a Personal Computer (PC) 
              via a USB serial Comm link at 9600 Baud, 8 Bits, No Parity, 1 Stop.

              This is an optionala "BareBones" version for a lower cost for those
              that dont needs all the functions and may have an Arduino R3 on hand already.
              This version identifies as the Trinket M0 so that it will work with
              the existing version of the GUI.

 Program via the Arduino IDE:
 Note: Be sure to use the Up and Down arrows on the pulldown fields as there may be 
 many more selections than are shown.
 
              Target: Arduino UNO R3 ($23 USD Board)
               Board: Arduino Uno
                Port: Select Manually
          Programmer: Arduino as ISP
 Compile Time Switch: #define TARGET_BOARD (TARGET_BOARD_ARDUINO_UNO_R3)

              Target: Adafruit Trinket M0 ($9 USD Board)
               Board: Adafruit Trinket M0 (SAMD 21)
           USB Stack: Arduino
            Optimize: Small (-Os) (standard)
               Debug: Off
                Port: Select Manually
          Programmer: USBtinyISP
 Compile Time Switch: #define TARGET_BOARD (TARGET_BOARD_ADAFRUIT_TRINKET_M0)

 For reliable uploads to the Adafruit Trinket M0: Before uploading, on the Trinket M0,
 press the Reset button twice within a half second.  This will put the Trinket M0 in 
 Bootloader mode (the Red LED will flash slowly between bright/dim and the Dotstar LED will be bright Green).
 After uploading and the code is running on the Trinket M0, the Dotstar LED will be normally bright 
 Magneta (by default) - ignore any color of the Dotstar LED as it is not controlled by this code.
 
 The Trinket M0 comes delivered with Circuit Python installed and acts as a mini thumb drive when plugged
 into the PC.  Once this code is installed on the Trinket M0, Circuit Python will be removed from the 
 Trinket M0 and it will not longer appear as a mini thumb drive when plugged into the PC. 
 If you want to later reuse the Trinket M0 for other applications and revert the Trinket M0 back to 
 Circuit Phyton, refer to the Adafruit website for details.  Do not port this code to Circuit Python 
 for the Trinket M0, Circuit Pyhton is too slow and does not have the microseconds counter function micros().

 Copyright (c) Paul Romsky 2022, 2023 - All rights reserved
  
 Revision History:
 
 Rev    Date        Name                   Description
 ------ ----------- ---------------------- ----------------------------------
  0.0.0 18 JUL 2024 Paul Romsky            Initial coding. Taken from the Trinket M0 V1.6.3 version

*******************************************************************************/

/* Compile Time Switches */

#define TARGET_BOARD_ARDUINO_UNO_R3      1              /* Target the Arduino UNO R3 board */
#define TARGET_BOARD_ADAFRUIT_TRINKET_M0 2              /* Target the Adafruit Trinket M0 board */
#define TARGET_BOARD (TARGET_BOARD_ARDUINO_UNO_R3)      /* Compile the code for Target board Selected (Comment out if not the board desired) */
//#define TARGET_BOARD (TARGET_BOARD_ADAFRUIT_TRINKET_M0) /* Compile the code for Target board Selected (Comment out if not the board desired) */
#define TEST_IO                          0              /* 0 = Normal, 1 = Bench Test */
#define TUNE                             0              /* 0=Normal Adapter operation,  1=Tune the Adapter */

#define STUB_OUT_SES                     1
#define STUB_OUT_UP_SHIFT                1

#define SIMULATE_DEFAULT  0 /* 0 = Normal, 1 = Turn on Simulation by Default */


/* Includes */

/* The Arduino IDE does not support Pre-Processor/Conditional Compiler includes, comment out the two following lines for Arduino UNO */
#include <Adafruit_DotStar.h> /* Requires SPI.h */
#include <SPI.h>              /* Requires Adafruit_BusIO library installed in the Arduino IDE */


/* Constants */

#if TARGET_BOARD == TARGET_BOARD_ARDUINO_UNO_R3

#define USER_LED_PIN                     13 /* Arduino UNO R3 Yellow 'L' LED */
#define ALDL_160_BAUD_PIN                 2 /* Arduino UNO R3 'DIGITAL'              Pin  2: Digtial Input from a 5.0V Zener limiter from the Fiero ALDL Connector 160 Baud Pin (ALDL Pin E) */
#define ALDL_SES_PIN_N                    4 /* Arduino UNO R3 'DIGITAL'              Pin  4: Active Low (GND) Digital Input from a 5.0V Zener limiter from the Fiero ALDL Connector SES Signal (ALDL Pin D) */
                                            /* The SES pin is not implentment in this version of the Adapter and is stubbed out */
#define ALDL_UP_SHIFT_PIN_N               5 /* Arduino UNO R3 'DIGITAL'              Pin  5: Active Low (GND) Digital Input from a 5.0V Zener limiter from the Fiero ALDL Connector Up Shift Signal (ALDL Pin F) */
                                            /* The Up SHIFT pin is not implentment in this version of the Adapter and is stubbed out */
#define ALDL_MODE_MONITOR_PIN             5 /* Arduino UNO R3 'ANALOG'               Pin A5: Analog Input from a 5.0V Zener limiter from the Q1 Collector */
#define ALDL_DIAG_MODE_SELECT_PIN         3 /* Arduino UNO R3 'DIGITAL – PWM~ (IOL)' Pin ~3: An 8-bit 0V to 5V PWM Output to a Transisitor to control the resistance to the Fiero ALDL Connector Diag Mode Select Pin (ALDL Pin B) */
                                            /* 
                                             *  The Arduino UNO R3 does not have a Digital to Analog Converter (DAC) output pin, so a PWM pin is used to generate continous PWM 
                                             *  pulses out of the pin, therefore, the transistor has a simple Resistor-Capacitor (RC) Low Pass Filter before its Base input to 
                                             *  convert the PWM stream to a steady analog DC voltage (effectively acting like a DAC).  
                                             */

                                            /* <TBD> These need to be re-tuned to the 008-00012-301 schematic circuitry */
#define TRANSISTOR_OFF                    0 /* A Constant Low Pulse Width Moulation (PWM) [  0.0% Duty Cycle] to Transistor Input Low Pass Filter [0.000V], Transitor is Off (Open [5.0V] between ALDL Connector DIAG Mode Select and Ground) */
#define TRANSISTOR_10K_OHMS             162 /* At 25C. A Constant PWM [ 63.5% Duty Cycle] to Transistor Input Low Pass Filter that sets Transitor partially On (10K Ohms [2.5V] between ALDL Connector DIAG Mode Select and Ground) */
#define TRANSISTOR_3_9K_OHMS            166 /* at 25C. A Constant PWM [ 65.1% Duty Cycle] to Transistor Input Low Pass Filter that sets Transitor partially On (3.9K [1.4V] Ohms between ALDL Connector DIAG Mode Select and Ground) */
#define TRANSISTOR_SATURATED            255 /* A Constant PWM [100.0% Duty Cycle] to Transistor Input Low Pass Filter that Saturates the Transistor (Short [0.0V] between ALDL Connector DIAG Mode Select and Ground) */

#define SES_ALIVE_COUNT               12200 /* The Service Engine Soon (SES) monitor will send an Error Code 0 (EC0) periodcially as it monitors the Error Codes to keep the Link active */ 

#elif TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0

#define USER_LED_PIN                     13 /* Adafruit Trinket M0 Red LED */
#define DOTSTAR_CLK_PIN                   8 /* DotStar LED Clock Pin */
#define DOTSTAR_DAT_PIN                   7 /* DotStar LED Data Pin */
#define DOTSTAR_PIXELS                    1 /* Only 1 Pixel is on the Trinket M0 */
#define ALDL_160_BAUD_PIN                 0 /* Adafruit Trinket M0 Pin  0: Digtial Input from a 3.3V Zener limiter from the Fiero ALDL Connector 160 Baud Pin (ALDL Pin E) */
#define ALDL_SES_PIN_N                    2 /* Adafruit Trinket M0 Pin  2: Active Low (GND) Digital Input from a 3.3V Zener limiter from the Fiero ALDL Connector SES Signal (ALDL Pin D) */
#define ALDL_UP_SHIFT_PIN_N               3 /* Adafruit Trinket M0 Pin  3: Active Low (GND) Digital Input from a 3.3V Zener limiter from the Fiero ALDL Connector Up Shift Signal (ALDL Pin F) */
#define ALDL_MODE_MONITOR_PIN            A4 /* Adafruit Trinket M0 Pin  4: Analog Input from a 3.3V Zener limiter from the Q1 Collector */
#define ALDL_DIAG_MODE_SELECT_PIN        A0 /* Adafruit Trinket M0 Pin ~1: A 10-bit 0.0V to 3.3V DAC Output to a Transisitor to control the resistance to the Fiero ALDL Connector Diag Mode Select Pin (ALDL Pin B) */
                                            /* 
                                             *  The Adafruit Trinket M0 has a 10-bit Digital to Analog Converter (DAC) output pinon A0 only, so there is no need for a simple 
                                             *  Resistor-Capacitor (RC) Low Pass Filter before the Base input of the Transistor. Note: In CircuitPython, the Analog value 
                                             *  passed in is the range of 0 to 65535, CiruitPython divides this by 64 to get a 10-bit range (0 to 1023) for the DAC.
                                             *  In Arduino C/C++, you simply pass in the 10-bit value (0 [0.0V] to 1023 [3.3V]).
                                             */

#define TRANSISTOR_OFF                    0 /* A Constant DAC voltage [0.000V] to Transistor Input, Transitor is Off (Open [5.03V] between ALDL Connector DIAG Mode Select and Ground) */
#define TRANSISTOR_10K_OHMS             318 /* At 25C.  A Constant DAC voltage [1.042V] to Transistor Input, Transitor partially On (10K Ohms [2.5V] between ALDL Connector DIAG Mode Select and Ground) */
#define TRANSISTOR_3_9K_OHMS            376 /* At 25C.  A Constant DAC voltage [1.232V] to Transistor Input, Transitor partially On (3.9K Ohms [1.4V] between ALDL Connector DIAG Mode Select and Ground) */
#define TRANSISTOR_SATURATED           1023 /* A Constant DAC voltage [3.300V] to Transistor Input Low Pass Filter, Transitor is Saturated (Short [0.05V] between ALDL Connector DIAG Mode Select and Ground) */

#define SES_ALIVE_COUNT               85000 /* The Service Engine Soon (SES) monitor will send an Error Code 0 (EC0) periodcially as it monitors the Error Codes to keep the Link active */ 
                       
#endif /* TARGET_BOARD */

#define ECM_SAMPLE_TIME_C3             1130 /* us */
#define ECM_SAMPLE_TIME_VN             2875 /* us */
#define ECM_SAMPLE_TIME_C3_OR_VN       1800 /* us */ 
#define ERROR_CODE_COARSE_DELAY_COUNTS    5 /* Allows EC0's (NULLs) to be sent over link to keep it active */
#define ERROR_CODE_PULSE_ON_MS          300 /* Light is On for a Count Pulse in milliseconds */
#define ERROR_CODE_PULSE_OFF_MS         200 /* Light is Off for a Count Pulse in milliseconds */
#define ERROR_CODE_PAUSE_MS            1000 /* Light in Off between Digit Counts of an Error Code in milliseconds (accumulates with prior ERROR_CODE_PULSE_OFF_MS to be 1.200 seconds total) */
#define ERROR_CODE_SPACE_MS            2000 /* Light is Off between Error Codes in milliseconds (accumulates with prior ERROR_CODE_PULSE_OFF_MS to be 3.200 seconds total) */
#define ALDL_BIT_DELAY_MS                 6 /* Actual is 6.25 ms for 160 Baud, this is close due to the limted resolution of delay() which is in whole milliseconds */


/* Create Objects */

#if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
Adafruit_DotStar DotStar_LED = Adafruit_DotStar(DOTSTAR_PIXELS, DOTSTAR_DAT_PIN, DOTSTAR_CLK_PIN, DOTSTAR_BGR);
#endif


/* Enumerations */

enum
{
  MODE_GET_ALDL_DATA   = 0,
  MODE_GET_ERROR_CODES = 1
};

enum
{
  DIAG_MODE_NORMAL      = 0, /*  >20K   Ohms (or Open)  */
  DIAG_MODE_ALDL        = 1, /*   10K   Ohms            */
  DIAG_MODE_FACTORY     = 2, /*    3.9K Ohms            */
  DIAG_MODE_DIAGNOSTICS = 3  /* <470    Ohms (or Short) */
};

enum
{
  SIMULATING_NO  = 0,
  SIMULATING_YES = 1
};

enum
{
  STATE_WAIT_SYNC = 0,
  STATE_SYNC      = 1,
  STATE_DATA      = 2
};

enum
{
  METHOD_SET      = 0,
  METHOD_MAINTAIN = 1
};


/* Global Variables */

static int           Data = 0x00;
static int           Mask;
static int           Simulating = SIMULATING_NO;
static String        Command;
static String        Argument;
static int           Mode_DAC;
static int           Mode_Monitor_ADC;
static int           Adapter_Mode = MODE_GET_ALDL_DATA; 
static int           Value;
static int           Diag_Mode = DIAG_MODE_NORMAL;
static int           Error_Code;
static int           Error_Code_Tens;
static int           Error_Code_Time_ms;
static int           Fine_Delay_ms;
static unsigned long Alive_Counter = 0;
static String        ECM_Data_Stream = "VN\n";
static unsigned long ECM_Sample_Time = ECM_SAMPLE_TIME_VN;
static int           Mode_Status = -1; /* ECM Mode Not yet Set or Error */

/* Function Prototypes */

void Monitor_160_ALDL_Link(void);
void Monitor_SES_Signal(void);
int  Adjust_Mode_Voltage(int Method, int Mode);
void Simulate_160_ALDL_Link(void);
void Simulate_160_ALDL_Partial_Link(void);
void Simulate_SES_Signal(void); 
int  Determine_Error_Code_Time_In_Millisec(int Error_Code);


/* Functions */

/********************************************************************************
 *
 * There is no main() used in the Ardunio IDE.
 * the Ardunio IDE compiler will call setup() once, then it calls loop() 
 * automatically.
 *
 ********************************************************************************/
void setup() 
{
  /* Set up the USB Console Comms */
  Serial.begin(9600);
  Serial.setTimeout(50); /* Needed for Serial.readString() because it waits for a Newline (the default timeout of 1000 ms is too slow). 
                            At 9600 baud it takes about 31.25ms for 30 bytes to come across the Serial Link, since no more than 
                            than 30 bytes are sent as a Command/Argument, 50ms is sufficient with plenty of design margin. */

  /* Set up Digital Pins */
  pinMode(USER_LED_PIN,        OUTPUT);
  pinMode(ALDL_160_BAUD_PIN,   INPUT_PULLUP);
  pinMode(ALDL_SES_PIN_N,      INPUT_PULLUP);
  pinMode(ALDL_UP_SHIFT_PIN_N, INPUT_PULLUP);

  digitalWrite(USER_LED_PIN, 0); /* Red LED Off */

  #if TARGET_BOARD == TARGET_BOARD_ARDUINO_UNO_R3
  TCCR2B = (TCCR2B & B11111000) | B00000001; /* Pin ~3 PWM frequency of 31372.55 Hz */
  #endif  

  /* Set the Diag Mode Select Transitor to Normal Mode: Provides an Open to Ground at the Fiero ALDL Diag Mode Select pin */
  Mode_DAC = TRANSISTOR_OFF;
  analogWrite(ALDL_DIAG_MODE_SELECT_PIN, Mode_DAC);

  /* Set to Mode Monitor Analog Pin to Input */
  Mode_Monitor_ADC = analogRead(ALDL_MODE_MONITOR_PIN);
  
  #if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
  /* Setup the DotStar LED */
  DotStar_LED.begin(); /* Initialize pins for output */
  DotStar_LED.setPixelColor(0, 0x00FF00); /* Green */
  DotStar_LED.show();
  #endif

  #if TEST_IO
  {
    int  Data;
    int  SES;
    int  Shift;
    int  Select_ADC = 0;
    char Line[80 + 1];

    #if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
    DotStar_LED.setPixelColor(0, 0xFFA0A0); /* Pink */
    DotStar_LED.show();
    #endif
  
    while(true)
    {
      Data  = digitalRead(ALDL_160_BAUD_PIN);
      SES   = digitalRead(ALDL_SES_PIN_N);
      Shift = digitalRead(ALDL_UP_SHIFT_PIN_N);
        
      analogWrite(ALDL_DIAG_MODE_SELECT_PIN, Select_ADC);

      sprintf(Line, "Data=%i  SES_N=%i  Shift_N=%i  ADC=%i", Data, SES, Shift, Select_ADC);
      Serial.println(Line);
  
      Select_ADC++;
      if(Select_ADC >= 256) Select_ADC = 0;
    }
  }
  #endif
  
  #if TUNE
  /* This is used to step through the Trinket M0 DAC values to determine the best 
     values at 72 degreees F before Q1 temperature compensation of these settings: 
     TRANSISTOR_OFF        5.000V   DAC    0            
     TRANSISTOR_10K_OHMS   2.500V   DAC  318 Typical but varies
     TRANSISTOR_3_9K_OHMS  1.400V   DAC  376 Typical but varies
     TRANSISTOR_SATURATED  0.000V   DAC 1023
     This must be done for every Adapter as each Q1 transitor reacts differently. 
  */
  while(true)
  {
    for(int DAC = 0; DAC < 1024; DAC++)
    {
      Serial.println(DAC); /* Show DAC value in Console Window */
      analogWrite(ALDL_DIAG_MODE_SELECT_PIN, DAC);
      while(Serial.available() == 0) 
      {
        asm(""); /* Do Not optimize out an empty loop */
      }
      Serial.read(); /* Press <enter> in the Console Window Send field to go to the next value */
    }
  }
  #endif
     
  #if SIMULATE_DEFAULT
  Simulating = SIMULATING_YES;        
  #endif 
}


/********************************************************************************
 *
 * Monitor or Simulate the Fiero ALDL 160 Baud Data Link or Fiero Error Codes 
 * via the SES Signal (if the Fiero ALDL connector is modified to include the
 * SES signal).
 *
 ********************************************************************************/
void loop() 
{
  /* Initialization */
    
  /* Process */

  /* Loop Forever */
  while(1)
  {
    /* Get Command (if any) */
    if(Serial.available() > 0)
    {   
      /* Wait for Command with terminating Newline to arrive */
      Command = Serial.readString();
     
      /* Parse Command */
      if(Command.compareTo("IDENTIFY\n") == 0)
      { 
        Serial.print("Fiero_ALDL_Adapter_V1.6.3\n");
      }       
      else        
      if(Command.compareTo("SET_ECM_DATA_STREAM\n") == 0)
      {  
        /* Wait for Argument with terminating Newline to arrive */
        while(Serial.available() == 0)
        {
          asm(""); /* Keeps Compiler Optimizer from removing empty loop */ 
        }
        Argument = Serial.readString();
      
        if(Argument.compareTo("C3\n") == 0)
        {
          ECM_Data_Stream = Command;
          ECM_Sample_Time = ECM_SAMPLE_TIME_C3;
        }
        else
        if(Argument.compareTo("VN\n") == 0)
        {        
          ECM_Data_Stream = Command;
          ECM_Sample_Time = ECM_SAMPLE_TIME_VN;
        }
        else
        if(Argument.compareTo("C3_OR_VN\n") == 0) 
        {
          ECM_Data_Stream = "C3_OR_VN\n";
          ECM_Sample_Time = ECM_SAMPLE_TIME_C3_OR_VN;
        }
      }
      else
      if(Command.compareTo("MODE_GET_ALDL_DATA\n") == 0)
      {  
        Adapter_Mode = MODE_GET_ALDL_DATA;
      }
      else
      if(Command.compareTo("MODE_GET_ERROR_CODES\n") == 0)
      {
        Adapter_Mode = MODE_GET_ERROR_CODES;
      }
      else
      if(Command.compareTo("SET_DIAGNOSTIC_MODE\n") == 0)
      {  
       /* Wait for Argument with terminating Newline to arrive */
        while(Serial.available() == 0)
        {
          asm(""); /* Keeps Compiler Optimizer from removing empty loop */  
        }
        Argument = Serial.readString();
        
        if(Argument[0] == '0') Value = DIAG_MODE_NORMAL;      /*  >20K   Ohms (or Open)  */
        else
        if(Argument[0] == '1') Value = DIAG_MODE_ALDL;        /*   10K   Ohms            */
        else
        if(Argument[0] == '2') Value = DIAG_MODE_FACTORY;     /*    3.9K Ohms            */
        else
        if(Argument[0] == '3') Value = DIAG_MODE_DIAGNOSTICS; /* <470    Ohms (or Short) */
        else                   Value = DIAG_MODE_NORMAL;
         
        switch(Value)
        {
          default:
          case DIAG_MODE_NORMAL: /* NORM (>20K Ohms  or Open) */
            Diag_Mode = DIAG_MODE_NORMAL;
            /* Set the Diag Mode Select Transitor provide an Open to Ground at the Fiero ALDL Diag Mode Select pin */  
            Mode_DAC = TRANSISTOR_OFF;
            Mode_Status = Adjust_Mode_Voltage(METHOD_SET, Diag_Mode);
            if(Mode_Status == 0)
            {
              #if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
              DotStar_LED.setPixelColor(0, 0xFFFFFF); /* White */
              DotStar_LED.show();
              #endif
              Serial.print("ACK\n");
            }
            else
            {
              Serial.print("NAK\n");
            }
          break;

          case DIAG_MODE_ALDL: /* DMALDL (10K Ohms) */
            Diag_Mode = DIAG_MODE_ALDL;
            /* Set Diag Mode Select Transitor to provide 10K Ohms to Ground at the Fiero ALDL Diag Mode Select pin */  
            Mode_DAC = TRANSISTOR_10K_OHMS;
            Mode_Status = Adjust_Mode_Voltage(METHOD_SET, Diag_Mode);
            if(Mode_Status == 0)
            {
              #if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
              DotStar_LED.setPixelColor(0, 0x2060FF); /* Sky Blue */
              DotStar_LED.show();
              #endif
              Serial.print("ACK\n");
            }
            else
            {
              Serial.print("NAK\n");
            }
          break;

          case DIAG_MODE_FACTORY: /* DMFACT (3.9K Ohms) */
            Diag_Mode = DIAG_MODE_FACTORY;
            /* Set Diag Mode Select Transitor to provide 3.9K Ohms to Ground at the Fiero ALDL Diag Mode Select pin */  
            Mode_DAC = TRANSISTOR_3_9K_OHMS;
            Mode_Status = Adjust_Mode_Voltage(METHOD_SET, Diag_Mode);
            if(Mode_Status == 0)
            {
              #if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
              DotStar_LED.setPixelColor(0, 0x2000FF); /* Violet */
              DotStar_LED.show(); 
              #endif
              Serial.print("ACK\n");
            }  
            else
            {
              Serial.print("NAK\n");
            }    
          break;
          
          case DIAG_MODE_DIAGNOSTICS: /* DMDIAG (<470 Ohms or Short) */
            Diag_Mode = DIAG_MODE_DIAGNOSTICS;
            /* Set Diag Mode Select Transitor to provide a Short to Ground at the Fiero ALDL Diag Mode Select pin */  
            Mode_DAC = TRANSISTOR_SATURATED;
            Mode_Status = Adjust_Mode_Voltage(METHOD_SET, Diag_Mode);
            if(Mode_Status == 0)
            {
              #if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
              DotStar_LED.setPixelColor(0, 0xFFA000); /* Amber */
              DotStar_LED.show();
              #endif
              Serial.print("ACK\n");
            }
            else
            {
              Serial.print("NAK\n");
            }
          break;
        }
      }
      else
      if(Command.compareTo("GET_DIAGNOSTIC_MODE\n") == 0)
      {
        /* Retrun Diag Mode of ALDL Link */
        Serial.write('0' + Diag_Mode);
        Serial.write('\n');
      }
      else
      if(Command.compareTo("MODE_SIMULATION\n") == 0)
      {
        /* Enable Simulation */
        Simulating = SIMULATING_YES;
        digitalWrite(USER_LED_PIN, 1); /* Red LED On */  
      }
      else
      if(Command.compareTo("MODE_MONITOR\n") == 0)
      {
        /* Disable Simulation */
        Simulating = SIMULATING_NO;
        digitalWrite(USER_LED_PIN, 0); /* Red LED Off */  
      }
    }
    
    /* Process Current Command(s) to Modes */
    switch(Adapter_Mode)
    {
      default:
      case MODE_GET_ALDL_DATA: 
        if(Simulating == SIMULATING_YES)
        {
          Simulate_160_ALDL_Link();
          //Simulate_160_ALDL_Partial_Link();
        }
        else
        {
          Monitor_160_ALDL_Link();
        }
      break;
  
      case MODE_GET_ERROR_CODES:
        if(Simulating == SIMULATING_YES)
        {
          Simulate_SES_Signal();
        }
        else
        {
          Monitor_SES_Signal();
        }
      break;
    }  
  }

  /* Termination */

  /* loop() is automatically restarted when compiled using the Arduino IDE */
}


/********************************************************************************
 *
 * 
 *
 ********************************************************************************/
void Monitor_160_ALDL_Link(void) 
{
  /* Initialization */
  
  int           Break_Flag;
  unsigned long Microseconds = 0;
  unsigned long Start_Time = 0;
  unsigned long Pulse_Width = 0;
  int           Level;
  int           Bit_Count = 0;
  int           Sync_Count = 0;
  int           State = STATE_WAIT_SYNC;

  /* Process */
      
  Break_Flag = 0;
  while(1)
  {
    /* 
       Each ALDL Bit lasts 6.25ms in duration, a Active Low Pulse for a Bit '0' is a much less than 3.125ms, 
       but an Active Low Pulse for a Bit '1' is a much more than 3.125ms. 
       pusleIn() could not used because a '1' bit's active pulse is over 5ms and this does not leave enough 
       time to send an ASCII '1' byte (Start + 8Data + Stop at 9600 Baud) over the serial port.
       Measuring the pulse using the microsceond timer allows to abort the pulse measurement after it reaches
       midway through the bit period (a '1' is occuring), thus giving plently of time to send the ASCII '1'
       over the serial port tothe PC at 9600 Baud. 
    */

    /* 
      The ALDL Bit could be in either state (Inactive High or Active Low) at this point.
      Toss out first pulse to be sure not within a partial pulse as the stream is entered.
    */

    /*
                        ms
              ---------------------
                     ECM Type
              ---------------------
                VN     C3  VN or C3
              -----  ----- --------
      Start   0.500  0.351
      Data    4.750  1.557
      Sample  2.875  1.130    1.800
      Stop    1.000  4.381
      Total   6.250  6.289

                         Sample 
                           |
           .Start.    Data |        .Stop .
  VN       .     .         v        .     .
     '0' --|_____|------------------------|
     '1' --|________________________|-----|
                     ^
                     |
                    
                 Sample
                   |
           .Start.Data.        Stop        . 
  C3       .    .  v  .                    .
     '0' --|____|--------------------------|
     '1' --|__________|--------------------|
                     ^
                     |
                  Sample for both VN or C3
                  1.800 ms    
    */

    /* Start by finding an Inactive High to Active Low transition */
    while(digitalRead(ALDL_160_BAUD_PIN) == 1)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      if(Mode_Status == 0)
      {
        /* Adjust Diag Mode DAC Value to compensate for Temperature changes */
        Adjust_Mode_Voltage(METHOD_MAINTAIN, Diag_Mode); /* Should return quickly as the DAC should already be close */     
      }
    }
    if(Break_Flag == 1) break;
    while(digitalRead(ALDL_160_BAUD_PIN) == 0)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}
    }
    if(Break_Flag == 1) break;
  
    /* The ALDL Bit has returned to Inactive High at this point, ready for first full pulse */
    /* Detect ALDL Bits */
    while(1)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}
      
      /* Wait for ALDL Bit to go Active Low */
      while(digitalRead(ALDL_160_BAUD_PIN) == 1)
      {
        /* Check for Command Change */
        if(Serial.available() > 0) {Break_Flag = 1; break;}
      }
      if(Break_Flag == 1) break;
      
      /* Start Timer */
      Start_Time = micros();
      
      /* ALDL Bit just whent Active Low, Timer Started for Pulse Width */ 
      while(1)
      {
        /* Check for Command Change */
        if(Serial.available() > 0) {Break_Flag = 1; break;}
  
        /* Check Timer */
        Microseconds = micros();
        if(Microseconds < Start_Time)
        {
          /* About every 71.583 minutes, micros() will modulated back past 0, adjust for this */
          Microseconds = (0xFFFFFFFF - Start_Time) + Microseconds;
          Start_Time   = 0;
        }
        
        /* Measure Pulse Width */
        Pulse_Width =  Microseconds - Start_Time;
  
        /* Sample Pulse at "Sweet Spot" After Falling Edge (Start) of pulse (SAMPLE_TIME_VN_OR_C3 is a compromise between VN and C3 ECMs of 1500 us) */
        if(Pulse_Width > ECM_SAMPLE_TIME_VN)
        {
          Level = digitalRead(ALDL_160_BAUD_PIN);

          /* Bit Count State Machine */
          switch(State)
          {
            case STATE_WAIT_SYNC:
              if(Level == 0) /* Active Low Pulse is Long (returns to 1 later) a '1' is detected */
              { 
                Sync_Count++;
                if(Sync_Count == 9)
                {
                  /* Nine 1's in a Row */ 
                  State = STATE_SYNC;
                }
              }
            break;

            case STATE_SYNC:
              if(Level == 1) /* Active Low Pulse is Short (returned to 1 early) '0' is detected */
              {
                /* First 0 after at least 9 1's (first data bit) */
                Bit_Count++;
                State = STATE_DATA;
              }
            break;

            case STATE_DATA:
              Bit_Count++; /* Bit_Count 0 is the first Byte's MSB Sync Bit */ 
              if(Bit_Count == 225)
              {
                /* End of Data, Look for next Sync */
                Bit_Count = 0;
                Sync_Count = 0;
                State = STATE_WAIT_SYNC;
              }
            break;
          }

          switch(Bit_Count)
          {
            case 146: // was 137 Wrong Word
              /* Take bit from SES pin not ALDL Stream - Convert Active Low signal to Active High bit */
              #if STUB_OUT_SES
              Serial.print("0");
              #else
              if(digitalRead(ALDL_SES_PIN_N) == 0)  Serial.print("1");
              else                                  Serial.print("0");
              #endif
            break;

            case 147: // was 138 Wrong Word
              /* Take bit from Up Shift pin not ALDL Stream - Convert Active Low signal to Active High bit  */
              #if STUB_OUT_UP_SHIFT
              Serial.print("0");
              #else
              if(digitalRead(ALDL_UP_SHIFT_PIN_N) == 0)  Serial.print("1"); 
              else                                       Serial.print("0");
              #endif
            break;

            default:
              /* Take bit from ALDL Stream */
              if(Level == 1) /* ALDL_160_BAUD_PIN == 1 */
              {
                /* Active Low Pulse is Short (returned to 1 early) '0' is detected */
                Serial.print("0");
              }
              else /* ALDL_160_BAUD_PIN == 0 */
              {
                /* Active Low Pulse is Long (returns to 1 later) a '1' is detected */
                Serial.print("1");
              }
            break;
          }
         
          /* Wait for pulse to return to Inactive High */
          while(digitalRead(ALDL_160_BAUD_PIN) == 0)
          {
            /* Check for Command Change */
            if(Serial.available() > 0) {Break_Flag = 1; break;}
          }
          if(Break_Flag == 1) break;
          
          /* Get Ready for Next Pulse */
          break;
        }
      }
      if(Break_Flag == 1) break;
    }
    if(Break_Flag == 1) break;
  }
      
  /* Termination */
      
  return;
}


/********************************************************************************
 *
 * 
 *
 ********************************************************************************/
void Monitor_SES_Signal(void) 
{
  /* Initialization */

  #if STUB_OUT_SES

  while(true)
  {
    Error_Code_Tens = 0;
    Alive_Counter   = 0;
    
    /* Check for Command Change */
    if(Serial.available() > 0) break;
  }
  
  return;

  #else

  int           Break_Flag;
  unsigned long Microseconds = 0;
  unsigned long Start_Time = 0;
  unsigned long Pulse_Width = 0;
  
  /* Process */

  Break_Flag = 0;
  while(1)
  {
    /* 
       Could not use pulseIn() because timing of both the Active Low and Inactive High states
       would cause missing the On(Active Low)/Off(Inactive High) edges, which is required by pulseIn().
       Measuring the pulses and timing between them using the microsceond timer allows both
       to be measured accurately without missing edges.
    */
      
    /* 
      The SES signal could be in either state (Inactive High or Active Low) at this point.
      Synchronize to Error Code stream, wait for SES to be Inactive High for at least 1.5 seconds (Between Error Codes)   
    */
      
    /* Start by finding an Inactive High to Active Low transition */
    while(digitalRead(ALDL_SES_PIN_N) == 1)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      if(Mode_Status == 0)
      {
        /* Adjust Diag Mode DAC Value to compensate for Temperature changes */
        Adjust_Mode_Voltage(METHOD_MAINTAIN, Diag_Mode); /* Should return quickly as the DAC should already be close */     
      }
      
      Alive_Counter++;
      if((Alive_Counter % SES_ALIVE_COUNT) == 0) Serial.write((unsigned char)0); /* Send not possible EC0 to keep Comms Link Active */
    }
    if(Break_Flag == 1) break;
    while(digitalRead(ALDL_SES_PIN_N) == 0)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      Alive_Counter++;
      if((Alive_Counter % SES_ALIVE_COUNT) == 0) Serial.write((unsigned char)0); /* Send not possible EC0 to keep Comms Link Active */
    }
    if(Break_Flag == 1) break;
    
    /* SES is Inactive High, Start Timer for the Inactive High Pulse Width */
    Start_Time = micros();
    
    /* Wait for SES to go Active Low */
    while(digitalRead(ALDL_SES_PIN_N) == 1)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      Alive_Counter++;
      if((Alive_Counter % SES_ALIVE_COUNT) == 0) Serial.write((unsigned char)0); /* Send not possible EC0 to keep Comms Link Active */
    }
    if(Break_Flag == 1) break;
    
    /*  Stop Timer */ 
    Microseconds = micros();
    if(Microseconds < Start_Time)
    {
      /* About every 71.583 minutes, micros() will modulated back past 0, adjust for this */
      Microseconds = (0xFFFFFFFF - Start_Time) + Microseconds;
      Start_Time   = 0;
    } 
    
    /* Determine Inctive High Pulse Width */
    Pulse_Width = Microseconds - Start_Time;
    
    /* If greater than 1.5 seconds, now Syncronized to the next Error Code Pattern */ 
    if(Pulse_Width > 1500000)
    {
      /* SES is Active Low, Start Timer for the Active Low Pulse Width */
      Error_Code_Tens = 1;
      Error_Code = 0;
      /* In Sync, ready for Error Code Pulse Measurements */
      Start_Time = micros();
      break;
    }
    else
    {
      ; /* Try Next Inactive High Pulse */
    }
  }
  
  /* 
    Measure Error Code pulses. 
    SES just went Active Low after being Inactive High > 1.5 seconds at this point, 
    Pulse Width Timer was started. 
  */
  while(1)
  {
    /* Check for Command Change */
    if(Serial.available() > 0) {Break_Flag = 1; break;}

    Alive_Counter++;
    if((Alive_Counter % SES_ALIVE_COUNT) == 0) Serial.write((unsigned char)0); /* Send not possible EC0 to keep Comms Link Active */
    
    /* Wait for pulse to return to Inactive High */
    while(digitalRead(ALDL_SES_PIN_N) == 0)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      Alive_Counter++;
      if((Alive_Counter % SES_ALIVE_COUNT) == 0) Serial.write((unsigned char)0); /* Send not possible EC0 to keep Comms Link Active */
    }
    if(Break_Flag == 1) break;
    
    /* Stop Timer */ 
    Microseconds = micros();
    if(Microseconds < Start_Time)
    {
      /* About every 71.583 minutes, micros() will modulated back past 0, adjust for this */
      Microseconds = (0xFFFFFFFF - Start_Time) + Microseconds;
      Start_Time   = 0;
    } 
    
    /* Determine Active Low Pulse Width */
    Pulse_Width = Microseconds - Start_Time;
    
    /* Act on Active Low Pulse Width */
    if(Pulse_Width < 500000)
    {
      /* If Inactive Time < 0.5 seconds, count the digit */
      if(Error_Code_Tens == 1) Error_Code += 10;
      else                     Error_Code +=  1;
    }
    else
    {
      /* Error - Active Low pulse should never be more than 0.5 seconds */
    }
    
    /* SES is Inactive High, Start Timer for the Inactive High Pulse Width */
    Start_Time = micros();
    
    /* Wait for pulse to go Active Low */
    while(digitalRead(ALDL_SES_PIN_N) == 1)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      Alive_Counter++;
      if((Alive_Counter % SES_ALIVE_COUNT) == 0) Serial.write((unsigned char)0); /* Send not possible EC0 to keep Comms Link Active */
    }
    if(Break_Flag == 1) break;
    
    /* Stop Timer */ 
    Microseconds = micros();
    if(Microseconds < Start_Time)
    {
      /* About every 71.583 minutes, micros() will modulated back past 0, adjust for this */
      Microseconds = (0xFFFFFFFF - Start_Time) + Microseconds;
      Start_Time   = 0;
    } 
    
    /* Measure Inactive High Pulse Width */
    Pulse_Width = Microseconds - Start_Time;
     
    /* Act on Inactive High Pulse Width */
    if(Pulse_Width < 500000)
    {
      ; /* Inactive Time < 0.5 seconds, still in Current Digit */
    }
    else
    if(Pulse_Width < 1500000)
    {
      /* Inactive Time > 0.5 and < 1.5 seconds, going to next digit */
      Error_Code_Tens = 0;
    }
    else
    {
      /* Inactive Time > 1.5 seconds (on its way to 2.5 seconds), going to next Error Code */
      /* Send Error Code to PC */
      Serial.write(Error_Code);
      /* Prep for Net Error Code */
      Error_Code_Tens = 1;
      Error_Code = 0;
    }

    /* Wait for pulse to go Active Low */
    while(digitalRead(ALDL_SES_PIN_N) == 1)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      Alive_Counter++;
      if((Alive_Counter % SES_ALIVE_COUNT) == 0) Serial.write((unsigned char)0); /* Send not possible EC0 to keep Comms Link Active */
    }
    if(Break_Flag == 1) break;
    
    /* Start Timer for Active Low Pulse Width */
    Start_Time = micros();
  }

  #endif
  
  /* Termination */
  
  return;
}


/********************************************************************************
 *
 * Adjust Tranisitor Base voltage to compensate for delta temperature from 25C.
 *
 ********************************************************************************/
int Adjust_Mode_Voltage(int Method, int Mode)
{
  /* Initialization */

  int    Status = 0;
  double Volts;
  double Target_Volts = 0.0;
  int    Index;
  int    Iterations;

  /* Process */

  if(Method == METHOD_SET)
  {
    unsigned int Timeout;

    /* Show LED Yellow while checking for Pullup Volltage */
    #if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
    DotStar_LED.setPixelColor(0, 0xFFFF00); /* Yellow */
    DotStar_LED.show();
    #endif  
    
    /* Temporarily turn off Transistor to check if 5V is present (with Timeout) */
    analogWrite(ALDL_DIAG_MODE_SELECT_PIN, TRANSISTOR_OFF);
    for(Timeout = 5000; Timeout > 0; Timeout--)
    {
      /* Wait for Ignition to be On (5V present) */ 
      Mode_Monitor_ADC = analogRead(ALDL_MODE_MONITOR_PIN);
      Volts = ((double)Mode_Monitor_ADC / 1023.0) * 3.3;
      /* 
       *  Becuase the Trinket M0 has 3.3V I/O, the inputs are protected with
       *  3.3V Zener diodes, and since 1 1M Ohm isolation resistor is used in the feedback,
       *  this attenuates the 5V to about 1.9V thus, when 5V is present the ADC will see that as about 1.9V
       */
      if(Volts > 1.19) break;
    }
    
    if(Timeout == 0) 
    {
      /* Show LED Red, ECM Mode Pullup 5V is not present (Ignition Off) */
      #if TARGET_BOARD == TARGET_BOARD_ADAFRUIT_TRINKET_M0
      DotStar_LED.setPixelColor(0, 0xFF0000); /* Red */
      DotStar_LED.show();
      #endif
      
      Status = -1;
      goto Termination;
    }
  } 

  /* Set Mode */
  switch(Mode)
  {
    case DIAG_MODE_ALDL:
      if(Method == METHOD_SET)
      {
        /* Set above Set Point to ease down into range */
        Mode_DAC = TRANSISTOR_10K_OHMS;
        analogWrite(ALDL_DIAG_MODE_SELECT_PIN, Mode_DAC);
      }
      Target_Volts = 1.191; /* In circuit attenuation from 2.5V */
    break;

    case DIAG_MODE_FACTORY:
     if(Method == METHOD_SET)
     {
       /* Set below Set Point to ease up into range */
       Mode_DAC = TRANSISTOR_3_9K_OHMS;
       analogWrite(ALDL_DIAG_MODE_SELECT_PIN, Mode_DAC);
     }
     Target_Volts = 1.037; /* In circuit attenuation from 1.4V */
    break;
         
    case DIAG_MODE_DIAGNOSTICS:
      analogWrite(ALDL_DIAG_MODE_SELECT_PIN, Mode_DAC);
      /* No need to adjust as this as tranistor fully on (0V). */
      goto Termination;
    break;

    case DIAG_MODE_NORMAL:
      analogWrite(ALDL_DIAG_MODE_SELECT_PIN, Mode_DAC);
      /* No need to adjust as this as tranistor is fully off (5V).
       Also, Normal Mode is 5V but the ADC Input can be only up to 3.3V Max so it is limited to
       3.3V with an external Zener Diode to protect the input and thus the Normal 5V can't 
       be monitored. 
      */
      goto Termination;
    break;
    
    default:
    break;
  }

  /* Adjust */

  if(Method == METHOD_MAINTAIN) Iterations =    1; /* Adjust many times to Set */
  else                          Iterations = 1024; /* Adjust one time to Maintain */
  
  for(Index = 0; Index < Iterations; Index++)
  {
    Mode_Monitor_ADC = analogRead(ALDL_MODE_MONITOR_PIN);
    Volts = ((double)Mode_Monitor_ADC / 1023.0) * 3.3;

    if(Volts > (Target_Volts + 0.002))
    {
      Mode_DAC++; /* Turn on DAC More to lower Voltage */
      if(Mode_DAC > 1023)
      {
        Mode_DAC = 1023;
        break;
      }
      analogWrite(ALDL_DIAG_MODE_SELECT_PIN, Mode_DAC);
    }
    else 
    if(Volts < (Target_Volts - 0.002)) 
    {
      Mode_DAC--; /* Turn off DAC More to raise Voltage */
      if(Mode_DAC < 0) 
      {
        Mode_DAC = 0;
        break;
      }
      analogWrite(ALDL_DIAG_MODE_SELECT_PIN, Mode_DAC);
    }
    else
    {
      break;
    }
  }
  
  /* Termination */
  
  Termination:
  
  return Status;
}


/********************************************************************************
 *
 * 
 *
 ********************************************************************************/
void Simulate_160_ALDL_Link(void) 
{
  /* Initialization */
    
  int Break_Flag;

  /* Process */

  Break_Flag = 0;
  while(1)
  {
    if(Mode_Status == 0)
    {
      /* Adjust Diag Mode DAC Value to compensate for Temperature changes */
      Adjust_Mode_Voltage(METHOD_MAINTAIN, Diag_Mode); /* Should return quickly as the DAC should already be close */     
    }
      
    /* Sync Byte: 9 Bits (all '1's) */
    for(int Bit = 0; Bit < 9; Bit++)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      /* Send Bit */
      Serial.print("1");
     
      /* Simulate Bit Timing */
      delay(ALDL_BIT_DELAY_MS);
    }
    if(Break_Flag == 1) break;
            
    /* 24 Incrementing Data Bytes: 9 Bits Each */
    for(int Byte = 0; Byte < 25; Byte++)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}
      
      /* Send a Sync Bit to PC ('0' for Data) */
      Serial.print("0");

       /* Simulate Bit Timing */
      delay(ALDL_BIT_DELAY_MS);
            
      /* 8 Data Bits */ 
      Mask = 0x80;
      for(int Bit = 0; Bit < 8; Bit++)
      {
        /* Check for Command Change */
        if(Serial.available() > 0) {Break_Flag = 1; break;}

        /* Send Bit to PC */
        if((Data & Mask) == Mask) Serial.print("1");
        else                      Serial.print("0");
        Mask >>= 1;
        Mask &= 0x7F;
             
        /* Simulate Bit Timing */
        delay(ALDL_BIT_DELAY_MS);
      }   
      if(Break_Flag == 1) break; 

      /* Next Data Pattern */
      Data++;
      if(Data > 0xFF) Data = 0x00;
    }
    if(Break_Flag == 1) break;
  }

  /* Termination */
  
  return;
}


/********************************************************************************
 *
 * 
 *
 ********************************************************************************/
void Simulate_160_ALDL_Partial_Link(void) 
{
  /* Initialization */
    
  int Break_Flag;

  /* Process */

  Break_Flag = 0;
  while(1)
  {
    if(Mode_Status == 0)
    {
      /* Adjust Diag Mode DAC Value to compensate for Temperature changes */
      Adjust_Mode_Voltage(METHOD_MAINTAIN, Diag_Mode); /* Should return quickly as the DAC should already be close */     
    }
      
    /* Sync Byte: 9 Bits (all '1's) */
    for(int Bit = 0; Bit < 9; Bit++)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}

      /* Send Bit */
      Serial.print("1");
     
      /* Simulate Bit Timing */
      delay(ALDL_BIT_DELAY_MS);
    }
    if(Break_Flag == 1) break;
            
    /* 4 Incrementing Data Bytes: 9 Bits Each */
    for(int Byte = 0; Byte < 4; Byte++)
    {
      /* Check for Command Change */
      if(Serial.available() > 0) {Break_Flag = 1; break;}
      
      /* Send a Sync Bit to PC ('0' for Data) */
      Serial.print("0");

       /* Simulate Bit Timing */
      delay(ALDL_BIT_DELAY_MS);
            
      /* 8 Data Bits */ 
      Mask = 0x80;
      for(int Bit = 0; Bit < 8; Bit++)
      {
        /* Check for Command Change */
        if(Serial.available() > 0) {Break_Flag = 1; break;}

        /* Send Bit to PC */
        if((Data & Mask) == Mask) Serial.print("1");
        else                      Serial.print("0");
        Mask >>= 1;
        Mask &= 0x7F;
             
        /* Simulate Bit Timing */
        delay(ALDL_BIT_DELAY_MS);
      }   
      if(Break_Flag == 1) break; 

      /* Next Data Pattern */
      Data++;
      if(Data > 0xFF) Data = 0x00;
    }
    if(Break_Flag == 1) break;
  }

  /* Termination */
  
  return;
}

/********************************************************************************
 *
 * 
 *
 ********************************************************************************/
void Simulate_SES_Signal(void) 
{
  /* Initialization */
    
  int Break_Flag;

  /* Process */

  Break_Flag = 0;
  while(1)
  { 
    if(Mode_Status == 0)
    {
      /* Adjust Diag Mode DAC Value to compensate for Temperature changes */
      Adjust_Mode_Voltage(METHOD_MAINTAIN, Diag_Mode); /* Should return quickly as the DAC should already be close */     
    }
      
    /* Emulate 3 EC12's */
    Error_Code = 12;
    Error_Code_Time_ms = Determine_Error_Code_Time_In_Millisec(Error_Code);
    Fine_Delay_ms = (int)(Error_Code_Time_ms / ERROR_CODE_COARSE_DELAY_COUNTS);
    for(int Repeat = 0; Repeat < 3; Repeat++)
    {
      for(int Coarse_Delay = 0; Coarse_Delay < ERROR_CODE_COARSE_DELAY_COUNTS; Coarse_Delay++)
      {
        /* Check for Command Change */
        if(Serial.available() > 0) {Break_Flag = 1; break;}

        /* Send not possible EC0 to keep Comms Link Active */
        Serial.write((unsigned char)0); 
        
        /* Simulate Error Code Timing */
        delay(Fine_Delay_ms);
      } 
      if(Break_Flag == 1) break;
      
      /* Send the Error Code */
      Serial.write(Error_Code); 
      
      /* Simulate Error Code Timing */
      delay(ERROR_CODE_SPACE_MS);
    }
    if(Break_Flag == 1) break;

    /* Emulate 3 EC13's */
    Error_Code = 13;
    Error_Code_Time_ms = Determine_Error_Code_Time_In_Millisec(Error_Code);
    Fine_Delay_ms = (int)(Error_Code_Time_ms / ERROR_CODE_COARSE_DELAY_COUNTS);
    for(int Repeat = 0; Repeat < 3; Repeat++)
    {
      for(int Coarse_Delay = 0; Coarse_Delay < ERROR_CODE_COARSE_DELAY_COUNTS; Coarse_Delay++)
      {
        /* Check for Command Change */
        if(Serial.available() > 0) {Break_Flag = 1; break;}

        /* Send not possible EC0 to keep Comms Link Active */
        Serial.write((unsigned char)0); 

        /* Simulate Error Code Timing */
        delay(Fine_Delay_ms);
      }
      if(Break_Flag == 1) break;
      
      /* Send the Error Code */
      Serial.write(Error_Code); 
      
      /* Simulate Error Code Timing */
      delay(ERROR_CODE_SPACE_MS);
    }
    if(Break_Flag == 1) break;
    
    /* Emulate 3 EC61's */

    Error_Code = 61;
    Error_Code_Time_ms = Determine_Error_Code_Time_In_Millisec(Error_Code);
    Fine_Delay_ms = (int)(Error_Code_Time_ms / ERROR_CODE_COARSE_DELAY_COUNTS);
    for(int Repeat = 0; Repeat < 3; Repeat++)
    {
      for(int Coarse_Delay = 0; Coarse_Delay < ERROR_CODE_COARSE_DELAY_COUNTS; Coarse_Delay++)
      {
        /* Check for Command Change */
        if(Serial.available() > 0) {Break_Flag = 1; break;}

        /* Send not possible EC0 to keep Comms Link Active */
        Serial.write((unsigned char)0); 

        /* Simulate Error Code Timing */
        delay(Fine_Delay_ms);
      }
      if(Break_Flag == 1) break;
      
      /* Send the Error Code */
      Serial.write(Error_Code);
      
      /* Simulate Error Code Timing */
      delay(ERROR_CODE_SPACE_MS);
    }
    if(Break_Flag == 1) break;
  }
  
  /* Termination */
  
  return;
} 
 

/********************************************************************************
 *
 * Error Code 99 may take up to 13.0 seconds but the highest expected Error Code 
 * for a Fiero is 55 which takes about 11.5 seconds. EC12 takes 6.0 seconds.
 *
 ********************************************************************************/
int Determine_Error_Code_Time_In_Millisec(int Error_Code)
{
  /* Initialization */
    
  unsigned long Time_ms;
  unsigned int  Tens;
  unsigned int  Ones;
  unsigned long Pause;

  /* Process */
  
  if(Error_Code > 0)
  {
    Tens = Error_Code / 10;
    Ones = Error_Code - (Tens * 10);

    if(Tens > 0) Pause = ERROR_CODE_PAUSE_MS;
    else         Pause = 0;
   
    Time_ms = (Tens * (ERROR_CODE_PULSE_ON_MS + ERROR_CODE_PULSE_OFF_MS)) + 
               Pause + 
              (Ones * (ERROR_CODE_PULSE_ON_MS + ERROR_CODE_PULSE_OFF_MS)) +
               Pause;
  }
  else
  {
    Time_ms = 0;
  }

  /* Termination */
  
  return Time_ms;
}

/* End */

