#ifndef IDLE_H
#define IDLE_H

#include "globals.h"
#include "table2d.h"
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

#define IAC_ALGORITHM_NONE        0
#define IAC_ALGORITHM_ONOFF       1
#define IAC_ALGORITHM_PWM_OL      2
#define IAC_ALGORITHM_PWM_CL      3
#define IAC_ALGORITHM_STEP_OL     4
#define IAC_ALGORITHM_STEP_CL     5
#define IAC_ALGORITHM_PWM_OLCL    6 //Openloop plus closedloop IAC control
#define IAC_ALGORITHM_STEP_OLCL   7 //Openloop plus closedloop IAC control

//#define IAC_ALGORITHM_HB          8
#define IAC_ALGORITHM_HB_ITPS_OL  8 //Test for ITPS based idle
#define IAC_ALGORITHM_HB_ITPS_OL2 9 //For Using PID
#define IAC_ALGORITHM_HB_ITPS_FFT 10 //For FeedForwardTerm test

#define IDLE_PIN_LOW()  *idle_pin_port &= ~(idle_pin_mask)
#define IDLE_PIN_HIGH() *idle_pin_port |= (idle_pin_mask)
#define IDLE2_PIN_LOW()  *idle2_pin_port &= ~(idle2_pin_mask)
#define IDLE2_PIN_HIGH() *idle2_pin_port |= (idle2_pin_mask)

#define HB_DIR_PIN_1_LOW()  *hbDir1_pin_port &= ~(hbDir1_pin_mask)
#define HB_DIR_PIN_1_HIGH()  *hbDir1_pin_port |= (hbDir1_pin_mask)
#define HB_DIR_PIN_2_LOW()  *hbDir2_pin_port &= ~(hbDir2_pin_mask)
#define HB_DIR_PIN_2_HIGH()  *hbDir2_pin_port |= (hbDir2_pin_mask)

#define STEPPER_FORWARD 0
#define STEPPER_BACKWARD 1
#define IDLE_TABLE_SIZE 10

enum StepperStatus {SOFF, STEPPING, COOLING}; //The 2 statuses that a stepper can have. STEPPING means that a high pulse is currently being sent and will need to be turned off at some point.

struct StepperIdle
{
  int curIdleStep; //Tracks the current location of the stepper
  int targetIdleStep; //What the targeted step is
  volatile StepperStatus stepperStatus;
  volatile unsigned long stepStartTime;
  byte lessAirDirection;
  byte moreAirDirection;
};

struct table2D iacPWMTable;
//struct table2D iacStepTable;
struct table2D hbITPSTable;
//Open loop tables specifically for cranking
//struct table2D iacCrankStepsTable;
struct table2D iacCrankDutyTable;
struct table2D hbCrankPositionTable;

struct StepperIdle idleStepper;
bool idleOn; //Simply tracks whether idle was on last time around
byte idleInitComplete = 99; //Tracks which idle method was initialised. 99 is a method that will never exist
unsigned int iacStepTime_uS;
unsigned int iacCoolTime_uS;
unsigned int completedHomeSteps;

volatile PORT_TYPE *idle_pin_port;
volatile PINMASK_TYPE idle_pin_mask;
volatile PORT_TYPE *idle2_pin_port;
volatile PINMASK_TYPE idle2_pin_mask;
volatile PORT_TYPE *idleUpOutput_pin_port;
volatile PINMASK_TYPE idleUpOutput_pin_mask;
volatile PORT_TYPE *hbDir1_pin_port;
volatile PINMASK_TYPE hbDir1_pin_mask;
volatile PORT_TYPE *hbDir2_pin_port;
volatile PINMASK_TYPE hbDir2_pin_mask;

volatile bool idle_pwm_state;
bool lastDFCOValue;
unsigned int idle_pwm_max_count; //Used for variable PWM frequency
volatile unsigned int idle_pwm_cur_value; //current PWM value
long idle_pid_target_value;
uint16_t idle_pid_hb_target_value;
long FeedForwardTerm;
unsigned long idle_pwm_target_value; //Target PWM value
long idle_cl_target_rpm;
byte idleCounter; //Used for tracking the number of calls to the idle control function
uint8_t idleTaper;

bool lastIdleStatus;
bool itpsIn;
byte holdValue;
byte lastValue;
byte hbValue;
byte incrDuty;
long idleLoad_value;
long idle_target_itps;
long idle_pid_itps_target_value;
uint16_t idle_pidi_itps_target_value;
uint16_t hb_idle_target;

byte idleUpOutputHIGH = HIGH; // Used to invert the idle Up Output 
byte idleUpOutputLOW = LOW;   // Used to invert the idle Up Output 

void initialiseIdle();
void initialiseIdleUpOutput();
void disableIdle();
void idleInterrupt();

#endif
