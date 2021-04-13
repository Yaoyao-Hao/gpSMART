/*
   gpSMART: A general purpose State MAchine Runner for Training animal behaviors.
   This library enables constructing and running state machine on a single Teensy (3.6) board.
   Created by Yaoyao Hao (yaoyaoh90@gmail.com) 2021.
*/


// Inspired by Bpod Finite State Machine v0.5 https://github.com/sanworks/Bpod_StateMachine_Firmware


#ifndef GPSMART_H
#define GPSMART_H

#include "Arduino.h"
#include "String.h"

#define MAX_STATE_NUM     64    // max number of state in gpSMART
#define GLOBAL_TC_NUM     2     // number of Timer/Counter in gpSMART
#define SOFT_EVENT_NUM    4     // number of Soft events
#define FREQ_NUM_PER_PWM  4     // number of frequencies per true PWM

#define MAX_EVENT_NUM     4096  // max number of events in a trial
#define MAX_VISIT_NUM     512   // max number of state transitions in a trial

/* Teensy 3.6 PWM hardware timers
 * FTM0  5, 6, 9, 10, 20, 21, 22, 23 488.28 Hz
 * FTM1  3, 4                        488.28 Hz
 * FTM2  29, 30                      488.28 Hz
 * FTM3  2, 7, 8, 14, 35, 36, 37, 38 488.28 Hz
 * TPM1  16, 17                      488.28 Hz
 */
 /* Assignment
  * 0, 1 for Serial1
  * 4P, 7P, 8P, 37PA, 38PA, 39A for Controller (P FOR PWM, A FOR ANALOG)
  * 11, 12 for MOSI
  * 13 for LED
  * 18, 19 for I2C
  */
const byte gpSMART_DI_Lines[]   = {14, 15, 17, 24, 25, 26, 27, 28};
const byte gpSMART_DO_Lines[]   = {29, 30, 31, 32, 33, 34, 35, 36};
const byte gpSMART_PWM_Lines[]  = { 5,  6,  9, 10, 20, 21, 22, 23};
const byte gpSMART_tPWM_Lines[] = { 2,  3, 16, 29}; // independent PWMs

const byte nDIOs  = sizeof(gpSMART_DI_Lines) / sizeof(gpSMART_DI_Lines[0]);      // 8
const byte nPWMs  = sizeof(gpSMART_PWM_Lines) / sizeof(gpSMART_PWM_Lines[0]);    // 8
const byte ntPWMs  = sizeof(gpSMART_tPWM_Lines) / sizeof(gpSMART_tPWM_Lines[0]); // 4 

// Constant variables
const PROGMEM String EventNames[] = { // 25 in total = 2 * nDIOs + SOFT_EVENT_NUM + 1 + 2 * GLOBAL_TC_NUM
  // Digital input rising edge or falling edge
  "DI1Rising", "DI1Falling", 	// Digital Input Line 1 Rising and Falling
  "DI2Rising", "DI2Falling", 	// Digital Input Line 2 Rising and Falling
  "DI3Rising", "DI3Falling", 	// Digital Input Line 3 Rising and Falling
  "DI4Rising", "DI4Falling", 	// Digital Input Line 4 Rising and Falling
  "DI5Rising", "DI5Falling", 	// Digital Input Line 5 Rising and Falling
  "DI6Rising", "DI6Falling", 	// Digital Input Line 6 Rising and Falling
  "DI7Rising", "DI7Falling", 	// Digital Input Line 7 Rising and Falling
  "DI8Rising", "DI8Falling", 	// Digital Input Line 8 Rising and Falling

  // softevent sent from Controller
  "SoftEvent1", 		// SoftEvent == 1
  "SoftEvent2",  		// SoftEvent == 2
  "SoftEvent3",  		// SoftEvent == 3
  "SoftEvent4",  		// SoftEvent == 4

  "Tup",				// Time up event (i.e., State Duration is up)

  "GlobalTimer1_End", 	// Global Timer 1 is up
  "GlobalTimer2_End", 	// Global Timer 2 is up

  "GlobalCounter1_End", // Global Counter 1 reached threshold
  "GlobalCounter2_End"  // Global Counter 2 reached threshold

};

const PROGMEM String OutputActionNames[] = { // 25 in total = 8 nDIOs + 8 nPWMs + 4 ntPWMs + 2 Serial + 3 Global
  // Output action name list.
  "DO1", 				// Digital Output Line 1
  "DO2", 				// Digital Output Line 2
  "DO3", 				// Digital Output Line 3
  "DO4", 				// Digital Output Line 4
  "DO5", 				// Digital Output Line 5
  "DO6", 				// Digital Output Line 6
  "DO7", 				// Digital Output Line 7
  "DO8", 				// Digital Output Line 8

  "PWM1", 				// regular PWM Line 1 i.e, defined by analogwrite()
  "PWM2", 				// regular PWM Line 2 ...  Frequency can be changed by analogWriteFrequency()
  "PWM3", 				// regular PWM Line 3 ...  If change one pin's frequency, all other pins change
  "PWM4", 				// regular PWM Line 4 ...  Default frequency 488.28 Hz
  "PWM5",         // regular PWM Line 5
  "PWM6",         // regular PWM Line 6
  "PWM7",         // regular PWM Line 7
  "PWM8",         // regular PWM Line 8

  "tPWM1", 				// true PWM Line 1 i.e., each tPWM can be independently set to different frequencies
  "tPWM2", 				// true PWM Line 2 ...   each tPWM can have upto 4 frequencies available
  "tPWM3", 				// true PWM Line 3
  "tPWM4", 				// true PWM Line 4

  "Serial1Code",   		  // Send code (1-255, 0 will be ignored) to Serial1
  "SerialCode", 		    // Send code (1-255, 0 will be ignored) to Serial (connected to PC)

  "GlobalTimerTrig",    // Start global timer 1-GLOBAL_TC_NUM
  "GlobalTimerCancel",  // Cancel global timer 1-GLOBAL_TC_NUM
  "GlobalCounterReset"  // Reset global counter 1-GLOBAL_TC_NUM

  /* other output actions to be included in future */
  // "Wire"
  // "SPI"
  // "DAC0/1"
  // "CANTX"
};

const byte nInputs  = sizeof(EventNames) / sizeof(EventNames[0]);               // 25
const byte nOutputs = sizeof(OutputActionNames) / sizeof(OutputActionNames[0]); // 25

const int TimerScaleFactor = 10; // from msec (defined in state machine) to gpSMART resolution (0.1 ms)

// important structures
struct OutputAction {
  String ActionType;
  byte ActionValue;
};
struct StateTransition {
  String TransitionTrigger;
  String TransitionTarget;
};

struct gpSMART_State {
  String Name;
  float Duration;	// msec (instead of sec)
  int nTransition = 0;
  StateTransition *Transition;
  int nAction = 0;
  OutputAction *Action;
};

struct StateMatrix {
  byte nStates                                           = 0;   	// number of States
  String StateNames[MAX_STATE_NUM]                   	   = {}; 		// State names in the order they were added
  unsigned long StateTimers[MAX_STATE_NUM]           	   = {};
  byte StatesDefined[MAX_STATE_NUM]                  	   = {}; 		// Referenced states are set to 0. Defined states are set to 1. Both occur with AddState

  byte InputMatrix[MAX_STATE_NUM][nInputs]               = {};
  byte OutputMatrix[MAX_STATE_NUM][nOutputs]             = {};

  unsigned long GlobalTimerThresholds[GLOBAL_TC_NUM]     = {}; 		//
  byte GlobalCounterAttachedEvents[GLOBAL_TC_NUM]        = {nInputs, nInputs};	// Default event of nInputs is code for "no event attached".
  unsigned long GlobalCounterThresholds[GLOBAL_TC_NUM]   = {};
};

struct TrialResult {
  uint16_t nEvent;
  unsigned long eventTimeStamps[MAX_EVENT_NUM]  = {};
  byte EventID[MAX_EVENT_NUM]                   = {};
  uint16_t nVisited;
  byte stateVisited[MAX_VISIT_NUM] = {};
};

extern TrialResult trial_res;	// store events and state transition in one trial
extern volatile bool smartFinished; 		// indicating if current trial is finished
extern bool smartRunning;  		// indicating if state matrix for current trial is running

// main class
class gpSMART {
  public:
    // Construction
    gpSMART();
    // Fucntions
    void setDigitalInputsEnabled(byte* PortEnabled);
    void setTruePWMFrequency(byte tPWM_num, byte freq_num, uint32_t freq, byte duty);
    void EmptyMatrix();
    gpSMART_State CreateState(String Name, float Duration, int nTransition, StateTransition* Transition, int nAction, OutputAction* Action);
    int AddBlankState(String statename);
    int AddState(gpSMART_State *State);
    void SetGlobalTimer(byte TimerNumber, float TimerDuration);
    void SetGlobalCounter(byte CounterNumber, String TargetEventName, uint16_t Threshold);
    void PrintMatrix();
    void Run();
    void Stop();
    void ManualOverride(String TargetAction, byte DataByte);
    void SendSoftEvent(byte EventNum);

  private:

};

// other functions for gpSMART
void gpSMART_Runner(); // ~15 usec
void setStateOutputs(byte State);
void setTruePWMOutput(byte tPWM_num, byte freq_num);

// other public functions
int find_idx(const String * str_array, int array_length, String target);

#endif
