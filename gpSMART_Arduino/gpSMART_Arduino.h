/*
   gpSMART: A general purpose State MAchine Runner for Training animal behaviors.
   This library enables constructing and running state machine on a single Arduino (due) board.
   Created by Yaoyao Hao (yaoyaoh90@gmail.com) 2020.
*/


// Based on Bpod Finite State Machine v0.5 https://github.com/sanworks/Bpod_StateMachine_Firmware
// Requires the DueTimer library from: https://github.com/ivanseidel/DueTimer
// Requires pwm_lib library from: https://github.com/antodom/pwm_lib


#ifndef GPSMART_H
#define GPSMART_H

#include "Arduino.h"
#include "String.h"
#include "DueTimer.h" // gpSMART_Timer
#include "pwm_lib.h"  // pwm lib for tPWM1-4

#define MAX_STATE_NUM     64    // max number of state in gpSMART
#define GLOBAL_TC_NUM     2     // number of Timer/Counter in gpSMART
#define SOFT_EVENT_NUM    4     // number of Soft events
#define FREQ_NUM_PER_PWM  4     // number of frequencies per true PWM

#define MAX_EVENT_NUM     4096  // max number of events in a trial
#define MAX_VISIT_NUM     512   // max number of state transitions in a trial

using namespace arduino_due::pwm_lib;

const byte gpSMART_DI_Lines[]   = {26, 28, 30, 32, 34, 36, 38, 40};
const byte gpSMART_DO_Lines[]   = {27, 29, 31, 33, 35, 37, 39, 41};
const byte gpSMART_PWM_Lines[]  = { 2,  3,  4,  5};
const byte gpSMART_tPWM_Lines[] = {42, 43, 44, 45};

const byte nDIOs  = sizeof(gpSMART_DI_Lines) / sizeof(gpSMART_DI_Lines[0]);
const byte nPWMs  = sizeof(gpSMART_PWM_Lines) / sizeof(gpSMART_PWM_Lines[0]);

// Constant variables
const PROGMEM String EventNames[] = { // 25 in total = 2 * nDIOs + SOFT_EVENT_NUM + 1 + 2 * GLOBAL_TC_NUM
  // Event codes list.
  "DI1Rising", "DI1Falling", 	// Digital Input Line 1 Rising and Falling
  "DI2Rising", "DI2Falling", 	// Digital Input Line 2 Rising and Falling
  "DI3Rising", "DI3Falling", 	// Digital Input Line 3 Rising and Falling
  "DI4Rising", "DI4Falling", 	// Digital Input Line 4 Rising and Falling
  "DI5Rising", "DI5Falling", 	// Digital Input Line 5 Rising and Falling
  "DI6Rising", "DI6Falling", 	// Digital Input Line 6 Rising and Falling
  "DI7Rising", "DI7Falling", 	// Digital Input Line 7 Rising and Falling
  "DI8Rising", "DI8Falling", 	// Digital Input Line 8 Rising and Falling

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

const PROGMEM String OutputActionNames[] = { // 21 in total = nDIOs + 2 * nPWMs + 5
  // Output action name list.
  "DO1", 				// Digital Output Line 1
  "DO2", 				// Digital Output Line 2
  "DO3", 				// Digital Output Line 3
  "DO4", 				// Digital Output Line 4
  "DO5", 				// Digital Output Line 5
  "DO6", 				// Digital Output Line 6
  "DO7", 				// Digital Output Line 7
  "DO8", 				// Digital Output Line 8

  "PWM1", 				// regular PWM Line 1 (i.e, defined by analogwrite())
  "PWM2", 				// regular PWM Line 2
  "PWM3", 				// regular PWM Line 3
  "PWM4", 				// regular PWM Line 4

  "tPWM1", 				// true PWM Line 1 (i.e., both frequency and duty changable)
  "tPWM2", 				// true PWM Line 2
  "tPWM3", 				// true PWM Line 3
  "tPWM4", 				// true PWM Line 4

  "Serial3Code",   		  // Send code (1-255, 0 will be ignored) to Serial3
  "SerialUSBCode", 		  // Send code (1-255, 0 will be ignored) to SerialUSB (connected to PC)

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
const byte nOutputs = sizeof(OutputActionNames) / sizeof(OutputActionNames[0]); // 21

const int TimerScaleFactor = 10; // from msec to gpSMART resolution (0.1 ms)

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
extern volatile bool smartRunning;  		// indicating if state matrix for current trial is running

// main class
class gpSMART {
  public:
    // Construction
    gpSMART();
    // Fucntions
    void setDigitalInputsEnabled(byte* PortEnabled);
    void setTruePWMFrequency(byte tPWM_num, byte freq_num, float frequency, float duty);
    void EmptyMatrix();
    gpSMART_State CreateState(String Name, float Duration, int nTransition, StateTransition* Transition, int nAction, OutputAction* Action);
    int AddBlankState(String statename);
    int AddState(gpSMART_State *State);
    int SetGlobalTimer(byte TimerNumber, float TimerDuration);
    int SetGlobalCounter(byte CounterNumber, String TargetEventName, uint16_t Threshold);
    void PrintMatrix();
    void Run();
    void Stop();
    void ManualOverride(String TargetAction, byte DataByte);
    void SendSoftEvent(byte EventNum);

  private:

};

// other functions for gpSMART
void gpSMART_Runner();
void setStateOutputs(byte State);
void setTruePWMOutput(byte tPWM_num, byte freq_num);

// other public functions
void digitalWriteDirect(byte pin, bool val);
byte digitalReadDirect(byte pin);
int find_idx(const String * str_array, int array_length, String target);

#endif
