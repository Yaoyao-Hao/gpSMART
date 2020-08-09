/*
   gpSMART: A general purpose State MAchine Runner for Training animal behaviors.
   This library enables constructing and running state machine on a single Arduino (due) board.
   Created by Yaoyao Hao (yaoyaoh90@gmail.com) 2020.
*/


#include "gpSMART.h"


/******************************************************************/
/************************ hardware related ************************/
/******************************************************************/
// using Timer3 for gpSMART
DueTimer gpSMART_Timer = DueTimer(3);
// true PWMs
pwm<pwm_pin::PWMH1_PA19> tPWM1;
pwm<pwm_pin::PWML2_PA20> tPWM2;
pwm<pwm_pin::PWMH5_PC19> tPWM3;
pwm<pwm_pin::PWMH6_PC18> tPWM4;
// period and duty for true PWMs
uint32_t tPWM1_period[FREQ_NUM_PER_PWM] = {100000, 200000, 300000, 400000}; // 1000-4000 us (i.e., 1000-4000 Hz)
uint32_t tPWM1_duty[FREQ_NUM_PER_PWM]   = {50000,  100000, 150000, 200000}; // 500-2000  us (i.e., 50% duty)
uint32_t tPWM2_period[FREQ_NUM_PER_PWM] = {100000, 200000, 300000, 400000}; //
uint32_t tPWM2_duty[FREQ_NUM_PER_PWM]   = {50000,  100000, 150000, 200000}; //
uint32_t tPWM3_period[FREQ_NUM_PER_PWM] = {100000, 200000, 300000, 400000}; //
uint32_t tPWM3_duty[FREQ_NUM_PER_PWM]   = {50000,  100000, 150000, 200000}; //
uint32_t tPWM4_period[FREQ_NUM_PER_PWM] = {100000, 200000, 300000, 400000}; //
uint32_t tPWM4_duty[FREQ_NUM_PER_PWM]   = {50000,  100000, 150000, 200000}; //
// Digital Input Enables
byte DigitalInputsEnabled[nDIOs] = {1, 1, 1, 1, 1, 1, 1, 1};


/*******************************************************************/
/********** public variables for both gpSMART and main.ino *********/
/*******************************************************************/
TrialResult trial_res;		// store events and state transitions in one trial
bool volatile smartFinished = false; 	// indicating if current trial is finished
bool smartRunning = false; 		// indicating if state matrix for current trial is running


/*****************************************************************/
/****************** public variables for gpSMART *****************/
/*****************************************************************/
StateMatrix sma;   // State Matrix
// Other public variables
bool PortInputLineValue[nDIOs] = {0}; // Direct reads of digital values of IR beams
bool PortInputLineLastKnownStatus[nDIOs] = {0}; // Last known status of IR beams
int CurrentState = 0; // What state is the state machine currently in?
int NewState = 0;
byte CurrentEvent[nDIOs + 2 + 2 * GLOBAL_TC_NUM] = {0}; // What event code just happened and needs to be handled. Up to 14 can be acquired per loop.
byte nCurrentEvents = 0; // Index of current event
byte SoftEvent = 0; // What soft event code just happened
bool GlobalTimersActive[GLOBAL_TC_NUM] = {0}; // 0 if timer x is inactive, 1 if it's active.
unsigned long GlobalTimerEnd[GLOBAL_TC_NUM] = {0}; // Future Times when active global timers will elapse
unsigned long GlobalCounterCounts[GLOBAL_TC_NUM] = {0}; // Event counters
unsigned long StateStartTime = 0; // Session Start Time
unsigned long CurrentTime = 0; // Current time in gpSMART state machine (units = timer cycles since start; used to control state transitions)


/******************************************************************/
/************************ Class Functions *************************/
/******************************************************************/
gpSMART::gpSMART() {
  // init gpSMART hardware ports
  for (int x = 0; x < nDIOs; x++) {
    pinMode(gpSMART_DI_Lines[x], INPUT_PULLUP);
  }
  for (int x = 0; x < nDIOs; x++) {
    pinMode(gpSMART_DO_Lines[x], OUTPUT);
    digitalWriteDirect(gpSMART_DO_Lines[x], 0);
  }
  for (int x = 0; x < nPWMs; x++) {
    pinMode(gpSMART_PWM_Lines[x], OUTPUT);
    analogWrite(gpSMART_PWM_Lines[x], 0);
    setTruePWMOutput(x + 1, 0);
  }
  // init Serial ports used by gpSMART
  Serial3.begin(115200);
  SerialUSB.begin(115200);
  // attach ISR to gpSMART_Timer
  gpSMART_Timer.attachInterrupt(gpSMART_Runner);
  gpSMART_Timer.setPeriod(100); // Runs every 100us
}

void gpSMART::setDigitalInputsEnabled(byte* PortEnabled) {
  for (int i = 0; i < nDIOs; i++) {
    DigitalInputsEnabled[i] = PortEnabled[i];
  }
}

void gpSMART::setTruePWMFrequency(byte tPWM_num, byte freq_num, float frequency, float duty) {
  if (freq_num > 0 && freq_num <= FREQ_NUM_PER_PWM) {
    uint32_t period = 100000000 / frequency;
    uint32_t duty_period = period * duty;
    switch (tPWM_num) {
      case 1:
        tPWM1_period[freq_num - 1] = period;
        tPWM1_duty[freq_num - 1] = duty_period;
        break;
      case 2:
        tPWM2_period[freq_num - 1] = period;
        tPWM2_duty[freq_num - 1] = duty_period;
        break;
      case 3:
        tPWM3_period[freq_num - 1] = period;
        tPWM3_duty[freq_num - 1] = duty_period;
        break;
      case 4:
        tPWM4_period[freq_num - 1] = period;
        tPWM4_duty[freq_num - 1] = duty_period;
        break;
      default:
        break;
    }
  }
}

void gpSMART::EmptyMatrix() {
  sma = StateMatrix();
}

gpSMART_State gpSMART::CreateState(String Name,                          // State Name
                                   float Duration,                       // State Timer
                                   int nTransition,                      // Number of Conditions
                                   StateTransition* Transition,          // State Transition Conditions
                                   int nAction,                          // Number of Output Actions
                                   OutputAction* Action)                 // Output Actions
{
  gpSMART_State newState;
  newState.Name = Name;
  newState.Duration = Duration;
  newState.nTransition = nTransition;
  newState.Transition = Transition;
  newState.nAction = nAction;
  newState.Action = Action;
  return newState;
}

int gpSMART::AddBlankState(String statename) {
  if (statename.compareTo("exit") == 0) {
    return 0;
  }
  sma.StateNames[sma.nStates] = statename;
  sma.StatesDefined[sma.nStates] = 0;
  sma.nStates++;
  return 0;
}

int gpSMART::AddState(gpSMART_State *State)
{
  // Check whether the new state has already been referenced. Add new blank state to matrix.
  int CurrentStateNum;
  int referred = 0;
  for (int i = 0; i < sma.nStates; i++) {
    // Exit if state is already defined.
    if (sma.StateNames[i].compareTo(State->Name) == 0 && sma.StatesDefined[i] == 1) {
      return -1;
    }

    // Check if the state is already referred.
    if (sma.StateNames[i].compareTo(State->Name) == 0 && sma.StatesDefined[i] == 0) {
      referred = 1;
      CurrentStateNum = i;
      break;
    }
  }

  if (referred == 0) {
    CurrentStateNum = sma.nStates;
    sma.nStates++;
    sma.StateNames[CurrentStateNum] = State->Name;
  }

  // Make sure all the states in "StateChangeConditions" exist, and if not, create them as undefined states.
  for (int i = 0; i < State->nTransition; i++) {
    int flag = 0;
    for (int j = 0; j < sma.nStates; j++) {
      if (State->Transition[i].TransitionTarget.compareTo(sma.StateNames[j]) == 0) {
        flag = 1;
        break;
      }
    }
    if (flag == 0) {
      if (State->Transition[i].TransitionTarget.compareTo("exit") != 0) {
        sma.StateNames[sma.nStates] = State->Transition[i].TransitionTarget;
        sma.StatesDefined[sma.nStates] = 0;
        sma.nStates++;
      }
    }
  }

  // Add state transitions.
  for (int i = 0; i < nInputs; i++) {
    sma.InputMatrix[CurrentStateNum][i] = CurrentStateNum;
  }
  for (int i = 0; i < State->nTransition; i++) {
    int CandidateEventCode = find_idx(EventNames, nInputs, State->Transition[i].TransitionTrigger);
    if (CandidateEventCode < 0) {
      return -1;
    }
    String TargetState = State->Transition[i].TransitionTarget;
    int TargetStateNumber = 0;
    if (TargetState.compareTo("exit") == 0) {
      TargetStateNumber = sma.nStates;
      /* This statement is true only if all the states were pre-added by AddBlankState().
         another way is to set TargetStateNumber to a big impossible number (e.g., MAX_STATE_NUM),
         and set to sma.nStates when all the states are added.
      */
    } else {
      TargetStateNumber = find_idx(sma.StateNames, sma.nStates, TargetState);
    }
    sma.InputMatrix[CurrentStateNum][CandidateEventCode] = TargetStateNumber;
  }

  // Add output actions.
  for (int i = 0; i < nOutputs; i++) {
    sma.OutputMatrix[CurrentStateNum][i] = 0;
  }
  for (int i = 0; i < State->nAction; i++) {
    int TargetEventCode = find_idx(OutputActionNames, nOutputs, State->Action[i].ActionType);
    if (TargetEventCode >= 0) {
      sma.OutputMatrix[CurrentStateNum][TargetEventCode] = State->Action[i].ActionValue;
    } else {
      return -1;
    }
  }

  // Add self timer.
  sma.StateTimers[CurrentStateNum] = State->Duration * TimerScaleFactor;

  sma.StatesDefined[CurrentStateNum] = 1;

  // Return 0 if success.
  return 0;
}

int gpSMART::SetGlobalTimer(byte TimerNumber, float TimerDuration) {
  // TimerNumber: The number of the timer you are setting (an integer, 1-GLOBAL_TC_NUM).
  // TimerDuration: The duration of the timer, following timer start (ms)
  if (TimerNumber > 0 && TimerNumber <= GLOBAL_TC_NUM) {
    sma.GlobalTimerThresholds[TimerNumber - 1] = TimerDuration * TimerScaleFactor;
  }
}

int gpSMART::SetGlobalCounter(byte CounterNumber, String TargetEventName, uint16_t Threshold) {
  // CounterNumber: The number of the counter you are setting (an integer, 1-GLOBAL_TC_NUM).
  // TargetEventName: The name of the event to count (a string; see Input Event Codes: EventNames[25])
  // Threshold: The number of event instances to count. (an integer).
  if (CounterNumber > 0 && CounterNumber <= GLOBAL_TC_NUM) {
    byte TargetEventCode = find_idx(EventNames, nInputs, TargetEventName);
    if (TargetEventCode > -1) {
      sma.GlobalCounterThresholds[CounterNumber - 1] = Threshold;
      sma.GlobalCounterAttachedEvents[CounterNumber - 1] = TargetEventCode;
    }
  }
}

void gpSMART::PrintMatrix() {
  SerialUSB.print("Number of States: ");
  SerialUSB.println(sma.nStates);
  SerialUSB.println("StateNames\tTimer\tDefined");
  for (int i = 0; i < sma.nStates; i++) {
    SerialUSB.print(sma.StateNames[i]);
    SerialUSB.print("\t");
    SerialUSB.print(sma.StateTimers[i]);
    SerialUSB.print("\t");
    SerialUSB.print(sma.StatesDefined[i]);
    SerialUSB.println();
  }
  SerialUSB.println("InputMatrix: ");
  for (int i = 0; i < sma.nStates; i++) {
    for (int j = 0; j < nInputs; j++) {
      SerialUSB.print(sma.InputMatrix[i][j]);
    }
    SerialUSB.println();
  }
  SerialUSB.println("OutputMatrix: ");
  for (int i = 0; i < sma.nStates; i++) {
    for (int j = 0; j < nOutputs; j++) {
      SerialUSB.print(sma.OutputMatrix[i][j]);
    }
    SerialUSB.println();
  }
  SerialUSB.println("GTthres\tGCevt\tGCthres");
  for (int i = 0; i < GLOBAL_TC_NUM; i++) {
    SerialUSB.print(sma.GlobalTimerThresholds[i]);
    SerialUSB.print("\t");
    SerialUSB.print(sma.GlobalCounterAttachedEvents[i]);
    SerialUSB.print("\t");
    SerialUSB.print(sma.GlobalCounterThresholds[i]);
    SerialUSB.println();
  }
}

void gpSMART::Run() {
  if (smartRunning == 1) {
    smartRunning = 0;
    gpSMART_Timer.stop();
  }
  NewState = 0;
  CurrentState = 0;
  SoftEvent = 0;    // No event
  smartFinished = false;

  trial_res.nEvent = 0;
  trial_res.nVisited = 0;
  trial_res.stateVisited[trial_res.nVisited++] = CurrentState;

  // Reset event counters
  for (int x = 0; x < GLOBAL_TC_NUM; x++) {
    GlobalCounterCounts[x] = 0;
    GlobalTimersActive[x] = false;
  }
  // Read initial state of sensors
  for (int x = 0; x < nDIOs; x++) {
    if (DigitalInputsEnabled[x] == 1) {
      PortInputLineValue[x] = digitalReadDirect(gpSMART_DI_Lines[x]); // Read each photogate's current state into an array
      if (PortInputLineValue[x] == HIGH) {
        PortInputLineLastKnownStatus[x] = HIGH; // Update last known state of input line
      } else {
        PortInputLineLastKnownStatus[x] = LOW;
      }
    } else {
      PortInputLineLastKnownStatus[x] = LOW;
      PortInputLineValue[x] = LOW;
    }
  }
  // Reset timers
  StateStartTime = 0;
  CurrentTime = 0;

  // Adjust outputs
  setStateOutputs(CurrentState);
  smartRunning = 1;
  gpSMART_Timer.start(); // Runs every 100us
  SerialUSB.println("M: State machine is running...");
}

void gpSMART::Stop() {
  // stop Timer
  smartRunning = 0;
  gpSMART_Timer.stop();

  // reset hardware ports
  setStateOutputs(sma.nStates);
}

void gpSMART::ManualOverride(String TargetAction, byte DataByte) {
  int TargetActionCode = find_idx(OutputActionNames, nOutputs, TargetAction);
  if (TargetActionCode >= 0) {
    if (TargetActionCode < nDIOs) { // DO1-8
      if (DataByte > 0) {
        digitalWriteDirect(gpSMART_DO_Lines[TargetActionCode], 1);
      } else {
        digitalWriteDirect(gpSMART_DO_Lines[TargetActionCode], 0);
      }
    } else if (TargetActionCode < nDIOs + nPWMs) { // PWM
      analogWrite(gpSMART_PWM_Lines[TargetActionCode - nDIOs], DataByte);
    } else if (TargetActionCode < nDIOs + 2 * nPWMs) { // tPWM
      setTruePWMOutput(TargetActionCode - (nDIOs + nPWMs) + 1, DataByte);
    } else if (TargetActionCode < nDIOs + 2 * nPWMs + 1) { // Serial3
      Serial3.write(DataByte);
    } else if (TargetActionCode < nDIOs + 2 * nPWMs + 2) { // SerialUSB
      SerialUSB.write(DataByte);
    } else if (TargetActionCode < nDIOs + 2 * nPWMs + 3) { // GlobalTimerTrig
      if (DataByte > 0 && DataByte <= GLOBAL_TC_NUM) {
        DataByte = DataByte - 1; // Convert to 0 index
        GlobalTimersActive[DataByte] = true;
        GlobalTimerEnd[DataByte] = CurrentTime + sma.GlobalTimerThresholds[DataByte];
      }
    } else if (TargetActionCode < nDIOs + 2 * nPWMs + 4) { // GlobalTimerCancel
      if (DataByte > 0 && DataByte <= GLOBAL_TC_NUM) {
        DataByte = DataByte - 1; // Convert to 0 index
        GlobalTimersActive[DataByte] = false;
      }
    } else if (TargetActionCode < nDIOs + 2 * nPWMs + 5) { // GlobalCounterReset
      if (DataByte > 0 && DataByte <= GLOBAL_TC_NUM) {
        DataByte = DataByte - 1; // Convert to 0 index
        GlobalCounterCounts[DataByte] = 0;
      }
    }
  }
}

void gpSMART::SendSoftEvent(byte EventNum) {
  if (EventNum > 0 && EventNum <= SOFT_EVENT_NUM) {
    SoftEvent = EventNum;
  }
}


/******************************************************************/
/*********************** Callback Functions ***********************/
/******************************************************************/

// Timer Callback Function
void gpSMART_Runner() {
  if (smartRunning) {
    nCurrentEvents = 0;
    CurrentEvent[0] = 254; // Event 254 = No event // not necessary
    CurrentTime++;         // 0.1 ms step increment
    // Refresh state of sensors and inputs
    for (int x = 0; x < nDIOs; x++) {
      if (DigitalInputsEnabled[x] == 1) {
        PortInputLineValue[x] = digitalReadDirect(gpSMART_DI_Lines[x]);
      }
    }
    // Determine which port event occurred
    int Ev = 0; // Since port-in and port-out events are indexed sequentially, Ev replaces x in the loop.
    for (int x = 0; x < nDIOs; x++) {
      // Determine port entry events
      if ((PortInputLineValue[x] == HIGH) && (PortInputLineLastKnownStatus[x] == LOW)) {
        PortInputLineLastKnownStatus[x] = HIGH;
        CurrentEvent[nCurrentEvents] = Ev;
        nCurrentEvents++;
      }
      Ev = Ev + 1;
      // Determine port exit events
      if ((PortInputLineValue[x] == LOW) && (PortInputLineLastKnownStatus[x] == HIGH)) {
        PortInputLineLastKnownStatus[x] = LOW;
        CurrentEvent[nCurrentEvents] = Ev;
        nCurrentEvents++;
      }
      Ev = Ev + 1;
    } // now Ev = 16

    // Determine if Soft Events occur
    if (SoftEvent > 0 && SoftEvent <= SOFT_EVENT_NUM) { // 1~4
      CurrentEvent[nCurrentEvents] = SoftEvent + Ev - 1; // 16-19
      nCurrentEvents++;
      SoftEvent = 0;
    }
    Ev = Ev + SOFT_EVENT_NUM;

    // Determine if a state timer expired
    if (sma.InputMatrix[CurrentState][2 * nDIOs + SOFT_EVENT_NUM] != CurrentState) {
      if ((CurrentTime - StateStartTime) >= sma.StateTimers[CurrentState]) {
        CurrentEvent[nCurrentEvents] = Ev;
        nCurrentEvents++;
      }
    }
    Ev = Ev + 1;

    // Determine if a global timer expired
    for (int x = 0; x < GLOBAL_TC_NUM; x++) {
      if (GlobalTimersActive[x] == true) {
        if (CurrentTime >= GlobalTimerEnd[x]) {
          CurrentEvent[nCurrentEvents] = Ev;
          nCurrentEvents++;
          GlobalTimersActive[x] = false;
        }
      }
      Ev = Ev + 1;
    }
    // Determine if a global event counter threshold was exceeded
    for (int x = 0; x < GLOBAL_TC_NUM; x++) {
      if (sma.GlobalCounterAttachedEvents[x] < nInputs) {
        // Check for and handle threshold crossing
        if (GlobalCounterCounts[x] == sma.GlobalCounterThresholds[x]) {
          CurrentEvent[nCurrentEvents] = Ev;
          nCurrentEvents++;
        }
        // Add current event to count (Crossing triggered on next cycle)
        for (int i = 0; i < nCurrentEvents; i++) {
          if (CurrentEvent[i] == sma.GlobalCounterAttachedEvents[x]) {
            GlobalCounterCounts[x] = GlobalCounterCounts[x] + 1;
          }
        }
      }
      Ev = Ev + 1;
    }

    // Now determine if a state transition should occur. The first event linked to a state transition takes priority.
    for (int i = 0; i < nCurrentEvents; i++) {
      NewState = sma.InputMatrix[CurrentState][CurrentEvent[i]];
      if (NewState != CurrentState) {
        break;
      }
    }

    // Store timestamp of events captured in this cycle
    if ((trial_res.nEvent + nCurrentEvents) < MAX_EVENT_NUM) {
      for (int x = 0; x < nCurrentEvents; x++) {
        trial_res.eventTimeStamps[trial_res.nEvent] = CurrentTime;
        trial_res.EventID[trial_res.nEvent] = CurrentEvent[x];
        trial_res.nEvent++;
      }
    }

    // Make state transition if necessary
    if (NewState != CurrentState) {
      if (trial_res.nVisited < MAX_VISIT_NUM) {
        trial_res.stateVisited[trial_res.nVisited++] = NewState;
      }
      setStateOutputs(NewState);
      if (NewState == sma.nStates) {
        smartRunning = false;
        smartFinished = true;
        gpSMART_Timer.stop();
      } else {
        StateStartTime = CurrentTime;
        CurrentState = NewState;
      }
    }
  } // End running state matrix
  return;
} // End for gpSMART_Runner

void setTruePWMOutput(byte tPWM_num, byte freq_num ) {
  switch (tPWM_num) {
    case 1:
      if (freq_num > 0 && freq_num <= FREQ_NUM_PER_PWM) {
        tPWM1.start(tPWM1_period[freq_num], tPWM1_duty[freq_num]);
      } else {
        tPWM1.stop();
      }
      break;
    case 2:
      if (freq_num > 0 && freq_num <= FREQ_NUM_PER_PWM) {
        tPWM2.start(tPWM2_period[freq_num], tPWM2_duty[freq_num]);
      } else {
        tPWM2.stop();
      }
      break;
    case 3:
      if (freq_num > 0 && freq_num <= FREQ_NUM_PER_PWM) {
        tPWM3.start(tPWM3_period[freq_num], tPWM3_duty[freq_num]);
      } else {
        tPWM3.stop();
      }
      break;
    case 4:
      if (freq_num > 0 && freq_num <= FREQ_NUM_PER_PWM) {
        tPWM4.start(tPWM4_period[freq_num], tPWM4_duty[freq_num]);
      } else {
        tPWM4.stop();
      }
      break;
    default:
      break;
  }
}

void setStateOutputs(byte iState) {
  // Column 0~7: Digital Output Line 1-nDIOs; Value: 0 or 1
  for (int x = 0; x < nDIOs; x++) {
    if (sma.OutputMatrix[iState][x] > 0) {
      digitalWriteDirect(gpSMART_DO_Lines[x], 1);
    } else {
      digitalWriteDirect(gpSMART_DO_Lines[x], 0);
    }
  }
  // Column 8~11: Regular PWM Output Line 1-nPWMs; Value: 0-255
  for (int x = 0; x < nPWMs; x++) {
    analogWrite(gpSMART_PWM_Lines[x], sma.OutputMatrix[iState][x + nDIOs]);
  }
  // Column 12-15: True PWM Output Line 1-nPWMs; Value: 1-4
  for (int x = 0; x < nPWMs; x++) {
    setTruePWMOutput(x + 1, sma.OutputMatrix[iState][x + nDIOs + nPWMs]);
  }
  // Column 16: Serial3; Value: 1-255, 0 will be ignored
  if (sma.OutputMatrix[iState][nDIOs + 2 * nPWMs] > 0) {
    Serial3.write(sma.OutputMatrix[iState][nDIOs + 2 * nPWMs]);
  }
  // Column 17: SerialUSB; Value: 1-255, 0 will be ignored
  if (sma.OutputMatrix[iState][nDIOs + 2 * nPWMs + 1] > 0) {
    SerialUSB.write('S');                         // State Message starts with 'S'
    SerialUSB.write(sma.OutputMatrix[iState][nDIOs + 2 * nPWMs + 1]); // message 1-255
    SerialUSB.println();
  }
  // Column 18: Trigger global timers; Vaue: 1-GLOBAL_TC_NUM(2)
  byte CurrentTimer = sma.OutputMatrix[iState][nDIOs + 2 * nPWMs + 2];
  if (CurrentTimer > 0 && CurrentTimer <= GLOBAL_TC_NUM) {
    CurrentTimer = CurrentTimer - 1; // Convert to 0 index
    GlobalTimersActive[CurrentTimer] = true;
    GlobalTimerEnd[CurrentTimer] = CurrentTime + sma.GlobalTimerThresholds[CurrentTimer];
  }
  // Column 19: Cancel global timers; Vaue: 1-GLOBAL_TC_NUM(2)
  CurrentTimer = sma.OutputMatrix[iState][nDIOs + 2 * nPWMs + 3];
  if (CurrentTimer > 0 && CurrentTimer <= GLOBAL_TC_NUM) {
    CurrentTimer = CurrentTimer - 1; // Convert to 0 index
    GlobalTimersActive[CurrentTimer] = false;
  }
  // Column 20: Reset event counters; Vaue: 1-GLOBAL_TC_NUM(2)
  byte CurrentCounter = sma.OutputMatrix[iState][nDIOs + 2 * nPWMs + 4];
  if (CurrentCounter > 0 && CurrentCounter <= GLOBAL_TC_NUM) {
    CurrentCounter = CurrentCounter - 1; // Convert to 0 index
    GlobalCounterCounts[CurrentCounter] = 0;
  }
}



/******************************************************************/
/************************ Other Functions *************************/
/******************************************************************/
void digitalWriteDirect(byte pin, bool val) {
  if (val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

byte digitalReadDirect(byte pin) {
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

int find_idx(const String * str_array, int array_length, String target) {
  for (int i = 0; i < array_length; i++) {
    if (target.compareTo(str_array[i]) == 0) {
      return i;
    }
  }
  return -1;
}
