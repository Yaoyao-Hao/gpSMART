/*
   gpSMART: A general purpose State MAchine Runner for Training animal behaviors.
   This library enables constructing and running state machine on a single Teensy (3.6) board.
   Created by Yaoyao Hao (yaoyaoh90@gmail.com) 2021.
*/


#include "gpSMART_Teensy.h"


/******************************************************************/
/************************ hardware related ************************/
/******************************************************************/
// using Timer for gpSMART
IntervalTimer gpSMART_Timer;

// period and duty for true PWMs
uint32_t tPWM_frequency[ntPWMs][FREQ_NUM_PER_PWM] = {{3000, 6000, 9000, 12000}, {3000, 6000, 9000, 12000}, {3000, 6000, 9000, 12000}, {3000, 6000, 9000, 12000}}; // 3-12 kHz
byte tPWM_duty[ntPWMs][FREQ_NUM_PER_PWM]          = {{128,  128, 128, 128}, {128,  128, 128, 128}, {128,  128, 128, 128}, {128,  128, 128, 128}}; // Deafault: 50% duty cycle

// Digital Input Enables
byte DigitalInputsEnabled[nDIOs] = {1, 1, 1, 1, 1, 1, 1, 1};


/*******************************************************************/
/********** public variables for both gpSMART and main.ino *********/
/*******************************************************************/
TrialResult trial_res;		// store events and state transitions in one trial
bool volatile smartFinished = false; 	// indicating if current trial is finished
bool volatile smartRunning = false; 		// indicating if state matrix for current trial is running


/*****************************************************************/
/****************** public variables for gpSMART *****************/
/*****************************************************************/
StateMatrix sma;   // State Matrix
// Other public variables
bool PortInputLineValue[nDIOs] = {0}; // Direct reads of digital values
bool PortInputLineLastKnownStatus[nDIOs] = {0}; // Last known status
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
    digitalWrite(gpSMART_DO_Lines[x], 0);
  }
  for (int x = 0; x < nPWMs; x++) {
    pinMode(gpSMART_PWM_Lines[x], OUTPUT);
    analogWrite(gpSMART_PWM_Lines[x], 0);
  }
  for (int x = 0; x < ntPWMs; x++) {
    setTruePWMOutput(x + 1, 0);
  }
  // init Serial ports used by gpSMART
  Serial1.begin(115200);
  Serial.begin(115200);
  // attach ISR to gpSMART_Timer (not necessary for Teensy)
  //gpSMART_Timer.begin(gpSMART_Runner, 100); // Runs every 100us
  //gpSMART_Timer.end();
  analogWriteFrequency(5, 1000); // FTM0  5, 6, 9, 10, 20, 21, 22, 23; default: 488.28 Hz
}

void gpSMART::setDigitalInputsEnabled(byte* PortEnabled) {
  for (int i = 0; i < nDIOs; i++) {
    DigitalInputsEnabled[i] = PortEnabled[i];
  }
}

void gpSMART::setTruePWMFrequency(byte tPWM_num, byte freq_num, uint32_t freq, byte duty) {
  if (tPWM_num > 0 && tPWM_num <= ntPWMs && freq_num > 0 && freq_num <= FREQ_NUM_PER_PWM) {
    tPWM_frequency[tPWM_num - 1][freq_num - 1] = freq;
    tPWM_duty[tPWM_num - 1][freq_num - 1] = duty;
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

void gpSMART::SetGlobalTimer(byte TimerNumber, float TimerDuration) {
  // TimerNumber: The number of the timer you are setting (an integer, 1~GLOBAL_TC_NUM).
  // TimerDuration: The duration of the timer, following timer start (ms)
  if (TimerNumber > 0 && TimerNumber <= GLOBAL_TC_NUM) {
    sma.GlobalTimerThresholds[TimerNumber - 1] = TimerDuration * TimerScaleFactor;
  }
}

void gpSMART::SetGlobalCounter(byte CounterNumber, String TargetEventName, uint16_t Threshold) {
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
  Serial.print("Number of States: ");
  Serial.println(sma.nStates);
  Serial.println("StateNames\tTimer\tDefined");
  for (int i = 0; i < sma.nStates; i++) {
    Serial.print(sma.StateNames[i]);
    Serial.print("\t");
    Serial.print(sma.StateTimers[i]);
    Serial.print("\t");
    Serial.print(sma.StatesDefined[i]);
    Serial.println();
  }
  Serial.println("InputMatrix: ");
  for (int i = 0; i < sma.nStates; i++) {
    for (int j = 0; j < nInputs; j++) {
      Serial.print(sma.InputMatrix[i][j]);
    }
    Serial.println();
  }
  Serial.println("OutputMatrix: ");
  for (int i = 0; i < sma.nStates; i++) {
    for (int j = 0; j < nOutputs; j++) {
      Serial.print(sma.OutputMatrix[i][j]);
    }
    Serial.println();
  }
  Serial.println("GT-thres\tGC-evt\tGC-thres");
  for (int i = 0; i < GLOBAL_TC_NUM; i++) {
    Serial.print(sma.GlobalTimerThresholds[i]);
    Serial.print("\t");
    Serial.print(sma.GlobalCounterAttachedEvents[i]);
    Serial.print("\t");
    Serial.print(sma.GlobalCounterThresholds[i]);
    Serial.println();
  }
}

void gpSMART::Run() {
  if (smartRunning == 1) {
    smartRunning = 0;
    gpSMART_Timer.end();
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
      PortInputLineValue[x] = digitalRead(gpSMART_DI_Lines[x]); // Read each photogate's current state into an array
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
  gpSMART_Timer.begin(gpSMART_Runner, 100); // Runs every 100us
  Serial.println("M: State machine is running...");
}

void gpSMART::Stop() {
  // stop Timer
  smartRunning = 0;
  gpSMART_Timer.end();

  // reset hardware ports
  setStateOutputs(sma.nStates);
}

void gpSMART::ManualOverride(String TargetAction, byte DataByte) {
  int TargetActionCode = find_idx(OutputActionNames, nOutputs, TargetAction);
  if (TargetActionCode >= 0) {
    if (TargetActionCode < nDIOs) { // DO1-8
      if (DataByte > 0) {
        digitalWrite(gpSMART_DO_Lines[TargetActionCode], 1);
      } else {
        digitalWrite(gpSMART_DO_Lines[TargetActionCode], 0);
      }
    } else if (TargetActionCode < nDIOs + nPWMs) { // PWM
      analogWrite(gpSMART_PWM_Lines[TargetActionCode - nDIOs], DataByte);
    } else if (TargetActionCode < nDIOs + nPWMs + ntPWMs) { // tPWM
      setTruePWMOutput(TargetActionCode - (nDIOs + nPWMs) + 1, DataByte);
    } else if (TargetActionCode < nDIOs + nPWMs + ntPWMs + 1) { // Serial
      Serial1.write(DataByte);
    } else if (TargetActionCode < nDIOs + nPWMs + ntPWMs + 2) { // Serial
      Serial.write(DataByte);
    } else if (TargetActionCode < nDIOs + nPWMs + ntPWMs + 3) { // GlobalTimerTrig
      if (DataByte > 0 && DataByte <= GLOBAL_TC_NUM) {
        DataByte = DataByte - 1; // Convert to 0 index
        GlobalTimersActive[DataByte] = true;
        GlobalTimerEnd[DataByte] = CurrentTime + sma.GlobalTimerThresholds[DataByte];
      }
    } else if (TargetActionCode < nDIOs + nPWMs + ntPWMs + 4) { // GlobalTimerCancel
      if (DataByte > 0 && DataByte <= GLOBAL_TC_NUM) {
        DataByte = DataByte - 1; // Convert to 0 index
        GlobalTimersActive[DataByte] = false;
      }
    } else if (TargetActionCode < nDIOs + nPWMs + ntPWMs + 5) { // GlobalCounterReset
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
        PortInputLineValue[x] = digitalRead(gpSMART_DI_Lines[x]);
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

    // Determine if a state timer expired, Tup (event 20)
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
        gpSMART_Timer.end();
      } else {
        StateStartTime = CurrentTime;
        CurrentState = NewState;
      }
    }
  } // End running state matrix
  return;
} // End for gpSMART_Runner

void setTruePWMOutput(byte tPWM_num, byte freq_num ) {
  if (freq_num > 0 && freq_num <= FREQ_NUM_PER_PWM) {
    analogWriteFrequency(gpSMART_tPWM_Lines[tPWM_num - 1], tPWM_frequency[tPWM_num - 1][freq_num - 1]);
    analogWrite(gpSMART_tPWM_Lines[tPWM_num - 1], tPWM_duty[tPWM_num - 1][freq_num - 1]);
  } else {
    analogWrite(gpSMART_tPWM_Lines[tPWM_num - 1], 0);
  }
}

void setStateOutputs(byte iState) {
  // Column 0~7: Digital Output Line 1-nDIOs; Value: 0 or 1
  for (int x = 0; x < nDIOs; x++) {
    if (sma.OutputMatrix[iState][x] > 0) {
      digitalWrite(gpSMART_DO_Lines[x], 1);
    } else {
      digitalWrite(gpSMART_DO_Lines[x], 0);
    }
  }
  // Column 8~15: Regular PWM Output Line 1-nPWMs; Value: 0-255
  for (int x = 0; x < nPWMs; x++) {
    analogWrite(gpSMART_PWM_Lines[x], sma.OutputMatrix[iState][x + nDIOs]);
  }
  // Column 16-19: True PWM Output Line 1-ntPWMs; Value: 0-4
  for (int x = 0; x < ntPWMs; x++) {
    setTruePWMOutput(x + 1, sma.OutputMatrix[iState][x + nDIOs + nPWMs]);
  }
  // Column 20: Serial1; Value: 1-255, 0 will be ignored
  if (sma.OutputMatrix[iState][nDIOs + nPWMs + ntPWMs] > 0) {
    Serial1.write(sma.OutputMatrix[iState][nDIOs + nPWMs + ntPWMs]);
  }
  // Column 21: Serial; Value: 1-255, 0 will be ignored
  if (sma.OutputMatrix[iState][nDIOs + nPWMs + ntPWMs + 1] > 0) {
    Serial.write('S');                         // State Message starts with 'S'
    Serial.write(sma.OutputMatrix[iState][nDIOs + nPWMs + ntPWMs + 1]); // message 1-255
    Serial.println();
  }
  // Column 22: Trigger global timers; Vaue: 1-GLOBAL_TC_NUM(2)
  byte CurrentTimer = sma.OutputMatrix[iState][nDIOs + nPWMs + ntPWMs + 2];
  if (CurrentTimer > 0 && CurrentTimer <= GLOBAL_TC_NUM) {
    CurrentTimer = CurrentTimer - 1; // Convert to 0 index
    GlobalTimersActive[CurrentTimer] = true;
    GlobalTimerEnd[CurrentTimer] = CurrentTime + sma.GlobalTimerThresholds[CurrentTimer];
  }
  // Column 23: Cancel global timers; Vaue: 1-GLOBAL_TC_NUM(2)
  CurrentTimer = sma.OutputMatrix[iState][nDIOs + nPWMs + ntPWMs + 3];
  if (CurrentTimer > 0 && CurrentTimer <= GLOBAL_TC_NUM) {
    CurrentTimer = CurrentTimer - 1; // Convert to 0 index
    GlobalTimersActive[CurrentTimer] = false;
  }
  // Column 24: Reset event counters; Vaue: 1-GLOBAL_TC_NUM(2)
  byte CurrentCounter = sma.OutputMatrix[iState][nDIOs + nPWMs + ntPWMs + 4];
  if (CurrentCounter > 0 && CurrentCounter <= GLOBAL_TC_NUM) {
    CurrentCounter = CurrentCounter - 1; // Convert to 0 index
    GlobalCounterCounts[CurrentCounter] = 0;
  }
}



/******************************************************************/
/************************ Other Functions *************************/
/******************************************************************/

int find_idx(const String * str_array, int array_length, String target) {
  for (int i = 0; i < array_length; i++) {
    if (target.compareTo(str_array[i]) == 0) {
      return i;
    }
  }
  return -1;
}
