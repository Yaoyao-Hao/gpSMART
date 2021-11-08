#include "gpSMART_Habits.h"
/*
   constructing and running state machine on the same Arduino!
*/

#define MAX_TRIAL_NUM 9999

gpSMART smart;
extern TrialResult trial_res;
extern volatile bool smartFinished; // Has the system exited the matrix (final state)?
extern volatile bool smartRunning; // 1 if state matrix is running

byte TrialType = 1;

const byte led_port = 13;

OutputAction LeftWaterOutput  = {"DO1", 1};
OutputAction RightWaterOutput = {"DO2", 1};
OutputAction LowSoundOutput   = {"tPWM2", 1};
OutputAction HighSoundOutput  = {"tPWM2", 2};
OutputAction CueOutput        = {"tPWM2", 3};

const byte switchPin       = 4;  // ToggerSwitch pin to start/pause experiment
int trial_num = 0;

void setup() {

  //delay(3000); // for debug

  Serial.begin(115200);   // To PC
  // while (!Serial) {}

//  pinMode(led_port, OUTPUT);
//  digitalWrite(led_port, 1);
//  delay(1000);
//  digitalWrite(led_port, 0);
  pinMode(switchPin, INPUT_PULLUP); // low if switch on; hight if switch off

  byte PortEnabled[4] = {0, 0, 0, 0};
  smart.setDigitalInputsEnabled(PortEnabled);
  // tPWM1: left sound; tPWM2: top sound; tPWM3: right sound
  smart.setTruePWMFrequency(2, 1, 3000, 128);  // (low sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 2, 10000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 3, 6500, 128);  // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty

  // free reward
  smart.ManualOverride("DO1", 1); // override valve
  delay(100);
  smart.ManualOverride("DO1", 0);
}

void loop() {
  if (digitalRead(switchPin) == 0) {
    Serial.print("Starting trial number: ");
    Serial.println(trial_num++);

    // clear matrix at the begining of each trial
    smart.EmptyMatrix();

    OutputAction SampleOutput;
    OutputAction RewardOutput;
    String LeftLickAction;
    String RightLickAction;
    String ActionAfterDelay;
    float reward_dur = 30;
    switch (TrialType) {
      case 1: // left-low sound
        SampleOutput    = LowSoundOutput;
        LeftLickAction  = "Reward";
        RightLickAction = "AnswerPeriod"; // no error trial
        RewardOutput    = LeftWaterOutput;
        reward_dur      = 30;
        break;
      case 2: // right-high sound
        SampleOutput    = HighSoundOutput;
        LeftLickAction  = "AnswerPeriod"; // no error trial
        RightLickAction = "Reward";
        RewardOutput    = RightWaterOutput;
        reward_dur      = 30;
        break;
    }
    if (random(100) < 100) { // free reward probability
      ActionAfterDelay = "GiveFreeDrop";
    } else {
      ActionAfterDelay = "ResponseCue";
    }

    StateTransition TrialStart_Cond[1]     = {{"Tup", "SamplePeriod"}}; //
    StateTransition SamplePeriod_Cond[1]   = {{"Tup", "DelayPeriod"}};
    StateTransition DelayPeriod_Cond[1]    = {{"Tup", ActionAfterDelay}};
    StateTransition ResponseCue_Cond[1]    = {{"Tup", "AnswerPeriod"}};
    StateTransition GiveFreeDrop_Cond[1]   = {{"Tup", "ResponseCue"}};
    StateTransition AnswerPeriod_Cond[3]   = {{"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1]         = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1]       = {{"Tup", "exit"}};
    StateTransition NoResponse_Cond[3]     = {{"Lick1In", "exit"}, {"Lick2In", "exit"}, {"Tup", "exit"}};

    OutputAction Sample_Output[1]          = {SampleOutput};
    OutputAction ResponseCue_Output[1]     = {CueOutput};
    OutputAction Reward_Output[1]          = {RewardOutput};
    OutputAction NoOutput[0]               = {};

    gpSMART_State states[9] = {};

    states[0]  = smart.CreateState("TrialStart",        10,                   1, TrialStart_Cond,      0, NoOutput); // msec
    states[1]  = smart.CreateState("SamplePeriod",      1000,       1, SamplePeriod_Cond,    1, Sample_Output);
    states[2]  = smart.CreateState("DelayPeriod",       500,        1, DelayPeriod_Cond,     0, NoOutput);
    states[3]  = smart.CreateState("ResponseCue",       100,                  1, ResponseCue_Cond,     1, ResponseCue_Output);
    states[4]  = smart.CreateState("GiveFreeDrop",      10000,           1, GiveFreeDrop_Cond,    1, Reward_Output);
    states[5]  = smart.CreateState("AnswerPeriod",      4000,       3, AnswerPeriod_Cond,    0, NoOutput);
    states[6]  = smart.CreateState("Reward",            reward_dur,           1, Reward_Cond,          1, Reward_Output);
    states[7]  = smart.CreateState("RewardConsumption", 750,  1, Tup_Exit_Cond,        0, NoOutput);
    states[8]  = smart.CreateState("NoResponse",        1000, 3, NoResponse_Cond,      0, NoOutput); // 1-hr: 60*60*1000 msec

    // Predefine State sequence.
    for (int i = 0; i < 9; i++) {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 9; i++) {
      smart.AddState(&states[i]);
    }

    smart.PrintMatrix(); // for debug

    digitalWrite(led_port, 0);
    smart.Run();
    while (smartFinished == 0) {} // wait until a trial is done
    analogWrite(led_port, 128);

    /* data will be stored in public variable 'trial_res', which includes:
       trial_res.nEvent:           number of event happened in last trial
       trial_res.eventTimeStamps[]: time stamps for each event
       trial_res.EventID[]:          event id for each event
       trial_res.nVisited:       number of states visited in last trial
       trial_res.stateVisited[]:   the states visited in last trail
    */
    Serial.print("State Transitions: ");
    Serial.println(trial_res.nVisited);
    for (int i = 0; i < trial_res.nVisited; i++) {
      Serial.print(trial_res.stateVisited[i]);
      Serial.print(" ");
    }
    Serial.println();

    Serial.print("Events Number: ");
    Serial.println(trial_res.nEvent);
//    for (int i = 0; i < trial_res.nEvent; i++) {
//      Serial.print(trial_res.EventID[i]);
//      Serial.print(" ");
//      Serial.print(trial_res.eventTimeStamps[i]);
//      Serial.print(" ");
//    }
//    Serial.println();

    // Change parameters based on smart.trial_res for next trial
    UpdateParameters();

    // inter-trial interval
    delay(5000);
  }
}

void UpdateParameters() {
  // update parameters for next trial

  // using trial_res to calculate e.g., performance, etc.

  if (random(100) < 50) {
    TrialType = 1;
  } else {
    TrialType = 1;
  }
}
