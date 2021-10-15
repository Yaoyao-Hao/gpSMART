#include "gpSMART_Teensy.h"
/*
   constructing and running state machine on the same Arduino!
*/

#define MAX_TRIAL_NUM 9999

gpSMART smart;
extern TrialResult trial_res;
extern volatile bool smartFinished; // Has the system exited the matrix (final state)?
extern volatile bool smartRunning; // 1 if state matrix is running

byte TrialType = 0;

const byte led_port = 13;

void setup() {

  delay(3000); // for debug

  Serial.begin(115200);   // To PC
  // while (!Serial) {}

  pinMode(led_port, OUTPUT);
  digitalWrite(led_port, 1);
  delay(1000);
  digitalWrite(led_port, 0);

  // Port and wire parameters for gpSMART.
  byte PortEnabled[8] = {1, 1, 0, 0, 0, 0, 0, 0};
  smart.setDigitalInputsEnabled(PortEnabled);

  smart.setTruePWMFrequency(1, 1, 3500, 128); // byte tPWM_num, byte freq_num, uint32 frequency, byte duty

  // free reward
  smart.ManualOverride("DO1", 1); // override valve
  delay(100);
  smart.ManualOverride("DO1", 0);

  for (int trial_num = 0; trial_num < MAX_TRIAL_NUM; trial_num++)
  {
    Serial.print("Starting trial number: ");
    Serial.println(trial_num + 1);

    // clear matrix at the begining of each trial
    smart.EmptyMatrix();

    String LeftLickAction;
    String RightLickAction;
    OutputAction LeftWaterOutput = {"DO1", 1};
    OutputAction RightWaterOutput = {"DO2", 1};
    OutputAction CueOutput = {"tPWM1", 1};
    OutputAction PoleOutput = {"PWM1", 255};
    OutputAction WaveSurferTrig = {"DO3", 1};
    OutputAction RewardOutput;
    switch (TrialType) {
      case 1:
        LeftLickAction  = "Reward";
        RightLickAction = "TimeOut";
        RewardOutput    = LeftWaterOutput;
        break;
      case 0:
        LeftLickAction  = "TimeOut";
        RightLickAction = "Reward";
        RewardOutput    = RightWaterOutput;
        break;
    }

    StateTransition TrigTrialStart_Cond[1]     = {{"Tup", "SamplePeriod"}}; //
    StateTransition SamplePeriod_Cond[3]       = {{"DI1Rising", "EarlyLickSample"}, {"DI2Rising", "EarlyLickSample"}, {"Tup", "DelayPeriod"}};
    StateTransition Sample_noforce_Cond[1]     = {{"Tup", "DelayPeriod"}};
    StateTransition EarlyLickSample_Cond[1]    = {{"Tup", "SamplePeriod"}};
    StateTransition DelayPeriod_Cond[3]        = {{"DI1Rising", "EarlyLickDelay"}, {"DI2Rising", "EarlyLickDelay"}, {"Tup", "ResponseCue"}};
    StateTransition EarlyLickDelay_Cond[1]     = {{"Tup", "DelayPeriod"}};
    StateTransition ResponseCue_Cond[1]        = {{"Tup", "AnswerPeriod"}};
    StateTransition GiveFreeDrop_Cond[1]       = {{"Tup", "ResponseCue"}};
    StateTransition AnswerPeriod_Cond[3]       = {{"DI1Rising", LeftLickAction}, {"DI2Rising", RightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1]             = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_StopLicking_Cond[1]    = {{"Tup", "StopLicking"}};
    StateTransition Tup_EndTrial_Cond[1]       = {{"Tup", "TrialEnd"}};
    StateTransition StopLicking_Cond[3]        = {{"DI1Rising", "StopLickingReturn"}, {"DI2Rising", "StopLickingReturn"}, {"Tup", "TrialEnd"}}; //
    StateTransition TrialEnd_Cond[1]           = {{"Tup", "exit"}};

    OutputAction TrigTrialStart_Output[1]  = {WaveSurferTrig};
    OutputAction Sample_Pole_Output[1]     = {PoleOutput};
    OutputAction ResponseCue_Output[1]     = {CueOutput};
    OutputAction GiveRightDrop_Output[1]   = {RightWaterOutput};
    OutputAction GiveLeftDrop_Output[1]    = {LeftWaterOutput};
    OutputAction Reward_Output[1]          = {RewardOutput};
    OutputAction NoOutput[0]               = {};

    gpSMART_State states[16] = {};
    states[0]  = smart.CreateState("TrigTrialStart",    1000,                     1, TrigTrialStart_Cond,  1, TrigTrialStart_Output); // msec
    states[1]  = smart.CreateState("SamplePeriod",      3000,                      3, SamplePeriod_Cond,    1, Sample_Pole_Output);
    states[2]  = smart.CreateState("EarlyLickSample",   50,                     1, EarlyLickSample_Cond, 1, Sample_Pole_Output);
    states[3]  = smart.CreateState("DelayPeriod",       3000,                      3, DelayPeriod_Cond,     0, NoOutput);
    states[4]  = smart.CreateState("EarlyLickDelay",    50,                     1, EarlyLickDelay_Cond,  0, NoOutput);
    states[5]  = smart.CreateState("ResponseCue",       10,                      1, ResponseCue_Cond,     1, ResponseCue_Output);
    states[6]  = smart.CreateState("GiveRightDrop",     30,                      1, GiveFreeDrop_Cond,    1, GiveRightDrop_Output);
    states[7]  = smart.CreateState("GiveLeftDrop",      10, 1, GiveFreeDrop_Cond,    1, GiveLeftDrop_Output);
    states[8]  = smart.CreateState("AnswerPeriod",      10,    3, AnswerPeriod_Cond,    0, NoOutput);
    states[9]  = smart.CreateState("Reward",            100, 1, Reward_Cond,          1, Reward_Output);
    states[10] = smart.CreateState("RewardConsumption", 100,        1, Tup_StopLicking_Cond, 0, NoOutput);
    states[11] = smart.CreateState("NoResponse",        200,                    1, Tup_StopLicking_Cond, 0, NoOutput);
    states[12] = smart.CreateState("TimeOut",           300,                1, Tup_StopLicking_Cond, 0, NoOutput);
    states[13] = smart.CreateState("StopLicking",       1500,        3, StopLicking_Cond,     0, NoOutput);
    states[14] = smart.CreateState("StopLickingReturn", 100,                     1, Tup_StopLicking_Cond, 0, NoOutput);
    states[15] = smart.CreateState("TrialEnd",          100,                     1, TrialEnd_Cond,        0, NoOutput);

    // Predefine State sequence.
    for (int i = 0; i < 16; i++) {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 16; i++) {
      smart.AddState(&states[i]);
    }

    smart.PrintMatrix();

    digitalWrite(led_port, 0);
    smart.Run();
    while (smartFinished == 0) {} // wait until a trial is done
    digitalWrite(led_port, 1);

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
    for (int i = 0; i < trial_res.nEvent; i++) {
      Serial.print(trial_res.EventID[i]);
      Serial.print(" ");
      Serial.print(trial_res.eventTimeStamps[i]);
      Serial.println();
    }
    Serial.println();

    // Change parameters based on smart.trial_res for next trial
    UpdateParameters();

    // inter-trial interval
    delay(5000);
  }
}

void loop() {
}

void UpdateParameters() {
  // update parameters for next trial

  // using trial_res to calculate e.g., performance, etc.

  if (random(100) < 50) {
    TrialType = 1;
  } else {
    TrialType = 0;
  }
}
