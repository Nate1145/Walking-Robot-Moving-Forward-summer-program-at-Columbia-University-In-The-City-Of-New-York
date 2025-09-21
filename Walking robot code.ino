#include <Servo.h>


float Height = 9.65;
float L1 = 4.65;
float L2 = 5.04;

float Angle_Hip_at_equil = 0;
float Angle_Range = 20;
float KneeAngle = 10;

float Angle_Hip_Max = Angle_Hip_at_equil + Angle_Range / 2;
float Angle_Hip_Min = Angle_Hip_at_equil - Angle_Range / 2;

float PhaseStep = 0.01;       
int   CycleTime = 2500;       


int HipOffset = 135;
int KneeOffset = 135;


Servo FR_Hip, FR_Knee;
Servo FL_Hip, FL_Knee;
Servo HR_Hip, HR_Knee;
Servo HL_Hip, HL_Knee;


int FR_Hip_Pin = 8,  FR_Knee_Pin = 9;
int FL_Hip_Pin = 10, FL_Knee_Pin = 11;
int HR_Hip_Pin = 4,  HR_Knee_Pin = 3;
int HL_Hip_Pin = 6,  HL_Knee_Pin = 5;


void setup() {
  Serial.begin(115200);

  
  FR_Hip.attach(FR_Hip_Pin);
  FR_Knee.attach(FR_Knee_Pin);
  FL_Hip.attach(FL_Hip_Pin);
  FL_Knee.attach(FL_Knee_Pin);
  HR_Hip.attach(HR_Hip_Pin);
  HR_Knee.attach(HR_Knee_Pin);
  HL_Hip.attach(HL_Hip_Pin);
  HL_Knee.attach(HL_Knee_Pin);

  
  setLeg(FR_Hip, FR_Knee, Angle_Hip_at_equil, KneeAngle);
  setLeg(FL_Hip, FL_Knee, Angle_Hip_at_equil, KneeAngle);
  setLeg(HR_Hip, HR_Knee, Angle_Hip_at_equil, KneeAngle);
  setLeg(HL_Hip, HL_Knee, Angle_Hip_at_equil, KneeAngle);
  delay(500);
}


void loop() {
  unsigned long now = millis();
  float phaseBase = fmod((float)now / CycleTime, 1.0); 

  
  moveLegWithPhase(HR_Hip, HR_Knee, phaseBase, false);             
  moveLegWithPhase(FL_Hip, FL_Knee, phaseBase, false);             

  moveLegWithPhase(HL_Hip, HL_Knee, fmod(phaseBase + 0.5, 1.0), false); 
  moveLegWithPhase(FR_Hip, FR_Knee, fmod(phaseBase + 0.5, 1.0), false); 
}


void moveLegWithPhase(Servo &hip, Servo &knee, float phase, bool stanceFirst) {
  
  bool stance = (phase < 0.5); 
  float hipAngle;

  if (stance) {
    hipAngle = mapf(phase, 0, 0.5, Angle_Hip_Min, Angle_Hip_Max);
    float rad = hipAngle * PI / 180;
    float tempVal = (Height - L1 * cos(rad)) / L2;
    tempVal = constrain(tempVal, -1, 1);
    float kneeAngle = rad + acos(tempVal);
    kneeAngle = kneeAngle * 180 / PI;
    setLeg(hip, knee, hipAngle, kneeAngle);
  } else {
    hipAngle = mapf(phase, 0.5, 1.0, Angle_Hip_Max, Angle_Hip_Min);
    float kneeAngle = KneeAngle; 
    setLeg(hip, knee, hipAngle, kneeAngle);
  }
}


void setLeg(Servo &hip, Servo &knee, float hipDeg, float kneeDeg) {
  hip.write(map(HipOffset - hipDeg, 0, 270, 0, 180));
  knee.write(map(KneeOffset - kneeDeg, 0, 270, 0, 180));
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
