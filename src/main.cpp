#include <Arduino.h>
#include <TMCStepper.h>
#include <AS5600.h>
#include <Wire.h>
#include <EEPROM.h>
#include "PID.h"

float angle_current, angle_current_previous, angle_desired;
bool moveInProgress, moveComplete,directionChanged = false, updateMem = false, dir= true;
int rate_HZ=50, steps_desired, turnState;
long SekTimer,SekTimer2, looptime;
volatile bool turnOff = false;

int steps;
int degreeOffset = 0; //set angle of actual motor to 0 degrees, read inner angle, set inner angle as offset.

#define EEPROM_SIZE 1 
#define gearRatio 3

//TMC2209 Stepper driver
#define BUZZPIN   7
#define EN_PIN    6
#define DIR_PIN   18
#define STEP_PIN  19
#define SW_RX     4  
#define SW_TX     5  
#define STEPSPRREVOLUTION 200
#define MICROSTEPRESOLUTION 128
#define DRIVER_ADDRESS 0b00
#define R_SENSE   0.11f
#define SERIAL_PORT Serial1
TMC2209Stepper TMCdriver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

//AS5600 Encoder
AS5600 as5600;
#define SDA_PIN 3
#define SCL_PIN 10

//PID
#define KP  30  
#define KI  80
#define KD  1
#define Looptime  0.001
#define MotorClamp  true 
PID pid(KP, KI, KD, Looptime, MotorClamp);

bool miliSEKtimer(int x){  
  SekTimer = millis();  
  if (SekTimer-SekTimer2 > x){
      SekTimer2 = SekTimer; 
      return true;
  }
  else{
      return false;
  }
}

void initializeTMC(){
  Serial1.begin(115200);
  delay(200); //adds delay to start Serial1
  Serial1.setPins(SW_RX, SW_TX); //works on own
  TMCdriver.begin();
  TMCdriver.test_connection() != 0 ? Serial.println("Connected!") : Serial.println("NOT Connected!");
  delay(500); 
  TMCdriver.rms_current(600); //set motor RMS current in mA
  TMCdriver.microsteps(MICROSTEPRESOLUTION);  //set microstepinterval
  TMCdriver.en_spreadCycle(false); //must be false to stealthchop
  TMCdriver.pwm_autoscale(true); //needed for stealthchop
  TMCdriver.pwm_autograd(true);
  TMCdriver.toff(5);  //TOFF off time and driver enable 0101
  TMCdriver.tbl(0);   //blank time select 0 or 1 recommended
}
void SetNullDegStart(){
  //sets the current angle to zero degs.
  degreeOffset = as5600.rawAngle()*AS5600_RAW_TO_DEGREES;
} 
void initializeAS5600(){
  as5600.begin();  
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  // default, just be explicit.
  SetNullDegStart();
  as5600.setOffset(degreeOffset);
}
void changeDirection(){
  dir = !dir;           //change direction
  TMCdriver.shaft(dir); // SET DIRECTION
}
void SetDirection(bool x){    
  TMCdriver.shaft(x); // SET DIRECTION
}
void set_pinmodes(){
  pinMode(BUZZPIN,INPUT);
  pinMode(EN_PIN,OUTPUT);
  digitalWrite(EN_PIN,HIGH); //disable motor at startup
  pinMode(STEP_PIN,OUTPUT);
  pinMode(DIR_PIN,OUTPUT);
}
void writeToEEPROM(int addr, uint8_t int_255values){
  EEPROM.write(addr,int_255values);
  EEPROM.commit();
}
float getAngleForEndGear(){
  float current = as5600.rawAngle()*AS5600_RAW_TO_DEGREES;
   Serial.print("\tinner_deg: ");
   Serial.print(current,4);
  //in order to keep track of what position magnet is beyond 360deg, states must be stored in EEPROM in order to
  //keep same position after restart, however notice that EEPROM has finite write possibility
  if((current-angle_current_previous) > 350){ 
    turnState--;
    //writeToEEPROM(0, turnState); //beware that EEPROM has a finite writes to it
  }
  else if((current-angle_current_previous) < -350){
    turnState++;
    //writeToEEPROM(0, turnState);
  }
  angle_current_previous = current;
  return (current/gearRatio)+(360/gearRatio)*(turnState-100);
}
int angleToSteps(double currentAngle, double wantedAngle, uint16_t stepsPrRevolution, uint16_t microstepResolution, uint16_t gear_ratio){
  return ((wantedAngle-currentAngle)*stepsPrRevolution*microstepResolution)/(360*gear_ratio);
}
int executeAngle(double currentAngle, double wantedAngle){
  int _steps = angleToSteps(currentAngle,wantedAngle, STEPSPRREVOLUTION, MICROSTEPRESOLUTION, gearRatio);
  //if steps are negative motordirection must switch and direction must be recorded
  if (_steps < 0 && !directionChanged){
    changeDirection();
    directionChanged = true;
  }
   if (steps < abs(_steps) && miliSEKtimer(3)){// sektimer sets time between steps
    moveComplete = false;
    TMCdriver.step();
    steps++;
  }
  if (steps >= abs(_steps)){
    directionChanged = false;
    if (!moveComplete){
      moveComplete = !moveComplete;
      angle_current += wantedAngle;
    } 
  }
  return _steps;
}

void setup() {
  Wire.begin(static_cast<int>(SDA_PIN),static_cast<int>(SCL_PIN), static_cast<uint32_t>(4000)); //sda scl config, static cast due to bug.
  Serial.begin(115200);
  Serial.println();
  set_pinmodes();
  initializeTMC();
  initializeAS5600();
  looptime = micros();
  angle_current = 0;
}

void loop() {
  if(digitalRead(BUZZPIN)) {
    turnOff = true;
  }

  turnOff ? digitalWrite(EN_PIN,HIGH) : digitalWrite(EN_PIN,LOW);//turnOff kan only be switched on by reboot
  angle_desired = 90;

  int _steps = executeAngle(angle_current, angle_desired);
  
  /*
  these can be used if you only want to drive the stepper motor
  TMCdriver.VACTUAL(50000);
  changeDirection();
  */  

  if (false)
  {
    Serial.print("steps desired: ");
    Serial.print(abs(_steps));
    Serial.print("\tsteps taken ");
    Serial.print(steps);
    Serial.print("\tmove Completed: ");
    Serial.print(moveComplete ? "Yes": "No");
    Serial.print("\touter_deg: ");
    Serial.print(angle_current,4);
    Serial.println();
  }

  while(micros() - looptime < 1000); //halts program to maintain 1ms cycle time = 1000hz
  looptime = micros(); 
}