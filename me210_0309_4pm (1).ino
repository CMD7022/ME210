

#include <ezButton.h>
#include <NewPing.h>
#include <Wire.h>
#include <SPI.h>
#include <pitches.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <gfxfont.h>
#include <./Fonts/FreeMonoBold9pt7b.h>
#include <./Fonts/FreeMonoBoldOblique9pt7b.h>


//Motor Pin Initialization~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pwm_MotorL; //enA
uint8_t pwm_MotorR; //enB
uint8_t in1_MotorL; //in1
uint8_t in2_MotorL; //in2
uint8_t in3_MotorR; //in3
uint8_t in4_MotorR; //in4

// Solenoid Pin Initialization~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define solenoidPin1 51
#define solenoidPin2 50

// Course Switch Initialization
#define courseSwitchPin 53

// Limit Switches Initialization~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ezButton limitSwitchR(23);
ezButton limitSwitchL(11);

// Celebration~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define StartCeleb_ParentPin 38 // StartCeleb_ChildPin should be 12 on child Arduino
#define FinishCeleb_ParentPin 39 // StartCeleb_ChildPin should be 13 on child Arduino
#define CelebDone_ParentPin 40 // CelebDone_ChildPin should be 2 on child Arduino
//#define ledPin 13
uint32_t solenoidToggleIntervalMS = 250;

// Ultrasonic Initialization~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define Ultra_FL_TrigPin 10
#define Ultra_FL_EchoPin 9
#define Ultra_FR_TrigPin 7
#define Ultra_FR_EchoPin 8

#define MAX_DISTANCE 300 // Maximum distance (in cm) to ping.
#define ITERATIONS 3
#define pingSpeed 35      // time between pings in milliseconds, minimum should be 29 ms
#define pingSpeed_Median  200 // time between a cycle of pings in milliseconds, minimum should be number of sensors times pingSpeed

#define startFarWallMax 103 // cm
#define startFarWallMin 79  // cm
#define ArenaTurnDistanceCM 55  //cm
#define ContactTurnDistanceCM 127 //cm
#define TurntoShootDistanceCM 15 //cm
#define BackWallDistanceCM 24 //cm
#define TurntoLobbyDistanceCM 30 //cm
#define TurntoStartDistanceCM 90 //cm
#define StartWallDistanceCM 27    // cm
#define TurnToFarWallDistanceCM 60  //cm

#define cm2uS 58.14        // cm to uS
#define mm2uS 5.81         // mm to uS
#define uS2mm 0.172        // uS to mm
#define uS2cm 0.0172        // uS to cm
#define alignmentThresholdMM  30.0   // mm
#define finealignThresholdMM  20.0   // mm
#define nonCornerThresholdMM  30.0   // mm
#define superfinealignThresholdMM  7.0   // mm

uint32_t startFarWallMax_uS = startFarWallMax*cm2uS;
uint32_t startFarWallMin_uS = startFarWallMin*cm2uS;
NewPing sonarFL(Ultra_FL_TrigPin, Ultra_FL_EchoPin, MAX_DISTANCE);
NewPing sonarFR(Ultra_FR_TrigPin, Ultra_FR_EchoPin, MAX_DISTANCE);
unsigned long pingTimerFR, pingTimerFL, pingTimerFL_Median, pingTimerFR_Median;
int32_t Ultra_array[2];

// IR Sensor Variables~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int irFarRight = 0;
int irRight = 0;
int irLeft = 0;
int irFarLeft = 0;

// Motors Variables~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define turnPWM 100
#define turnSlowPWM 75
#define turnSlowIntervalOn 100 // ms
#define turnSlowIntervalOff 300 // ms
uint32_t currentSlowTurnTime = millis();
bool SlowTurnToggleToggle = true;

uint8_t goStraightPWM = 175;
uint8_t goBackwardPWM = 175;

#define turn90degtime 540 // ms based off turn PWM of 100
#define turn65degtime 450 // ms based off turn PWM of 100
#define turn180degtime 1350
#define exitArenaTimeMS 4750
#define ReloadingMS 4000
#define game_over_time_MS 130000   // 130000 for 2 mins 10 seconds

uint32_t turn90degdonetime = 0;
uint32_t turn65degdonetime = 0;
uint32_t waitForDoneTime = 0;
uint32_t run_time_MS = 0;


/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_ScanningForStartWall, STATE_AligningWStartWall, STATE_ApproachingStartWall, STATE_TurningToArena, STATE_AligningWArenaWall,STATE_EnteringArena,
  STATE_TurningToContactWall, STATE_AligningWContactWall, STATE_ApproachingContact, STATE_ContactDisengage, STATE_TurningToFarWall,
  STATE_AligningWFarWall, STATE_ApproachingFarWall, STATE_TurningToGoalWall, STATE_AligningWGoalWall, STATE_ApproachingGoal, STATE_StartDispensing, STATE_Dispensing, 
  STATE_StartCelebration, STATE_FinishCelebration, STATE_EndState,
  STATE_DisengageGoal, STATE_TurnToReturnWall, STATE_AlignWReturnWall, STATE_ExitingArena, STATE_TurnToLobbyFront1, STATE_AlignBetweenTurns, STATE_TurnToLobbyFront2, 
  STATE_AlignWLobbyFrontWall, STATE_ApproachLobbyFrontWall, STATE_TurnToStart, STATE_AlignWStartForReload, STATE_ApproachStart, STATE_Reloading, 
  STATE_DisengageFromStart, STATE_TurnToFarWall, STATE_RerunTurnAround, STATE_ExitingArena2, STATE_TurnToLobbyFront180
} States_t;

String state_string = "STATE_ScanningForStartWall";

States_t state;

// State Flags~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool turn90degtimerset = false;
bool turn65degtimerset = false;
bool measureMedianToggle = false;
bool waitForTimerSet = false;
bool celeb_complete = false;
bool solenoid_toggle = false;
bool display_invert = false;
bool omitContact = false;
bool right_course = false;

//Screen Initialization~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_MOSI     36 //10
#define OLED_CLK      37 //8
#define OLED_DC       38 //7
#define OLED_CS       39
#define OLED_RST      40 //9
Adafruit_SH1106G display = Adafruit_SH1106G(128, 64,OLED_MOSI, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);

// unsigned long previousMillis = 0;   // Stores the time when the text was last updated
// const long intervalOn = 250;        // Interval for text on (in milliseconds)
// const long intervalOff = 200;       // Interval for text off (in milliseconds)
 bool displayText = true;           // Flag to toggle text display


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// void readIRValues(){ //threshold looks like it is 750
//   irFarRight = analogRead(A0);
//   irRight = analogRead(A1);
//   irLeft = analogRead(A2);
//   irFarLeft = analogRead(A3);
// }

void measureDistance(){
  if (measureMedianToggle == false){
    if (millis() >= pingTimerFL) {
      pingTimerFL += pingSpeed;
      int FL_distance = sonarFL.ping();
      Ultra_array[0] = FL_distance;
    }
    
    if (millis() >= pingTimerFR) {
      pingTimerFR = pingTimerFL + pingSpeed;
      int FR_distance = sonarFR.ping();
      Ultra_array[1] = FR_distance;
    }
  } else {
    if (millis() >= pingTimerFL_Median) {
      pingTimerFL_Median += pingSpeed_Median;
      int FL_distance_median = sonarFL.ping_median();
      Ultra_array[0] = FL_distance_median;
      }

      if (millis() >= pingTimerFR_Median) {
        pingTimerFR_Median = pingTimerFL_Median + pingSpeed_Median;
        int FR_distance_Median = sonarFR.ping_median();
        Ultra_array[1] = FR_distance_Median;
      }
  }
}

bool AlignedWithWall(){
  float dif_mm = abs((Ultra_array[0] - Ultra_array[1])*uS2mm); // 0.0172 factor to turn uS into mm
  if (dif_mm < alignmentThresholdMM){ 
    return true;
  }
  else {
    return false;
  }
}

bool AlignedWithWallFine(float thresholdMM){
  float dif_mm = abs((Ultra_array[0] - Ultra_array[1])*uS2mm); // 0.0172 factor to turn uS into mm
  if (dif_mm < thresholdMM){ 
    return true;
  }
  else{
    return false;
  }
}

bool ScanForRightWall(){
  if (((Ultra_array[0] > startFarWallMin_uS) && (Ultra_array[0] < startFarWallMax_uS)) && ((Ultra_array[1] > startFarWallMin_uS) && (Ultra_array[1] < startFarWallMax_uS))){
    return true;
  } else {
    return false;
  }
}

bool DetectRangeWithin(uint32_t distanceCM){
  if((Ultra_array[0]< distanceCM*cm2uS) && (Ultra_array[1]<distanceCM*cm2uS)){
    return true;
  } else {
    return false;
  }
}

bool DetectRangeFurtherThanOR(uint32_t distanceCM){
  if((Ultra_array[0]> distanceCM*cm2uS) || (Ultra_array[1]>distanceCM*cm2uS)){
    return true;
  } else {
    return false;
  }
}

bool DetectRangeFurtherThanAND(uint32_t distanceCM){
  if((Ultra_array[0]> distanceCM*cm2uS) && (Ultra_array[1]>distanceCM*cm2uS)){
    return true;
  } else {
    return false;
  }
}

void turnRight(){
  Serial.println("Turning Right!");
  digitalWrite(in1_MotorL, LOW); // turn right
  digitalWrite(in2_MotorL, HIGH); // turn right
  digitalWrite(in3_MotorR, HIGH); // turn right
  digitalWrite(in4_MotorR, LOW); // turn right
  analogWrite(pwm_MotorL, turnPWM);
  analogWrite(pwm_MotorR, turnPWM);
}

void turnLeft(){
  Serial.println("Turning Left!");
  digitalWrite(in1_MotorL, HIGH);
  digitalWrite(in2_MotorL, LOW);
  digitalWrite(in3_MotorR, LOW); // turn left
  digitalWrite(in4_MotorR, HIGH); // turn left
  analogWrite(pwm_MotorR, turnPWM);
  analogWrite(pwm_MotorL, turnPWM);
}

void turnRightSlowly(){
  checkSlowTurnToggle();
  if(SlowTurnToggleToggle == false){
    // Serial.println("turn right slowly");
    digitalWrite(in1_MotorL, LOW); // 
    digitalWrite(in2_MotorL, HIGH); // forward
    digitalWrite(in3_MotorR, HIGH); // backwards
    digitalWrite(in4_MotorR, LOW); // 
    analogWrite(pwm_MotorL, turnSlowPWM);
    analogWrite(pwm_MotorR, turnSlowPWM);
  } else {
    // Serial.println("switch off");
    analogWrite(pwm_MotorL, 0);
    analogWrite(pwm_MotorR, 0);
  }
}

void turnLeftSlowly(){
  checkSlowTurnToggle();
  if(SlowTurnToggleToggle == false){
    digitalWrite(in1_MotorL, HIGH); // turn right
    digitalWrite(in2_MotorL, LOW); // turn right
    digitalWrite(in3_MotorR, LOW); // turn left
    digitalWrite(in4_MotorR, HIGH); // turn left
    analogWrite(pwm_MotorL, turnSlowPWM);
    analogWrite(pwm_MotorR, turnSlowPWM);
  }else{
  analogWrite(pwm_MotorL, 0);
  analogWrite(pwm_MotorR, 0);
  }
}

void goStraight(){
  digitalWrite(in1_MotorL, LOW); // turn right
  digitalWrite(in2_MotorL, HIGH);
  digitalWrite(in3_MotorR, LOW); // turn left
  digitalWrite(in4_MotorR, HIGH);
  analogWrite(pwm_MotorL, goStraightPWM); //175
  analogWrite(pwm_MotorR, goStraightPWM); //155
}

float Pgain = 1.0;  //proportional gain for feedback control
int feedback_limit = 100; //feedback control will only work when the difference in ultrsonic sensor reading is less than this value in mm.
int32_t feedbackOffsetMMB = 0;
int32_t feedbackOffsetMMF = 0;

void goStraightWithFB(){
  digitalWrite(in1_MotorL, LOW); // turn right
  digitalWrite(in2_MotorL, HIGH);
  digitalWrite(in3_MotorR, LOW); // turn left
  digitalWrite(in4_MotorR, HIGH);
  float dif_mm = (Ultra_array[0] - Ultra_array[1])*uS2mm - feedbackOffsetMMF;
  float deltaLR = Pgain * dif_mm;
  int deltaLR_PWM = round(deltaLR);
  //float deltaLR_PWM = deltaLR;
  if (abs(dif_mm) > feedback_limit){
    deltaLR_PWM = 0;
  }
  // uint8_t leftMotorPWM = goStraightPWM + deltaLR_PWM;
  // uint8_t rightMotorPWM = goStraightPWM - deltaLR_PWM;
  //B
  uint8_t leftMotorPWM = goStraightPWM + deltaLR_PWM;
  uint8_t rightMotorPWM = goStraightPWM - deltaLR_PWM;
  if (digitalRead(courseSwitchPin) == LOW){ //A
    rightMotorPWM = goStraightPWM + deltaLR_PWM;
    Serial.println("rightMotorPWM");
    Serial.println(rightMotorPWM);
    leftMotorPWM = goStraightPWM - deltaLR_PWM;
    Serial.println(leftMotorPWM);
  }
  analogWrite(pwm_MotorL, leftMotorPWM); //175
  analogWrite(pwm_MotorR, rightMotorPWM); //155
}

void goBackwardsWithFB(){
  digitalWrite(in1_MotorL, HIGH); // turn right
  digitalWrite(in2_MotorL, LOW);
  digitalWrite(in3_MotorR, HIGH); // turn left
  digitalWrite(in4_MotorR, LOW);
  float dif_mm = (Ultra_array[0] - Ultra_array[1])*uS2mm+feedbackOffsetMMB;
  float deltaLR = Pgain * dif_mm;
  int deltaLR_PWM = round(deltaLR);
  if (abs(dif_mm) > feedback_limit){
    deltaLR_PWM = 0;
  }
  // uint8_t leftMotorPWM = goStraightPWM - deltaLR_PWM;
  // uint8_t rightMotorPWM = goStraightPWM + deltaLR_PWM;
  //B
  uint8_t leftMotorPWM = goStraightPWM - deltaLR_PWM;
  uint8_t rightMotorPWM = goStraightPWM + deltaLR_PWM;
  if (digitalRead(courseSwitchPin) == LOW){ //A
    leftMotorPWM = goStraightPWM + deltaLR_PWM;
    rightMotorPWM = goStraightPWM - deltaLR_PWM;
  }
  analogWrite(pwm_MotorL, leftMotorPWM);
  analogWrite(pwm_MotorR, rightMotorPWM);
}

void stopMoving(){
  Serial.println("Stop Moving!");
  digitalWrite(in1_MotorL, LOW); // turn right
  digitalWrite(in2_MotorL, LOW);
  digitalWrite(in3_MotorR, LOW); // turn left
  digitalWrite(in4_MotorR, LOW);
  analogWrite(pwm_MotorL, 0);
  analogWrite(pwm_MotorR, 0);
}

void goBackwards(){
  Serial.println("Go Backwards!");
  digitalWrite(in1_MotorL, HIGH); // turn right
  digitalWrite(in2_MotorL, LOW);
  digitalWrite(in3_MotorR, HIGH); // turn left
  digitalWrite(in4_MotorR, LOW);
  analogWrite(pwm_MotorL, goBackwardPWM);
  analogWrite(pwm_MotorR, goBackwardPWM); 
}

void adjustAlignment(){
  // if (Ultra_array[0] >= Ultra_array[1]){ //right ultrasonic sensor closer to wall
  //   turnRightSlowly();
  // } else { 
  //   //if (Ultra_array[0] < Ultra_array[1]){ //left ultrasonic sensor closer to wall
  //   turnLeftSlowly();
  //  // }
  // }
  if (digitalRead(courseSwitchPin) == HIGH){ //B
    if (Ultra_array[0] >= Ultra_array[1]){ //right ultrasonic sensor closer to wall
      turnRightSlowly(); //turn right slowly
    } else { 
    //if (Ultra_array[0] < Ultra_array[1]){ //left ultrasonic sensor closer to wall
      turnLeftSlowly();
   // }
    } 
  } else { //A
    if (Ultra_array[0] >= Ultra_array[1]){ //left ultrasonic sensor closer to wall
      turnRightSlowly(); //turn left slowly
    } else { 
    //if (Ultra_array[0] < Ultra_array[1]){ //right ultrasonic sensor closer to wall
      turnLeftSlowly();
   // }
    } 
  }
}

void updateDisplay(void){
  switch (state) {
    case STATE_StartCelebration:
    // state_string = "Celebrating";
    // printStateText(state_string);
      break;
    case STATE_ScanningForStartWall:
    state_string = "Scanning";
    printStateText(state_string); 
      break;
    case STATE_AligningWStartWall:
    state_string = "Align 1";
    printStateText(state_string);
      break;
    case STATE_ApproachingStartWall:
    state_string = "Approach 1";
    printStateText(state_string);
      break;
    case STATE_TurningToArena:
    state_string = "Turning 1";
    printStateText(state_string);
      break;
    case STATE_AligningWArenaWall:
    state_string = "Align 2";
    printStateText(state_string);
      break;
    case STATE_EnteringArena:
    state_string = "Entering";
      printStateText(state_string);
      break;
    case STATE_TurningToContactWall:
    state_string = "Turning 2";
      printStateText(state_string);
      break;
    case STATE_AligningWContactWall:
    state_string = "Align 3";
      printStateText(state_string);
      break;
    case STATE_ApproachingContact:
    state_string = "Approach 2";
      printStateText(state_string);
      break;
    case STATE_ContactDisengage:
    state_string = "Disengage 1";
      printStateText(state_string);
      break;
    case STATE_TurningToFarWall:
    state_string = "Turning 3";
      printStateText(state_string);
      break;
    case STATE_AligningWFarWall:
    state_string = "Align 4";
      printStateText(state_string);
      break;
    case STATE_ApproachingFarWall:
    state_string = "Approach 3";
      printStateText(state_string);
      break;
    case STATE_TurningToGoalWall:
    state_string = "Turning 4";
      printStateText(state_string);
      break;
    case STATE_ApproachingGoal:
    state_string = "Approach G";
      printStateText(state_string);
      break;
    case STATE_StartDispensing:
    state_string = "Dispensing";
      printStateText(state_string); 
      break;
    case STATE_FinishCelebration:
    state_string = "Woop Woop!";
      printStateText(state_string);
      break;
    case STATE_DisengageGoal:
      state_string = "Disengage G";
      printStateText(state_string);
      break;
    case STATE_TurnToReturnWall:
      state_string = "Turn2ReturnW";
      printStateText(state_string);
      break;
    case STATE_AlignWReturnWall:
      state_string = "AlignWReturnW";
      printStateText(state_string);
      break;
    case STATE_ExitingArena:
      state_string = "Exit Arena";
      printStateText(state_string);
      break;
    case STATE_TurnToLobbyFront1:
      state_string = "Turn2Lobby1";
      printStateText(state_string);
      break;
    case STATE_AlignBetweenTurns:
      state_string = "BtwnTurns";
      printStateText(state_string);
      break;
    case STATE_TurnToLobbyFront2:
      state_string = "Turn2Lobby2";
      printStateText(state_string);
      break;
    case STATE_AlignWLobbyFrontWall:
      state_string = "AlignWLobby";
      printStateText(state_string);
      break;
    case STATE_ApproachLobbyFrontWall:
      state_string = "Approach Lobby";
      printStateText(state_string);
      break;
    case STATE_TurnToStart:
      state_string = "Turn2Start";
      printStateText(state_string);
      break;
    case STATE_AlignWStartForReload:
      state_string = "Align4Reload";
      printStateText(state_string);
      break;
    case STATE_ApproachStart:
      state_string = "Approach Start";
      printStateText(state_string);
      break;
    case STATE_Reloading:
      state_string = "RELOAD";
      printStateText(state_string);
      break; 
    case STATE_DisengageFromStart:
      state_string = "Disengage Start";
      printStateText(state_string);
      break;
    case STATE_TurnToFarWall:
      state_string = "Turn2FarW";
      printStateText(state_string);
      break;
    case STATE_EndState:
      // state_string = "End State!";
      // printStateText(state_string);
      break; 
    default:    // Should never get into an unhandled state
    Serial.println("uh oh");
  }
  // Print or clear the text
  display.display();               // Update display
}

void printStateText(String state_text) {
  display.clearDisplay(); // Clear the display buffer
  display.invertDisplay(false);
  //display.setFont(&FreeMonoBoldOblique9pt7b);
  display.setFont(NULL);
  // Center text on two lines
  // int16_t x, y;
  // uint16_t textWidth, textHeight;
  // display.getTextBounds(state_text, 0, 0, &x, &y, &textWidth, &textHeight);
  // x = (SCREEN_WIDTH - textWidth) / 2;
  // y = (SCREEN_HEIGHT - textHeight * 2) / 2;
  int16_t x = 0;
  int16_t y = 0;

  // Print text if displayText is true
  if (displayText) {
    display.setTextSize(2);
    display.setCursor(x, y);
    display.print("S:");
    display.println(state_string);
    display.setTextSize(2);
    display.setCursor(x, y+30);
    display.print("L:");
    display.print(Ultra_array[0]*uS2cm);
    display.setTextSize(1);
    display.println("cm");
    display.setCursor(x, y+45);
    display.setTextSize(2);
    display.print("R:");
    display.print(Ultra_array[1]*uS2cm);
    display.setTextSize(1);
    display.println("cm");
  }
}

void startCelebration(){
  String text1 = "UNITY";
  String text2 = "GAINZ";
  printStartCelebration(text1,text2);
}

void finishCelebration(){
  String text1 = "GAME";
  String text2 = "OVER";
  printStartCelebration(text1,text2);
}

void printStartCelebration(String print_text1, String print_text2){
  display.clearDisplay(); // Clear the display buffer
  display.setFont(&FreeMonoBoldOblique9pt7b);
  display.setTextSize(2);
  if(display_invert){
    display.invertDisplay(true);
  } else {
    display.invertDisplay(false);
  }
  display_invert = !display_invert;
  int16_t x, y;
  uint16_t textWidth, textHeight;
  display.getTextBounds(print_text1, 0, 0, &x, &y, &textWidth, &textHeight);
  x = (SCREEN_WIDTH - textWidth) / 2;
  y = (SCREEN_HEIGHT - textHeight * 2) / 2;
  display.setCursor(x, y+15);
  display.println(print_text1);
  display.setCursor(x, y+45);
  display.println(print_text2);
}

bool turn90degtimercheck(){
  if(turn90degtimerset == false){
  turn90degdonetime = millis()+ turn90degtime;
  turn90degtimerset = true;
  }
  if (millis()>turn90degdonetime){
    turn90degtimerset = false;
    return true;
  } else {
    return false;
  }
}

bool turn65degtimercheck(){
  if(turn65degtimerset == false){
    turn65degdonetime = millis()+ turn65degtime;
    turn65degtimerset = true;
  }

  if (millis()>turn65degdonetime){
    turn65degtimerset = false;
    Serial.println("65 deg timer expired!");
    return true;
  } else{
    Serial.println("65 deg timer running!");
    return false;
  }
}

bool waitForMS(uint32_t waitTimeMS){
  if(waitForTimerSet == false){
  waitForDoneTime = millis()+ waitTimeMS;
  waitForTimerSet = true;
  }
  if (millis()>waitForDoneTime){
    waitForTimerSet = false;
    return true;
  } else{
    return false;
  }
}

void checkSlowTurnToggle(){
  if (millis() >= currentSlowTurnTime){
    if(SlowTurnToggleToggle){
      currentSlowTurnTime = currentSlowTurnTime + turnSlowIntervalOn;
    } else {
      currentSlowTurnTime = currentSlowTurnTime + turnSlowIntervalOff;
    }
    SlowTurnToggleToggle = !SlowTurnToggleToggle;
    return false;
  } else{
    return true;
  }
}

void solenoidCelebration(){
  if(solenoid_toggle){
    digitalWrite(solenoidPin1, LOW); 
    digitalWrite(solenoidPin2, HIGH);
    Serial.println("Solenoid Open");
  } else {
    digitalWrite(solenoidPin1, LOW); 
    digitalWrite(solenoidPin2, LOW);
    Serial.println("Solenoid Close");
  }
  solenoid_toggle = !solenoid_toggle;
}

void courseswitchtoggle(){
  //toggleSwitch.loop()
  //if (toggleSwitch.isPressed())
  if (digitalRead(courseSwitchPin) == HIGH){ //B
    //Serial.println("The switch: OFF -> ON");
    pwm_MotorL  = 3; //enA
    pwm_MotorR  = 2; //enB
    in1_MotorL  = 32; //in1
    in2_MotorL  = 33; //in2
    in3_MotorR  = 28; //in3
    in4_MotorR  = 29; //in4
  } else {
    //Serial.println("The switch: ON -> OFF");
    pwm_MotorL  = 2; //enA
    pwm_MotorR  = 3; //enB
    in1_MotorL  = 28; //in1
    in2_MotorL  = 29; //in2
    in3_MotorR  = 32; //in3
    in4_MotorR  = 33; //in4 
  }
}

void setup() {
  Serial.begin(9600);
  state = STATE_StartCelebration;

  // Flip Motor Pins
  pinMode(courseSwitchPin, INPUT);
  pwm_MotorL  = 3; //enA
  pwm_MotorR  = 2; //enB
  in1_MotorL  = 32; //in1
  in2_MotorL  = 33; //in2
  in3_MotorR  = 28; //in3
  in4_MotorR  = 29; //in4
  // if (digitalRead(courseSwitchPin) == HIGH){
  //   // Right Course
  //   pwm_MotorL = 2;
  //   pwm_MotorR = 3;
  //   in1_MotorL = 28;
  //   in2_MotorL = 29;
  //   in3_MotorR = 32;
  //   in4_MotorR = 33;
  // } else{
  //   // Left Course
  //   pwm_MotorL = 3; //enA
  //   pwm_MotorR = 2; //enB
  //   in1_MotorL = 32; //in1
  //   in2_MotorL = 33; //in2
  //   in3_MotorR = 28; //in3
  //   in4_MotorR = 29; //in4
  // }

  // Ultrasonics
  pinMode(Ultra_FL_TrigPin, OUTPUT);
  pinMode(Ultra_FL_EchoPin, INPUT);
  pinMode(Ultra_FR_TrigPin, OUTPUT);
  pinMode(Ultra_FR_EchoPin, INPUT);
  pingTimerFL = millis() + pingSpeed; // Sensor 1 fires after 100ms (pingSpeed)
  pingTimerFR = pingTimerFL + pingSpeed; // Sensor 2 fires 50ms later

  // Motors
  pinMode(pwm_MotorL, OUTPUT);
  pinMode(pwm_MotorR, OUTPUT);
  pinMode(in1_MotorL, OUTPUT);
  pinMode(in2_MotorL, OUTPUT);
  pinMode(in3_MotorR, OUTPUT);
  pinMode(in4_MotorR, OUTPUT);

  // Celebration
  pinMode(StartCeleb_ParentPin, OUTPUT);
  pinMode(FinishCeleb_ParentPin, OUTPUT);
  pinMode(CelebDone_ParentPin, INPUT_PULLUP);

  // Set initial rotation direction
  digitalWrite(in1_MotorL, LOW);
  digitalWrite(in2_MotorL, HIGH);
  digitalWrite(in3_MotorR, LOW);
  digitalWrite(in4_MotorR, HIGH);

  // Solenoid
  pinMode(solenoidPin1, OUTPUT); 
  pinMode(solenoidPin2, OUTPUT); 

  // Limit Switch
  limitSwitchR.setDebounceTime(50);  // set debounce time to 50 milliseconds, this is non blocking
  limitSwitchL.setDebounceTime(50);  // set debounce time to 50 milliseconds, this is non blocking

  // Display
  display.begin();
  display.clearDisplay(); // Clear the display buffer
  // Set up the text
  display.setTextColor(SH110X_WHITE); // Set text color to white
  display.setTextSize(1); // Set text size
  // Print text initially (turned off)
  display.println("Failure is always an option");
  // Update display
  display.display();

  uint32_t run_time_startMS = millis();
}

void loop() {
  //courseswitchtoggle();
  measureDistance();
  
  limitSwitchR.loop();    // Check the state of the buttons
  limitSwitchL.loop();

  // Turn off after 2 mins 10 seconds!
  run_time_MS = millis();
  if(run_time_MS > game_over_time_MS){
    state = STATE_EndState;
  }

  switch (state) {
    case STATE_StartCelebration:
      startCelebration();
      delay(300);
      if(waitForMS(1700)){
        state = STATE_ScanningForStartWall;
      }
      break;
    case STATE_ScanningForStartWall:
      if (digitalRead(courseSwitchPin) == HIGH){
        turnRightSlowly();
      }
      
      if(ScanForRightWall()){
        state = STATE_AligningWStartWall;
        //state = STATE_ApproachingStartWall;
        }
      break;
    case STATE_AligningWStartWall:
      adjustAlignment();
      if(AlignedWithWall()) {
      state = STATE_ApproachingStartWall;
      //measureMedianToggle = true;
      }
      break;
    case STATE_ApproachingStartWall:
      goStraightWithFB();
      //goStraight();
      if(DetectRangeWithin(ArenaTurnDistanceCM)){
        state = STATE_TurningToArena;
        //measureMedianToggle = false;
        }
      break;
    case STATE_TurningToArena:
      turnRight();
      if(turn65degtimercheck()){
          state = STATE_AligningWArenaWall;
          //state = STATE_EnteringArena;
          }
      break;
    case STATE_AligningWArenaWall:
      adjustAlignment();
      if(AlignedWithWallFine(finealignThresholdMM)){
          state = STATE_EnteringArena;
          }
      break;
    case STATE_EnteringArena:
      goBackwardsWithFB();
        if(DetectRangeFurtherThanOR(ContactTurnDistanceCM)){
          if(omitContact == false){
            state = STATE_TurningToContactWall;
            omitContact = true;
          } else {
            state = STATE_RerunTurnAround;
          }
        }
      break;
    case STATE_RerunTurnAround:
      turnLeft();
      if(waitForMS(turn180degtime)){
        state = STATE_ApproachingFarWall;
      }
      break;
    case STATE_TurningToContactWall:
      turnLeft();
      if(waitForMS(turn90degtime+40)){
          // state = STATE_AligningWContactWall;
          state = STATE_ApproachingContact;
          }
      break;
    case STATE_AligningWContactWall:
      adjustAlignment();
      if(AlignedWithWallFine(finealignThresholdMM)){
          state = STATE_ApproachingContact;
          }
      break;
    case STATE_ApproachingContact:
      goStraightWithFB();
      if (limitSwitchR.isPressed() || limitSwitchL.isPressed()){
        state = STATE_ContactDisengage;
        stopMoving();
        }
      break;
    case STATE_ContactDisengage:
      goBackwardsWithFB();
      if(DetectRangeFurtherThanOR(TurntoShootDistanceCM)){
        state = STATE_TurningToFarWall;
      }
      break;
    case STATE_TurningToFarWall:
      turnLeft();
        // if(turn65degtimercheck()){
        //     state = STATE_AligningWFarWall;
        //     }
          if(turn90degtimercheck()){
        state = STATE_ApproachingFarWall;
        }
      break;
    case STATE_AligningWFarWall:
      adjustAlignment();
        if(AlignedWithWallFine(finealignThresholdMM)){
            state = STATE_ApproachingFarWall;
            }
      break;
    case STATE_ApproachingFarWall:
      goStraightWithFB();
      //goStraight();
      if(DetectRangeWithin(BackWallDistanceCM)){
        state = STATE_TurningToGoalWall;
        //measureMedianToggle = false;
        }
      break;
    case STATE_TurningToGoalWall:
    turnRight();
      if(turn90degtimercheck()){
        // state = STATE_AligningWGoalWall;
        state = STATE_ApproachingGoal;
      }
      break;
    case STATE_AligningWGoalWall:
      adjustAlignment();
      if(AlignedWithWallFine(finealignThresholdMM)){
        state = STATE_ApproachingGoal;
      }
      break;
    case STATE_ApproachingGoal:
      goStraightWithFB();
      if (limitSwitchR.isPressed() || limitSwitchL.isPressed()){
        state = STATE_StartDispensing;
        }
      break;
    case STATE_StartDispensing:
      //Serial.println("dispense");
      digitalWrite(solenoidPin1, LOW); 
      digitalWrite(solenoidPin2, HIGH);
      stopMoving();
      state = STATE_Dispensing;
      break;
    case STATE_Dispensing:
      if(waitForMS(1000)){
        state = STATE_DisengageGoal;
      }
      break;
    case STATE_FinishCelebration:
      // if(!finishCelebration()){
      //   state = STATE_EndState;
      // }
      if(waitForMS(solenoidToggleIntervalMS)){
        solenoidCelebration();
        waitForMS(solenoidToggleIntervalMS);
      }
      break;
    case STATE_EndState:
      stopMoving();
      finishCelebration();
      break;
    case STATE_DisengageGoal:
      goBackwardsWithFB();
      digitalWrite(solenoidPin1, LOW); 
      digitalWrite(solenoidPin2, LOW);
      if(DetectRangeFurtherThanAND(TurntoLobbyDistanceCM)){
        state = STATE_TurnToReturnWall;
      }
      break;
    case STATE_TurnToReturnWall:
      turnLeft();
      if(turn65degtimercheck()){
        state = STATE_AlignWReturnWall;
      }
      break;
    case STATE_AlignWReturnWall:
      adjustAlignment();
      if(AlignedWithWallFine(finealignThresholdMM)){
        state = STATE_ExitingArena2;
      }
      break;
    case STATE_ExitingArena:
      goBackwardsWithFB();
      if(DetectRangeFurtherThanOR(TurntoStartDistanceCM)){
        state = STATE_TurnToLobbyFront1;
      }
      break;
    case STATE_ExitingArena2:
      goBackwardsWithFB();
      if(waitForMS(exitArenaTimeMS)){
        state = STATE_TurnToLobbyFront180;
      }
      break;
    case STATE_TurnToLobbyFront1:
      turnRight();
      if(turn90degtimercheck()){
        state = STATE_AlignBetweenTurns;
      }
      break;
    case STATE_AlignBetweenTurns:
      adjustAlignment();
      if(AlignedWithWallFine(superfinealignThresholdMM)){
        state = STATE_TurnToLobbyFront2;
      }
      break;
    case STATE_TurnToLobbyFront2:
      turnRight();
      if(waitForMS(turn90degtime)){
        state = STATE_AlignWLobbyFrontWall;
      }
      break;
    case STATE_TurnToLobbyFront180:
      turnRight();
      if(waitForMS(turn180degtime)){
        state = STATE_AlignWLobbyFrontWall;
      }
      break;
    case STATE_AlignWLobbyFrontWall:
      adjustAlignment();
      if(AlignedWithWallFine(finealignThresholdMM)){
        state = STATE_ApproachLobbyFrontWall;
      }
      break;
    case STATE_ApproachLobbyFrontWall:
      goStraightWithFB();
      if(DetectRangeWithin(StartWallDistanceCM)){
        state = STATE_TurnToStart;
      }
      break;
    case STATE_TurnToStart:
      turnRight();
      if(turn90degtimercheck()){
        // state = STATE_AligningWGoalWall;
        state = STATE_AlignWStartForReload;
      }
      break;
    case STATE_AlignWStartForReload:
      adjustAlignment();
      if(AlignedWithWallFine(finealignThresholdMM)){
        state = STATE_ApproachStart;
      }
      break;
    case STATE_ApproachStart:
      goStraightWithFB();
      if (limitSwitchR.isPressed() || limitSwitchL.isPressed()){
        state = STATE_Reloading;
      }
      break;
    case STATE_Reloading:
    stopMoving();
      if(waitForMS(ReloadingMS)){
        state = STATE_DisengageFromStart;
      }
      break;
    case STATE_DisengageFromStart:
      goBackwardsWithFB();
      if(DetectRangeFurtherThanAND(TurnToFarWallDistanceCM)){
        state = STATE_TurnToFarWall;
      }
      break;
    case STATE_TurnToFarWall:
      turnLeft();
      if(turn65degtimercheck()){
        state = STATE_AligningWArenaWall;
      }
      break;
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
  updateDisplay();
}


