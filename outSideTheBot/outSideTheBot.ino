#include <AbsoluteIMU.h>
#include <ACCLNx.h>
#include <AngleSensor.h>
#include <BaseI2CDevice.h>
#include <DISTNx.h>
//#include <EV3Color.h>
//#include <EV3Gyro.h>
//#include <EV3InfraRed.h>
#include <EV3SensorAdapter.h>
#include <EV3Sonar.h>
//#include <LineLeader.h>
#include <NXShield.h>
#include <NXShieldAGS.h>
#include <NXShieldI2C.h>
#include <NXTCam.h>
#include <NXTHID.h>
#include <NXTLight.h>
#include <NXTMMX.h>
#include <NXTPowerMeter.h>
#include <NXTServo.h>
#include <NXTTouch.h>
#include <NXTUS.h>
#include <PFMate.h>
#include <PSPNx.h>
#include <RTC.h>
#include <SoftI2cMaster.h>
#include <SumoEyes.h>
#include <Wire.h>
#include <NXShield.h>
#include <NXTUS.h>
#include <SoftwareSerial.h>

//**************Refactoring LOG JDS ************\\
/*
3:13PM changed a few delay values, made a function called fullRight()
3:19PM it seems that the orientation of our motor is hooked up backwards
physically, RailLeft actually goes right and likewise with RailRight. I have
not updated the code yet to give correct behavior
3:31PM Josh changed functionality of pullyUp and pullyDown, modfified to reflect
better behavior during field testing. Changed number of rotations.
4:25PM Josh tweaked number of rotations in pullyUp pully down. I believe that 3 rotations
instead of 6 is sufficient considering that we replaced a smaller gear with the bigger wyrmgear.
Tom changed fisherSpinup to 700
*/




/*
~~~~~~TABLE OF CONTENTS~~~~~
-INIT COMMANDS
-MAIN LOOP
-MOTOR COMMANDS
-LCD & SENSOR COMMMANDS


*/

/////////////////////////////////////////////////////////////////////////
///////////////////////////// INIT COMMANDS /////////////////////////////
/////////////////////////////////////////////////////////////////////////


// The shield: Global Variables & Setup
NXShield nxshield;
SoftwareSerial lcd(2, 10);
NXTUS       sonarFrontLeft;
NXTUS       sonarFrontRight;
NXTUS       sonarBall;
int updateDelay = 50; // X ms sensor / screen update time
int mainDelay = 40; // x ms sensor .. tweak value to allow arduino to think between function calls
int fisherPricePin = 9; //the pin we're using to control the fisher-price motor.
int fisherSpinup = 700; //amount of ms for fischer to spin up //5oo seemed like it may be not 
int centerOffset = 0;  //used in second loop of findCenter POSITIVE IS RIGHT


void setup() {
  nxshield.init(SH_HardwareI2C); //Initialize NXShield

  //Initialize the i2c motors.
  nxshield.bank_a.motorReset();
  nxshield.bank_b.motorReset();
  //Initialize the i2c sensors.
  sonarFrontRight.init( &nxshield, SH_BBS2 );
  sonarFrontLeft.init( &nxshield, SH_BAS2 );
  sonarBall.init( &nxshield, SH_BBS1 );
  //Initialize the fisher-price motor
  pinMode(fisherPricePin, OUTPUT);
  
  initializeDisplay();
  // Check battery voltage on startup. Warn if low.
  showVoltage();
  
  nxshield.waitForButtonPress(BTN_GO);  //This call allows for the button "go" to be pressed in order to start the robot.
}


/*
/////////////////////////////////////////////////////////////////////////
/////////////////////////////// MAIN LOOP ///////////////////////////////
/////////////////////////////////////////////////////////////////////////
This function is the 'main' part of the code that loops forever. This
will run functions that are defined below and will continue running
until the arduino is stopped in some way.
====>RUN TOP LEVEL PROCEDURE HERE<====
*/
void loop() {
  
////lcd.print(hasBall());

  //These work:
  //printLocation();//Just displays distance to walls on display
  //fullLeft();//Full speed to left wall, stop motor when hit.

  //findCenter(2);//Runs fast loop, then slow loop to refit to threshold (the argument).
  
  showVoltage();
  fisherOn();
  delay(fisherSpinup);
  ballLift();

  //delay(100);//2 seconds //@JDS 2:56PM Changed this to tenth of second from 2 seconds.
  //ballLift();
  //fisherOff();
  //Now that we just shot a ball, we want to move either left or right to retrieve a new ball to shoot.
  fullLeft(4500);//argument is timeout
  //delay(750);
  findCenter(3);//Runs fast loop, then slow loop to refit to threshold (the argument).
  showVoltage();
  fisherOn();
  delay(fisherSpinup);//2 seconds //@JDS 2:56PM Changed this to half a second from 2 seconds.
  ballLift();
  //fisherOff();
  fullRight(3000);//argument is timeout
  findCenter(3);
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////// MOTOR COMMANDS /////////////////////////////
/////////////////////////////////////////////////////////////////////////

//*****************************GENERAL****************************\\
void stopMoving() {
  nxshield.bank_a.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
  //nxshield.bank_b.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
}

//***********************RAIL MOTOR CONTROL***********************\\

void moveLeft(int motorSpeed) {
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, motorSpeed);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, motorSpeed);
}

void moveRight(int motorSpeed) {
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, motorSpeed);
  nxshield.bank_b.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, motorSpeed);
}

void fullLeft(int timeOut) {
  bool hasBallNow = 0;
  int distToWall = sonarFrontLeft.getDist();
  long startTime = millis();
  long deltaTime = 0;
  moveRight(100);

  while (distToWall > 10 && deltaTime < timeOut && !hasBallNow) {
    delay(updateDelay);
    printDistances("fullLeft:", distToWall, sonarFrontRight.getDist());
    distToWall = sonarFrontLeft.getDist();
    deltaTime = millis() - startTime;
    hasBallNow = hasBall();

  }
  //keep searching if ball not found
  if(!hasBallNow){
    fullRight(4500); 
  }
  printLocation();
  stopMoving();
}

void fullRight(int timeOut) {
  bool hasBallNow = 0;
  int distToWall = sonarFrontRight.getDist();
  long startTime = millis();
  long deltaTime = 0;

  moveLeft(100);

  while (distToWall > 10 && deltaTime < timeOut && !hasBallNow ) {
    delay(updateDelay);
    printDistances("fullRight:", sonarFrontLeft.getDist(), distToWall);
    distToWall = sonarFrontRight.getDist();
    deltaTime = millis() - startTime;
    hasBallNow = hasBall();

  }
  //keep searching if ball not found
  if(!hasBallNow){
    fullLeft(4500); 
  }
  printLocation();
  stopMoving();
}


void findCenter(int threshold) {
  int distLeft = sonarFrontLeft.getDist();
  int distRight = sonarFrontRight.getDist();
  long startTime = millis();
  long deltaTime = 0;
  
  while (abs(distLeft - distRight) > 30){ //First loop: get CLOSISH (trying 30) to middle
    delay(updateDelay);
    distLeft = sonarFrontLeft.getDist();
    distRight = sonarFrontRight.getDist();
    printDistances("Finding Center:", distLeft, distRight);
    if (distLeft > distRight) {
      if (distLeft == 255) { //the sonar is pointed at a ball and needs to be corrected
        moveLeft(50);
      } else {
        moveRight(100);
      }
    } else {
      if (distRight == 255) {
        moveRight(50);
      } else {
        moveLeft(100);
      }
    }

  }
  distLeft = sonarFrontLeft.getDist() - centerOffset;
  distRight = sonarFrontRight.getDist() + centerOffset;
  while (abs(distLeft - distRight) > threshold) { //Second loop: refine at slow speed
    delay(updateDelay);
    distLeft = sonarFrontLeft.getDist() - centerOffset;
    distRight = sonarFrontRight.getDist() + centerOffset;
    printDistances("Refining Center:", distLeft, distRight);
    if (distLeft > distRight) {
      moveRight(30);
    } else {
      moveLeft(30);
    }
  }
  stopMoving();
  printLocation();
}


//***********************PULLY MOTOR CONTROL***********************\\
void pullyDown(int motorSpeed, int rotations) {
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, motorSpeed);
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Forward, motorSpeed, rotations, SH_Completion_Wait_For, SH_Next_Action_Float);
}

void pullyUp(int motorSpeed, int rotations) {
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, motorSpeed);
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Reverse, motorSpeed, rotations, SH_Completion_Wait_For, SH_Next_Action_Brake);
}

void shootBall(int motorSpeed) {
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Reverse, motorSpeed, 1, SH_Completion_Dont_Wait, SH_Next_Action_Float);
}

void dropLift(int motorSpeed) {
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, motorSpeed);
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Forward, motorSpeed, 1, SH_Completion_Wait_For, SH_Next_Action_Float);
}

void ballLift() {
  pullyUp(100, 2); //speed, numRotations
  delay(mainDelay);
  shootBall(100);
  delay(250);
  fisherOff();
  //delay(mainDelay);
  pullyDown(100, 2);
  //delay(mainDelay);
  //dropLift(100);
}



//**********************RELAY MOTOR******************\\
void fisherOn() {
  digitalWrite(fisherPricePin, HIGH);
}

void fisherOff() {
  digitalWrite(fisherPricePin, LOW);
}


/////////////////////////////////////////////////////////////////////////
///////////////////////////// LCD & SENSOR COMMMANDS //////////////////////////////
/////////////////////////////////////////////////////////////////////////
void clearDisplay() {
  lcd.write(0xFE);
  lcd.write(0x01);
}

void setLCDCursor(byte cursor_position) {
  lcd.write(0xFE); // ready LCD for special command
  lcd.write(0x80); // ready LCD to recieve cursor potition
  lcd.write(cursor_position); // send cursor position
}

void initializeDisplay() {
  lcd.begin(9600);
  delay(500);
  clearDisplay();
  lcd.print("Setup Starting. ");
}

void displayTest() {
  clearDisplay();
  delay(500);
  lcd.print("Test");
  delay(500);
}

void printLocation() {
  int refreshDisplay = 0;
  int distLeft = sonarFrontRight.getDist();
  int distRight = sonarFrontLeft.getDist();

  if (refreshDisplay = updateDelay) {
    clearDisplay();
    lcd.print("Location: ");
    setLCDCursor(16);
    lcd.print("R: ");
    lcd.print(distLeft);
    lcd.print(" L: ");
    lcd.print(distRight);
    refreshDisplay = 0;
  }
}

void printDistances(const char* title, int left, int right) {
  int refreshDisplay = 0;
  if (refreshDisplay = updateDelay) {
    clearDisplay();
    lcd.print(title);
    setLCDCursor(16);
    lcd.print("L: ");
    lcd.print(left);
    lcd.print(" R: ");
    lcd.print(right);
    refreshDisplay = 0;
  }
}

void showVoltage() {
  float batVolt = (float) nxshield.bank_a.nxshieldGetBatteryVoltage() / 1000;
  if (batVolt < 7.50) {
    clearDisplay();
    lcd.print("LOW VOLTAGE!!!");
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt);
  } else {
    clearDisplay();
    lcd.print("L: ");
    lcd.print(sonarFrontLeft.getDist());
    lcd.print(" R: ");
    lcd.print(sonarFrontRight.getDist());
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt);
  }
}

//************SENSOR***************\\
bool hasBall(){
 return (sonarBall.getDist() > 250 || sonarBall.getDist() < 10); 
}
