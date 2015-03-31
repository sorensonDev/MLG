#include <AbsoluteIMU.h>
#include <ACCLNx.h>
#include <AngleSensor.h>
#include <BaseI2CDevice.h>
#include <DISTNx.h>
#include <EV3Color.h>
#include <EV3Gyro.h>
#include <EV3InfraRed.h>
#include <EV3SensorAdapter.h>
#include <EV3Sonar.h>
#include <LineLeader.h>
#include <MagicWand.h>
#include <MsTimer2.h>
#include <NumericPad.h>
#include <NXShield.h>
#include <NXShieldAGS.h>
#include <NXShieldI2C.h>
#include <NXTCam.h>
#include <NXTCurrentMeter.h>
#include <NXTHID.h>
#include <NXTLight.h>
#include <NXTMMX.h>
#include <NXTPowerMeter.h>
#include <NXTServo.h>
#include <NXTTouch.h>
#include <NXTUS.h>
#include <NXTVoltMeter.h>
#include <PFMate.h>
#include <PiLight.h>
#include <PSPNx.h>
#include <RCXLight.h>
#include <RTC.h>
#include <SHDefines.h>
#include <SoftI2cMaster.h>
#include <SumoEyes.h>
#include <Wire.h>
#include <NXShield.h>
#include <NXTUS.h>
#include <SoftwareSerial.h>
/*   
~~~~~~TABLE OF CONTENTS~~~~~
-INIT COMMANDS
-MAIN LOOP
-MOTOR COMMANDS
-LCD COMMANDS
-SANDBOX
-PROCEDURES



*/

/////////////////////////////////////////////////////////////////////////
///////////////////////////// INIT COMMANDS /////////////////////////////
/////////////////////////////////////////////////////////////////////////


// The shield: Global Variables & Setup
NXShield nxshield;
SoftwareSerial lcd(2, 8);
NXTUS       sonarFrontLeft;
NXTUS       sonarFrontRight;
NXTUS       infraSensor;
int updateDelay = 50; // X ms sensor / screen update time
int mainDelay = 40; // x ms sensor .. tweak value to allow arduino to think between function calls
int intSpeed = 75; //set default speed for testing/tweaking
int fisherPricePin = 9; //the pin we're using to control the fisher-price motor.


void setup() {  
  //Serial.begin(115200); //Initialize Baud Rate for Arduino
  //delay(500);
  nxshield.init(SH_HardwareI2C); //Initialize NXShield  
  
  //Initialize the i2c motors.
  nxshield.bank_a.motorReset();
  nxshield.bank_b.motorReset();
  //Initialize the i2c sensors.
  sonarFrontRight.init( &nxshield, SH_BBS2 );
  sonarFrontLeft.init( &nxshield, SH_BAS2 );
  //Initialize the fisher-price motor
  pinMode(fisherPricePin, OUTPUT);
  
                //tsStop.init( &nxshield, SH_BAS2 );
                //touchSensor.init(&nxshield, SH_BAS1);  
                //infraSensor.init(&nxshield, SH_BAS2);
  

  
  initializeDisplay();
  // Check battery voltage on startup. Warn if low.
  float batVolt = (float) nxshield.bank_a.nxshieldGetBatteryVoltage() / 1000;
  if(batVolt < 7.50) {
    clearDisplay();
    lcd.print("LOW VOLTAGE!!!");
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt);
  } else {
    clearDisplay();
    lcd.print("Press GO!");
    setLCDCursor(16);
    lcd.print("voltage: ");
    lcd.print(batVolt); 
  }
  nxshield.waitForButtonPress(BTN_GO);  //This call allows for the button "go" to be pressed in order to start the robot.
}


/*
/////////////////////////////////////////////////////////////////////////
/////////////////////////////// MAIN LOOP ///////////////////////////////
/////////////////////////////////////////////////////////////////////////
This function is the 'main' part of the code that loops forever. This 
will run functions that are defined below and will continue running 
until the arduino is stopped in some way.
*/
void loop(){
  
  fisherOn();
  //getInfraData();
  //displayTest();
  //clearDisplay();
  //fullLeft();
  //delay(mainDelay );
  //moveRight(100);
  //delay(mainDelay);
  //secureBall(intSpeed);
//  delay(mainDelay);
//  ballLift();
  delay(2000);//2 seconds
  fisherOff();
  delay(2000);//2 seconds
  //railLeft(100);
  //moveRight();
//delay(10);
  //findCenter(5);

 // ballLift();
 // delay(mainDelay);
 // dropLift(100);
  //stopMoving();
  //shootBall(intSpeed);
  //dropLift(intSpeed);
  //delay(mainDelay);
  //moveRight(intSpeed);
  //delay(mainDelay);
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////// MOTOR COMMANDS /////////////////////////////
/////////////////////////////////////////////////////////////////////////

//*****************************GENERAL****************************\\
void stopMoving(){
  nxshield.bank_a.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
  //nxshield.bank_b.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
}

//***********************RAIL MOTOR CONTROL***********************\\

void railLeft(int motorSpeed){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, motorSpeed); 
  //nxshield.ledSetRGB(0,0,8); 
}

void railRight(int motorSpeed){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, motorSpeed);
  //nxshield.ledSetRGB(8,0,0);  
}

void moveRight(){
  int sonarData = sonarFrontLeft.getDist();
  while(sonarData < 100){
    nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, 100); 
    sonarData = sonarFrontLeft.getDist();
  }
  stopMoving();
  secureBall(100);
  
}

//***********************PULLY MOTOR CONTROL***********************\\
void pullyDown(int motorSpeed){
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, motorSpeed); 
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Forward, motorSpeed,
                     4,
                     SH_Completion_Wait_For ,
                     SH_Next_Action_Float); 
}

void pullyUp(int motorSpeed){
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, motorSpeed);
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Reverse, motorSpeed,
                     4,
                     SH_Completion_Wait_For ,
                     SH_Next_Action_Brake);   
}

void shootBall(int motorSpeed){
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Reverse, motorSpeed,
                     1,
                     SH_Completion_Dont_Wait ,
                     SH_Next_Action_Float);   
}

void secureBall(int motorSpeed){
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, motorSpeed);
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Reverse, motorSpeed,
                     1,
                     SH_Completion_Wait_For ,
                     SH_Next_Action_Float);   
}

void dropLift(int motorSpeed){
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, motorSpeed);
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Forward, motorSpeed,
                     1, 
                     SH_Completion_Wait_For ,
                     SH_Next_Action_Float);   
}






//**********************RELAY MOTOR******************\\
void fisherOn() {
  digitalWrite(fisherPricePin, HIGH);
}

void fisherOff() {
  digitalWrite(fisherPricePin, LOW);
}




/////////////////////////////////////////////////////////////////////////
///////////////////////////// LCD COMMANDS //////////////////////////////
/////////////////////////////////////////////////////////////////////////
void clearDisplay() {
  lcd.write(0xFE);
  lcd.write(0x01); 
}

void setLCDCursor(byte cursor_position){
 lcd.write(0xFE); // ready LCD for special command
 lcd.write(0x80); // ready LCD to recieve cursor potition
 lcd.write(cursor_position); // send cursor position 
}

void initializeDisplay(){
  lcd.begin(9600); 
  delay(500);
  clearDisplay();
  lcd.print("Setup Starting. ");  
}

void displayTest(){
  clearDisplay();
  delay(500);
  lcd.print("Test");
  delay(500); 
}


/////////////////////////////////////////////////////////////////////////
//////////////////////////////// SANDBOX ////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/* Playing with touch sensor
void testTouch(){  
   while(!touchSensor.isPressed()){
     test(50);
     }
     stopMoving();
     delay(250);
  Serial.println("button is pressed");
}
*/


/* Playing with infared
void getInfraData(){
  char aa[80];
  char str[256];
  int  ab_us;
  int  bb_us;
  */

/* Using Serial to read sonar input
  ab_us = sonarFrontRight.getDist();
  sprintf (str, "sonarR: Obstacle at: %d mm", ab_us );
  Serial.println(str);
  Serial.println( "-------------" );
  delay (500);
  
  bb_us = sonarFrontLeft.getDist();
  sprintf (str, "SonarL: Obstacle at: %d mm", bb_us );
  Serial.println(str);
  Serial.println( "-------------" );
  delay (500);
  */
  
  void testSlow(){
     // Drive motor 1 forward and backward for a specific number of
    // rotations
    char            str[40];
    long            rotations = 6;
    delay(1000);
    Serial.println("Bank A motors >>");
    sprintf(str, "Motor 1 Forward %d Rotations", rotations);
    Serial.println(str);
    str[0] = '\0';
    nxshield.bank_a.motorRunRotations(SH_Motor_1, 
                     SH_Direction_Forward, 
                     SH_Speed_Medium,
                     rotations, 
                     SH_Completion_Wait_For,
                     SH_Next_Action_BrakeHold); 
  
}


void findCenter(int threshold){
  int distA = sonarFrontRight.getDist();
  int distB = sonarFrontLeft.getDist();

  
  while( abs(distA - distB) > threshold) {
  delay(updateDelay);
  distA = sonarFrontRight.getDist();
  distB = sonarFrontLeft.getDist();
  //clearDisplay();
    

//    lcd.print("Finding Center: ");
//    //setLCDCursor(16);
//    lcd.print("R: ");
//    lcd.print(distA);
//    lcd.print(" L: ");
//    lcd.print(distB);
//
    if(distA > distB) {
        railRight(100);
    } else {
        railLeft(100);
    }
  }
  stopMoving();
}

//void fullLeft(){
//    int distA = sonarFrontRight.getDist();
//    
//    while( distA > 6) {
//      //if(distA > 103) stopMoving();
//          clearDisplay();
//          lcd.print("Full Left       ");
//          lcd.print("A: ");
//          lcd.print(distA);
//          
//      delay(updateDelay);
//      distA = sonarFrontRight.getDist();
//      railLeft(100);
//    }
//    stopMoving();
//
//}

void fullLeft(){
    int distA = sonarFrontLeft.getDist();
    
    while( distA < 100) {
//          clearDisplay();
//          lcd.print("Full Left       ");
//          lcd.print("A: ");
//          lcd.print(distA);
          
      delay(updateDelay);
      distA = sonarFrontLeft.getDist();
      railRight(100);
    }
    //clearDisplay();
    stopMoving();

}

//HELPER FUNCTIONS\\
void ballLift(){
  secureBall(100);
  pullyUp(intSpeed);
  delay(mainDelay);
  shootBall(100);
  delay(mainDelay);
  pullyDown(intSpeed);
  delay(mainDelay);
  dropLift(100); 
}

/////////////////////////////////////////////////////////////////////////
///////////////////////////// PROCEDURES ////////////////////////////////
/////////////////////////////////////////////////////////////////////////

