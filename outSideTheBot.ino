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

// The shield
NXShield nxshield;
SoftwareSerial lcd(2, 8);
NXTUS       sonarB;
NXTUS       sonarA;
NXTUS       infraSensor;
int updateDelay = 50; // X ms sensor / screen update time



void setup() {  
  Serial.begin(115200); //Initialize Baud Rate for Arduino
  delay(500);
  initializeDisplay();
  nxshield.init(SH_HardwareI2C); //Initialize NXShield  
  nxshield.waitForButtonPress(BTN_GO);  
  
  nxshield.bank_a.motorReset();
  nxshield.bank_b.motorReset();
  
  //
  // Initialize the i2c sensors.
  //
  sonarA.init( &nxshield, SH_BAS2 );
  sonarB.init( &nxshield, SH_BBS2 );
//tsStop.init( &nxshield, SH_BAS2 );
//  touchSensor.init(&nxshield, SH_BAS1);  
  //infraSensor.init(&nxshield, SH_BAS2);
}

void loop(){
  
  
  //getInfraData();
  //displayTest();
  //clearDisplay();
  findCenter(5);
  //delay(500);
  //testSlow();
  
  //int senData = infraSensor.getDist();
  
  //if(senData > 30){
    //pullyDown(100);
    //delay(500);
  //}
  //pullyUp(100);
  //delay(500);
}



//***********************MOTOR CONTROL***********************\\

void railLeft(int motorSpeed){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, motorSpeed);  
}

void railRight(int motorSpeed){
  nxshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, motorSpeed);  
}

void pullyUp(int motorSpeed){
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, motorSpeed); 
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Forward, motorSpeed,
                     6, 
                     SH_Completion_Wait_For,
                     SH_Next_Action_BrakeHold); 
}

void pullyDown(int motorSpeed){
  //nxshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, motorSpeed);
  nxshield.bank_a.motorRunRotations(SH_Motor_2, SH_Direction_Reverse, motorSpeed,
                     6, 
                     SH_Completion_Wait_For,
                     SH_Next_Action_BrakeHold);   
}

void stopMoving(){
  nxshield.bank_a.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
  //nxshield.bank_b.motorRunSeconds(SH_Motor_Both, SH_Direction_Reverse, 0, 0, SH_Completion_Dont_Wait, SH_Next_Action_Brake);
}

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

//***********************MOTOR CONTROL***********************\\

//void testTouch(){  
//   while(!touchSensor.isPressed()){
//     test(50);
//     }
//     stopMoving();
//     delay(250);
//  Serial.println("button is pressed");
//}

void getInfraData(){
  char aa[80];
  char str[256];
  int  ab_us;
  int  bb_us;

  ab_us = sonarA.getDist();
  sprintf (str, "SonarA: Obstacle at: %d mm", ab_us );
  Serial.println(str);
  Serial.println( "-------------" );
  delay (500);
  
  bb_us = sonarB.getDist();
  sprintf (str, "SonarB: Obstacle at: %d mm", bb_us );
  Serial.println(str);
  Serial.println( "-------------" );
  delay (500);

}

//***********************LCD FUNCTIONS****************\\

void clearDisplay() {
  lcd.write(0xFE);
  lcd.write(0x01); 
}

void initializeDisplay(){
  lcd.begin(9600); 
  delay(500);
  lcd.print("Setup Starting...");  
}

void displayTest(){
  clearDisplay();
  delay(500);
  lcd.print("Test");
  delay(500); 
}

//***********************LCD FUNCTIONS****************\\

void findCenter(int threshold){
  Serial.println("Inside find center");
  int distA = sonarA.getDist();
  int distB = sonarB.getDist();

  
  while( abs(distA - distB) > threshold) {
  delay(updateDelay);
  distA = sonarA.getDist();
  distB = sonarB.getDist();
  clearDisplay();
    

    lcd.print("Finding Center: ");
    //setLCDCursor(16);
    lcd.print("A: ");
    lcd.print(distA);
    lcd.print(" B: ");
    lcd.print(distB);
//
    if(distA > distB) {
      railRight(100); 
    } else {
      railLeft(100);
   }
  }
  stopMoving();
}


//HELPER FUNCTIONS\\



//HELPER FUNCTIONS\\



