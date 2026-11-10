/*
* The PAC1 bot code.
*
* Made with tender, love, and care (and blood) by:
* The PAC1 Team
*
* Version 52
*
*/

//Libraries to include
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>

//Defined statements
#define DiameterOfWheel 3.0
#define PI 3.14159265
#define EncoderCounts 318
#define WheeltoWheel 7.45
#define ServoMin 575
#define ServoMax 2400
#define MotorSpeed 30.0
#define RPSSleep .3
#define ReverseSpeed 15
#define CountsPerDegree 2.23
#define CountsPerInch 33.7579617834
#define HeadingCheckCompensate 2.0
#define KeyRPSLocations 2
#define RPSDDRLightX 22.7
#define RPSDDRLightY 14.9
#define RPSCoinX 17.4
#define RPSCoinY 47
#define RPSStairX 22.5
#define RPSStairY 42.3

//Declare Encoders and CDS cell
DigitalEncoder rightEncoder(FEHIO::P2_7);
DigitalEncoder leftEncoder(FEHIO::P0_0);
AnalogInputPin Cds(FEHIO::P2_0);

//Declare the servo arm and motors
FEHServo arm(FEHServo::Servo7);
FEHMotor rightMotor(FEHMotor::Motor0, 9.0);
FEHMotor leftMotor(FEHMotor::Motor1, 9.0);

//Declare the bump_switches
//DigitalInputPin bumpFrontLeft(FEHIO::P0_3);
//DigitalInputPin bumpFrontRight(FEHIO::P0_3);

/*
* Prototypes for all methods.
*/
void ResetCounts();
void calibrate();
void moveArm(int d);
void waitForStart();
void turnLeft(float p, float d);
void turnRight(float p, float d);
void goForward(float p, float d, float t);
int check_heading(float d);
int check_x(float p, float x);
int check_y(float p, float y);
void pressDDRButton();
void goToDDRLight();
void goUpRamp();
void goToCoin();
void goToFoosball();
void flipLever();
void moveFoos();
void loadMenu();
void drawMenu();
void openMenu();

/*
* Declare global variables.
*/
FEHIcon::Icon startButton, backButton, calibrateRPS, keepRPS, setRPS, coordsButton, courseButton, turnRButton, turnLButton, forwardButton;
bool RPSInitialized = false;
//RPS key locations (0 = DDRLight, 1 = Coin position)
float rpsXPos[KeyRPSLocations] = {RPSDDRLightX, RPSStairX};
float rpsYPos[KeyRPSLocations] = {RPSDDRLightY, RPSStairY};
char* keyPos[KeyRPSLocations] = {"DDR left light", "Stairs corner"};

/*
* Method for removing counts based on speed
*/
//int calculateMomentumCounts(float percent) {
//   //A quadratic equation I made to solve for over counts from Exploration 2.
//   return 0.01*percent*percent + 0.35*percent - 7;
//}

/*
* Method for reseting the counters.
*/
void ResetCounts(){
  rightEncoder.ResetCounts();
  leftEncoder.ResetCounts();
}

/*
* Method for calibrating everything.
*/
void calibrate(){
  arm.SetMax(ServoMax);
  arm.SetMin(ServoMin);
  moveArm(0);
}

/*
* Method for moving the servo arm.
*/
void moveArm(int degree){
  arm.SetDegree(degree);
}

/*
* Waits for the start of the run.
*/
void waitForStart() {
  while(Cds.Value() > 0.70) {
      LCD.Clear();
      LCD.WriteLine(Cds.Value());
      Sleep(50);
  }
}

/*
* Method for turning left based on percent speed and degree.
*/
void turnLeft(float percent, float degree){
  //Reset Counts
  ResetCounts();

  //Find the number of counts to move forward
  float NumberOfCounts = CountsPerDegree*degree;

  //Set motor speeds to that percent
  rightMotor.SetPercent(-percent);
  leftMotor.SetPercent(-percent);

  //Drive until actual counts is greater than number of counts
  while((rightEncoder.Counts() + leftEncoder.Counts()) / 2 < NumberOfCounts) {
      int fix = 5;
      if (leftEncoder.Counts()>rightEncoder.Counts()){
          if(percent > 0) {
              rightMotor.SetPercent(-(percent+fix));
          } else {
              rightMotor.SetPercent(-(percent-fix));
          }
          leftMotor.SetPercent(-percent);
      } else if (leftEncoder.Counts()<rightEncoder.Counts()){
          rightMotor.SetPercent(-percent);
          if(percent > 0) {
              leftMotor.SetPercent(-(percent+fix));
          } else {
              leftMotor.SetPercent(-(percent-fix));
          }
      } else {
          rightMotor.SetPercent(-percent);
          leftMotor.SetPercent(-percent);
      }
  }

  //Stop Motors
  rightMotor.Stop();
  leftMotor.Stop();

  //Wait for actual counts then get actual counts and reset counts
  Sleep(200);
  int actualCounts = (rightEncoder.Counts() + leftEncoder.Counts()) / 2;
  ResetCounts();

  //Find how many counts to reverse then move back that much
  int reverseCounts = actualCounts - NumberOfCounts;
  rightMotor.SetPercent(ReverseSpeed);
  leftMotor.SetPercent(ReverseSpeed);
  while((rightEncoder.Counts() + leftEncoder.Counts()) / 2 < reverseCounts);

  //Stop Motors
  rightMotor.Stop();
  leftMotor.Stop();
  Sleep(200);
}

/*
* Method for turning right based on percent speed and degree.
*/
void turnRight(float percent, float degree){
  //Reset Counts
  ResetCounts();

  //Find the number of counts to move forward
  float NumberOfCounts = CountsPerDegree*degree;

  //Set motor speeds to that percent
  leftMotor.SetPercent(percent);
  rightMotor.SetPercent(percent);

  //Drive until actual counts is greater than number of counts
  while((rightEncoder.Counts() + leftEncoder.Counts()) / 2 < NumberOfCounts) {
      int fix = 5;
      if (leftEncoder.Counts()>rightEncoder.Counts()){
          if(percent > 0) {
              rightMotor.SetPercent(percent+fix);
          } else {
              rightMotor.SetPercent(percent-fix);
          }
          leftMotor.SetPercent(percent);
      } else if (leftEncoder.Counts()<rightEncoder.Counts()){
          rightMotor.SetPercent(percent);
          if(percent > 0) {
              leftMotor.SetPercent(percent+fix);
          } else {
              leftMotor.SetPercent(percent-fix);
          }
      } else {
          rightMotor.SetPercent(percent);
          leftMotor.SetPercent(percent);
      }
  }

  //Stop Motors
  rightMotor.Stop();
  leftMotor.Stop();


  //Wait for actual counts then get actual counts and reset counts
  Sleep(200);
  int actualCounts = (rightEncoder.Counts() + leftEncoder.Counts()) / 2;
  ResetCounts();

  //Find how many counts to reverse then move back that much
  int reverseCounts = actualCounts - NumberOfCounts;
  rightMotor.SetPercent(-ReverseSpeed);
  leftMotor.SetPercent(-ReverseSpeed);
  while((rightEncoder.Counts() + leftEncoder.Counts()) / 2 < reverseCounts);

  //Stop Motors
  rightMotor.Stop();
  leftMotor.Stop();
  Sleep(200);
}

/*
* Method for going forward at a percent speed and a distance in inches.
*/
void goForward(float percent, float distance, float timeOut){
  //Reset Counts
  ResetCounts();

  //Find the number of counts to move forward
  float NumberOfCounts = distance*CountsPerInch;

  //Set motor speeds to that percent
  rightMotor.SetPercent(-percent);
  leftMotor.SetPercent(percent); //To fix veering left

  //Get current time
  float timeStart = TimeNow();

  //Drive until actual counts is greater than number of counts
  while((rightEncoder.Counts() + leftEncoder.Counts()) / 2 < NumberOfCounts && TimeNow()-timeStart < timeOut) {
      int fix = 5;

      //Set motor speeds to that percent
      if (leftEncoder.Counts()>rightEncoder.Counts()){
          if(percent > 0) {
              rightMotor.SetPercent(-(percent+fix));
          } else {
              rightMotor.SetPercent(-(percent-fix));
          }
          leftMotor.SetPercent(percent);
      } else if (leftEncoder.Counts()<rightEncoder.Counts()){
          rightMotor.SetPercent(-percent);
          if(percent > 0) {
              leftMotor.SetPercent(percent+fix);
          } else {
              leftMotor.SetPercent(percent-fix);
          }
      } else {
          rightMotor.SetPercent(-percent);
          leftMotor.SetPercent(percent);
      }
  }

  //Stop Motors
  rightMotor.Stop();
  leftMotor.Stop();
}

void goForwardRamp(float percent, float distance, float timeOut){
  //Reset Counts
  ResetCounts();

  //Find the number of counts to move forward
  float NumberOfCounts = distance*CountsPerInch;

  //Set motor speeds to that percent
  rightMotor.SetPercent(-percent);
  leftMotor.SetPercent(percent); //To fix veering left

  //Get current time
  float timeStart = TimeNow();

  //Set fix and next time to check RPS
  int fix = 7;

  //Drive until actual counts is greater than number of counts
  while((rightEncoder.Counts() + leftEncoder.Counts()) / 2 < NumberOfCounts && TimeNow()-timeStart < timeOut) {
      int rpsfix = 0;
      if(RPS.Heading()>271.5&&RPS.Heading()>=0){
          rpsfix = -2;
      }
      else if(RPS.Heading()<270.5&&RPS.Heading()>=0){
          rpsfix = 2;
      }

      //Set motor speeds to that percent
      if (leftEncoder.Counts()>rightEncoder.Counts()){
          leftMotor.SetPercent(percent-rpsfix);
          if(percent > 0) {
              rightMotor.SetPercent(-(percent+fix));
          } else {
              rightMotor.SetPercent(-(percent-fix));
          }

      } else if (leftEncoder.Counts()<rightEncoder.Counts()){
          rightMotor.SetPercent(-percent-rpsfix);
          if(percent > 0) {
              leftMotor.SetPercent(percent+fix);
          } else {
              leftMotor.SetPercent(percent-fix);
          }
      } else {
          rightMotor.SetPercent(-percent);
          leftMotor.SetPercent(percent-rpsfix);
      }
  }

  //Stop Motors
  rightMotor.Stop();
  leftMotor.Stop();
}

int check_heading(float angle) //using RPS
{
  //Check to see if RPS is working
  if (RPS.Heading() != -1 || RPS.Heading() != -2) {
      //Check wether robot is within acceptable amount
      while(RPS.Heading() < angle - 1.5 || RPS.Heading() > angle + 1.5)
      {

          if(angle==0){
              if(RPS.Heading() > 180)
              {
                  //pulse the motors for a short duration in the correct direction
                  turnLeft(30.,360-RPS.Heading()); //1.2
                  leftMotor.Stop();
                  rightMotor.Stop();
                  Sleep(RPSSleep);
                  LCD.Write("   Heading: ");
                  LCD.Write(RPS.Heading());
              }
              else if(RPS.Heading() < 180)
              {
                  //pulse the motors for a short duration in the correct direction
                  turnRight(30.,RPS.Heading());
                  leftMotor.Stop();
                  rightMotor.Stop();
                  Sleep(RPSSleep);
                  LCD.Write("   Heading: ");
                  LCD.Write(RPS.Heading());
              }
          }

          else{
              if(RPS.Heading() > angle)
              {
                  //pulse the motors for a short duration in the correct direction
                  if(RPS.Heading() > 0) {
                      turnRight(30.,RPS.Heading()- angle-.5);
                  } else {
                      turnRight(30.,1.2);
                  }
                  leftMotor.Stop();
                  rightMotor.Stop();
                  Sleep(RPSSleep);
                  LCD.Write("   Heading: ");
                  LCD.Write(RPS.Heading());
              }
              else if((RPS.Heading() < angle))
              {
                  //pulse the motors for a short duration in the correct direction
                  if(RPS.Heading() > 0) {
                      turnLeft(30.,angle-RPS.Heading()-.5);
                  } else {
                      turnLeft(30.,1.2);
                  }
                  leftMotor.Stop();
                  rightMotor.Stop();
                  Sleep(RPSSleep);
                  LCD.Write("   Heading: ");
                  LCD.Write(RPS.Heading());
              }
          }
      }
      //Succesfull checked and adjusted, return 0
      return 0;
  }
  //RPS not working, return -1
  return -1;
}

int check_x(float percent, float x_coordinate) //using RPS while robot is in the x direction
{
  //Check to see if RPS is working
  if (RPS.X() != -1 || RPS.X() != -2) {
      //check whether the robot is within an acceptable range
      while(RPS.X() < x_coordinate - 0.35 || RPS.X() > x_coordinate + 0.35)
      {
          if(RPS.X() > x_coordinate)
          {
              //pulse the motors for a short duration in the correct direction
              LCD.Clear();
              LCD.Write(RPS.X());
              if(RPS.X() > 0) {
                  if(RPS.Heading() < 90 || RPS.Heading() > 270) {
                      goForward(percent, RPS.X()-x_coordinate-0.3, 50.0);
                  } else {
                      goForward(-percent, RPS.X()-x_coordinate-0.3, 50.0);
                  }
              } else {
                  goForward(-percent, 2, 2.0);
              }
          }
          else if(RPS.X() < x_coordinate)
          {
              LCD.Clear();
              LCD.Write(RPS.X());
              if(RPS.X() > 0) {
                  if(RPS.Heading() < 90 || RPS.Heading() > 270) {
                      goForward(-percent, x_coordinate-RPS.X()-0.3, 50.0);
                  } else {
                      goForward(percent, x_coordinate-RPS.X()-0.3, 50.0);
                  }
              } else {
                  goForward(percent, 2, 2.0);
              }
          }
          Sleep(RPSSleep);
      }
      //Succesfull checked and adjusted, return 0
      return 0;
  }
  //RPS not working, return -1
  return -1;
}

/*
* Method for checking the y position with RPS.
*/
int check_y(float percent, float y_coordinate) //using RPS while robot is in the y direction
{
  //Check to see if RPS is working
  if (RPS.Y() != -1 || RPS.Y() != -2) {
      //check whether the robot is within an acceptable range
      while(RPS.Y() < y_coordinate - 0.35 || RPS.Y() > y_coordinate + 0.35)
      {
          LCD.Clear();
          if(RPS.Y() > y_coordinate)
          {
              //pulse the motors for a short duration in the correct direction
              LCD.WriteLine(RPS.Y());
              if(RPS.Y() > 0) {
                  if(RPS.Heading() < 180) {
                      goForward(percent, RPS.Y()-y_coordinate-0.3, 50.0);
                  } else {
                      goForward(-percent, RPS.Y()-y_coordinate-0.3, 50.0);
                  }
              } else {
                  goForward(-percent, 2, 1.0);
              }
          }
          else if(RPS.Y() < y_coordinate)
          {
              //pulse the motors for a short duration in the correct direction
              LCD.WriteLine(RPS.Y());
              if(RPS.Y() > 0) {
                  if(RPS.Heading() < 180) {
                      goForward(-percent, y_coordinate-RPS.Y()-0.3, 50.0);
                  } else {
                      goForward(percent, y_coordinate-RPS.Y()-0.3, 50.0);
                  }
              } else {
                  goForward(percent, 2, 1.0);
              }
          }
          Sleep(RPSSleep);
      }
      //Succesfull checked and adjusted, return 0
      return 0;
  }
  //RPS not working, return -1
  return -1;
}

/*
* Presses the button based on the color seen.
*/
void pressDDRButton() {
  LCD.Clear();
  LCD.WriteLine(Cds.Value());
  //Red
  if (Cds.Value() < 0.4){
      LCD.Clear();
      LCD.WriteLine("Red");
      //check_heading(270);
      goForward(-MotorSpeed-15,30,6.5);


      turnRight(30.0, 30);
      goForward(MotorSpeed+5, 6, 5.0);
      turnRight(30.0, 70);
      check_heading(180);
      check_x(MotorSpeed-10, rpsXPos[0]+4);
      turnLeft(30.0, 90);
      goForward(MotorSpeed, 2, 3.0);
      check_heading(270);

      //Blue (Might want to add a case for if its not blue either (<.75))
  } else if (Cds.Value() > 0.4){
      LCD.Clear();
      LCD.WriteLine("Blue");
      turnLeft(30.0,90);
      goForward(-MotorSpeed-5, 4, 5.0);
      check_x(MotorSpeed-10,rpsXPos[0]+5.5);
      turnRight(30,90);
      //check_heading(270);
      goForward(-MotorSpeed-15,40,7.0);

      goForward(MotorSpeed+5, 10, 50.0);
//       turnRight(30.0, 95);
//       check_heading(180);
//       check_x(MotorSpeed-10, rpsXPos[0]+5);
//       turnLeft(30.0, 90);
      goForward(MotorSpeed, 1, 3.0);
      check_heading(270);
  }
}

/*
* Method for going to the DDR light.
*/
void goToDDRLight(){
  goForward(-MotorSpeed-5, 3, 30.0);
  turnRight(30.,45);
  //check_heading(0);
  goForward(-MotorSpeed-5, 11, 5.0);
  check_x(MotorSpeed-10.,rpsXPos[0]+0.5);
  turnRight(30,92);
  check_heading(270);
  check_y(MotorSpeed-10., rpsYPos[0]);
}

/*
* Method for driving up the ramp
*/
void goUpRamp(){
  goForwardRamp(MotorSpeed+20, 18, 30.0);
  Sleep(300);
  //if(RPS.Heading()>270){
  //   turnLeft();
  //   check_heading();
  //   check_x()
  //   turnLeft();
  //}

  //else{
  //   turnRight();
  //   check_heading();
  //   check_x()
  //   turnRight();

  //}
  check_y(MotorSpeed-10, rpsYPos[1]-4.5);
  turnRight(30, 180);
}

/*
* Method for driving up the ramp
*/
void goToCoin(){
  //Cant check heading  after steps, too close to bad zone
  goForward(-MotorSpeed+5, 15, 30.0);
  turnRight(30., 90);
  goForward(MotorSpeed+5, 3.5, 30.0);
  turnLeft(30., 45);
  goForward(MotorSpeed+5, 9, 30.0);
  turnLeft(30., 50);
  check_heading(90);
  check_y(MotorSpeed-10, rpsYPos[1]+4);
  turnRight(30., 90);
  check_x(MotorSpeed-10, rpsXPos[1]-4.5);
  check_heading(0);
}

/*
* Method for driving to foosball
*/
void goToFoosball() {
  turnLeft(30., 50);
  //check_y(MotorSpeed-10,47);
  //turnRight(25.,45);
  goForward(-MotorSpeed-5, 8.2, 4.0);
  turnRight(30., 40);
  check_heading(0);
  goForward(-MotorSpeed-5, 4.5, 4.0);
  Sleep(250);
  check_x(MotorSpeed-10, rpsXPos[1]+8);
  turnLeft(30.0,90);
  check_heading(91.0);
  check_y(MotorSpeed-10, rpsYPos[1]+9.7);
  goForward(-MotorSpeed, 13, 5.0);
  turnLeft(25., 92);
  goForward(MotorSpeed, 5, 1.0);
  goForward(-MotorSpeed, 5, 1.0);
}

/*
* Method for driving to foosball
*/
void goToLeverFromFoos() {
  goForward(MotorSpeed+5, 2, 2.0);
  turnRight(30.0, 45);
  goForward(MotorSpeed+5, 7, 30.0);
  turnRight(30.0, 45);
  goForward(MotorSpeed+5, 5, 3.0);
  check_y(MotorSpeed-10, rpsYPos[1]+11);
  check_heading(90);
  turnRight(30.0, 75);
  goForward(MotorSpeed+5, 14, 3.0);
  turnRight(30.0, 15);
  check_heading(0);
  goForward(MotorSpeed+5, 4, 3.0);
  check_x(MotorSpeed-10, rpsXPos[1]-15.8);
  turnRight(30., 90);
  check_heading(270);
  check_y(MotorSpeed-10,rpsYPos[1]+9);
  goForward(MotorSpeed, 7.3, 5.0);
  turnRight(25., 98);
  goForward(-MotorSpeed, 3, 5.0);
}

/*
* Method for driving to the ramp from the top
*/
void goToRampTop(){
  goForward(MotorSpeed, 3.5, 5.0);
  turnLeft(30., 105);
  goForward(-MotorSpeed-10, 15, 5.0);
  turnRight(25.0, 90);
  check_x(MotorSpeed-10, rpsXPos[1]-18);
  turnLeft(25.0, 90);
  check_heading(273);
  check_y(MotorSpeed+5, rpsYPos[1]-2.4);
}

/*
* Method for driving down the ramp and too the button from anywhere on the first level
*/
void goToFinalButtonTheta(){
  goForward(-MotorSpeed-10, 25, 6.0);
  Sleep(333);
  //Calculate a theta to turn based on the x and y
  //(think of a line extending from the button (0,0) to the position of the robot)
  float theta = 180/3.14159265 * atan((RPS.Y()-6)/(RPS.X()-6));
//   if (RPS.Heading() < theta+180) {
//       turnLeft(25., theta);
//   } else {
//       turnRight(25., theta);
//   }
  if (RPS.X() >= 0 && RPS.Y() >= 0) {
      if(RPS.X()-6 < 0) {
           check_heading(theta + 360);
      } else {
           check_heading(theta + 180);
      }
  }
  goForward(-MotorSpeed-35., 40, 10.0);
}

/*
* Method for driving down the ramp and too the button
*/
void goToFinalButton(){
  goForward(-MotorSpeed-5, 20, 2.0);
  //  turnRight(25.0, 90);
  //  check_x(20.0, 6.5);
  //  turnLeft(25.0, 90);
  check_heading(275);
  goForward(-MotorSpeed-35, 60, 30.0);

}

/*
* Method for dropping the coin
*/
void dropCoin(){
  moveArm(85);
  Sleep(1500);
  moveArm(0);
}

/*
* Method for grabbing the foosball sliders and moving forward
*/
void moveFoos(){
  moveArm(78.1);
  Sleep(800);
  rightMotor.SetPercent(30.0);
  leftMotor.SetPercent(-35.0);
  Sleep(1.60);
  rightMotor.Stop();
  leftMotor.Stop();
  moveArm(0);
  Sleep(500);
  //Move back a little and push it the rest of the way

  //goForward(MotorSpeed, 5, 4.0);
  //moveArm(78);
  //Sleep(800);
  //rightMotor.SetPercent(30.0);
  //leftMotor.SetPercent(-32.0);
  //Sleep(0.5);
  //rightMotor.Stop();
  //leftMotor.Stop();
  //moveArm(0);
  //Sleep(500);

}

/*
* Method for flipping the lever
*/
void flipLever(){
  moveArm(80);
  Sleep(800);
  moveArm(0);
  Sleep(800);

  //  float n = .10;
  //  int m = -1;
  //  int k = 1;
  //  while (true) {
  //      m = m * -1;
  //      goForward(20.*m, n, 5.0);
  //      moveArm(65);
  //      Sleep(1500);
  //      moveArm(0);
  //      Sleep(1500);
  //      n = .25 * k;
  //      k = k+1;
  //  }
}

/*
* Method for loading the menu.
*/
void loadMenu() {
  //Clear the screen to white
  LCD.Clear(BLACK);
  startButton.SetProperties("Get Bread", 100, 180, 120, 40, GOLD, GRAY);
  courseButton.SetProperties("Run", 35, 10, 120, 40, WHITE, WHITE);
  calibrateRPS.SetProperties("RPS Spots", 165, 10, 120, 40, WHITE, WHITE);
  coordsButton.SetProperties("Coord/Light", 35, 60, 120, 40, WHITE, WHITE);
  forwardButton.SetProperties("Forward", 165, 60, 120, 40, WHITE, WHITE);
  turnLButton.SetProperties("Turn L 90", 35, 110, 120, 40, WHITE, WHITE);
  turnRButton.SetProperties("Turn R 90", 165, 110, 120, 40, WHITE, WHITE);
  backButton.SetProperties("Back", 250, 200, 60, 35, WHITE, WHITE);
  keepRPS.SetProperties("Keep", 35, 110, 60, 35, WHITE, WHITE);
  setRPS.SetProperties("Set", 165, 110, 60, 35, WHITE, WHITE);

  //Draw the opening menu
  LCD.SetFontColor(GOLD);
  LCD.FillRectangle(25,20,30,90);
  LCD.FillCircle(60,50,30);
  LCD.SetFontColor(BLACK);
  LCD.FillCircle(60,50,10);

  LCD.SetFontColor(GOLD);
  LCD.FillRectangle(100,20,30,80);
  LCD.FillRectangle(160,20,30,80);
  LCD.FillRectangle(130,20,30,20);
  LCD.FillRectangle(130,60,30,20);

  LCD.FillRectangle(200,20,50,30);
  LCD.FillRectangle(200,50,20,30);
  LCD.FillRectangle(200,80,50,30);

  LCD.FillRectangle(280,20,30,80);

  //Draw the button
  startButton.Draw();
}

/*
* Method for drawing the menu.
*/
void drawMenu() {
  LCD.Clear();
  courseButton.Draw();
  calibrateRPS.Draw();
  coordsButton.Draw();
  forwardButton.Draw();
  turnRButton.Draw();
  turnLButton.Draw();
}

/*
* Method for openeing the menu.
*/
void openMenu(){
  //Variables for touching the screen
  float xTouch, yTouch;

  //Boolean for the first screen
  bool init = true;

  //Calibrate
  calibrate();

  //While loop that waits at the menu screen
  while (init) {
      LCD.Touch(&xTouch, &yTouch);

      //If the button is pressed
      if(startButton.Pressed(xTouch, yTouch, 0)) {
          LCD.Clear();
          init = false;
      }

      //Sleep while waiting for touches
      Sleep(30);
  }

  //Draw Menu
  drawMenu();

  //While loop that actually has the menu on it
  while (true) {
      LCD.Touch(&xTouch, &yTouch);

      //If course button is pressed
      if(courseButton.Pressed(xTouch, yTouch, 0)) {
          //Check to see if PRS was initialized
          if (!RPSInitialized) {
              RPS.InitializeTouchMenu();
              RPSInitialized = true;
          }
          //Run the course
          LCD.Clear();

          waitForStart();
          goToDDRLight();
          pressDDRButton();
          goUpRamp();
          goToCoin();
          dropCoin();
          goToFoosball();
          moveFoos();
          goToLeverFromFoos();
          flipLever();
          goToRampTop();
          goToFinalButtonTheta();

          drawMenu();

          //If the calibrateRPS button was pressed
      } else if(calibrateRPS.Pressed(xTouch, yTouch, 0)) {
          //Check to see if PRS was initialized
          if (!RPSInitialized) {
              RPS.InitializeTouchMenu();
              RPSInitialized = true;
          }
          int i = 0;
          while(i < KeyRPSLocations) {
              LCD.Clear();
              keepRPS.Draw();
              setRPS.Draw();
              LCD.Touch(&xTouch, &yTouch);
              LCD.WriteAt(keyPos[i], 75, 200);
              LCD.Write("Default: ");
              LCD.Write(rpsXPos[i]);
              LCD.Write(",   ");
              LCD.WriteLine(rpsYPos[i]);
              LCD.Write("Current: ");
              LCD.Write(RPS.X());
              LCD.Write(",   ");
              LCD.WriteLine(RPS.Y());
              if(keepRPS.Pressed(xTouch, yTouch, 0)) {
                  i++;
              } else if(setRPS.Pressed(xTouch, yTouch, 0) && RPS.X() > 0 && RPS.Y() > 0) {
                  rpsXPos[i] = RPS.X();
                  rpsYPos[i] = RPS.Y();
                  i++;
              }
              Sleep(100);
          }

          drawMenu();

          //If the coordinate button was pressed
      } else if(coordsButton.Pressed(xTouch, yTouch, 0)) {
          //Check to see if PRS was initialized
          if (!RPSInitialized) {
              RPS.InitializeTouchMenu();
              RPSInitialized = true;
          }
          while(!backButton.Pressed(xTouch,yTouch,0)) {
              LCD.Clear();
              backButton.Draw();
              LCD.Touch(&xTouch, &yTouch);
              LCD.Write(RPS.X());
              LCD.Write(",   ");
              LCD.WriteLine(RPS.Y());
              LCD.Write("CDS: ");
              LCD.Write(Cds.Value());
              Sleep(100);
          }
          drawMenu();

          //If the forward button is pressed
      } else if(forwardButton.Pressed(xTouch, yTouch, 0)) {
          goForward(-30.0, 15, 10.0);

          //If the turn right button is pressed
      } else if(turnRButton.Pressed(xTouch, yTouch, 0)) {
          turnRight(20., 90);

          //If the turn left button is pressed
      } else if(turnLButton.Pressed(xTouch, yTouch, 0)) {
          turnLeft(20., 90);
      }

      //Sleep while waiting for touches
      Sleep(30);
  }
}

int main(void)
{
  //goForward(-MotorSpeed,10,30);
  //LCD.WriteLine(leftEncoder.Counts());
  // LCD.WriteLine(rightEncoder.Counts());
  loadMenu();
  openMenu();
}



