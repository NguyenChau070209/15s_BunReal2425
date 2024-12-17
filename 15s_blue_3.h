#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
motor leftMotorMotorA = motor(PORT10, ratio18_1, false);
motor leftMotorMotorB = motor(PORT8, ratio18_1, false);
motor_group leftMotor = motor_group(leftMotorMotorA, leftMotorMotorB);

motor rightMotorMotorA = motor(PORT9, ratio18_1, true);
motor rightMotorMotorB = motor(PORT7, ratio18_1, true);
motor_group rightMotor = motor_group(rightMotorMotorA, rightMotorMotorB);

motor ArmMotor = motor(PORT20, ratio18_1, false);

motor RollMotorMotorA = motor(PORT1, ratio6_1, false);
motor RollMotorMotorB = motor(PORT2, ratio6_1, false);
motor_group RollMotor = motor_group(RollMotorMotorA, RollMotorMotorB);

inertial Inertial11 = inertial(PORT11);



// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration


// Hàm điều khiển robot đi thẳng với khoảng cách, góc và tốc độ
void drivestraight(double Distance, double heading, double speed, double kp) {
  leftMotor.setPosition(0.0, degrees);
  rightMotor.setPosition(0.0, degrees);
  
  if (speed > 0.0) {
    // Đi thẳng
    while ((leftMotor.position(degrees) < Distance || rightMotor.position(degrees) < Distance)) {
      float error = (Inertial11.rotation() - heading) * kp;
      leftMotor.setVelocity((error - speed), percent);
      rightMotor.setVelocity((speed + error), percent);
      leftMotor.spin(fwd);
      rightMotor.spin(fwd);
      vex::task::sleep(20);  // Thay thế wait(20, msec)
    }
  }
  else {
    // Đi lùi
    while ((leftMotor.position(degrees) > 1.0 - Distance || rightMotor.position(degrees) > 1.0 - Distance)) {
      float error = (heading - Inertial11.rotation()) * kp;
      leftMotor.setVelocity((speed - error), percent);
      rightMotor.setVelocity((speed + error), percent);
      leftMotor.spin(fwd);
      rightMotor.spin(fwd);
      vex::task::sleep(20);  // Thay thế wait(20, msec)
    }
  }
  leftMotor.stop();
  rightMotor.stop();
}

// Hàm điều khiển robot quay đến góc mục tiêu sử dụng PID
void turn(double targetHeading) {
  leftMotor.spin(fwd);
  rightMotor.spin(fwd);

  // PID Constants
  float Kp = 1.0;  // Hệ số tỉ lệ
  float Ki = 0.08;  // Hệ số tích phân
  float Kd = 0.1;  // Hệ số đạo hàm

  // Các biến của PID
  float previousError = 0;
  float integral = 0;
  float derivative = 0;
  float error = targetHeading - Inertial11.rotation();

  while (fabs(error) > 2) { // Khi độ sai lệch giữa góc mục tiêu và góc hiện tại nhỏ hơn 2 độ, dừng lại
    error = targetHeading - Inertial11.rotation();
    integral += error * 0.02; // Lấy sai số tích lũy theo thời gian
    derivative = (error - previousError) / 0.02; // Tính đạo hàm theo thời gian
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Điều khiển động cơ bằng output từ PID
    leftMotor.setVelocity(output, percent);
    rightMotor.setVelocity(-output, percent);

    // Lưu giá trị error cũ để tính đạo hàm ở lần lặp sau
    previousError = error;

    vex::task::sleep(20);  // Thay thế wait(20, msec)
  }

  leftMotor.stop();
  rightMotor.stop();
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertial11.calibrate();
  
  RollMotor.setVelocity(100, percent);
  int rpm = 400;  // RPM mong muốn
  RollMotor.setVelocity(rpm, velocityUnits::rpm);

  leftMotor.setStopping(brake);
  rightMotor.setStopping(brake);

  //ArmMotor.setPosition(0, degrees);
  ArmMotor.setVelocity(100, percent);
  ArmMotor.setMaxTorque(100, percent);
  ArmMotor.setStopping(coast);

  //ArmMotor.spinFor(reverse, 112, degrees); // vị start để lấy vòng

  // Khởi động cảm biến Inertial
  while (Inertial11.isCalibrating()) {
    vex::task::sleep(100);
  
  Brain.Screen.print("Inertial Sensor Ready!"); }

  /* Code mẫu
        turn(90.0); // Quay đến góc 90 độ phải
      
        RollMotor.spin(reverse); //Cuộn

      // Đi thẳng
        leftMotor.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct); 
        rightMotor.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct); 
        vex::task::sleep(2000); 
        leftMotor.stop();
        rightMotor.stop();
      //Time
        wait(0.2, seconds);


  */ 

  // Code chạy 

    // Kẹp trụ

        leftMotor.spin(vex::directionType::rev, 100, vex::velocityUnits::pct); 
        rightMotor.spin(vex::directionType::rev, 100, vex::velocityUnits::pct); 
        vex::task::sleep(1000); 
        leftMotor.stop();
        rightMotor.stop();

        wait(0.2, seconds);

         // Kẹp trụ
    DigitalOutA.set(true);

    wait(0.2, seconds);

    //Chạy cuộn
    RollMotor.spin(reverse);

    turn(210.0);

    //Vòng1

    leftMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); // 
    rightMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); // 
    vex::task::sleep(550); 
    leftMotor.stop();
    rightMotor.stop();

    wait(0.2, seconds);

    leftMotor.spin(vex::directionType::rev, 70, vex::velocityUnits::pct); // 
    rightMotor.spin(vex::directionType::rev, 70, vex::velocityUnits::pct); // 
    vex::task::sleep(350); 
    leftMotor.stop();
    rightMotor.stop();

    wait(0.2, seconds);

    //Vòng 2

    turn(245.0);

    leftMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); // 
    rightMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); // 
    vex::task::sleep(450); 
    leftMotor.stop();
    rightMotor.stop();

    wait(0.2, seconds);

    leftMotor.spin(vex::directionType::rev, 70, vex::velocityUnits::pct); // 
    rightMotor.spin(vex::directionType::rev, 70, vex::velocityUnits::pct); // 
    vex::task::sleep(550); 
    leftMotor.stop();
    rightMotor.stop();

    wait(0.2, seconds);

    //Vòng 3

    turn(280.0);

    leftMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); // 
    rightMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); // 
    vex::task::sleep(450); 
    leftMotor.stop();
    rightMotor.stop();


    wait(0.2, seconds);

    turn(100.0);

    leftMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); // 
    rightMotor.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); // 
    vex::task::sleep(890); 
    leftMotor.stop();
    rightMotor.stop();



  }

        

