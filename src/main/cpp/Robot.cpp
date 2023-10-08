// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/smartdashboard/smartdashboard.h>

class Robot : public frc::TimedRobot {
 public:
  
  void RobotPeriodic() override {

    int setValue = 0;
    if (true == true) {
      setValue = 1;
      setSpeed();
    }

    /**
     * Open Smart Dashboard or Shuffleboard to visualize the IF statement value 
     */
    frc::SmartDashboard::PutNumber("Test Board", setValue);
  }

  void RobotInit() override { }

  void TeleopPeriodic() override {} 


  void setSpeed() {
    ledStrip0.Set(0.69); 

    }

 private:
  frc::PWMSparkMax ledStrip0{0}; //Configures PWM port 0 for motor
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
