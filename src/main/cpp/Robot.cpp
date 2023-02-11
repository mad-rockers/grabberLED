// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/smartdashboard/smartdashboard.h>
#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"


/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and motor controller inputs as
 * range from -1 to 1 making it easy to work together.
 *
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent to the Dashboard.
 */
class Robot : public frc::TimedRobot {
 public:

  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  rev::ColorSensorV3 m_colorSensor{i2cPort};

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  rev::ColorMatch m_colorMatcher;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
  static constexpr frc::Color kPurpleTarget = frc::Color(0.204, 0.292, 0.518);

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */

  //void Robot::LEDmonitor() override{}

  void RobotPeriodic() override {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    frc::Color detectedColor = m_colorSensor.GetColor();
    
       /**
     * Run the color match algorithm on our detected color
     */
    std::string colorString;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget) {
      colorString = "Blue";
      makeRainbow();
    } else if (matchedColor == kRedTarget) {
      colorString = "Red";
      makeRainbow();
    } else if (matchedColor == kGreenTarget) {
      colorString = "Green";
      makeRainbow();
    } else if (matchedColor == kYellowTarget) {
      colorString = "Cone";
      makeYellow();
    } else if (matchedColor == kPurpleTarget) {
      colorString = "Cube";
      makePurple();
    } else {
      colorString = "Unknown";
      makeRainbow();
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", colorString);
  }

  void RobotInit() override {
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
    m_colorMatcher.AddColorMatch(kPurpleTarget);
  }

  void TeleopPeriodic() override {} //statically sets the color of the led strip to when Teleo is enabled


  /*
  * Blinkin color table
  * Blue Violet = 0.89
  * Yellow = 0.69
  * Rainbow = -0.99
  * https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  */
  void makeYellow() {ledStrip.Set(0.69); }
  void makePurple() {ledStrip.Set(0.89); }
  void makeRainbow() {ledStrip.Set(-0.99); }


 private:
  frc::Joystick m_stick{0};
  frc::PWMSparkMax ledStrip{9}; //Configures PWM port 9 for the Blinkin device
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
