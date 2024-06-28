#pragma once

#include <string>
#include "drivetrain.h"

// #include <frc/drive/Vector2d.h>
// #include <frc/drive/MecanumDrive.h>

// #include <frc/kinematics/ChassisSpeeds.h>

// #include <frc/kinematics/MecanumDriveKinematics.h>
// #include <frc/kinematics/MecanumDriveOdometry.h>
// #include <frc/kinematics/MecanumDriveWheelSpeeds.h>

// #include <frc/kinematics/DifferentialDriveKinematics.h>
// #include <frc/kinematics/DifferentialDriveOdometry.h>
// #include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

// #include <frc/drive/DifferentialDrive.h>

#include <frc/Joystick.h>

#include <frc/TimedRobot.h>
// #include <frc/smartdashboard/SendableChooser.h>
#include <frc/SlewRateLimiter.h>

#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ramp.h>


#include <opencv2/opencv.hpp>
#include <cameraserver/CameraServer.h>

#include <cscore_oo.h>
#include <wpi/raw_ostream.h>
#include "frc/DigitalInput.h"
#include "studica/MockDS.h"



class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

    

 private:
    studica::MockDS ds; // Create the object and the instance
    bool active = false; // Active flag prevents calling mockds more than once
    frc::DigitalInput startButton{9};
    frc::DigitalInput stopButton{8};
    cs::UsbCamera camera;
    cs::CvSource outputStream;
    cs::CvSink cvSink;

    int m_ponteiro = 0.0;
    drivetrain OMNI3;
    frc::Joystick m_controller{0};

    frc2::PIDController PID_x {0.8, 0.0, 0.0}; //{Kp, Ki, Kd};
    frc2::PIDController PID_y {0.8, 0.0, 0.0}; //{Kp, Ki, Kd};
    frc2::PIDController PID_th {1.0, 0.0, 0.0}; //{Kp, Ki, Kd};

    ramp Rampa_X;
    ramp Rampa_Y;
    ramp Rampa_W;

    // std::vector<ControlPoint> controlMap = {
    //     {-600.0, -600.0},
    //     {-100.0, -200.0},
    //     {-50.0, -80.0},
    //     {-10.0, -60.0},
    //     {0.0, 0.0},
    //     {10.0, 60.0},
    //     {50.0, 80.0},
    //     {100.0, 200.0},
    //     {600.0, 600.0}
    // }; //vetor de interpolação Erro x Velocidade para X e Y
    std::vector<ControlPoint> controlMap = {
        {-600.0, -350.0},
        {-100.0, -200.0},
        {-50.0, -80.0},
        {-10.0, -60.0},
        {0.0, 0.0},
        {10.0, 60.0},
        {50.0, 80.0},
        {100.0, 200.0},
        {600.0, 350.0}
    }; //vetor de interpolação Erro x Velocidade para X e Y

    MotorController controllerVx {controlMap, 10.0};
    MotorController controllerVy {controlMap, 10.0};

    std::vector<ControlPoint> controlMap_Omega = {
        {-180.0*M_PI/180.0, -2.50},
        // {-100.0*M_PI/180.0, -200.0},
        {-10.0*M_PI/180.0,  -0.5},
        {-2.0*M_PI/180.0,  -0.175},
        {0.0,               0.0},
        {2.0*M_PI/180.0,    0.175},
        {10.0*M_PI/180.0,   0.5},
        // {100.0*M_PI/180.0,  200.0},
        {180.0*M_PI/180.0,  2.50} //máximo
    }; //vetor de interpolação Erro (em rad/s) x Velocidade para W (rad/s)

    MotorController controllerW {controlMap_Omega, 5.0*M_PI/180.0};


    frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{0.5 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{0.5 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{0.5 / 1_s};

    BR::ChassisSpeeds DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = m_controller.GetRawAxis(0) * 600.0; 

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = m_controller.GetRawAxis(1) * -510.0; 

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = m_controller.GetRawAxis(3) * -154*M_PI/180.0; //max 2.70 rad/s

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed);
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed);
    frc::SmartDashboard::PutNumber("rot", rot);
    frc::SmartDashboard::PutNumber("rot_deg", rot*180.0/M_PI);

    // OMNI3.drive(xSpeed, ySpeed, rot, fieldRelative);
        return {xSpeed, ySpeed, rot};
    }
};
