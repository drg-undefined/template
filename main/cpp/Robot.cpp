/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc2/Timer.h>



void Robot::RobotInit() {
    camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    camera.SetResolution(640, 480);
    outputStream = frc::CameraServer::GetInstance()->PutVideo("Processed", 640, 480);
    frc::SmartDashboard::PutNumber("PID_x kp:", 1.0);
    frc::SmartDashboard::PutNumber("PID_x ki:", 0.0);
    frc::SmartDashboard::PutNumber("PID_x kd:", 0.0);

    frc::SmartDashboard::PutNumber("PID_y kp:", 1.0);
    frc::SmartDashboard::PutNumber("PID_y ki:", 0.0);
    frc::SmartDashboard::PutNumber("PID_y kd:", 0.0);

    frc::SmartDashboard::PutNumber("PID_th kp:", 1.0);
    frc::SmartDashboard::PutNumber("PID_th ki:", 0.0);
    frc::SmartDashboard::PutNumber("PID_th kd:", 0.0);

    frc::SmartDashboard::PutNumber("SP_X:", 0.0);
    frc::SmartDashboard::PutNumber("SP_Y:", 0.0);
    frc::SmartDashboard::PutNumber("SP_TH:", 0.0);

    frc::SmartDashboard::PutNumber("taxa_X:", 0.0);
    frc::SmartDashboard::PutNumber("taxa_Y:", 0.0);
    frc::SmartDashboard::PutNumber("taxa_W:", 0.0);
    active = false;
    
}

void Robot::RobotPeriodic() {
    // cv::Mat frame;
    //     cvSink.GrabFrame(frame);
    //     if (frame.empty()) {
    //         frc::SmartDashboard::PutString("ERROR", "VAZIO");
    //         return;
    //     }
    //     else {
    //         frc::SmartDashboard::PutString("ERROR", "OK");
    //         outputStream.PutFrame(frame);
    //     }
    OMNI3.updateOdometry();
    // OMNI3.updateOdometryByDistance();
    frc::SmartDashboard::PutNumber("X:", m_controller.GetRawAxis(0));
    frc::SmartDashboard::PutNumber("Y:", m_controller.GetRawAxis(1));
    frc::SmartDashboard::PutNumber("TH:", m_controller.GetRawAxis(3));

    double ang = OMNI3.GetPose().th;
    frc::SmartDashboard::PutNumber("POSE_X:", OMNI3.GetPose().x);
    frc::SmartDashboard::PutNumber("POSE_Y:", OMNI3.GetPose().y);
    frc::SmartDashboard::PutNumber("POSE_W_deg:", ang*180.0/M_PI);

    frc::SmartDashboard::PutNumber("Vx: ", OMNI3.GetChassiSpeeds().vx);
    frc::SmartDashboard::PutNumber("Vy: ", OMNI3.GetChassiSpeeds().vy);
    frc::SmartDashboard::PutNumber("W_: ", OMNI3.GetChassiSpeeds().omega);

    driveWheelSpeeds EncoderSpeeds = OMNI3.getCurrentSpeed();
    frc::SmartDashboard::PutNumber("ENC_Roda0 RPM:", EncoderSpeeds.roda0/200.0);
    frc::SmartDashboard::PutNumber("ENC_Roda1 RPM:", EncoderSpeeds.roda1/200.0);
    frc::SmartDashboard::PutNumber("ENC_Roda2 RPM:", EncoderSpeeds.roda2/200.0);



    double kp_x = frc::SmartDashboard::GetNumber("PID_x kp:", 1.0);
    double ki_x = frc::SmartDashboard::GetNumber("PID_x ki:", 0.0);
    double kd_x = frc::SmartDashboard::GetNumber("PID_x kd:", 0.0);
    PID_x.SetPID(kp_x, ki_x, kd_x);

    double kp_y = frc::SmartDashboard::GetNumber("PID_y kp:", 1.0);
    double ki_y = frc::SmartDashboard::GetNumber("PID_y ki:", 0.0);
    double kd_y = frc::SmartDashboard::GetNumber("PID_y kd:", 0.0);
    PID_y.SetPID(kp_y, ki_y, kd_y);

    double kp_th = frc::SmartDashboard::GetNumber("PID_th kp:", 1.0);
    double ki_th = frc::SmartDashboard::GetNumber("PID_th ki:", 0.0);
    double kd_th = frc::SmartDashboard::GetNumber("PID_th kd:", 0.0);
    PID_th.SetPID(kp_th, ki_th, kd_th);


    // PID_x.SetPID(1.15, 10000.0, 1.0); //valores possível

    // PID_th.SetPID(2.50, 0.045, 0.000001); //valores bons
    // PID_th.SetPID(2.50, 0.25, 0.00001); //valores bons
    
    if (!startButton.Get() && !active)
    {
        ds.Enable(); // enable the robot
        active = true;
    }
    // If the e-stop is pushed and the system is active
    if (!stopButton.Get() && active)
    {
        active = false;
        ds.Disable(); // disable the robot
    }

    
}

void Robot::TeleopInit() {
    OMNI3.resetPosition(pose2d(), OMNI3.GetAngle());

    // driveWithPID(array pontos);

    // driveWithPID(array pontos) {
    //     i = pontos
    //     drive(PIDx.Calculate(PoseAtualRobo.X, ponto.X), 
    //           PIDy.Calculate(PoseAtualRobo.Y, ponto.Y),
    //           PIDth.Calculate(PoseAtualRobo.th, ponto.th));
    //     if(PIDx.AtSetPoint() && PIDy.AtSetPoint() && PIDth.AtSetPoint()){
    //         remove ponto
    //     }
    //     até não conter mais pontos
    // }

}

double NormalizedDeg_(double angle_Rad) { //resultado entre 0 e 360 graus;
    double normalizedAngle = fmod(angle_Rad*M_PI/180.0, 360.0);
    if (normalizedAngle < 0.0) {
        normalizedAngle += 360.0;
    }
    return normalizedAngle;
}
double NormalizedRad_(double angle_Rad) { //resultado entre 0 e 2*pi graus;
    double normalizedAngle = fmod(angle_Rad, 2.0*M_PI);
    if (normalizedAngle < 0.0) {
        normalizedAngle += 2.0*M_PI;
    }
    return normalizedAngle;
}

void Robot::TeleopPeriodic() {
    auto [SP_vx, SP_vy, SP_W] = DriveWithJoystick(false);
    auto [PV_vx, PV_vy, PV_W] = OMNI3.GetChassiSpeeds();

    // OMNI3.drive(PID_x.Calculate(PV_vx, SP_vx),
    //             PID_y.Calculate(PV_vy, SP_vy),
    //             PID_th.Calculate(NormalizedDeg_(PV_W), NormalizedDeg_(SP_W)), true);

    // OMNI3.drive(SP_vx, SP_vy, SP_W, true);
    OMNI3.drive(SP_vy/2.0, -SP_vx/2.0, SP_W/2.0, false);
}

void Robot::AutonomousInit() {
    // pose2d zeroPos = pose2d(0.25, 1.0, 45.0);
    // OMNI3.resetPosition(0.0, 0.0, 0.0, OMNI3.GetAngle());
    OMNI3.resetYaw();
    OMNI3.resetPosition(pose2d(0.0, 0.0, 0.0), OMNI3.GetAngle()); //45.0*M_PI/180.0
    // frc::SmartDashboard::PutNumber("POSE_X:", zeroPos.x);
    // frc::SmartDashboard::PutNumber("POSE_Y:", zeroPos.y);
    // frc::SmartDashboard::PutNumber("POSE_W_deg:", zeroPos.degrees());
    // frc::SmartDashboard::PutNumber("POSE_W_rad:", zeroPos.radians());
    PID_x.SetTolerance(1.0);
    PID_y.SetTolerance(1.0);
    PID_th.SetTolerance(1.0);
}


void Robot::AutonomousPeriodic() {
    // auto [SP_vx, SP_vy, SP_W] = DriveWithJoystick(true);


    auto [PV_x, PV_y, PV_th] = OMNI3.GetPose();
    double SP_x = 2000.0;
    double SP_y = 0.0;
    double SP_th = 0.0; //45.0*M_PI/180.0;

    SP_x = frc::SmartDashboard::GetNumber("SP_X:", 0.0);
    SP_y = frc::SmartDashboard::GetNumber("SP_Y:", 0.0);
    SP_th = frc::SmartDashboard::GetNumber("SP_TH:", 0.0);


    // std::vector<pose2d> trajetoria {
    //     pose2d(0.0, 0.0, 0.0),
    //     pose2d(1000.0, 0.0, 0.0),
    //     pose2d(1000.0, 250.0, 45.0),
    //     pose2d(1500.0, -150.0, 180.0),
    //     pose2d(500.0, -0.0, 180.0),
    //     pose2d(500.0, -0.0, 0.0),
    //     pose2d(0.0, -0.0, 0.0)
    // };

    // std::vector<pose2d> trajetoria = {
    //     {0.0,       0.0,        0.0},
    //     {2000.0,    0.0,        0.0},
    //     {1000.0,    250.0,      45.0},
    //     {1500.0,    -150.0,     180.0},
    //     {1500.0,    1050.0,     180.0},
    //     {500.0,     -0.0,       90.0},
    //     {500.0,     -0.0,       0.0},
    //     {2500.0,     -0.0,       0.0},
    //     {0.0,       -0.0,       0.0}
    // };
    std::vector<pose2d> trajetoria = {
        {0.0,       0.0,        0.0},
        {300.0,    0.0,        0.0},
        {300.0,    1100.0,        90.0},
        {200.0,    1100.0,        90.0},
        {200.0,    700.0,        0.0},
        {3200.0,    700.0,        0.0},
        {3200.0,    0.0,        -90.0},
        {3200.0,    700.0,        -90.0},
        {3200.0,    700.0,        -180.0},
        {300.0,    700.0,        -180.0},
        {300.0,    0.0,        -180.0},
        {0.0,    0.0,        -180.0},
        {0.0,       -0.0,       0.0}
        // {0.0,       -0.0,       90.0},
        // {0.0,       -0.0,       180.0},
        // {0.0,       -0.0,       270.0},
        // {0.0,       -0.0,       360.0}
    };

    auto [poseSP_x, poseSP_y, poseSP_th] = trajetoria[m_ponteiro];
    pose2d SP_traj = trajetoria[m_ponteiro];

    SP_x = SP_traj.x;
    SP_y = SP_traj.y;
    SP_th = SP_traj.deg2rad();

    SP_x = poseSP_x;
    SP_y = poseSP_y;
    SP_th = poseSP_th*M_PI/180.0;
    // SP_th = NormalizedDeg_(SP_th);

    PID_th.EnableContinuousInput(0, 2*M_PI);

    // OMNI3.drive(PID_x.Calculate(PV_x, SP_x),
    //             PID_y.Calculate(PV_y, SP_y),
    //             PID_th.Calculate(NormalizedDeg_(PV_th), NormalizedDeg_(SP_th))*-1.0, true);

    

    double taxaX = frc::SmartDashboard::GetNumber("taxa_X:", 0.0);
    double taxaY = frc::SmartDashboard::GetNumber("taxa_Y:", 0.0);
    double taxaW = frc::SmartDashboard::GetNumber("taxa_W:", 0.0);

    controllerVx.setTolerance(taxaX);
    controllerVy.setTolerance(taxaY);
    controllerW.setTolerance(taxaW);

    // controllerVx.setTolerance(10.0); //
    // controllerVy.setTolerance(10.0); //
    // controllerW.setTolerance(0.05); //
    //desde que robô não colida em paredes, pois nesse caso PV - e consequentemente o erro - é severamente comprometido

    controllerVx.setTolerance(15.0); //
    controllerVy.setTolerance(15.0); //
    controllerW.setTolerance(0.05); //


    double CO_x = PID_x.Calculate(PV_x, SP_x);
    double CO_y = PID_y.Calculate(PV_y, SP_y);
    double CO_th = PID_th.Calculate(PV_th, SP_th);

    frc::SmartDashboard::PutNumber("PID CO_x:", CO_x);
    frc::SmartDashboard::PutNumber("PID CO_y:", CO_y);
    frc::SmartDashboard::PutNumber("PID CO_th:", CO_th);

    // frc::SmartDashboard::PutNumber("CLAMP CO_x:", std::clamp(CO_x, -1.0, 1.0));
    // frc::SmartDashboard::PutNumber("CLAMP CO_y:", std::clamp(CO_y, -1.0, 1.0));
    // frc::SmartDashboard::PutNumber("CLAMP CO_th:", std::clamp(CO_th, -1.0, 1.0));

    // CO_x = std::abs(CO_x) < taxaX? (CO_x + CO_x*0.60) : CO_x;
    // CO_y = std::abs(CO_y) < taxaY? (CO_y + CO_y*0.60) : CO_y;
    // CO_th = std::abs(CO_th) < taxaW? (CO_th + CO_th*0.60) : CO_th;
    

    // controllerVx.setRampInc(taxaX);
    // controllerVy.setRampInc(taxaY);
    // controllerW.setRampInc(taxaW*M_PI/180.0);

    // CO_x = controllerVx.getMotorSpeed(SP_x - PV_x);
    // CO_y = controllerVy.getMotorSpeed(SP_y - PV_y);
    // CO_th = controllerW.getMotorSpeed(SP_th - PV_th);

    CO_x = controllerVx.getMotorSpeed(PV_x, SP_x);
    CO_y = controllerVy.getMotorSpeed(PV_y, SP_y);
    CO_th = controllerW.getMotorSpeed(PV_th, SP_th);

    frc::SmartDashboard::PutNumber("Erro_x:", SP_x - PV_x);
    frc::SmartDashboard::PutNumber("Erro_y:", SP_y - PV_y);
    frc::SmartDashboard::PutNumber("Erro_th:", SP_th - PV_th);
    frc::SmartDashboard::PutNumber("Erro_th_DEG:", (SP_th - PV_th)*180.0/M_PI);

    frc::SmartDashboard::PutNumber("CO_x:", CO_x);
    frc::SmartDashboard::PutNumber("CO_y:", CO_y);
    frc::SmartDashboard::PutNumber("CO_th:", CO_th);

    // OMNI3.drive(CO_x,CO_y,CO_th, true);

    bool AtSetPoint = controllerVx.atSP() && controllerVy.atSP() && controllerW.atSP();
    if (AtSetPoint && (m_ponteiro < trajetoria.size()) ){
        m_ponteiro += 1;
    }
    frc::SmartDashboard::PutNumber("Ponteiro:", m_ponteiro);

    OMNI3.drive(CO_x,CO_y,CO_th, true);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
    OMNI3.resetYaw();
    OMNI3.reserEncoders();
    controllerVx.reset();
    controllerVy.reset();
    controllerW.reset();
    m_ponteiro = 0.0;
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif





// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #include "Robot.h"
// #include <frc2/Timer.h>


// void Robot::RobotInit() {
//     frc::SmartDashboard::PutNumber("PID_x kp:", 1.0);
//     frc::SmartDashboard::PutNumber("PID_x ki:", 0.0);
//     frc::SmartDashboard::PutNumber("PID_x kd:", 0.0);

//     frc::SmartDashboard::PutNumber("PID_y kp:", 1.0);
//     frc::SmartDashboard::PutNumber("PID_y ki:", 0.0);
//     frc::SmartDashboard::PutNumber("PID_y kd:", 0.0);

//     frc::SmartDashboard::PutNumber("PID_th kp:", 1.0);
//     frc::SmartDashboard::PutNumber("PID_th ki:", 0.0);
//     frc::SmartDashboard::PutNumber("PID_th kd:", 0.0);

//     frc::SmartDashboard::PutNumber("SP_X:", 0.0);
//     frc::SmartDashboard::PutNumber("SP_Y:", 0.0);
//     frc::SmartDashboard::PutNumber("SP_TH:", 0.0);
    
// }

// void Robot::RobotPeriodic() {
//     OMNI3.updateOdometry();
//     // OMNI3.updateOdometryByDistance();
//     frc::SmartDashboard::PutNumber("X:", m_controller.GetRawAxis(0));
//     frc::SmartDashboard::PutNumber("Y:", m_controller.GetRawAxis(1));
//     frc::SmartDashboard::PutNumber("TH:", m_controller.GetRawAxis(3));

//     double ang = OMNI3.GetPose().th;
//     frc::SmartDashboard::PutNumber("POSE_X:", OMNI3.GetPose().x);
//     frc::SmartDashboard::PutNumber("POSE_Y:", OMNI3.GetPose().y);
//     frc::SmartDashboard::PutNumber("POSE_W_deg:", ang*180.0/M_PI);

//     frc::SmartDashboard::PutNumber("Vx: ", OMNI3.GetChassiSpeeds().vx);
//     frc::SmartDashboard::PutNumber("Vy: ", OMNI3.GetChassiSpeeds().vy);
//     frc::SmartDashboard::PutNumber("W_: ", OMNI3.GetChassiSpeeds().omega);

//     driveWheelSpeeds EncoderSpeeds = OMNI3.getCurrentSpeed();
//     frc::SmartDashboard::PutNumber("ENC_Roda0 RPM:", EncoderSpeeds.roda0/200.0);
//     frc::SmartDashboard::PutNumber("ENC_Roda1 RPM:", EncoderSpeeds.roda1/200.0);
//     frc::SmartDashboard::PutNumber("ENC_Roda2 RPM:", EncoderSpeeds.roda2/200.0);



//     double kp_x = frc::SmartDashboard::GetNumber("PID_x kp:", 1.0);
//     double ki_x = frc::SmartDashboard::GetNumber("PID_x ki:", 0.0);
//     double kd_x = frc::SmartDashboard::GetNumber("PID_x kd:", 0.0);
//     PID_x.SetPID(kp_x, ki_x, kd_x);

//     double kp_y = frc::SmartDashboard::GetNumber("PID_y kp:", 1.0);
//     double ki_y = frc::SmartDashboard::GetNumber("PID_y ki:", 0.0);
//     double kd_y = frc::SmartDashboard::GetNumber("PID_y kd:", 0.0);
//     PID_y.SetPID(kp_y, ki_y, kd_y);

//     double kp_th = frc::SmartDashboard::GetNumber("PID_th kp:", 1.0);
//     double ki_th = frc::SmartDashboard::GetNumber("PID_th ki:", 0.0);
//     double kd_th = frc::SmartDashboard::GetNumber("PID_th kd:", 0.0);
//     PID_th.SetPID(kp_th, ki_th, kd_th);


//     // PID_x.SetPID(1.15, 10000.0, 1.0); //valores possível

//     // PID_th.SetPID(2.50, 0.045, 0.000001); //valores bons
//     // PID_th.SetPID(2.50, 0.25, 0.00001); //valores bons
    
// }

// void Robot::AutonomousInit() {
//     OMNI3.resetPosition(pose2d(), OMNI3.GetAngle());

//     // driveWithPID(array pontos);

//     // driveWithPID(array pontos) {
//     //     i = pontos
//     //     drive(PIDx.Calculate(PoseAtualRobo.X, ponto.X), 
//     //           PIDy.Calculate(PoseAtualRobo.Y, ponto.Y),
//     //           PIDth.Calculate(PoseAtualRobo.th, ponto.th));
//     //     if(PIDx.AtSetPoint() && PIDy.AtSetPoint() && PIDth.AtSetPoint()){
//     //         remove ponto
//     //     }
//     //     até não conter mais pontos
//     // }

// }

// double NormalizedDeg_(double angle_Rad) { //resultado entre 0 e 360 graus;
//     double normalizedAngle = fmod(angle_Rad*M_PI/180.0, 360.0);
//     if (normalizedAngle < 0.0) {
//         normalizedAngle += 360.0;
//     }
//     return normalizedAngle;
// }
// double NormalizedRad_(double angle_Rad) { //resultado entre 0 e 2*pi graus;
//     double normalizedAngle = fmod(angle_Rad, 2.0*M_PI);
//     if (normalizedAngle < 0.0) {
//         normalizedAngle += 2.0*M_PI;
//     }
//     return normalizedAngle;
// }

// void Robot::AutonomousPeriodic() {
//     auto [SP_vx, SP_vy, SP_W] = DriveWithJoystick(true);
//     auto [PV_vx, PV_vy, PV_W] = OMNI3.GetChassiSpeeds();

//     // OMNI3.drive(PID_x.Calculate(PV_vx, SP_vx),
//     //             PID_y.Calculate(PV_vy, SP_vy),
//     //             PID_th.Calculate(NormalizedDeg_(PV_W), NormalizedDeg_(SP_W)), true);
//     OMNI3.drive(SP_vx, SP_vy, SP_W, true);
// }

// void Robot::TeleopInit() {
//     // pose2d zeroPos = pose2d(0.25, 1.0, 45.0);
//     // OMNI3.resetPosition(0.0, 0.0, 0.0, OMNI3.GetAngle());
//     OMNI3.resetPosition(pose2d(0.0, 0.0, 0.0), OMNI3.GetAngle()); //45.0*M_PI/180.0
//     // frc::SmartDashboard::PutNumber("POSE_X:", zeroPos.x);
//     // frc::SmartDashboard::PutNumber("POSE_Y:", zeroPos.y);
//     // frc::SmartDashboard::PutNumber("POSE_W_deg:", zeroPos.degrees());
//     // frc::SmartDashboard::PutNumber("POSE_W_rad:", zeroPos.radians());
//     PID_x.SetTolerance(1.0);
//     PID_y.SetTolerance(1.0);
//     PID_th.SetTolerance(1.0);
// }



// void Robot::TeleopPeriodic() {
//     // auto [SP_vx, SP_vy, SP_W] = DriveWithJoystick(true);


//     auto [PV_x, PV_y, PV_th] = OMNI3.GetPose();
//     double SP_x = 2000.0;
//     double SP_y = 0.0;
//     double SP_th = 0.0; //45.0*M_PI/180.0;

//     SP_x = frc::SmartDashboard::GetNumber("SP_X:", 0.0);
//     SP_y = frc::SmartDashboard::GetNumber("SP_Y:", 0.0);
//     SP_th = frc::SmartDashboard::GetNumber("SP_TH:", 0.0);

//     SP_th = SP_th*M_PI/180.0;

//     PID_th.EnableContinuousInput(0, 2*M_PI);

//     // OMNI3.drive(PID_x.Calculate(PV_x, SP_x),
//     //             PID_y.Calculate(PV_y, SP_y),
//     //             PID_th.Calculate(NormalizedDeg_(PV_th), NormalizedDeg_(SP_th))*-1.0, true);

    

//     double CO_x = PID_x.Calculate(PV_x, SP_x);
//     double CO_y = PID_y.Calculate(PV_y, SP_y);
//     double CO_th = PID_th.Calculate(PV_th, SP_th);

//     frc::SmartDashboard::PutNumber("CO_x:", CO_x);
//     frc::SmartDashboard::PutNumber("CO_y:", CO_y);
//     frc::SmartDashboard::PutNumber("CO_th:", CO_th);

//     OMNI3.drive(CO_x,CO_y,CO_th, false);
// }

// void Robot::DisabledInit() {}

// void Robot::DisabledPeriodic() {
//     OMNI3.resetYaw();
//     OMNI3.reserEncoders();
// }

// void Robot::TestInit() {}

// void Robot::TestPeriodic() {}

// #ifndef RUNNING_FRC_TESTS
// int main() { return frc::StartRobot<Robot>(); }
// #endif

