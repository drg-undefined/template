

#include <frc/drive/Vector2d.h>

#include "drivetrain.h"


#define DEBUG true

drivetrain::drivetrain(){ //inicialização
    M0.SetInverted(true);      
    M1.SetInverted(true);
    M2.SetInverted(true);
    //M3.SetInverted(false);    //equações construídas observando sentido real dos motores
    // M0.InvertRPM();
    // M1.InvertRPM();
    // M2.InvertRPM();
    // M1.InvertRPM();
    // ENC1.SetReverseDirection();
    ENC0.InvertSpeed();
    
    // setSpeeds(driveWheelSpeeds{0_mps, 0_mps, 0_mps, 0_mps});
    drive(); //parar robô; Equivale a drive(0.0, 0.0, 0.0);
    reserEncoders();
    resetYaw();
    // pose::pose poseRobo{0, 0, 0};
}

// void drivetrain::drive(double vx, double vy, double w_rad, bool fieldRel){


void drivetrain::setSpeeds(const driveWheelSpeeds& wheelSpeeds){
    // std::clamp(wheelSpeeds.roda0.to<double>(), -1.0, 1.0);
    double minValSetMotor = 0.05;
    double roda0Adj = std::abs(wheelSpeeds.roda0) > minValSetMotor? wheelSpeeds.roda0 : 0.0;
    double roda1Adj = std::abs(wheelSpeeds.roda1) > minValSetMotor? wheelSpeeds.roda1 : 0.0;
    double roda2Adj = std::abs(wheelSpeeds.roda2) > minValSetMotor? wheelSpeeds.roda2 : 0.0;
    double roda3Adj = std::abs(wheelSpeeds.roda3) > minValSetMotor? wheelSpeeds.roda3 : 0.0;

    // double roda0Adj = wheelSpeeds.roda0;
    // double roda1Adj = wheelSpeeds.roda1;
    // double roda2Adj = wheelSpeeds.roda2;
    // double roda3Adj = wheelSpeeds.roda3;

    // if (std::abs(output) < threshold) {
    //     output = 0;
    // } else if (output > 0) {
    //     output = std::max(output, threshold);
    // } else {
    //     output = std::min(output, -threshold);
    // }


    M1.Set(roda0Adj*1440.0/1464.0);
    M2.Set(roda1Adj*1440.0/1464.0);
    M0.Set(roda2Adj*1464.0/1440.0);
    // M3.Set(roda3Adj);
    // M3.Set(wheelSpeeds.roda3);
}


driveWheelSpeeds drivetrain::getCurrentSpeed() {
    driveWheelSpeeds wheelsCurrentSpeed;
    wheelsCurrentSpeed.roda0 = ENC1.GetSpeed();
    wheelsCurrentSpeed.roda1 = ENC2.GetSpeed();
    wheelsCurrentSpeed.roda2 = ENC0.GetSpeed();//ajuste devido uso de motor diferente
    // wheelsCurrentSpeed.roda3 = ENC3.GetSpeed();
  return wheelsCurrentSpeed;
}


BR::ChassisSpeeds drivetrain::GetChassiSpeeds(){
    driveWheelSpeeds wSpeeds = getCurrentSpeed();
    wSpeeds.rpm2rad_s();
    auto chassiSpeedsV = m_kinematics.toChassisSpeeds(wSpeeds);
    return chassiSpeedsV;
}

void drivetrain::drive(double vx, double vy, double rot, bool fieldRel){
    
    auto wheelSpeeds = m_kinematics.toWheelSpeeds(
    fieldRel ? BR::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, rot, GetAngle())
                : BR::ChassisSpeeds{vx, vy, rot});

      
    auto val = BR::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, rot, GetAngle());
    
    frc::SmartDashboard::PutNumber("auto ANGLE", GetAngle().rad2deg());

    frc::SmartDashboard::PutNumber("auto Vx", val.vx);
    frc::SmartDashboard::PutNumber("auto Vy", val.vy);
    frc::SmartDashboard::PutNumber("auto Th", val.omega);    

    // frc::SmartDashboard::PutNumber("w0", wheelSpeeds.roda0);
    // frc::SmartDashboard::PutNumber("w1", wheelSpeeds.roda1);
    // frc::SmartDashboard::PutNumber("w2", wheelSpeeds.roda2);    

    wheelSpeeds.rad_s2rpm(); //converte rad_s para RPM [-1 a 1]
    wheelSpeeds.Normalize(m_kinematics.GetFrame()); //Normaliza [-1 a 1]

    frc::SmartDashboard::PutNumber("w0_Norm", wheelSpeeds.roda0);
    frc::SmartDashboard::PutNumber("w1_Norm", wheelSpeeds.roda1);
    frc::SmartDashboard::PutNumber("w2_Norm", wheelSpeeds.roda2);
    frc::SmartDashboard::PutNumber("w3_Norm", wheelSpeeds.roda3);

    setSpeeds(wheelSpeeds);

    // auto chassiSpeedsV2 = m_kinematics.toChassisSpeeds(wheelSpeeds);
    // frc::SmartDashboard::PutNumber("Vx_teste:", chassiSpeedsV2.vx);
    // frc::SmartDashboard::PutNumber("Vy_teste:", chassiSpeedsV2.vy);
    // frc::SmartDashboard::PutNumber("W__teste:", chassiSpeedsV2.omega);

    // frc::SmartDashboard::PutNumber("Vx")

    // SP = rampa.get(setPoint, frc::SmartDashboard::GetNumber("taxa", 1.0));

    // CO = std::clamp(PID1.Calculate(ENC0.GetEncoderDistance(), SP), -1.0, 1.0);
    // // CO = 0.5;
    // M0.Set(CO);
    
    // frc::SmartDashboard::PutNumber("Valor SP funcao", SP);
    // frc::SmartDashboard::PutNumber("Valor CO funcao", PID1.Calculate(ENC0.GetSpeed(), SP));
    // frc::SmartDashboard::PutNumber("Valor CO funcao clamp", CO);

    // SP = setPoint;
}


void drivetrain::reserEncoders(){
    ENC0.Reset(); 
    ENC1.Reset(); 
    ENC2.Reset();
    // ENC3.Reset();
}

void drivetrain::resetYaw(){
    navX.Reset();
}

void drivetrain::updateOdometry(){
    driveWheelSpeeds wSpeeds = getCurrentSpeed();
    wSpeeds.rpm2rad_s();
    m_odometry.Update(GetAngle(), wSpeeds);
    // m_odometry.Update(GetAngle(), getCurrentSpeed());
}

// void drivetrain::updateOdometryByDistance(){
//     driveWheelSpeeds actualEncDistances;
//     driveWheelSpeeds wSpeeds;
//     double currentTime = frc2::Timer::GetFPGATimestamp().to<double>();

//     double deltaTime =
//         (m_prevTimerEncDistance >= 0.0) ? currentTime - m_prevTimerEncDistance : 0.0;
//     m_prevTimerEncDistance = currentTime;

//     actualEncDistances.roda0 = ENC1.GetRaw();
//     actualEncDistances.roda1 = ENC2.GetRaw();
//     actualEncDistances.roda2 = ENC0.GetRaw();
//     // actualEncDistance.roda3 = ENC3.GetRaw();

//     wSpeeds.roda0 = ((actualEncDistances.roda0 - m_prevEncDistances.roda0) / deltaTime) * constant::fatorTicks_s2RPM_MAV *-1.0;
//     wSpeeds.roda1 = ((actualEncDistances.roda1 - m_prevEncDistances.roda1) / deltaTime) * constant::fatorTicks_s2RPM_MAV *-1.0;
//     wSpeeds.roda2 = ((actualEncDistances.roda2 - m_prevEncDistances.roda2) / deltaTime) * constant::fatorTicks_s2RPM_TOR * 1.0;
//     wSpeeds.roda3 = ((actualEncDistances.roda3 - m_prevEncDistances.roda3) / deltaTime) * constant::fatorTicks_s2RPM_MAV * 1.0;

//     m_prevEncDistances = actualEncDistances;

//     frc::SmartDashboard::PutNumber("w0 CALC by RAW", wSpeeds.roda0);
//     frc::SmartDashboard::PutNumber("w1 CALC by RAW", wSpeeds.roda1);
//     frc::SmartDashboard::PutNumber("w2 CALC by RAW", wSpeeds.roda2);
//     frc::SmartDashboard::PutNumber("w3 CALC by RAW", wSpeeds.roda3);

//     wSpeeds.rpm2rad_s();
//     m_odometry.Update(GetAngle(), wSpeeds);
//     // m_odometry.Update(GetAngle(), getCurrentSpeed());
// }