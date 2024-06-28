#pragma once

#define _USE_MATH_DEFINES
#include <math.h>


#include "studica/TitanQuad.h"
#include "studica/TitanQuadEncoder.h"
#include "AHRS.h"
#include "ramp.h"
// #include "OI.h"

#include <frc/controller/PIDController.h>
#include "frame.h"

#include <frc/smartdashboard/SmartDashboard.h>

namespace constant
{
    //Motors
    static constexpr int TITAN_ID       = 42;
    // static constexpr int FRONT_LEFT     = 2;
    // static constexpr int BACK_LEFT      = 3;
    // static constexpr int FRONT_RIGHT    = 0;
    // static constexpr int BACK_RIGHT     = 1;

    //Encoder
    static constexpr double WHEEL_RADIUS    = 51; //mm
    static constexpr double PULSE_PER_REV   = 1440; //1464;
    static constexpr double GEAR_RATIO      = 1/1;
    static constexpr double ENCODER_PULSE_RATIO = PULSE_PER_REV * GEAR_RATIO;
    static constexpr double WHEEL_DIST_PER_TICK   =   (M_PI * 2 * WHEEL_RADIUS) / ENCODER_PULSE_RATIO;

    static constexpr double OMS_DIST_PER_TICK = 1;

    //Fator Conversão ticks/s para RPM
    static constexpr double fatorTicks_s2RPM_TOR = 2*60.0/1440.0/100.0;
    static constexpr double fatorTicks_s2RPM_MAV = 2*60.0/1464.0/100.0;

    // //Inputs
    // static constexpr int START_BUTTON   = 9;
    // static constexpr int STOP_BUTTON    = 11;
    // //Outputs
    // static constexpr int RUNNING_LED    = 20;
    // static constexpr int STOPPED_LED    = 21;
}


class drivetrain {
    public:
        drivetrain(); //construtor da classe; é chamado quando um objeto é criado. Inicializa o drivetrain;

        void resetYaw();
        void reserEncoders();
        void setSpeeds(const driveWheelSpeeds& wheelSpeeds);
        driveWheelSpeeds getCurrentSpeed();
        // driveWheelSpeeds getCurrentSpeed(void);

        // void setMotors(const double& )
        // void drive(double vx, double vy, double w_rad, bool fieldRel);
        void drive(double vx = 0.0, double vy = 0.0, double rot = 0.0, bool fieldRel = false);
        void updateOdometry();
        // void updateOdometryByDistance();

        BR::ChassisSpeeds GetChassiSpeeds();

        // void driveWithTrajectory();
        // void driveWithPID(pose2d poseFinal)
        const pose2d& GetPose() const { return m_odometry.GetPose(); }
        // pose2d robotPose = m_odometry.GetPose();

        pose2d GetAngle() const {
            // Negating the angle because WPILib Gyros are CW positive.
            return pose2d(0.0, 0.0, -navX.GetAngle()*M_PI/180.0);
        }

        double GetYawDeg() const {
            // Negating the angle because WPILib Gyros are CW positive.
            // frc::SmartDashboard::PutData(navX);
            return navX.GetAngle();
        }

        // pose2d GetPosition_navX() {
        //     return pose2d(navX.GetDisplacementX(), navX.GetDisplacementY(), navX.GetAngle());
        // }

        // pose2d GetVelocity_navX() {
        //     return pose2d(navX.GetVelocityX(), navX.GetVelocityY(), navX.GetRate());
        // }

        // AHRS& GetNavX() {
        //     return navX;
        // }

        // void printEncoder(){
        //     auto Encoder = getCurrentSpeed();
        //     frc::SmartDashboard::PutNumber("ENC_Roda0:", Encoder.roda0);
        //     frc::SmartDashboard::PutNumber("ENC_Roda1:", Encoder.roda1);
        //     frc::SmartDashboard::PutNumber("ENC_Roda2:", Encoder.roda2);
        // }

        void resetPosition(const pose2d pose, const pose2d& gyroAngle){
            m_odometry.ResetPosition(pose, gyroAngle);
        }


    private:
        double cm(double medida){ return double(medida)/100.0; }
        double mm(double medida){ return double(medida)/1000.0; }
        
        double m_prevTimer = -1.0;
        driveWheelSpeeds m_wheelsCurrentDistance;

        studica::TitanQuad M0{constant::TITAN_ID, 0};
        studica::TitanQuad M1{constant::TITAN_ID, 1};
        studica::TitanQuad M2{constant::TITAN_ID, 2};
        // studica::TitanQuad M3_OMS{constant::TITAN_ID, 3};
        // studica::TitanQuadEncoder ENC0{M0, 0, ((M_PI*2*mm(51))/1440)}; //Motor diferente
        // studica::TitanQuadEncoder ENC1{M1, 1, ((M_PI*2*mm(51))/1464)};
        // studica::TitanQuadEncoder ENC2{M2, 2, ((M_PI*2*mm(51))/1464)};    
        studica::TitanQuadEncoder ENC0{M0, 0, ((M_PI*2.0*51.0)/1440.0)}; //Motor diferente
        studica::TitanQuadEncoder ENC1{M1, 1, ((M_PI*2.0*51.0)/1464.0)};
        studica::TitanQuadEncoder ENC2{M2, 2, ((M_PI*2.0*51.0)/1464.0)};        
        // studica::TitanQuadEncoder ENC3_OMS{M3_OMS, 3, constant::OMS_DIST_PER_TICK};

        studica::TitanQuad M3_OMS{constant::TITAN_ID, 3};
        studica::TitanQuadEncoder ENC3_OMS{M3_OMS, 3, ((M_PI*2.0*51.0)/1440.0) };

        double m_mavTick = ((M_PI*2.0*51.0)/1464.0) * 12; //no Maverick a relação é 61:1
        double m_torqTick = ((M_PI*2.0*51.0)/1440.0) * 12; //no Torquenado a relação é 60:1

        AHRS navX{frc::SPI::Port::kMXP};
        // frc2::PIDController PID_x {0.8, 0.0, 0.0}; //{Kp, Ki, Kd};
        // frc2::PIDController PID_y {0.8, 0.0, 0.0}; //{Kp, Ki, Kd};
        // frc2::PIDController PID_th {1.0, 0.0, 0.0}; //{Kp, Ki, Kd};
        
        ramp rampa_Vx;
        ramp rampa_Vy;
        ramp rampa_W;

        double Tanterior = 0.0; 

        // wheelConfig roda0{mm(51), 60.0,  cm(18)};
        // wheelConfig roda1{mm(51), 180.0, cm(19.5)};
        // wheelConfig roda2{mm(51), 300.0, cm(18)};
        wheelConfig roda0{51.0, 60.0,  190.0}; //trabalhando em mm
        wheelConfig roda1{51.0, 180.0, 200.0};
        wheelConfig roda2{51.0, 300.0, 190.0};
        // wheelConfig roda2{0.051, 300.0, 0.18};
        wheelConfig roda3;
        driveWheelSpeeds m_prevEncDistances;
        double m_prevTimerEncDistance = -1.0;
        
        // driveWheelsConfig rodasConfig;
        // rodasConfig.
        
        BR::cinematica m_kinematics{frame::Omni3Wheels, roda0, roda1, roda2, roda3};
        BR::driveOdometry m_odometry{m_kinematics, GetAngle()};
};