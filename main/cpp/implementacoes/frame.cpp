#include "frame.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace BR;
using namespace frc;

driveOdometry::driveOdometry(cinematica kinematics,
                             const pose2d& gyroAngle,
                             const pose2d& initialPose)
    : m_kinematics(kinematics), m_pose(initialPose) {
  m_previousAngle = m_pose.Rotation();
  m_gyroOffset = m_pose.Rotation() - gyroAngle.Rotation();
}

const pose2d& driveOdometry::UpdateWithTime_ENC(
                double currentTime,
                const pose2d& gyroAngle,
                driveWheelSpeeds wheelSpeeds) {
    
    double deltaTime =
        (m_previousTime >= 0.0) ? currentTime - m_previousTime : 0.0;
    m_previousTime = currentTime;

    auto angle = gyroAngle.Rotation() + m_gyroOffset;
    frc::SmartDashboard::PutNumber("ANGULO", angle);

    auto [dx, dy, dtheta] = m_kinematics.toChassisSpeeds(wheelSpeeds);
    
    // // A expressão static_cast<void>(dtheta); em C++ é usada para suprimir avisos de compilação relacionados a variáveis não utilizadas.
    // static_cast<void>(dtheta);


    // auto newPose = m_pose.Exp(dx * deltaTime,
    //                           dy * deltaTime,
    //                           (angle - m_previousAngle));
    // m_angle = dtheta*deltaTime;
    // angle = m_pose.Rotation() + m_gyroOffset + m_angle;
    // auto newPose = m_pose.Exp(dx * deltaTime,
    //                           dy * deltaTime,
    //                           (angle - m_previousAngle));
    // frc::SmartDashboard::PutNumber("m_angle", m_angle);
    // frc::SmartDashboard::PutNumber("dtheta", dtheta);
    // frc::SmartDashboard::PutNumber("m_previousAngle", m_previousAngle);
    // m_previousAngle = angle;

    m_angle = m_angle + dtheta*deltaTime;
    angle = m_gyroOffset + m_angle;
    auto newPose = m_pose.Exp(dx * deltaTime,
                              dy * deltaTime,
                              (angle ));
                            //   (angle - m_previousAngle));
    m_previousAngle = angle;
    

    // m_previousAngle = angle;
    // m_pose = {newPose.x, newPose.y, angle.Rotation()};
    m_pose = {newPose.x, newPose.y, angle};

    return m_pose;
}

const pose2d& driveOdometry::UpdateWithTime(
                double currentTime,
                const pose2d& gyroAngle,
                driveWheelSpeeds wheelSpeeds) {
    
    double deltaTime =
        (m_previousTime >= 0.0) ? currentTime - m_previousTime : 0.0;
    m_previousTime = currentTime;

    auto angle = gyroAngle.th + m_gyroOffset;

    auto [dx, dy, dtheta] = m_kinematics.toChassisSpeeds(wheelSpeeds);
    
    // // A expressão static_cast<void>(dtheta); em C++ é usada para suprimir avisos de compilação relacionados a variáveis não utilizadas.
    static_cast<void>(dtheta);
    auto newPose = m_pose.Exp(dx * deltaTime,
                              dy * deltaTime,
                              (angle ));
                            //   (angle - m_previousAngle));
    frc::SmartDashboard::PutNumber("ANGULO", angle);
    frc::SmartDashboard::PutNumber("DIFF ANGULO", angle - m_previousAngle);                              
    m_previousAngle = angle;
    m_pose = {newPose.x, newPose.y, angle};

    return m_pose;
}

// cinematica::cinematica(frame tipo) : m_frame(tipo) {}

driveWheelSpeeds cinematica::toWheelSpeeds(const ChassisSpeeds& chassisSpeeds) {
    double Vx = chassisSpeeds.vx;
    double Vy = chassisSpeeds.vy;
    double Omega = chassisSpeeds.omega;

    double w0 = 0.0; double w1=0.0; double w2=0.0; double w3=0.0;

    Eigen::Vector3d chassisSpeedsVector;
    chassisSpeedsVector << Vx, Vy, Omega;

    Eigen::Matrix<double, 3, 1> wheelsMatrix_3x1;
    Eigen::Matrix<double, 4, 1> wheelsMatrix_4x1;

    // driveWheelSpeeds wheelSpeeds;
    switch (cinematica::m_frame) {
    case frame::Omni3Wheels :
        wheelsMatrix_3x1 = m_inverseKinematics_3x3 * chassisSpeedsVector;
        w0 = wheelsMatrix_3x1(0, 0);
        w1 = wheelsMatrix_3x1(1, 0);
        w2 = wheelsMatrix_3x1(2, 0);
        // w3 = 0.0;
        break;
    case frame::Mecanum :
        // // We have a new center of rotation. We need to compute the matrix again.
        // if (centerOfRotation != m_previousCoR) {
        //     auto fl = m_frontLeftWheel - centerOfRotation;
        //     auto fr = m_frontRightWheel - centerOfRotation;
        //     auto rl = m_rearLeftWheel - centerOfRotation;
        //     auto rr = m_rearRightWheel - centerOfRotation;

        //     SetInverseKinematics(fl, fr, rl, rr);

        //     m_previousCoR = centerOfRotation;
        // } //Para utilizar centerOfRotation, necessário informar na entrada da função (X,Y)
        wheelsMatrix_4x1 = m_inverseKinematics_4x3 * chassisSpeedsVector;
        w0 = wheelsMatrix_4x1(0, 0);
        w1 = wheelsMatrix_4x1(1, 0);
        w2 = wheelsMatrix_4x1(2, 0);
        w3 = wheelsMatrix_4x1(3, 0);
        break;
    // case frame::Omni4Wheels :
    //     break;
    // case frame::Differential2Wheels :
    //     break;
    // case frame::Differential4Wheels :
    //     break;
    default:
        // w0 = 0.0; w1 = 0.0; w2 = 0.0; w3 = 0.0;
        break;
    }
    
  return { w0, w1, w2, w3 }; //retorna em rad/s
}

ChassisSpeeds cinematica::toChassisSpeeds(const driveWheelSpeeds& wheelSpeeds) {
    // units::meters_per_second_t Vx = 0_mps;
    // units::meters_per_second_t vy = 0_mps;
    // units::radians_per_second_t omega = 0_rad_per_s;
    double Vx = 0.0; double Vy = 0.0; double Rot = 0.0;

    double w0 = wheelSpeeds.roda0;
    double w1 = wheelSpeeds.roda1;
    double w2 = wheelSpeeds.roda2;
    double w3 = wheelSpeeds.roda3;
    // double R = 0.0;
    
    Eigen::Matrix<double, 3, 1> wheelSpeedMatrix_3x1;
    Eigen::Matrix<double, 4, 1> wheelSpeedsMatrix_4x1;
    Eigen::Vector3d chassisSpeedsVector;

    switch (cinematica::m_frame) {
    case frame::Omni3Wheels :
        // Cinemática Direta: Calcular velocidades linear Vx e Vy e velocidade angular omega do robô
        
        // clang-format off
        wheelSpeedMatrix_3x1 << w0, w1, w2;
        // clang-format on
        chassisSpeedsVector = m_forwardKinematics_3x3.solve(wheelSpeedMatrix_3x1);
        Vx = chassisSpeedsVector(0);
        Vy = chassisSpeedsVector(1);
        Rot = chassisSpeedsVector(2);
        break;
    case frame::Mecanum :
        
        // // clang-format off
        wheelSpeedsMatrix_4x1 << w0, w1, w2, w3;
        // // clang-format on

        chassisSpeedsVector = m_forwardKinematics_4x3.solve(wheelSpeedsMatrix_4x1);
        Vx = chassisSpeedsVector(0);
        Vy = chassisSpeedsVector(1);
        Rot = chassisSpeedsVector(2);
        break;
    // case frame::Omni4Wheels :
    //     break;
    // case frame::Differential2Wheels :
    //     break;
    // case frame::Differential4Wheels :
    //     break;
    default:
        Vx = 0.0; Vy = 0.0; Rot = 0.0;
        break;
    }

    return { Vx, Vy, Rot };
}

void cinematica::SetInverseKinematics(){
    double soma_dist  = m_roda0.distance + m_roda1.distance + m_roda2.distance;
    switch (cinematica::m_frame) {
        case frame::Omni3Wheels : //wi para Chassi (Vx, Vy, W)
            // clang-format off

            
            m_inverseKinematics_3x3 << cos(m_roda0.angle+M_PI/2),  sin(m_roda0.angle+M_PI/2),   m_roda0.distance,
                                       cos(m_roda1.angle+M_PI/2),  sin(m_roda1.angle+M_PI/2),   m_roda1.distance,
                                       cos(m_roda2.angle+M_PI/2),  sin(m_roda2.angle+M_PI/2),   m_roda2.distance;
                                    //    m_roda0.distance/soma_dist, m_roda1.distance/soma_dist,  m_roda2.distance/soma_dist;

            m_inverseKinematics_3x3 /= m_roda0.raio;
            // clang-format on
            // m_inverseKinematics /= std::sqrt(2);
            
            break;
        case frame::Mecanum :
            // clang-format off
            m_inverseKinematics_4x3 << 1, -1, (-(m_roda0.offsetX + m_roda0.offsetY)),
                                       1,  1, (m_roda1.offsetX - m_roda1.offsetY),
                                       1,  1, (m_roda2.offsetX - m_roda2.offsetY),
                                       1, -1, (-(m_roda3.offsetX + m_roda3.offsetY));
            // clang-format on
            m_inverseKinematics_4x3 /= std::sqrt(2);
            break;
        // case frame::Omni4Wheels :
        //     break;
        // case frame::Differential2Wheels :
        //     break;
        // case frame::Differential4Wheels :
        //     break;
        default:
            break;
        }
}

