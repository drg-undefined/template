#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/QR>

// #include <units/units.h>

// #include "frc/geometry/Pose2d.h"
// #include "frc/geometry/Rotation2d.h"

#include "frc2/Timer.h"

#include <frc/smartdashboard/SmartDashboard.h>



#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
// #include <thread>
// Estrutura para um ponto no espaço
struct ControlPoint {
    double error;   // Erro de posição (SP_x - PV_x)
    double speed;   // Velocidade do motor correspondente
};

class MotorController {
public:
    MotorController(const std::vector<ControlPoint>& controlMap, double rampIncrement)
        : controlMap(controlMap), rampIncrement(rampIncrement), currentSpeed(0.0) {}

    double getMotorSpeed(double PV, double SP) {
        distanceError = SP-PV;
    // double getMotorSpeed(double _distanceError) {
        // distanceError = _distanceError;
        // Encontrar o ponto de controle mais próximo para o erro fornecido
        auto it = std::lower_bound(controlMap.begin(), controlMap.end(), distanceError,
            [](const ControlPoint& cp, double value) {
                return cp.error < value;
            });

        double targetSpeed;
        if (it == controlMap.end()) {
            targetSpeed = controlMap.back().speed;
        } else if (it == controlMap.begin()) {
            targetSpeed = controlMap.front().speed;
        } else {
            // Se o valor exato não for encontrado, interpolar entre os dois pontos mais próximos
            auto itPrev = std::prev(it);
            double ratio = (distanceError - itPrev->error) / (it->error - itPrev->error);
            targetSpeed = itPrev->speed + ratio * (it->speed - itPrev->speed);
        }

        // Aplicar rampa de aceleração
        if (currentSpeed < targetSpeed) {
            currentSpeed += rampIncrement;
            if (currentSpeed > targetSpeed) {
                currentSpeed = targetSpeed;
            }
        } else if (currentSpeed > targetSpeed) {
            currentSpeed -= rampIncrement;
            if (currentSpeed < targetSpeed) {
                currentSpeed = targetSpeed;
            }
        }

        return currentSpeed;
    }
    void reset() {
        currentSpeed = 0.0;
        // targetSpeed = 0.0
    }
    void setRampInc(double newRampInc){
        rampIncrement = newRampInc;
    }
    bool atSP(){
        return std::abs(distanceError) < std::abs(m_tolerance);
    }
    void setTolerance(double tol){
        m_tolerance = std::abs(tol);
    }

private:
    std::vector<ControlPoint> controlMap;
    double rampIncrement;
    double currentSpeed;
    double distanceError;
    double m_tolerance = 0.0;
};

enum class frame {
    Omni3Wheels,
    Omni4Wheels,
    Mecanum,
    Differential2Wheels,
    Differential4Wheels,
    Custom,
    Default
};

enum class units_ {
    rad_s,
    rpm,
    rpm2,
    rad_s2rpmNorm,
    rpm2rad_s,
    none
};

struct wheelConfig {
    // units::meter_t raio;
    double raio;
    double angle;    // Ângulo da roda em radianos (entrada em graus)
    double distance; // Distância da roda até o centro do robô
    double offsetX;  // Distância em x da roda até o centro do robô (para mecanum e diferencial)
    double offsetY;  // Distância em y da roda até o centro do robô (para mecanum e diferencial)
    

    wheelConfig(double _r = 0.0, double _a = 0.0, double _d = 0.0, double _x = 0.0, double _y = 0.0) :
        raio(_r), angle(_a*M_PI/180.0), distance(_d), offsetX(_x), offsetY(_y) {}

    // wheelConfig(units::meter_t _r = 0.0_m, double _a = 0.0, double _d = 0.0, double _x = 0.0, double _y = 0.0) :
    //     raio(_r), angle(_a), distance(_d), offsetX(_x), offsetY(_y) {}    
};

struct driveWheelsConfig {
    wheelConfig configRoda0;
    wheelConfig configRoda1;
    wheelConfig configRoda2;
    wheelConfig configRoda3;
};

// struct rotation2d {
//     double angle;
//     rotation2d(double _th = 0.0) : angle(_th){}
// };

struct pose2d {
    double x;
    double y;
    double th; //sempre em rad

    pose2d(double _x = 0.0, double _y = 0.0, double _th = 0.0) : x(_x), y(_y), th(_th) {}
        //Construtor recebe angulo em GRAUS
    pose2d operator+(const pose2d& other) const{
        return {x+other.x, y+other.y, th+other.th};
    }
    pose2d operator-(const pose2d& other) const{
        return {x-other.x, y-other.y, th-other.th};
    }

    double Rotation() const { return th; }

    void Rotate(double angle) {
        double cosA = std::cos(angle * (M_PI / 180.0));
        double sinA = std::sin(angle * (M_PI / 180.0));
        double out[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        x = out[0];
        y = out[1];
    }
    void Rotate() {
        double angle = th;
        double cosA = std::cos(angle);
        double sinA = std::sin(angle);
        double out[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        x = out[0];
        y = out[1];
    }
    double Magnitude() const { return std::sqrt(x * x + y * y); }
    double distance(const pose2d& other) const {
        return std::sqrt( (x-other.x)*(x-other.x) + (y-other.y)*(y-other.y) ); }
    
    //Implementar distância entre pontos [?]
    double radians() const { return th; }
    double degrees() const { return th*(180.0/M_PI); }
    double rad2deg() const { return th*(180.0/M_PI); }
    double deg2rad() const { return th*(M_PI/180.0); }
    double Cos() const { return std::cos(radians()); }
    double Sin() const { return std::sin(radians()); }
    // double Tan() const { return 0.0; }
    double NormalizedDeg() { //resultado entre 0 e 360 graus;
        double normalizedAngle = fmod(degrees(), 360.0);
        if (normalizedAngle < 0.0) {
            normalizedAngle += 360.0;
        }
        return normalizedAngle;
    }
    double NormalizedRad() { //resultado entre 0 e 2*pi graus;
        double normalizedAngle = fmod(th, 2.0*M_PI);
        if (normalizedAngle < 0.0) {
            normalizedAngle += 2.0*M_PI;
        }
        return normalizedAngle;
    }

    pose2d Exp(const double& dx, const double& dy, const double& dtheta) const {
        const auto sinTheta = std::sin(dtheta);
        const auto cosTheta = std::cos(dtheta);

        double s, c;
        if (std::abs(dtheta) < 1E-9) {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        } else {
            // s = sinTheta / dtheta;
            // c = (1 - cosTheta) / dtheta;

            s = sinTheta;
            c = cosTheta;
        }

        // const pose2d transform{dx * s - dy * c, dx * c + dy * s, dtheta};
        const pose2d transform{dx * c - dy * s, dx * s + dy * c, dtheta};


        return *this + transform;
    }
};




struct driveWheelSpeeds{
    double roda0 = 0.0;
    double roda1 = 0.0;
    double roda2 = 0.0;
    double roda3 = 0.0;
    driveWheelSpeeds(double _roda0 = 0.0, double _roda1 = 0.0, double _roda2 = 0.0, double _roda3 = 0.0) : 
            roda0(_roda0), roda1(_roda1), roda2(_roda2), roda3(_roda3) {}

    void Normalize(frame tipo, double attainableMaxSpeed = 1.0) {
        int nRodas = 4;

        // if (tipo == frame::Differential2Wheels) {nRodas = 2;}
        // if (tipo == frame::Omni3Wheels) {nRodas = 3;}
        switch (tipo) {
            case frame::Differential2Wheels :
                nRodas = 2;
                break;
            case frame::Omni3Wheels :
                nRodas = 3;
                break;
            default:
                nRodas = 4;
                break;
        }
        
        std::array<double, 4> wheelSpeeds{roda0, roda1, roda2, roda3};
        double realMaxSpeed = *std::max_element(
            wheelSpeeds.begin(), wheelSpeeds.end()-(4-nRodas), [](const auto& a, const auto& b) {
                return std::abs(a) < std::abs(b);
            });
        
        realMaxSpeed = std::abs(realMaxSpeed);
        if (realMaxSpeed > attainableMaxSpeed) {
            for (int i = 0; i < nRodas; ++i) {
            wheelSpeeds[i] = wheelSpeeds[i] / realMaxSpeed * attainableMaxSpeed;
            }
            roda0 = wheelSpeeds[0];
            roda1 = wheelSpeeds[1];
            roda2 = wheelSpeeds[2];
            roda3 = wheelSpeeds[3];
        }
    }

    void rad_s2rpm(){
        //converte de rad/s para RPM [-1 a 1]
        // std::clamp(wheelSpeeds.roda0.to<double>(), -1.0, 1.0);
        roda0 = roda0*60.0/(2.0*M_PI)/100; 
        roda1 = roda1*60.0/(2.0*M_PI)/100;
        roda2 = roda2*60.0/(2.0*M_PI)/100;
        roda3 = roda3*60.0/(2.0*M_PI)/100;
    }
    void rpm2rad_s(){
        //GetSpeed()/2 -> RPM * 2.0 * M_PI = rad/min -> rad/min / 60.0 s = rad/s
        //GetSpeed()/2.0 * 2.0*M_PI / 60.0 = GetSpeed()*M_PI/60.0 [rad/s]
        roda0 = roda0*M_PI/60.0;
        roda1 = roda1*M_PI/60.0;
        roda2 = roda2*M_PI/60.0;
        roda3 = roda3*M_PI/60.0;
    }
};



namespace BR {

struct ChassisSpeeds {
  /**
   * Represents forward velocity w.r.t the robot frame of reference. (Fwd is +)
   */
  double vx = 0.0;

  /**
   * Represents strafe velocity w.r.t the robot frame of reference. (Left is +)
   */
  double vy = 0.0;

  /**
   * Represents the angular velocity of the robot frame. (CCW is +)
   */
  double omega = 0.0;

  /**
   * Converts a user provided field-relative set of speeds into a robot-relative
   * ChassisSpeeds object.
   *
   * @param vx The component of speed in the x direction relative to the field.
   * Positive x is away from your alliance wall.
   * @param vy The component of speed in the y direction relative to the field.
   * Positive y is to your left when standing behind the alliance wall.
   * @param omega The angular rate of the robot.
   * @param robotAngle The angle of the robot as measured by a gyroscope. The
   * robot's angle is considered to be zero when it is facing directly away from
   * your alliance station wall. Remember that this should be CCW positive.
   *
   * @return ChassisSpeeds object representing the speeds in the robot's frame
   * of reference.
   */
  static ChassisSpeeds FromFieldRelativeSpeeds(
      double vx, double vy,
      double omega, const pose2d& robotPose) {
    return { vx * robotPose.Cos() + vy * robotPose.Sin(),
            -vx * robotPose.Sin() + vy * robotPose.Cos(), omega};
  }
};

class cinematica {
    public:
        explicit cinematica(frame tipo, const wheelConfig& roda0, const wheelConfig& roda1, const wheelConfig roda2, const wheelConfig roda3) : 
                m_frame(tipo),
                m_roda0(roda0), m_roda1(roda1), m_roda2(roda2), m_roda3(roda3) {
                    SetInverseKinematics();
                    switch (GetFrame()) {
                        case frame::Omni3Wheels :
                            m_forwardKinematics_3x3 = m_inverseKinematics_3x3.householderQr();
                            // frc::SmartDashboard::PutNumber("A-1 [0 0]: ", m_inverseKinematics_3x3(0,0));
                            // frc::SmartDashboard::PutNumber("A-1 [0 1]: ", m_inverseKinematics_3x3(0,1));
                            // frc::SmartDashboard::PutNumber("A-1 [1 0]: ", m_inverseKinematics_3x3(1,0));
                            // m_forwardKinematics_3x3 = m_inverseKinematics_3x3.inverse();
                            break;
                        case frame::Mecanum :
                            m_forwardKinematics_4x3 = m_inverseKinematics_4x3.householderQr();
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
        driveWheelSpeeds toWheelSpeeds(const ChassisSpeeds& chassisSpeeds);
        ChassisSpeeds toChassisSpeeds(const driveWheelSpeeds& wheelSpeeds);
        // frame GetFrame();
        frame GetFrame(){ return m_frame; }
    private:
        Eigen::Matrix<double, 3, 3> m_inverseKinematics_3x3;
        Eigen::HouseholderQR<Eigen::Matrix<double, 3, 3>> m_forwardKinematics_3x3;
        
        Eigen::Matrix<double, 4, 3> m_inverseKinematics_4x3;
        Eigen::HouseholderQR<Eigen::Matrix<double, 4, 3>> m_forwardKinematics_4x3;

        frame m_frame;
        wheelConfig m_roda0;
        wheelConfig m_roda1;
        wheelConfig m_roda2;
        wheelConfig m_roda3;
        void SetInverseKinematics();
};


class driveOdometry {
 public:

  /**
   * Constructs a MecanumDriveOdometry object.
   *
   * @param tipo 
   * @param gyroAngle The angle reported by the gyroscope.
   * @param initialPose The starting position of the robot on the field.
   */
  explicit driveOdometry(cinematica m_kinematics,
                                const pose2d& gyroAngle,
                                const pose2d& initialPose = pose2d());

  /**
   * Resets the robot's position on the field.
   *
   * The gyroscope angle does not need to be reset here on the user's robot
   * code. The library automatically takes care of offsetting the gyro angle.
   *
   * @param pose The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  void ResetPosition(const pose2d& pose, const pose2d& gyroAngle) {
    m_pose = pose;
    m_angle = 0.0;
    m_previousAngle = pose.Rotation();
    m_gyroOffset = m_pose.Rotation() - gyroAngle.Rotation();
  }

  /**
   * Returns the position of the robot on the field.
   * @return The pose of the robot.
   */
  const pose2d& GetPose() const { return m_pose; }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method takes in the current time as
   * a parameter to calculate period (difference between two timestamps). The
   * period is used to calculate the change in distance from a velocity. This
   * also takes in an angle parameter which is used instead of the
   * angular rate that is calculated from forward kinematics.
   *
   * @param currentTime The current time.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelSpeeds The current wheel speeds.
   *
   * @return The new pose of the robot.
   */
  const pose2d& UpdateWithTime(double currentTime,
                               const pose2d& gyroAngle,
                               driveWheelSpeeds wheelSpeeds);
                               

  const pose2d& UpdateWithTime_ENC(double currentTime,
                               const pose2d& gyroAngle,
                               driveWheelSpeeds wheelSpeeds);
  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This method automatically calculates
   * the current time to calculate period (difference between two timestamps).
   * The period is used to calculate the change in distance from a velocity.
   * This also takes in an angle parameter which is used instead of the
   * angular rate that is calculated from forward kinematics.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelSpeeds The current wheel speeds.
   *
   * @return The new pose of the robot.
   */
  const pose2d& Update(const pose2d& gyroAngle,
                       driveWheelSpeeds wheelSpeeds) {
    return UpdateWithTime(frc2::Timer::GetFPGATimestamp().to<double>(), gyroAngle,
                          wheelSpeeds);
  }

 private:
    cinematica m_kinematics;
    pose2d m_pose;

    double m_previousTime = -1.0;
    double m_previousAngle;
    double m_gyroOffset;

    double m_angle = 0.0;
};


}  // namespace BR

// #pragma once

// #include <cmath>

// struct WheelConfig {
//     double angle;    // Ângulo da roda em graus
//     double distance; // Distância da roda até o centro do robô
//     double offsetX;  // Distância em x da roda até o centro do robô (para mecanum e diferencial)
//     double offsetY;  // Distância em y da roda até o centro do robô (para mecanum e diferencial)
// };

// class RobotBase {
// public:
//     enum class BaseType {
//         Omni3Wheels,
//         Omni4Wheels,
//         Mecanum,
//         Differential2Wheels,
//         Differential4Wheels
//     };
//     // Construtor para configurar a base do robô
//     // RobotBase(BaseType type, const WheelConfig config[4]);
//     void omni(const WheelConfig w1, const WheelConfig w2, const WheelConfig w3);
//     void omni(const WheelConfig w1, const WheelConfig w2, const WheelConfig w3, const WheelConfig w4);
//     void mecanum(const WheelConfig fr, const WheelConfig fl, const WheelConfig br, const WheelConfig bl);
//     void diff(const WheelConfig right, const WheelConfig left);
//     void diff(const WheelConfig fr, const WheelConfig fl, const WheelConfig br, const WheelConfig bl);
// private:
//     // BaseType m_type;
//     // WheelConfig m_wheelConfigs[4];
// };
