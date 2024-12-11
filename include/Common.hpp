#ifndef ROBOSIGNAL_COMMON_HPP
#define ROBOSIGNAL_COMMON_HPP

#include <array>
#include <string>

#include <QMetaType>
#include <QMutex>
#include <QObject>
#include <QString>
#include <QSerialPortInfo>
#include <QString>
#include <QStringList>

#include "robosignal_global.hpp"

namespace rc {

enum Axis : int { X = 1, Y, Z, RX, RY, RZ };
enum Joint : int { J1 = 1, J2, J3, J4, J5, J6 };
constexpr const int Axes = 6;
constexpr const int Joints = 6;
using Coords = std::array<double, Axes>;
using Angles = std::array<double, Joints>;
using IoSates = std::array<int, 11>;
using PortsInfo = QList<QSerialPortInfo>;

QDebug ROBOSIGNALSHARED_EXPORT operator << (QDebug debug, const Coords& c);

constexpr const double DoubleEpsilon = 0.001;
#if defined ROBOT_MYCOBOT
constexpr const double CoordsEpsilon = 5.0;
constexpr const double EncodersEpsilon = 50.0;
#elif defined ROBOT_MYCOBOTPRO
constexpr const double CoordsEpsilon = 1.0;
constexpr const double EncodersEpsilon = 10.0;
#elif defined ROBCTL_PHOENIX
constexpr const double CoordsEpsilon = 0.50;
#endif

constexpr const double AnalogPinEpsilon = 0.001;

bool ROBOSIGNALSHARED_EXPORT DoubleEqual(double a, double b, double epsilon = DoubleEpsilon);
bool ROBOSIGNALSHARED_EXPORT CoordEqual(double a, double b, double epsilon = CoordsEpsilon);
bool ROBOSIGNALSHARED_EXPORT CoordsEqual(const Coords& c1, const Coords& c2, double epsilon = CoordsEpsilon);
bool ROBOSIGNALSHARED_EXPORT IsCoordsZero(const Coords& c1);

enum DI {
    CatPhysicalStart = 15,
    CatPhysicalStop = 16,
    CatPhysicalUserDefine = 17,

    J1Communication = 34,
    J2Communication = 35,
    J3Communication = 36,
    J4Communication = 37,
    J5Communication = 38,
    J6Communication = 39,

    IoStopTriggered = 40,  //0->1 triger
    IoRunTriggered = 41,  //0->1 triger
    HardwarePausePressed = 43,  //1-->pressed
    BrakeActivationRunning = 44,  //1-->runing
    J1ServoEnabled = 45,
    J2ServoEnabled = 46,
    J3ServoEnabled = 47,
    J4ServoEnabled = 48,
    J5ServoEnabled = 49,
    J6ServoEnabled = 50,

    HardwareFreeMove = 51,
    MotionEnable = 52,

    J1Status = 56, //0->ok
    J2Status = 57, //0->ok
    J3Status = 58, //0->ok
    J4Status = 59, //0->ok
    J5Status = 60, //0->ok
    J6Status = 61, //0->ok

    EmergenceStop = 62,  //0->pressed, 1->released
    PowerOnStatus = 63,  //1->ready
};

enum DO {
    BrakeActiveAuto = 47,

    BrakeManualModeEnable = 48, //1->enable
    J1BrakeRelease = 49, //1->release
    J2BrakeRelease = 50, //1->release
    J3BrakeRelease = 51, //1->release
    J4BrakeRelease = 52, //1->release
    J5BrakeRelease = 53, //1->release
    J6BrakeRelease = 54, //1->release

    SkipInitErrors = 55,
    ProgramAutoRunning = 56,
    PowerOnRelay1 = 57,
    PowerOnRelay2 = 58,
    SoftwareFreeMove = 60,
};

enum AI {
    J1Voltage = 31,
    J2Voltage = 32,
    J3Voltage = 33,
    J4Voltage = 34,
    J5Voltage = 35,
    J6Voltage = 36,

    J1Temperature = 37,
    J2Temperature = 38,
    J3Temperature = 39,
    J4Temperature = 40,
    J5Temperature = 41,
    J6Temperature = 42,

    J1WindingACurrent = 43,
    J2WindingACurrent = 44,
    J3WindingACurrent = 45,
    J4WindingACurrent = 46,
    J5WindingACurrent = 47,
    J6WindingACurrent = 48,

    J1WindingBCurrent = 49,
    J2WindingBCurrent = 50,
    J3WindingBCurrent = 51,
    J4WindingBCurrent = 52,
    J5WindingBCurrent = 53,
    J6WindingBCurrent = 54,

    Robot = 55,

    RobotAvgPower = 56,
    ControlerTemperature = 57,

    J1Current = 58,
    J2Current = 59,
    J3Current = 60,
    J4Current = 61,
    J5Current = 62,
    J6Current = 63,
};

enum AO {
    LedLight = 16,

    Acceleration = 41,

    ToolForce = 48,
    ToolSpeed = 49,
    StoppingDistance = 50,
    StoppingTime = 51,
    PowerLimit = 52,

    Payload = 53,

    J1Torque = 56,
    J2Torque = 57,
    J3Torque = 58,
    J4Torque = 59,
    J5Torque = 60,
    J6Torque = 61,

    XYAxisTorque = 62,
    ZAxisTorque = 63,
};

/// AO::Robot return values
enum Robots {
    Elephant = 101,
    Panda3 = 201,
    Panda5 = 202,
    Cat3 = 301,
#if defined ROBCTL_ATOMMAIN
    Mycobot = 401,
    MycobotPro = 402,
#endif
};

enum LedColors {
    RedAlways = 20,
    GreenAlways = 21,
    YellowAlways = 22,
    RedBlink = 23,
    GreenBlink = 24,
    YellowBlink = 25,
};

constexpr const char *const RobotName[] = {
    "Elephant5",
    "Panda3",
    "Panda5",
    "Catbot3",
    "MyCobot"
};

extern int ROBOSIGNALSHARED_EXPORT current_robot;

/**
 * 1. Min, max Joint angle limits
 * 2. min, default, max Joint velocity limits
 * 3. min, default, max Joint acceleration limits
 * 4. Minimum, default, maximum Collision torque limits
 * 5. Payload (min, max), default is min
 */

constexpr const int MaxLinearSpeed = 100;
constexpr const int DefaultLinearSpeed = static_cast<int>(MaxLinearSpeed / 2.5);
constexpr const int MaxAngleSpeed = 100; 
constexpr const int DefaultAngleSpeed = MaxAngleSpeed / 5;
constexpr const int DefaultXYTorqueLimit = 55;
constexpr const int DefaultZTorqueLimit = 30;

std::string ROBOSIGNALSHARED_EXPORT CoordsToString(const Coords& coords);
std::string ROBOSIGNALSHARED_EXPORT AnglesToString(const Angles& angles);
Coords ROBOSIGNALSHARED_EXPORT StringToCoords(const std::string& coords_str);
Angles ROBOSIGNALSHARED_EXPORT StringToAngles(const std::string& angles_str);
std::string ROBOSIGNALSHARED_EXPORT CoordsToGcode(const Coords& coords);
std::string ROBOSIGNALSHARED_EXPORT AnglesToGcode(const Angles& angles);
Angles ROBOSIGNALSHARED_EXPORT GcodeToAngles(const QString& gcode);
Coords ROBOSIGNALSHARED_EXPORT InvalidCoords();
Angles ROBOSIGNALSHARED_EXPORT InvalidAngles();
}
Q_DECLARE_METATYPE(rc::Coords)
Q_DECLARE_METATYPE(rc::PortsInfo)
#endif
