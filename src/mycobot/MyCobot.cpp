#include "mycobot/MyCobot.hpp"

#include <system_error>

#include <QCoreApplication>

#include "MyCobot.hpp"

namespace mycobot {

class MYCOBOTCPP_LOCAL MyCobotImpl{};

MyCobot MyCobot::I()
{
    static MyCobot singleton{};
    if (!singleton.impl) {
        auto impl = std::make_shared<MyCobotImpl>();
        int ret = rc::MyCobot::Instance().Init();
        if (ret != 0) {
            throw std::error_code(5, std::generic_category());
        }
        if (!rc::MyCobot::Instance().IsCncConnected()) {
            throw std::error_code(4, std::generic_category());
        }
        rc::MyCobot::Instance().InitRobot();
        singleton.impl = impl;
    }
    QCoreApplication::processEvents();
    return singleton;
}

void MyCobot::PowerOn()
{
    rc::MyCobot::Instance().PowerOn();
    QCoreApplication::processEvents();
}

void MyCobot::PowerOff()
{
    rc::MyCobot::Instance().PowerOff();
    QCoreApplication::processEvents();
}

bool MyCobot::IsControllerConnected() const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().IsCncConnected();
}

void MyCobot::StopRobot()
{
    rc::MyCobot::Instance().TaskStop();
    QCoreApplication::processEvents();
}

bool MyCobot::IsInPosition(const Coords& coords, bool is_linear) const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().IsInPosition(coords, is_linear);
}

bool MyCobot::IsMoving() const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().CheckRunning();
}

int MyCobot::GetSpeed() const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().GetSpeed();
}

void MyCobot::SetSpeed(int percentage)
{
    rc::MyCobot::Instance().SetSpeed(percentage);
    QCoreApplication::processEvents();
}

double MyCobot::GetJointMin(Joint joint) const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().GetJointMinPosLimit(static_cast<rc::Joint>(joint));
}

double MyCobot::GetJointMax(Joint joint) const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().GetJointMaxPosLimit(static_cast<rc::Joint>(joint));
}

void MyCobot::SetFreeMoveMode(bool free_move)
{
    rc::MyCobot::Instance().SetFreeMove(free_move);
    QCoreApplication::processEvents();
}

bool MyCobot::IsFreeMoveMode() const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().IsSoftwareFreeMove();
}

Angles MyCobot::GetAngles() const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().GetAngles();
}

void MyCobot::WriteAngles(const Angles& angles, int speed)
{
    rc::MyCobot::Instance().WriteAngles(angles, speed);
    QCoreApplication::processEvents();
}

void MyCobot::WriteAngle(Joint joint, double value, int speed)
{
    rc::MyCobot::Instance().WriteAngle(static_cast<rc::Joint>(joint), value, speed);
    QCoreApplication::processEvents();
}

Coords MyCobot::GetCoords() const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().GetCoords();
}

void MyCobot::WriteCoords(const Coords& coords, int speed)
{
    rc::MyCobot::Instance().WriteCoords(coords, speed);
    QCoreApplication::processEvents();
}

void MyCobot::WriteCoord(Axis axis, double value, int speed)
{
    rc::MyCobot::Instance().WriteCoord(static_cast<rc::Axis>(axis), value, speed);
    QCoreApplication::processEvents();
}

void MyCobot::JogCoord(Axis axis, int direction, int speed)
{
    rc::MyCobot::Instance().JogCoord(static_cast<rc::Axis>(axis), direction, speed);
    QCoreApplication::processEvents();
}

void MyCobot::JogAngle(Joint joint, int direction, int speed)
{
    rc::MyCobot::Instance().JogAngle(static_cast<rc::Joint>(joint), direction, speed);
    QCoreApplication::processEvents();
}

void MyCobot::JogCoordAbsolute(Axis axis, double value, int speed)
{
    rc::MyCobot::Instance().WriteCoord(static_cast<rc::Axis>(axis), value, speed);
    QCoreApplication::processEvents();
}

void MyCobot::JogAngleAbsolute(Joint joint, double value, int speed)
{
    rc::MyCobot::Instance().WriteAngle(static_cast<rc::Joint>(joint), value, speed);
    QCoreApplication::processEvents();
}

void MyCobot::JogCoordIncrement(Axis axis, double increment, int speed)
{
    rc::MyCobot::Instance().SendJogIncrement(axis, speed, increment, 0);
    QCoreApplication::processEvents();
}

void MyCobot::JogAngleIncrement(Joint joint, double increment, int speed)
{
    rc::MyCobot::Instance().SendJogIncrement(joint, speed, increment, 1);
    QCoreApplication::processEvents();
}

int MyCobot::GetBasicIn(int pin_number) const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().GetBasicIn(pin_number);
}
void MyCobot::SetBasicOut(int pin_number, int pin_signal)
{
    rc::MyCobot::Instance().SetBasicOut(pin_number, pin_signal);
    QCoreApplication::processEvents();
}
int MyCobot::GetDigitalIn(int pin_number) const
{
    QCoreApplication::processEvents();
    return rc::MyCobot::Instance().GetDigitalIn(pin_number);
}
void MyCobot::SetDigitalOut(int pin_number, int pin_signal)
{
    rc::MyCobot::Instance().SetDigitalOut(pin_number, pin_signal);
    QCoreApplication::processEvents();
}

void MyCobot::SetGriper(int open)
{
    if (open == 1)
        rc::MyCobot::Instance().SetEncoder(6, 2048);
    else
        rc::MyCobot::Instance().SetEncoder(6, 1500);
    QCoreApplication::processEvents();
}
void MyCobot::SetElectricGriper(int open)
{
    if (open == 1) {
        rc::MyCobot::Instance().SerialWrite(QByteArray("\x01\x06\x01\x03\x03\xE8\x78\x88", 8));
    } else if (open == 0) {
        rc::MyCobot::Instance().SerialWrite(QByteArray("\x01\x06\x01\x03\x00\x00\x78\x36", 8));
    }
    QCoreApplication::processEvents();
}

void MyCobot::SleepSecond(unsigned time)
{
#if defined WIN32 
    Sleep(time*1000);//ms
#else
    sleep(time);//s
#endif
}

}
