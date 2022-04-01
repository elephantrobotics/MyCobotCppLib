#include "MyCobot.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>

#include <qeventloop.h>
#include <qnamespace.h>
#include <QCoreApplication>
#include <QDebug>
#include <QStringList>
#include <QThread>
#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>

#include "Common.hpp"
#include "Firmata.hpp"
#include "SystemInfo.hpp"
#define log_category ::rc::log::robot_controller
#include "log/Log.hpp"

namespace rc {

    constexpr const double EMC_COMMAND_DELAY = 0.1;

    enum class ROBOT_STATE {
        COORDS_STATE,
        ANGLES_STATE,
        RUN_PROGRAME_STATE,
        OTHER_STATE,
    };

    std::list<std::pair<int, std::function<void()>>> MyCobot::commands{};
    std::mutex MyCobot::serial_data_mutex{};
    std::mutex MyCobot::command_mutex{};
    std::atomic<bool> MyCobot::drop_all_commands{ false };

    MyCobot::~MyCobot()
    {
        if (command_thread) {
            command_thread_running = false;
            command_thread->quit();
            command_thread->wait();
        }
    }

    MyCobot& MyCobot::Instance()
    {
        static MyCobot singleton;
        return singleton;
    }

    int MyCobot::Init()
    {
        InitFirmata();
        int ret = InitSerialPort();
        if (ret) {
            return ret;
        }

        DetectRobot();
        if (command_thread == nullptr) {
            command_thread_running = true;
            command_thread = QThread::create(&MyCobot::CommandThreadWorker, this);
            command_thread->start();
        }
        return 0;
    }

    bool MyCobot::IsCncConnected()
    {
        return serial_port->isOpen();
    }

    void MyCobot::InitRobot()
    {
        LogTrace;
        InitIo();
        //FIXME: for test, power on is in settings
        StateOn();
        // Init Cache
        GetAngles();
        GetCoords();
        //GetEncoders();//if not,when first trajectory,will return 0.
        //set io
        //InitIo();
    }

    void MyCobot::DetectRobot()
    {
        LogTrace;
        current_robot = 4;
    }

    void MyCobot::EnableOutPinCache(bool enable)
    {
    }

    /**
     * High-level Interface follows
     */

    bool MyCobot::PowerOn()
    {
        for (int i = 0; i < 6; i++) {
            FocusServo(static_cast<Joint>(i));
        }
        return true;
    }

    bool MyCobot::PowerOnOnly()
    {
        LogDebug << "PowerOnOnly() => PowerOn()";
        return PowerOn();
    }

    bool MyCobot::PowerOff()
    {
        ReleaseAllServos();
        return true;
    }

    Coords MyCobot::GetCoords() const
    {
        // LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::GetCoords)) {
            std::function<void()> f = [this]() {
                SerialWrite(CommandGetCoords);
            };
            command_bits.set(Command::GetCoords);
            commands.push_back(std::make_pair(Command::GetCoords, f));
        }
        return cur_coords;
    }

    void MyCobot::WriteCoords(const Coords& coords, int speed)
    {
        //LogTrace << "(" << coords << ", " << speed << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        UpdateCacheInChangePositionFunction();
        std::function<void()> f =
            [this, coords, speed]() {
            UpdateCacheInChangePositionFunction();
            QByteArray command(CommandWriteCoords);
            for (int i = 0; i < 3; ++i) {
                //xyz计算方式：高：坐标乘以10 再取十六进制的高字节 低：坐标乘以10 再取十六进制的低字节
                signed short tenth_mm = static_cast<signed short>(coords[i] * 10);
                command += static_cast<char>(tenth_mm >> 8);
                command += static_cast<char>(tenth_mm & 0x00FF);
            }
            for (int i = 3; i < rc::Axes; ++i) {
                //xyz计算方式：高：坐标乘以100 再取十六进制的高字节 低：坐标乘以100 再取十六进制的低字节
                signed short centi_deg = static_cast<signed short>(coords[i] * 100);
                command += static_cast<char>(centi_deg >> 8);
                command += static_cast<char>(centi_deg & 0x00FF);
            }
            command += static_cast<char>(speed * 100 / MaxLinearSpeed);
            command += static_cast<char>(2);
            command += FIRMATA_FOOTER;
            // LogTrace << "WriteCoords(" << coords << ", " << speed << ")";
            SerialWrite(command);
            LogInfo << "WriteCoords(" << coords << ", " << speed << ")";
        };
        command_bits.set(Command::WriteCoords);
        commands.remove_if(
            [](std::pair<int, std::function<void()>> p) {
                return p.first == Command::WriteCoords || p.first == Command::IsInPosition;
            });
        command_bits.reset(Command::IsInPosition);
        commands.push_back(std::make_pair(Command::WriteCoords, f));
    }

    void MyCobot::WriteCoord(Axis axis, double value, int speed)
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        UpdateCacheInChangePositionFunction();
        if (!command_bits.test(Command::WriteCoord)) {
            std::function<void()> f =
                [this, axis, value, speed]() {
                UpdateCacheInChangePositionFunction();
                QByteArray command(CommandWriteCoord);
                command += char(axis);
                signed short tenth_mm = static_cast<signed short>(value * 10);
                command += static_cast<char>(tenth_mm >> 8);
                command += static_cast<char>(tenth_mm & 0x00FF);
                command += char(speed * 100 / MaxLinearSpeed);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::WriteCoord);
            commands.remove_if(
                [](std::pair<int, std::function<void()>> p) {
                    return p.first == Command::WriteCoord || p.first == Command::IsInPosition;
                });
            command_bits.reset(Command::IsInPosition);
            commands.push_back(std::make_pair(Command::WriteCoord, f));
        }
    }

    Angles MyCobot::GetAngles() const
    {
        // LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::GetAngles)) {
            std::function<void()> f = [this]() {
                SerialWrite(CommandGetAngles);
                // QByteArray command(CommandSetLedRgb);
                // command += static_cast<char>(255);
                // command += static_cast<char>(255);
                // command += static_cast<char>(0);
                // command += FIRMATA_FOOTER;
                // SerialWrite(command);
            };
            command_bits.set(Command::GetAngles);
            commands.push_back(std::make_pair(Command::GetAngles, f));
        }
        return cur_angles;
    }

    void MyCobot::WriteAngles(const Angles& angles, int speed)
    {
        // LogTrace << "(" << angles << ", " << speed << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        UpdateCacheInChangePositionFunction();
        std::function<void()> f =
            [this, angles, speed]() {
            UpdateCacheInChangePositionFunction();
            QByteArray command(CommandWriteAngles);
            for (const double& angle : angles) {
                signed short centi_deg = static_cast<signed short>(angle * 100);
                command += static_cast<char>(centi_deg >> 8);
                command += static_cast<char>(centi_deg & 0x00FF);
            }
            command += static_cast<char>(speed * 100 / MaxAngleSpeed);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        command_bits.set(Command::WriteAngles);
        commands.remove_if(
            [](std::pair<int, std::function<void()>> p) {
                return p.first == Command::WriteAngles || p.first == Command::IsInPosition;
            });
        command_bits.reset(Command::IsInPosition);
        commands.push_back(std::make_pair(Command::WriteAngles, f));
    }

    void MyCobot::WriteAngle(Joint joint, double angle, int speed)
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        UpdateCacheInChangePositionFunction();
        if (!command_bits.test(Command::WriteAngle)) {
            std::function<void()> f =
                [this, joint, angle, speed]() {
                UpdateCacheInChangePositionFunction();
                QByteArray command(CommandWriteAngle);
                command += char(joint);
                signed short centi_deg = static_cast<signed short>(angle * 100);
                command += static_cast<char>(centi_deg >> 8);
                command += static_cast<char>(centi_deg & 0x00FF);
                command += static_cast<char>(speed * 100 / MaxAngleSpeed);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::WriteAngle);
            commands.remove_if(
                [](std::pair<int, std::function<void()>> p) {
                    return p.first == Command::WriteAngle || p.first == Command::IsInPosition;
                });
            command_bits.reset(Command::IsInPosition);
            commands.push_back(std::make_pair(Command::WriteAngle, f));
        }
    }

    double MyCobot::GetSpeed() const
    {
        LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::GetSpeed)) {
            std::function<void()> f = [this]() {
                SerialWrite(CommandGetSpeed);
            };
            command_bits.set(Command::GetSpeed);
            commands.push_back(std::make_pair(Command::GetSpeed, f));
        }
        return cur_speed;
    }

    void MyCobot::SetSpeed(int percentage)
    {
        LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this, percentage]() {
            QByteArray command(CommandSetSpeed);
            command += static_cast<char>(percentage);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        commands.push_back(std::make_pair(Command::GetSpeed, f));
    }

    int MyCobot::StateOn()
    {
        LogTrace;
        return InitSerialPort();
        //return ReconnectCurrentSerialPort();
    }

    int MyCobot::StateOff()
    {
        LogTrace;
        serial_port->close();
        return 0;
    }

    bool MyCobot::StateCheck()
    {
        bool state = false;
#if defined ROBCTL_ATOMMAIN
        //return (IsCncConnected() && IsPowerOn());
        state = IsCncConnected() && IsPowerOn();
#elif defined ROBCTL_PHOENIX
        //return IsCncConnected();
        state = IsCncConnected();
#endif
        return state;
    }

    int MyCobot::TaskStop()
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::TaskStop)) {
            std::function<void()> f = [this]() {
                LogTrace << ": TaskStop";
                SerialWrite(CommandTaskStop);
            };
            command_bits.set(Command::TaskStop);
            commands.push_back(std::make_pair(Command::TaskStop, f));
        }
        return 0;
    }

    void MyCobot::Wait(double time)
    {
        LogTrace << "(" << time << ")";
        QTime finish_time = QTime::currentTime().addMSecs(static_cast<int>(time * 1000));
        while (QTime::currentTime() < finish_time) {
            QCoreApplication::processEvents();
        }
    }

    void MyCobot::SetCarteTorqueLimit(Axis axis, double value)
    {
        // LogTrace << "(" << axis << ", " << value << ") -> NOT USED IN THIS ROBOT";
    }

    void MyCobot::JogAngle(Joint joint, int direction, int speed)
    {
        LogTrace << "(joint=" << joint << ", direction=" << direction << ", speed=" << speed << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        UpdateCacheInChangePositionFunction();
        LogInfo << "jogangle1";
        if (!command_bits.test(Command::JogAngle)) {
            std::function<void()> f = [this, joint, direction, speed]() {
                LogInfo << "jogangle2";
                QByteArray command(CommandJogAngle);
                LogInfo << "jogangle3";
                command += char(joint + 1);
                command += char((direction < 0) ? 0 : 1);
                command += char(speed * 100 / MaxAngleSpeed);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::JogAngle);
            commands.push_back(std::make_pair(Command::JogAngle, f));
        }
    }

    void MyCobot::JogAbsolute(int joint_or_axis, double speed, double pos)
    {
        LogTrace << "(joint_or_axis=" << joint_or_axis << ", speed=" << speed << ", pos=" << pos << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        UpdateCacheInChangePositionFunction();
        if (!command_bits.test(Command::JogAbsolute)) {
            std::function<void()> f = [this, joint_or_axis, speed, pos]() {
                QByteArray command(CommandJogAbsolute);
                command += char(joint_or_axis);
                signed short centi_deg = static_cast<signed short>(pos * 100);
                command += static_cast<char>(centi_deg >> 8);
                command += static_cast<char>(centi_deg & 0x00FF);
                command += static_cast<char>(speed * 100 / MaxAngleSpeed);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::JogAbsolute);
            commands.push_back(std::make_pair(Command::JogAbsolute, f));
        }
    }

    void MyCobot::JogCoord(Axis axis, int direction, int speed)
    {
        // JogCoord does not work good in Atom
        LogTrace << ": Disabled for Atom";
    }

    int MyCobot::JogStop(int joint_or_axis, int jog_mode)
    {
        LogTrace << "(joint_or_axis=" << joint_or_axis << ", jog_mode=" << jog_mode << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::JogStop)) {
            std::function<void()> f = [this]() {
                SerialWrite(CommandJogStop);
            };
            command_bits.set(Command::JogStop);
            commands.push_back(std::make_pair(Command::JogStop, f));
        }
        return 0;
    }

    bool MyCobot::CheckRunning() const
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::CheckRunning)) {
            //LogInfo << "check running";
            std::function<void()> f = [this]() {
                SerialWrite(CommandCheckRunning);
            };
            command_bits.set(Command::CheckRunning);
            commands.push_back(std::make_pair(Command::CheckRunning, f));
        }
        return robot_is_moving;
    }

    int MyCobot::IsInPositionEncoders(const Angles& encoders) const
    {
        //GetEncoders();
        is_in_position = rc::CoordsEqual(cur_encoders, encoders, EncodersEpsilon);
        return is_in_position;
    }

    int MyCobot::IsInPosition(const Coords& coords, bool is_linear) const
    {
        // LogTrace << "(" << coords << ", is_linear=" << is_linear << ")";
#if defined IS_IN_POSITION_USE_COMMAND
        const std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::IsInPosition)) {
            std::function<void()> f = [this, coords, is_linear]() {
                QByteArray command(CommandIsInPosition);
                if (is_linear) {
                    for (int i = 0; i < 3; ++i) {
                        signed short tenth_mm = static_cast<signed short>(coords[i] * 10);
                        command += static_cast<char>(tenth_mm >> 8);
                        command += static_cast<char>(tenth_mm & 0x00FF);
                    }
                    for (int i = 3; i < rc::Axes; ++i) {
                        //计算方式：rxyz坐标乘以100 先转换为int类型 再取十六进制高低字节
                        signed short centi_deg = static_cast<signed short>(coords[i] * 100);
                        command += static_cast<char>(centi_deg >> 8);
                        command += static_cast<char>(centi_deg & 0x00FF);
                    }
                }
                else {
                    for (const double& angle : coords) {
                        signed short centi_deg = static_cast<signed short>(angle * 100);
                        command += static_cast<char>(centi_deg >> 8);
                        command += static_cast<char>(centi_deg & 0x00FF);
                    }
                }
                command += static_cast<char>(is_linear);
                command += FIRMATA_FOOTER;
                // LogTrace << "IsInPosition(" << coords << ", " << is_linear << ")";
                SerialWrite(command);
            };
            command_bits.set(Command::IsInPosition);
            commands.push_back(std::make_pair(Command::IsInPosition, f));
        }
#else
        if (is_linear) {
            is_in_position = rc::CoordsEqual(cur_coords, coords);
        }
        else {
            is_in_position = rc::CoordsEqual(cur_angles, coords);
        }
#endif
        if (is_in_position) {
            // LogTrace << " -> true";
        }
        return is_in_position;
    }

    void MyCobot::FocusServo(Joint joint)
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this, joint]() {
            QByteArray command(CommandFocusServo);
            command += char(joint + 1);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        command_bits.set(Command::FocusServo);
        commands.push_back(std::make_pair(Command::FocusServo, f));
    }

    bool MyCobot::GetServoData(Joint joint, int data_id)
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::GetServoData)) {
            std::function<void()> f = [this, joint, data_id]() {
                QByteArray command(CommandGetServoData);
                command += char(joint + 1);
                command += char(data_id);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::GetServoData);
            commands.push_back(std::make_pair(Command::GetServoData, f));
        }
        return is_power_on;
    }

    void MyCobot::ReleaseAllServos()
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::ReleaseAllServos)) {
            std::function<void()> f = [this]() {
                QByteArray command(CommandReleaseAllServos);
                SerialWrite(command);
            };
            command_bits.set(Command::ReleaseAllServos);
            commands.push_back(std::make_pair(Command::ReleaseAllServos, f));
        }
    }

    void MyCobot::SetFreeMove(bool on)
    {
        LogTrace << "(" << on << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::SetFreeMoveMode)) {
            std::function<void()> f = [this, on]() {
                QByteArray command(CommandSetFreeMoveMode);
                command += static_cast<char>(on);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::SetFreeMoveMode);
            commands.push_back(std::make_pair(Command::SetFreeMoveMode, f));
        }

    }

    bool MyCobot::IsSoftwareFreeMove() const
    {
        // LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::IsFreeMoveMode)) {
            std::function<void()> f = [this]() {
                SerialWrite(CommandIsFreeMoveMode);
            };
            command_bits.set(Command::IsFreeMoveMode);
            commands.push_back(std::make_pair(Command::IsFreeMoveMode, f));
        }
        return is_free_move;
    }

    bool MyCobot::IsHardwareFreeMove() const
    {
        return IsSoftwareFreeMove();
    }

    int MyCobot::SetPayload(double payload)
    {
        LogTrace << "(" << payload << ") -> NOT USED IN THIS ROBOT";
        return 0;
    }

    bool MyCobot::SetUpsideDown(bool new_upside_down)
    {
        LogTrace << "(" << new_upside_down << ") -> NOT USED IN THIS ROBOT";
        return true;
    }

    int MyCobot::ProgramOpen(const std::string& program_file_path)
    {
        LogTrace << "(\"" << program_file_path.c_str() << "\")";
        //TODO
        return 0;
    }

    int MyCobot::ProgramRun(int start_line)
    {
        LogTrace << "(start_line=" << start_line << ")";
        //TODO
        return 0;
    }

    bool MyCobot::ProgramRunFinished()
    {
        LogTrace;
        //TODO
        return true;
    }

    int MyCobot::ProgramPause()
    {
        LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::ProgramPause)) {
            std::function<void()> f = [this]() {
                SerialWrite(CommandProgramPause);
            };
            command_bits.set(Command::ProgramPause);
            commands.push_back(std::make_pair(Command::ProgramPause, f));
        }
        return 0;
    }

    bool MyCobot::IsProgramPaused() const
    {
        LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::IsProgramPaused)) {
            std::function<void()> f = [this]() {
                SerialWrite(CommandIsProgramPaused);
            };
            command_bits.set(Command::IsProgramPaused);
            commands.push_back(std::make_pair(Command::IsProgramPaused, f));
        }
        return is_program_paused;
    }

    int MyCobot::ProgramResume()
    {
        LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::ProgramResume)) {
            std::function<void()> f = [this]() {
                SerialWrite(CommandProgramResume);
            };
            command_bits.set(Command::ProgramResume);
            commands.push_back(std::make_pair(Command::ProgramResume, f));
        }
        return 0;
    }

    int MyCobot::ReadNextError(std::string& error_string)
    {
        error_string = next_error;
        next_error = "";
        return 0;
    }

    int MyCobot::IsPowerOn()
    {
        is_powered_on = true;
        for (int i = 0; i < 6; i++) {
            if (!GetServoData(static_cast<Joint>(i), 40)) {
                is_powered_on = false;
                break;
            }
        }
        /*
        //LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::IsPoweredOn)) {
            std::function<void()> f = [this] () {
                SerialWrite(CommandIsPoweredOn);
            };
            command_bits.set(Command::IsPoweredOn);
            commands.push_back(std::make_pair(Command::IsPoweredOn, f));
        }*/
        return is_powered_on;
    }

    bool MyCobot::IsControllerConnected() {
        //LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this]() {
            QByteArray command(CommandIsControllerConnected);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        command_bits.set(Command::IsControllerConnected);
        commands.push_back(std::make_pair(Command::IsControllerConnected, f));
        return is_controller_connected;
    }

    bool MyCobot::IsServoEnabled(Joint j) const
    {
        //LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this, j]() {
            QByteArray command(CommandIsServoEnabled);
            command += static_cast<char>(j + 1);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        command_bits.set(Command::IsServoEnabled);
        commands.push_back(std::make_pair(Command::IsServoEnabled, f));
        return servo_enabled[j];
    }

    bool MyCobot::IsAllServoEnabled() const
    {
        LogTrace;
        constexpr static const bool use_command{ true };
        if (use_command) {
            std::lock_guard<std::mutex> lock(command_mutex);
            if (!command_bits.test(Command::IsAllServoEnabled)) {
                std::function<void()> f = [this]() {
                    SerialWrite(CommandIsAllServoEnabled);
                };
                command_bits.set(Command::IsAllServoEnabled);
                commands.push_back(std::make_pair(Command::IsAllServoEnabled, f));
            }
        }
        else {
            // calculate from cached servo_enabled
            is_all_servo_enabled = true;
            for (const bool& enabled : servo_enabled) {
                is_all_servo_enabled = is_all_servo_enabled & enabled;
            }
        }
        return is_all_servo_enabled;
    }

    double MyCobot::GetJointMinPosLimit(Joint j) const
    {
        LogTrace << "(" << j << ") -> ";
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::GetJointMin)) {
            std::function<void()> f = [this, j]() {
                QByteArray command(CommandGetJointMin);
                command += static_cast<char>(j);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::GetJointMin);
            commands.push_back(std::make_pair(Command::GetJointMin, f));
        }
        return joint_min[j];
    }

    int MyCobot::SetJointMinPosLimit(Joint joint, double angle_limit)
    {
        LogTrace << "(" << joint << ", " << angle_limit << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::SetJointMin)) {
            std::function<void()> f = [this, joint, angle_limit]() {
                QByteArray command(CommandSetJointMin);
                command += static_cast<char>(joint);
                //计算方式：角度值乘以10 先转换成int形式 再取十六进制的高字节
                signed short deci_deg = static_cast<signed short>(angle_limit * 10);
                command += static_cast<char>(deci_deg >> 8);
                command += static_cast<char>(deci_deg & 0x00FF);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::SetJointMin);
            commands.push_back(std::make_pair(Command::SetJointMin, f));
        }
        return 0;
    }

    double MyCobot::GetJointMaxPosLimit(Joint j) const
    {
        LogTrace << "(" << j << ") -> ";
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::GetJointMax)) {
            std::function<void()> f = [this, j]() {
                QByteArray command(CommandGetJointMax);
                command += static_cast<char>(j);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::GetJointMax);
            commands.push_back(std::make_pair(Command::GetJointMax, f));
        }
        return joint_max[j];
    }

    int MyCobot::SetJointMaxPosLimit(Joint joint, double angle_limit)
    {
        LogTrace << "(" << joint << ", " << angle_limit << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::SetJointMax)) {
            std::function<void()> f = [this, joint, angle_limit]() {
                QByteArray command(CommandSetJointMax);
                command += static_cast<char>(joint);
                //计算方式：角度值乘以10 先转换成int形式 再取十六进制的高字节
                signed short deci_deg = static_cast<signed short>(angle_limit * 10);
                command += static_cast<char>(deci_deg >> 8);
                command += static_cast<char>(deci_deg & 0x00FF);
                command += FIRMATA_FOOTER;
                SerialWrite(command);
            };
            command_bits.set(Command::SetJointMax);
            commands.push_back(std::make_pair(Command::SetJointMax, f));
        }
        return 0;
    }

    int MyCobot::SetJointMaxVelocity(Joint joint, double limit)
    {
        LogTrace << "(" << joint << ", " << limit << ")";
        return 0;
    }

    void MyCobot::SetEncoders(Angles encoders, int speed)
    {
        LogTrace << "(SetEncoders=" << encoders << "speed = " << speed;
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this, encoders, speed]() {
            LogInfo << "into setencoders";
            QByteArray command(CommandSetEncoders);
            for (const double& angle_encode : encoders) {
                signed short centi_deg = static_cast<signed short>(angle_encode);
                command += static_cast<char>(centi_deg >> 8);
                command += static_cast<char>(centi_deg & 0x00FF);
            }
            command += speed;
            command += FIRMATA_FOOTER;
            SerialWrite(command);
            LogInfo << "serialwrite setEncoders";
        };
        command_bits.set(Command::SetEncoders);
        commands.push_back(std::make_pair(Command::SetEncoders, f));
    }

    Angles MyCobot::GetEncoders() const
    {

        std::lock_guard<std::mutex> lock(command_mutex);
        if (!command_bits.test(Command::GetEncoders)) {
            std::function<void()> f = [this] {
                SerialWrite(CommandGetEncoders);
            };
            command_bits.set(Command::GetEncoders);
            commands.remove_if(
                [](std::pair<int, std::function<void()>> p) {
                    return p.first == Command::GetEncoders || p.first == Command::CheckRunning || p.first == Command::IsFreeMoveMode;
                });
            command_bits.reset(Command::CheckRunning);
            command_bits.reset(Command::IsFreeMoveMode);
            commands.push_back(std::make_pair(Command::GetEncoders, f));
        }
        return cur_encoders;
    }

    void MyCobot::SetEncoder(int joint, int val)
    {
        LogTrace << "joint:" << joint << "encoder:" << val;
        if (joint < 0 || joint >= MaxEncoders)
            return;
        LogTrace << "(joint=" << joint << ", Encoder=" << val << ")";
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this, joint, val]() {
            QByteArray command(CommandSetEncoder);
            command += char(joint + 1);
            signed short encoder = static_cast<signed short>(val);
            command += static_cast<char>(encoder >> 8);
            command += static_cast<char>(encoder & 0x00FF);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        command_bits.set(Command::SetEncoder);
        commands.push_back(std::make_pair(Command::SetEncoder, f));
    }

    int MyCobot::EmcCommandWaitDone()
    {
        return -1;
    }

    PHOENIX_TASK_MODE_ENUM MyCobot::GetCurrentCncMode() const
    {
        LogTrace;
        return PHOENIX_TASK_MODE_ENUM::PHOENIX_TASK_MODE_MANUAL;
    }

    int MyCobot::SendFeedOverride(double override)
    {
        LogTrace << "(" << override << ")";
        return 0;
    }

    int MyCobot::GetFeedOverride()
    {
        return 0;
    }

    int MyCobot::GetAcceleration()
    {
        LogTrace;
        return static_cast<int>(GetAnalogOut(AO::Acceleration));
    }

    int MyCobot::SendMode(PHOENIX_TASK_MODE_ENUM mode)
    {
        LogTrace << "(" << mode << ")";
        return 0;
    }

    void MyCobot::JointBrake(Joint joint, int release)
    {
        LogTrace << "(joint=" << joint << ", release=" << release << ")";
    }

    bool MyCobot::IsCncInMdiMode() const
    {
        return false;
    }

    void MyCobot::SetCncInMdiMode()
    {
        LogTrace;
    }

    int MyCobot::GetCurrentLine() const
    {
        LogTrace;
        return 0;
    }

    int MyCobot::GetMotionLine() const
    {
        LogTrace;
        return 0;
    }

    int MyCobot::SendTeleop(int enable)
    {
        LogTrace << "(" << enable << ")";
        return 0;
    }

    int MyCobot::GetRobotStatus()
    {
        return StateCheck();
    }

    double MyCobot::GetRobotTemperature()
    {
        return rc::SystemInfo::GetCpuTemperature();
    }

    double MyCobot::GetRobotPower()
    {
        return 0;
    }

    int MyCobot::GetJointState(Joint joint_num)
    {
        uint32_t status_word = 2;
        if (status_word & (1 << 1)) {
            if (status_word & (1 << 3)) {
                return JointState::Error;
            }
            return JointState::OK;
        }
        return JointState::PoweredOff;
    }

    double MyCobot::GetJointTemperature(Joint joint_num)
    {
        return GetAnalogIn(joint_num + AI::J1Temperature);
    }

    int MyCobot::GetJointCommunication(Joint joint_num)
    {
        return !IsServoEnabled(joint_num);
    }

    double MyCobot::GetJointVoltage(Joint joint_num)
    {
        return GetAnalogIn(joint_num + AI::J1Voltage);
    }

    double MyCobot::GetJointCurrent(Joint joint_num)
    {
        return GetAnalogIn(joint_num + AI::J1WindingACurrent);
    }

    uint32_t MyCobot::GetJointErrorMask(Joint joint_num)
    {
        return 0;
    }

    void MyCobot::SetPowerLimit(int power_limit)
    {
        SetAnalogOut(AO::PowerLimit, power_limit);
    }

    int MyCobot::GetPowerLimit() const
    {
        return static_cast<int>(GetAnalogOut(AO::PowerLimit));
    }

    void MyCobot::SetStoppingTime(int stopping_time)
    {
        SetAnalogOut(AO::StoppingTime, stopping_time);
    }

    int MyCobot::GetStoppingTime() const
    {
        return static_cast<int>(GetAnalogOut(AO::StoppingTime));
    }

    void MyCobot::SetStoppingDistance(int stopping_distance)
    {
        SetAnalogOut(AO::StoppingDistance, stopping_distance);
    }

    int MyCobot::GetStoppingDistance() const
    {
        return static_cast<int>(GetAnalogOut(AO::StoppingDistance));
    }

    void MyCobot::SetToolSpeed(int tool_speed)
    {
        SetAnalogOut(AO::ToolSpeed, tool_speed);
    }

    int MyCobot::GetToolSpeed() const
    {
        return static_cast<int>(GetAnalogOut(AO::ToolSpeed));
    }

    void MyCobot::SetToolForce(int tool_force)
    {
        SetAnalogOut(AO::ToolForce, tool_force);
    }

    int MyCobot::GetToolForce() const
    {
        return static_cast<int>(GetAnalogOut(AO::ToolForce));
    }

    void MyCobot::StartForceSensor(char* device, int speed)
    {
    }

    void MyCobot::StopForceSensor()
    {
    }

    void MyCobot::SetMotionFlexible(uint32_t flexible)
    {
        LogTrace << "(" << flexible << ")";
    }

    int MyCobot::SendMdiCmd(const std::string& gcode)
    {
        return 0;
    }

    int MyCobot::GetDigitalIn(int pin_number) const
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this, pin_number]() {
            QByteArray command(CommandGetDigitalIn);
            command += char(pin_number);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        command_bits.set(Command::GetDigitalIn);
        commands.push_back(std::make_pair(Command::GetDigitalIn, f));
        return atom_digital_in[pin_number];
    }

    int MyCobot::GetDigitalOut(int pin_number) const
    {
        return 0;
    }

    void MyCobot::SetPinMode(int pin_number, int pin_mode)
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f =
            [this, pin_number, pin_mode]() {
            LogTrace << "SetPinMode(pin=" << pin_number << ", value=" << pin_mode << ")";
            QByteArray command(CommandSetPinMode);
            command += char(pin_number);
            command += char(pin_mode);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        command_bits.set(Command::SetPinMode);
        commands.remove_if(
            [](std::pair<int, std::function<void()>> p) {
                return p.first == Command::SetPinMode || p.first == Command::GetBasicIn || p.first == Command::GetDigitalIn
                    || p.first == Command::CheckRunning || p.first == Command::GetAngles || p.first == Command::GetCoords
                    || p.first == Command::SetJointMax || p.first == Command::SetJointMin;
            });
        command_bits.reset(Command::SetJointMax);
        command_bits.reset(Command::SetJointMin);
        command_bits.reset(Command::CheckRunning);
        command_bits.reset(Command::GetCoords);
        command_bits.reset(Command::GetAngles);
        command_bits.reset(Command::GetBasicIn);
        command_bits.reset(Command::GetDigitalIn);
        commands.push_back(std::make_pair(Command::SetPinMode, f));
    }

    int MyCobot::SetDigitalOut(int pin_number, int pin_signal)
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f =
            [this, pin_number, pin_signal]() {
            LogTrace << "(pin=" << pin_number << ", value=" << pin_signal << ")";
            QByteArray command(CommandSetDigitalOut);
            command += char(pin_number);
            command += char(pin_signal);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
            LogInfo << "SerialWrite SetDigitalOutPut";
        };
        command_bits.set(Command::SetDigitalOut);
        commands.remove_if(
            [](std::pair<int, std::function<void()>> p) {
                return p.first == Command::SetDigitalOut || p.first == Command::GetBasicIn || p.first == Command::GetDigitalIn;
            });
        command_bits.reset(Command::GetBasicIn);
        command_bits.reset(Command::GetDigitalIn);
        commands.push_back(std::make_pair(Command::SetDigitalOut, f));
        atom_digital_out[pin_number] = pin_signal;
        return 0;
    }

    int MyCobot::GetBasicIn(int pin_number) const
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this, pin_number]() {
            QByteArray command(CommandGetBasicIn);
            command += char(pin_number);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
        };
        command_bits.set(Command::GetBasicIn);
        commands.push_back(std::make_pair(Command::GetBasicIn, f));
        return basic_digital_in[pin_number];
    }

    int MyCobot::SetBasicOut(int pin_number, int pin_signal)
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f =
            [this, pin_number, pin_signal]() {
            LogTrace << "(pin=" << pin_number << ", value=" << pin_signal << ")";
            QByteArray command(CommandSetBasicOut);
            command += char(pin_number);
            command += char(pin_signal);
            command += FIRMATA_FOOTER;
            SerialWrite(command);
            LogInfo << "SerialWrite setbaiscoutput";
        };
        command_bits.set(Command::SetBasicOut);
        commands.remove_if(
            [](std::pair<int, std::function<void()>> p) {
                return p.first == Command::SetBasicOut || p.first == Command::GetBasicIn || p.first == Command::GetDigitalIn;
            });
        command_bits.reset(Command::GetBasicIn);
        command_bits.reset(Command::GetDigitalIn);
        commands.push_back(std::make_pair(Command::SetBasicOut, f));
        basic_digital_out[pin_number] = pin_signal;
        return 0;
    }

    void MyCobot::InitIo()
    {
        int s = (sizeof(out_pin_num) / sizeof(out_pin_num[0]));
        out_signal.resize(s);
        //set out low level
        for (int i = 0; i < s; i++) {
            if (i > 1) {
                //don't use SetDigitalOrBasic if digital will wait 500ms,here don't
                SetDigitalOut(out_pin_num[i], 0);
            }
            else {
                SetBasicOut(out_pin_num[i], 0);
            }
            if (i == 0 || i == 2) {
                Wait(0.2);
            }
        }
    }

    int MyCobot::GetDigtalOrBasic(int pin_number, bool is_out)
    {
        int signal = 0;
#if defined ROBOT_MYCOBOT
        //only atom should set pin mode 0-->input 1-->output
        if (pin_number > 1) {
            if (is_out) {
                signal = out_signal[pin_number];
            }
            else {
                signal = GetDigitalIn(in_pin_num[pin_number]);
            }
        }
        else {
#endif
            if (is_out) {
                signal = out_signal[pin_number];
            }
            else {
                signal = GetBasicIn(in_pin_num[pin_number]);
            }
#if defined ROBOT_MYCOBOT
        }
#endif
        return signal;
    }

    void MyCobot::SetDigitalOrBasic(int pin_number, int pin_signal)
    {
        out_signal[pin_number] = pin_signal;
#if defined ROBOT_MYCOBOTPRO
        //pro 0--high level  mycobot 1--high
        (!pin_signal) ? (pin_signal = 1) : (pin_signal = 0);
        SetBasicOut(out_pin_num[pin_number], pin_signal);
#endif
#if defined ROBOT_MYCOBOT
        if (pin_number > 1) {
            SetDigitalOut(out_pin_num[pin_number], pin_signal);
            Wait(0.5);
        }
        else {
            SetBasicOut(out_pin_num[pin_number], pin_signal);
        }
#endif
    }

    int MyCobot::SetGriper(int open)
    {
        LogTrace;
        std::lock_guard<std::mutex> lock(command_mutex);
        std::function<void()> f = [this, open]() {
            if (open == 1) {
                SerialWrite(CommandSetGriperOpen);
                LogTrace << CommandSetGriperOpen;
            }
            else {
                SerialWrite(CommandSetGriperClose);
                LogTrace << CommandSetGriperClose;
            }
        };
        commands.push_back(std::make_pair(Command::GripperMode, f));
        return 0;
    }

    double MyCobot::GetAnalogIn(int pin_number) const
    {
        return 0;
    }

    double MyCobot::GetAnalogOut(int pin_number) const
    {
        return 0;
    }

    int MyCobot::SetAnalogOut(int pin_number, double pin_value)
    {
        return 0;
    }

    void MyCobot::SetAcceleration(int acceleration)
    {
        // LogTrace << "(" << acceleration << ")";
    }

    void MyCobot::SendJogIncrement(int joint_or_axis, double speed, double incr, int jjogmode)
    {
        LogTrace << "(joint_or_axis=" << joint_or_axis << ", speed=" << speed << ", incr=" << incr
            << ", jjogmode=" << jjogmode << ")";
        //first judge joint or axis 0-->axis 1-->joint,if judge set angle,axis set coordinate
        Coords new_coords = cur_coords;
        Angles new_angles = cur_angles;
        if (jjogmode == 0) {
            new_coords[joint_or_axis] += incr;
            // XXX: WriteCoord doesn't work in Atom
            WriteCoords(new_coords, speed);
        }
        else {
            new_angles[joint_or_axis] += incr;
            // XXX: WriteAngle doesn't work in Atom
            WriteAngles(new_angles, speed);
        }
        while (!IsInPosition(jjogmode ? new_angles : new_coords, !jjogmode)) {
            QCoreApplication::processEvents();
        }
        TaskStop();
    }

    MyCobot::MyCobot()
    {
        LogTrace;
    }

    void MyCobot::SerialWrite(const QByteArray& data) const
    {
        // LogTrace << "(" << data.toHex(' ').toUpper() << ")";
        std::lock_guard<std::mutex> lock(serial_data_mutex);
        const qint64 bytes_written = serial_port->write(data);
        if (bytes_written == -1) {
            LogError << "Could not write data";
        }
        else if (bytes_written != data.size()) {
            LogError << "Failed to write all data";
        }
        serial_port->flush();
        //LogTrace << " " << bytes_written;
        //serial_timer.start(SERIAL_TIMEOUT);
    }

    int64_t MyCobot::GetCurrentTimeMs()
    {
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        return t.count();
    }

    bool MyCobot::IsSerialPort(int vid, int pid)
    {
        int len = sizeof(m5BasicId) / sizeof(m5BasicId[0]);
        for (int i = 0; i < len - 1; i += 2) {
            if ((vid == m5BasicId[i] && pid == m5BasicId[i + 1]))
                return true;//直接return
        }
        return false;
    }

    int MyCobot::InitSerialPort()
    {
        QString serial_port_name = "";
        int baud_rate = 0;
        if (serial_port == nullptr)
            serial_port = new QSerialPort(this);
        QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
        if (ports.size() == 0) {
            LogError << "Please Insert serial cable first!";
            return 1;
        }
        bool serial_port_found{ false };
        for (int i = 0; i < ports.size(); ++i) {
            LogInfo << "Detected serial port name: " << ports[i].portName();
            if (ports[i].portName() == SERIAL_PORT_PI) {
                serial_port_name = SERIAL_PORT_PI;
                baud_rate = 1000000;
                LogInfo << "Insert " << serial_port_name;
                serial_port_found = true;
                break;
            }
            if (IsSerialPort(ports[i].vendorIdentifier(), ports[i].productIdentifier())) {
                serial_port_name = ports[i].portName();
                baud_rate = QSerialPort::BaudRate::Baud115200;
                LogInfo << "Insert " << serial_port_name;
                serial_port_found = true;
                break;
            }
        }
        if (!serial_port_found) {
            LogError << "Serial device not found. Please, make sure robot is connected to corresponding port.";
            return 1;
        }
        serial_port->setPortName(serial_port_name);
        serial_port->setBaudRate(baud_rate);
        serial_port->open(QIODevice::OpenModeFlag::ReadWrite);

        serial_timer = new QTimer();
        serial_timer->setSingleShot(true);
        connect(serial_port, &QSerialPort::readyRead,
            this, &MyCobot::HandleReadyRead, Qt::ConnectionType::QueuedConnection);
        connect(serial_port, &QSerialPort::bytesWritten,
            this, &MyCobot::HandleBytesWritten, Qt::ConnectionType::QueuedConnection);
        connect(serial_port, &QSerialPort::errorOccurred,
            this, &MyCobot::HandleError, Qt::ConnectionType::QueuedConnection);
        connect(serial_timer, &QTimer::timeout,
            this, &MyCobot::HandleTimeout, Qt::ConnectionType::QueuedConnection);
        return 0;
    }

    /*
    PortsInfo MyCobot::GetSerialPortInfo()
    {

        serial_port_info.clear();
        QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
        if (ports.size() == 0) {
            return serial_port_info;
        }
        for (int i = 0; i < ports.size(); ++i) {
            //LogInfo << "Detected serial port name: " << ports[i].portName();
            if (ports[i].portName() == SERIAL_PORT_PI ||
                    (ports[i].vendorIdentifier() == M5BASIC_VID && ports[i].productIdentifier() == M5BASIC_PID)) {
                serial_port_info.push_back(ports[i]);
            }
        }
        return serial_port_info;
    }

    bool MyCobot::SetSerialPort(int index)
    {
        if (serial_port_info.size() == 0) {
            return false;
        }
        int baud_rate = 0;

        LogInfo << "Detected serial port name: " << serial_port_info[index].portName();
        if (serial_port_info[index].portName() == SERIAL_PORT_PI) {
    #if defined ROBOT_MYCOBOT
            baud_rate = 1000000;
    #elif defined ROBOT_MYCOBOTPRO
            baud_rate = 115200;
    #endif
        } else if (serial_port_info[index].vendorIdentifier() == M5BASIC_VID &&
                serial_port_info[index].productIdentifier() == M5BASIC_PID) {
            baud_rate = QSerialPort::BaudRate::Baud115200;
        } else {
            return false;
        }
        serial_port.close();
        serial_port.setPortName(serial_port_info[index].portName());
        LogInfo << "new serial port : " << serial_port.portName();
        serial_port.setBaudRate(baud_rate);
        serial_port.open(QIODevice::OpenModeFlag::ReadWrite);
        serial_timer = new QTimer();
        serial_timer->setSingleShot(true);

        connect(&serial_port, &QSerialPort::readyRead,
                this, &MyCobot::HandleReadyRead, Qt::ConnectionType::QueuedConnection);
        connect(&serial_port, &QSerialPort::bytesWritten,
                this, &MyCobot::HandleBytesWritten, Qt::ConnectionType::QueuedConnection);
        connect(&serial_port, &QSerialPort::errorOccurred,
                this, &MyCobot::HandleError, Qt::ConnectionType::QueuedConnection);
        connect(serial_timer, &QTimer::timeout,
                this, &MyCobot::HandleTimeout, Qt::ConnectionType::QueuedConnection);
        return true;
    }

    int MyCobot::ReconnectCurrentSerialPort()
    {

        if (!serial_port.isOpen()) {
            serial_port.open(QIODevice::OpenModeFlag::ReadWrite);
            return 1;
        }
        return 0;
    }
    */
    void MyCobot::UpdateCacheInChangePositionFunction()
    {
        LogTrace << ": is_in_position = false";
        is_in_position = false;
    }

    void MyCobot::CommandThreadWorker()
    {
        int64_t last_time{ 0 };
        constexpr const int64_t wait_time = COMMAND_INTERVAL;
        while (command_thread_running) {
            std::unique_lock<std::mutex> lock(command_mutex);
            if ((GetCurrentTimeMs() - last_time) < wait_time || commands.empty()) {
                lock.unlock();
                QCoreApplication::processEvents();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if (!serial_port->isOpen()) {
                commands.clear();
                command_bits.reset();
                continue;
            }
            last_time = GetCurrentTimeMs();
            std::pair<int, std::function<void()>> f = commands.front();
            command_bits.reset(f.first);
            if (f.first == Command::JogStop || f.first == Command::GetAngles || f.first == Command::GetEncoders || Command::GetCoords) {
                drop_all_commands = false;
            }
            if (!drop_all_commands) {
                f.second();
            }
            commands.pop_front();
            if (f.first == Command::JogAngle || f.first == Command::JogAbsolute || f.first == Command::JogCoord) {
                drop_all_commands = true;
            }
            lock.unlock();
        }
        LogInfo << "Exit CommandThreadWorker thread";
    }

    void MyCobot::HandleReadyRead()
    {
        std::lock_guard<std::mutex> lock(serial_data_mutex);
        read_data.append(serial_port->readAll());
        //LogTrace << ": " << read_data.toHex(' ').toUpper();
        // first: command, second: data
        std::vector<std::pair<unsigned char, QByteArray>> parsed_commands = Parse(read_data);
        if (parsed_commands.empty()) {
            return;
        }
        FixupCommands(parsed_commands);
        //LogTrace << ": parsed_commands.size() = " << parsed_commands.size();
        for (const auto& content : parsed_commands) {
            //LogTrace << ": " << content;
            switch (content.first) {
            case Command::IsPoweredOn:
                is_powered_on = static_cast<bool>(content.second[0]);
                break;
            case Command::IsControllerConnected:
                is_controller_connected = static_cast<bool>(content.second[0]);
                break;
            case Command::ReadNextError:
                //TODO
                break;
            case Command::IsFreeMoveMode:
                is_free_move = static_cast<bool>(content.second[0]);
                break;
            case Command::GetAngles:
                //LogTrace << ": GetAngles";
                for (int i = 0; i < rc::Joints; ++i) {
                    short centi_deg = (short)((unsigned short)(content.second[i * 2] << 8) | (unsigned char)(content.second[i * 2 + 1]));
                    double deg = static_cast<double>(centi_deg) / 100;
                    cur_angles[i] = deg;
                }
                break;
            case Command::GetCoords:
                //LogTrace << ": GetCoords";
                // x y z
                for (int i = 0; i < 3; ++i) {
                    short tenth_mm = (short)((unsigned short)(content.second[i * 2] << 8) | (unsigned char)(content.second[i * 2 + 1]));
                    double mm = static_cast<double>(tenth_mm) / 10;
                    cur_coords[i] = mm;
                }
                // rx ry rz
                for (int i = 3; i < rc::Axes; ++i) {
                    short centi_deg = (short)((unsigned short)(content.second[i * 2] << 8) | (unsigned char)(content.second[i * 2 + 1]));
                    double deg = static_cast<double>(centi_deg) / 100;
                    cur_coords[i] = deg;
                }
                break;
            case Command::IsProgramPaused:
                is_program_paused = static_cast<bool>(content.second[0]);
                break;
            case Command::IsInPosition:
                is_in_position = static_cast<bool>(content.second[0]);
                break;
            case Command::CheckRunning:
                robot_is_moving = static_cast<bool>(content.second[0]);
                break;
            case Command::GetSpeed:
                cur_speed = double(content.second[0]);
                break;
            case Command::GetFeedOverride:
                //TODO
                break;
            case Command::GetAcceleration:
                //TODO
                break;
            case Command::GetJointMin: {
                short deci_deg = (short)((unsigned short)(content.second[1] << 8) | (unsigned char)(content.second[2]));
                double deg = static_cast<double>(deci_deg) / 10;
                joint_min[content.second[0]] = deg;
                break;
            }
            case Command::GetJointMax: {
                short deci_deg = (short)((unsigned short)(content.second[1] << 8) | (unsigned char)(content.second[2]));
                double deg = static_cast<double>(deci_deg) / 10;
                joint_max[content.second[0]] = deg;
                break;
            }
            case Command::GetEncoders: {
                //LogInfo << "getencoders";
                for (int i = 0; i < rc::Joints; ++i) {
                    short centi_deg = (short)((unsigned short)(content.second[i * 2] << 8) | (unsigned char)(content.second[i * 2 + 1]));
                    double deg = static_cast<double>(centi_deg);
                    cur_encoders[i] = deg;
                }
                LogInfo << "cur_encoders = " << cur_encoders;
                //LogInfo << "get encoders finish";
                break;
            }
            case Command::IsServoEnabled:
                servo_enabled[static_cast<size_t>(content.second[0]) - 1] = static_cast<bool>(content.second[1]);
                break;
            case Command::IsAllServoEnabled:
                is_all_servo_enabled = static_cast<bool>(content.second[0]);
                break;
            case Command::GetServoData:
                is_power_on = static_cast<bool>(content.second[0]);
                break;
            case Command::GetDigitalIn:
                atom_digital_in[content.second[0]] = content.second[1];
                break;
            case Command::GetBasicIn:
                basic_digital_in[content.second[0]] = content.second[1];
                break;
            default:
                LogDebug << "Unhandled command: <" << content.first << ", " << content.second.toHex(' ').toUpper() << ">";
                break;
            }
        }
    }

    void MyCobot::HandleBytesWritten(qint64 bytes)
    {
        // written bytes
        //LogTrace << " " << bytes;
        //serial_timer.stop();
    }

    void MyCobot::HandleTimeout()
    {
        // serial port timeout
    }

    void MyCobot::HandleError(QSerialPort::SerialPortError error)
    {
        if (error == QSerialPort::SerialPortError::WriteError) {
            // write error
        }
        else if (error == QSerialPort::ResourceError) {
            // close serial port, and disconnection after connection, set next_error
            if (IsCncConnected()) {
                StateOff();
                next_error = QObject::tr("Device removed or became unavailable. Please, check connection with the Robot.").toStdString();
            }
        }
    }

}
