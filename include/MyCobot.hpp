#ifndef ROBOSIGNAL_MYCOBOT_HPP
#define ROBOSIGNAL_MYCOBOT_HPP

#include <array>
#include <string>
#include <list>
#include <functional>
#include <bitset>
#include <mutex>
#include <thread>
#include <condition_variable>

#include <QtSerialPort/qserialport.h>
#include <QTimer>
#include <QByteArray>

#include "robosignal_global.hpp"
#include "Common.hpp"
#define ROBCTL_ATOMMAIN
#define ROBOT_MYCOBOT

// types for EMC_TASK mode
enum PHOENIX_TASK_MODE_ENUM {
    PHOENIX_TASK_MODE_MANUAL = 1,
    PHOENIX_TASK_MODE_AUTO = 2,
    PHOENIX_TASK_MODE_MDI = 3
};

namespace rc
{

    enum M5BasicId : int {
        CP210X_VID = 0x10C4, CP210X_PID = 0xEA60,
        CH9102_VID = 0x1A86, CH9102_PID = 0x55D4
    };

    constexpr const int MaxEncoders = Joints + 1;

    class ROBOSIGNALSHARED_EXPORT MyCobot : public QObject
    {
        Q_OBJECT

    public:
        virtual ~MyCobot();
        static MyCobot &Instance();
        int Init();
        bool IsCncConnected();
        bool IsControllerConnected();
        void InitRobot();
        void DetectRobot();
        void EnableOutPinCache(bool enable = true);

        bool PowerOn();
        bool PowerOnOnly();
        bool PowerOff();
        Coords GetCoords() const;
        /* double GetCoord(int axis); */
        void WriteCoords(const Coords &coords, int speed = DefaultLinearSpeed);
        void WriteCoord(Axis axis, double value, int speed = DefaultLinearSpeed);
        Angles GetAngles() const;
        /* double GetAngle(int joint); */
        void WriteAngles(const Angles &angles, int speed = DefaultAngleSpeed);
        void WriteAngle(Joint joint, double value, int speed = DefaultAngleSpeed);
        double GetSpeed() const;
        /**
         * Sets speed percentage (feed override).
         * @parcentage in range [0; 100], 0 means do not move.
         */
        void SetSpeed(int percentage);
        int StateOn();
        int StateOff();
        bool StateCheck();
        int TaskStop();
        /* void Freeze(); */
        /* void Unfreeze(); */
        void Wait(double time);
        /* void SetHomePosition(Angles home_position); */
        /* Angles GetHomePosition(); */
        bool CheckRunning() const;
        /**
         * Checks whether robot is in requested position.
         * @returns 0 - not in position, 1 - in position,
         *          -1 - backend/robot/Phoenix error.
         */
        void ReleaseAllServos();
        void FocusServo(Joint joint);
        bool GetServoData(Joint joint, int data_id);
        int IsInPosition(const Coords &coord, bool is_linear = true) const;
        int IsInPositionEncoders(const Angles &angles) const;
        void SetFreeMove(bool on);
        bool IsSoftwareFreeMove() const;
        bool IsHardwareFreeMove() const;
        /* void ToolheadOpen(); */
        /* void ToolheadClose(); */
        /* void ToolheadStop(); */
        /* void ToolheadControl(int force, int pos, double error); */
        /* void ToolheadClose(int force, int pos, double error); */
        /* int ToolheadGetStatus(int pos, double error); */
        /* double ReadTorqueLimit(int axis); */
        void SetCarteTorqueLimit(Axis axis, double value);

        void JogAngle(Joint joint, int direction, int speed = DefaultAngleSpeed);
        void JogAbsolute(int joint_or_axis, double speed, double pos);
        /* void JogAngleDistance(int joint, int direction, double distance); */
        void JogCoord(Axis axis, int direction, int speed = DefaultLinearSpeed);
        void SendJogIncrement(int joint_or_axis, double speed, double incr,
                              int jjogmode);
        int JogStop(int joint_or_axis, int jog_mode);
        /* void JogCoordDistance(int axiBrakes, int direction, double distance); */

        /* void UpdateWatchDog(); */
        int SetPayload(double payload);
        bool SetUpsideDown(bool new_upside_down = true);
        int ProgramOpen(const std::string &program_file_path);
        int ProgramRun(int start_line);
        bool ProgramRunFinished();
        int ProgramPause();
        bool IsProgramPaused() const;
        int ProgramResume();
        int ReadNextError(std::string &error_string);

        int IsPowerOn();
        bool IsServoEnabled(Joint j) const;
        bool IsAllServoEnabled() const;

        double GetJointMinPosLimit(Joint j) const;
        int SetJointMinPosLimit(Joint joint, double limit);
        double GetJointMaxPosLimit(Joint j) const;
        int SetJointMaxPosLimit(Joint joint, double limit);
        int SetJointMaxVelocity(Joint joint, double limit);

        void SetEncoder(int joint, int val);
        void SetEncoders(Angles encodes, int speed);
        Angles GetEncoders() const;
        /* void SetCoordsLimit(const Coords& min_coords, const Coords& max_coords); */
        /* void SetAnglesLimit(const Angles& min_angles, const Coords& max_angles); */
        /* void BoundToCoordsLimit(Coords& coords); */
        /* void BoundToAnglesLimit(Angles& angles); */

    public:
        void InitIo();
        int GetDigtalOrBasic(int pin_number, bool is_out);
        void SetDigitalOrBasic(int pin_number, int pin_signal);
        int GetDigitalIn(int pin_number) const;
        int GetDigitalOut(int pin_number) const;
        int SetDigitalOut(int pin_number, int pin_signal);
#if defined ROBOT_MYCOBOT || ROBOT_MYCOBOTPRO
        int GetBasicIn(int pin_number) const;
        int SetBasicOut(int pin_number, int pin_signal);
        void SetPinMode(int pin_number, int pin_mode);
#endif
        //TODO: rename to SetGripper()
        int SetGriper(int open);
        double GetAnalogIn(int pin_number) const;
        double GetAnalogOut(int pin_number) const;
        int SetAnalogOut(int pin_number, double pin_value);

        int SendFeedOverride(double override);
        int GetFeedOverride();
        int GetAcceleration();
        void SetAcceleration(int acceleration);
        int EmcCommandWaitDone();
        PHOENIX_TASK_MODE_ENUM GetCurrentCncMode() const;
        /**
         * Set mode.
         * @mode can be PHOENIX_TASK_MODE_MANUAL, PHOENIX_TASK_MODE_AUTO, PHOENIX_TASK_MODE_MDI
         */
        int SendMode(PHOENIX_TASK_MODE_ENUM mode);

        void JointBrake(Joint joint, int release);
        bool IsCncInMdiMode() const;
        void SetCncInMdiMode();

        int GetCurrentLine() const;
        int GetMotionLine() const;
        int SendMdiCmd(const std::string &gcode);
        int SendTeleop(int enable);

        /**
         * Robot status
         */
        int GetRobotStatus();
        double GetRobotTemperature();
        double GetRobotPower();

        /**
         * Joint status
         */
        enum JointState : int { OK = 0, Error, PoweredOff };
        int GetJointState(Joint joint_num);
        double GetJointTemperature(Joint joint_num);
        int GetJointCommunication(Joint joint_num);
        double GetJointVoltage(Joint joint_num);
        double GetJointCurrent(Joint joint_num);
        uint32_t GetJointErrorMask(Joint joint_num);

        void SetPowerLimit(int power_limit);
        int GetPowerLimit() const;
        void SetStoppingTime(int stopping_time);
        int GetStoppingTime() const;
        void SetStoppingDistance(int stopping_distance);
        int GetStoppingDistance() const;
        void SetToolSpeed(int tool_speed);
        int GetToolSpeed() const;
        void SetToolForce(int tool_force);
        int GetToolForce() const;

        void StartForceSensor(char *device, int speed);
        void StopForceSensor();
        void SetMotionFlexible(uint32_t flexible);

        void SerialWrite(const QByteArray &data) const;
        //PortsInfo GetSerialPortInfo();
        //bool SetSerialPort(int index);
        int InitSerialPort();
        bool IsSerialPort(int vid, int pid);
    protected:
        MyCobot();

    private:
        void CommandThreadWorker();
        int64_t GetCurrentTimeMs();
        //int ReconnectCurrentSerialPort();
        void UpdateCacheInChangePositionFunction();

    private slots:
        void HandleReadyRead();
        void HandleBytesWritten(qint64 bytes);
        void HandleTimeout();
        void HandleError(QSerialPort::SerialPortError error);

    private:
        bool testing_mode{ false };
        bool upside_down{ false };
        quint64 command_id{ 0 };
        /// Serial Port
        //mutable QSerialPort serial_port{};
        //PortsInfo serial_port_info{};
        QSerialPort *serial_port{ nullptr };
        QTimer *serial_timer{ nullptr };
        QByteArray read_data{};
        bool is_power_on{ false };
        static std::mutex serial_data_mutex;
        static constexpr const int SERIAL_TIMEOUT{ 1000 };
#if defined ENABLE_OUT_PIN_CACHE && ENABLE_OUT_PIN_CACHE == 1
        bool out_pin_cache_enabled { false };
        /// Cache for SetDigitalOut()
        mutable int do_cache[64] { 0 };
        /// Cache for SetAnalogOut()
        mutable double ao_cache[64] { 0.0 };
#endif
        static std::list<std::pair<int, std::function<void()>>> commands;
        static std::mutex command_mutex;
        static std::atomic<bool> drop_all_commands;
        mutable std::bitset<256> command_bits{};
        /// Cached values
        bool is_controller_connected{ false };
        bool is_powered_on{ false };
        Coords cur_coords{};
        Angles cur_angles{};
        Angles cur_encoders;
        mutable bool is_in_position{ false };
        bool is_free_move{ false };
        bool is_program_paused{ false };
        std::string next_error{};
        double cur_speed{ 0.0 };
        bool servo_enabled[Joints] = { false };
        mutable bool is_all_servo_enabled{ false };
        double joint_min[Joints] = { 0 };
        double joint_max[Joints] = { 0 };
        bool robot_is_moving{ true };
        mutable std::bitset<2> atom_digital_out{};
        mutable std::bitset<2> atom_digital_in{};
        int atom_io_state{ 0 };
#if defined ROBOT_MYCOBOT || ROBOT_MYCOBOTPRO
        mutable std::bitset<16> basic_digital_out {};
        mutable std::bitset<16> basic_digital_in{};
        IoSates io_states{};
#elif defined ROBOT_MYCOBOT_PI
        mutable std::bitset<6> gpio_out {}
        mutable std::bitset<6> gpio_in{}
#endif
#if defined ROBOT_MYCOBOT
        //0-1 basic 2-3 atom
        int in_pin_num[4] = { 36, 35, 22, 19 };
        int out_pin_num[4] = { 5, 2, 33, 23 };
#elif defined ROBOT_MYCOBOTPRO
        //5--out2、15--out1 35--in2 36--in1
        int in_pin_num[2] = { 36, 35 };
        int out_pin_num[2] = { 15, 5 };
#endif
        bool command_thread_running { false };
        QThread *command_thread{ nullptr };
        static constexpr const char *const SERIAL_PORT_PI = "ttyAMA0";
        bool flag{ false };
        std::vector<bool> out_signal;//out can't record High and low level
        int m5BasicId[4] = { CP210X_VID, CP210X_PID, CH9102_VID, CH9102_PID };
    };

}
#endif
