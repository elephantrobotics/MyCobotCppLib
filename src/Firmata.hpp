#ifndef ROBOSIGNAL_FIRMATA_HPP
#define ROBOSIGNAL_FIRMATA_HPP

#include <vector>
#include <utility>

#include <QByteArray>

namespace rc {

const QByteArray FIRMATA_HEADER = QByteArrayLiteral("\xFE\xFE");
const QByteArray FIRMATA_FOOTER = QByteArrayLiteral("\xFA");

enum Command : unsigned char {
    Undefined = 0x0,
    PowerOn = 0x10,
    PowerOff = 0x11,
    IsPoweredOn = 0x12,
    ReleaseAllServos = 0x13,
    IsControllerConnected = 0x14,
    ReadNextError = 0x15,
    SetFreeMoveMode = 0x1A,
    IsFreeMoveMode = 0x1B,

    GetAngles = 0x20,
    WriteAngle = 0x21,
    WriteAngles = 0x22,
    GetCoords = 0x23,
    WriteCoord = 0x24,
    WriteCoords = 0x25,
    ProgramPause = 0x26,
    IsProgramPaused = 0x27,
    ProgramResume = 0x28,
    TaskStop = 0x29,
    IsInPosition = 0x2A,
    CheckRunning = 0x2B,

    JogAngle = 0x30,
    JogAbsolute = 0x31,
    JogCoord = 0x32,
    SendJogIncrement = 0x33,
    JogStop = 0x34,
    SetEncoder = 0x3A,
    SetEncoders = 0x3C,
    GetEncoders = 0x3D,

    GetSpeed = 0x40,
    SetSpeed = 0x41,
    GetFeedOverride = 0x42,
    SendFeedOverride = 0x43,
    GetAcceleration = 0x44,
    SetAcceleration = 0x45,
    GetJointMin = 0x4A,
    GetJointMax = 0x4B,
    SetJointMin = 0x4C,
    SetJointMax = 0x4D,

    IsServoEnabled = 0x50,
    IsAllServoEnabled = 0x51,
    SetServoData = 0x52,
    GetServoData = 0x53,
    SetServoCalibration = 0x54,
    JointBrake = 0x55,
    FocusServo = 0x57,

    SetPinMode = 0x60,
    SetDigitalOut = 0x61,
    GetDigitalIn = 0x62,

    GripperMode = 0x66,
    SetLedRgb = 0x6A,

    SetBasicOut = 0xA0,
    GetBasicIn = 0xA1,
};

extern std::pair<int, int> RETURNED_COMMAND_SIZES[256];

/// Basic Control commands
const QByteArray CommandPowerOn = FIRMATA_HEADER + char(2) + char(Command::PowerOn) + FIRMATA_FOOTER;
const QByteArray CommandPowerOff = FIRMATA_HEADER + char(2) + char(Command::PowerOff) + FIRMATA_FOOTER;
const QByteArray CommandIsPoweredOn = FIRMATA_HEADER + char(2) + char(Command::IsPoweredOn) + FIRMATA_FOOTER;
const QByteArray CommandReleaseAllServos = FIRMATA_HEADER + char(2) + char(Command::ReleaseAllServos) + FIRMATA_FOOTER;
const QByteArray CommandFocusServo = FIRMATA_HEADER + char(3) + char(Command::FocusServo);
const QByteArray CommandIsControllerConnected = FIRMATA_HEADER + char(2) + char(Command::IsControllerConnected) + FIRMATA_FOOTER;
const QByteArray CommandReadNextError = FIRMATA_HEADER + char(2) + char(Command::ReadNextError) + FIRMATA_FOOTER;
const QByteArray CommandSetFreeMoveMode = FIRMATA_HEADER + char(3) + char(Command::SetFreeMoveMode);
const QByteArray CommandIsFreeMoveMode = FIRMATA_HEADER + char(2) + char(Command::IsFreeMoveMode) + FIRMATA_FOOTER;
const QByteArray CommandGetAngles = FIRMATA_HEADER + char(2) + char(Command::GetAngles) + FIRMATA_FOOTER;
const QByteArray CommandWriteAngle = FIRMATA_HEADER + char(6) + char(Command::WriteAngle);
const QByteArray CommandWriteAngles = FIRMATA_HEADER + char(15) + char(Command::WriteAngles);
const QByteArray CommandGetCoords = FIRMATA_HEADER + char(2) + char(Command::GetCoords) + FIRMATA_FOOTER;
const QByteArray CommandWriteCoord = FIRMATA_HEADER + char(2) + char(Command::WriteCoord);
const QByteArray CommandWriteCoords = FIRMATA_HEADER + char(16) + char(Command::WriteCoords);
const QByteArray CommandProgramPause = FIRMATA_HEADER + char(2) + char(Command::ProgramPause) + FIRMATA_FOOTER;
const QByteArray CommandIsProgramPaused = FIRMATA_HEADER + char(2) + char(Command::IsProgramPaused) + FIRMATA_FOOTER;
const QByteArray CommandProgramResume = FIRMATA_HEADER + char(2) + char(Command::ProgramResume) + FIRMATA_FOOTER;
const QByteArray CommandTaskStop = FIRMATA_HEADER + char(2) + char(Command::TaskStop) + FIRMATA_FOOTER;
const QByteArray CommandIsInPosition = FIRMATA_HEADER + char(15) + char(Command::IsInPosition);
const QByteArray CommandCheckRunning = FIRMATA_HEADER + char(2) + char(Command::CheckRunning) + FIRMATA_FOOTER;
const QByteArray CommandJogAngle = FIRMATA_HEADER + char(5) + char(Command::JogAngle);
const QByteArray CommandJogAbsolute = FIRMATA_HEADER + char(6) + char(Command::JogAbsolute);
const QByteArray CommandJogCoord = FIRMATA_HEADER + char(5) + char(Command::JogCoord);
const QByteArray CommandSendJogIncrement = FIRMATA_HEADER + char(6) + char(Command::SendJogIncrement);
const QByteArray CommandJogStop = FIRMATA_HEADER + char(2) + char(Command::JogStop) + FIRMATA_FOOTER;
const QByteArray CommandSetEncoder = FIRMATA_HEADER + char(5) + char(Command::SetEncoder);
const QByteArray CommandSetEncoders = FIRMATA_HEADER + char(15) + char(Command::SetEncoders);
const QByteArray CommandGetEncoders = FIRMATA_HEADER + char(2) + char(Command::GetEncoders) + FIRMATA_FOOTER;
const QByteArray CommandGetSpeed = FIRMATA_HEADER + char(2) + char(Command::GetSpeed) + FIRMATA_FOOTER;
const QByteArray CommandSetSpeed = FIRMATA_HEADER + char(3) + char(Command::SetSpeed);
const QByteArray CommandGetFeedOverride = FIRMATA_HEADER + char(2) + char(Command::GetFeedOverride) + FIRMATA_FOOTER;
const QByteArray CommandSendFeedOverride = FIRMATA_HEADER + char(2) + char(Command::SendFeedOverride);
const QByteArray CommandGetAcceleration = FIRMATA_HEADER + char(2) + char(Command::GetAcceleration) + FIRMATA_FOOTER;
const QByteArray CommandSetAcceleration = FIRMATA_HEADER + char(2) + char(Command::SetAcceleration);
const QByteArray CommandGetJointMin = FIRMATA_HEADER + char(3) + char(Command::GetJointMin);
const QByteArray CommandGetJointMax = FIRMATA_HEADER + char(3) + char(Command::GetJointMax);
const QByteArray CommandSetJointMin = FIRMATA_HEADER + char(3) + char(Command::SetJointMin);
const QByteArray CommandSetJointMax = FIRMATA_HEADER + char(3) + char(Command::SetJointMax);
const QByteArray CommandIsServoEnabled = FIRMATA_HEADER + char(3) + char(Command::IsServoEnabled);
const QByteArray CommandIsAllServoEnabled = FIRMATA_HEADER + char(2) + char(Command::IsAllServoEnabled) + FIRMATA_FOOTER;
const QByteArray CommandSetServoData = FIRMATA_HEADER + char(5) + char(Command::SetServoData);
const QByteArray CommandGetServoData = FIRMATA_HEADER + char(4) + char(Command::GetServoData);
const QByteArray CommandSenServoCalibration = FIRMATA_HEADER + char(3) + char(Command::SetServoCalibration);
const QByteArray CommandJointBrake = FIRMATA_HEADER + char(3) + char(Command::JointBrake);

/// Atom Features
const QByteArray CommandSetGriperOpen = FIRMATA_HEADER + char(4) + char(Command::GripperMode) + char(0) + char(100) + FIRMATA_FOOTER;
const QByteArray CommandSetGriperClose = FIRMATA_HEADER + char(4) + char(Command::GripperMode) + char(1) + char(100) + FIRMATA_FOOTER;
const QByteArray CommandSetLedRgb = FIRMATA_HEADER + char(5) + char(Command::SetLedRgb);

/// IO
const QByteArray CommandSetDigitalOut = FIRMATA_HEADER + char(4) + char(Command::SetDigitalOut);
const QByteArray CommandGetDigitalIn = FIRMATA_HEADER + char(3) + char(Command::GetDigitalIn);
const QByteArray CommandSetBasicOut = FIRMATA_HEADER + char(4) + char(Command::SetBasicOut);
const QByteArray CommandGetBasicIn = FIRMATA_HEADER + char(3) + char(Command::GetBasicIn);
const QByteArray CommandSetPinMode = FIRMATA_HEADER + char(4) + char(Command::SetPinMode);

enum ParsingState {
    NOT_STARTED,
    GOT_FIRST_HEADER,
    GOT_FULL_HEADER,
    GOT_LENGTH,
    GOT_COMMAND,
    GOT_PARAMS,
    GOT_FOOTER,
};

void InitFirmata();
std::vector<std::pair<unsigned char, QByteArray>> Parse(QByteArray& data);
void FixupCommands(std::vector<std::pair<unsigned char, QByteArray>>& command);

}
#endif
