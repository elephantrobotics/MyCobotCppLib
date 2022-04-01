#include "Firmata.hpp"

#define log_category ::rc::log::robot_controller
#include "log/Log.hpp"

namespace rc {

std::pair<int, int> RETURNED_COMMAND_SIZES[256] = { {0, 0} };

void InitFirmata()
{
    RETURNED_COMMAND_SIZES[Undefined] = {0, 0};
    RETURNED_COMMAND_SIZES[PowerOn] = {0, 0};
    RETURNED_COMMAND_SIZES[PowerOff] = {0, 0};
    RETURNED_COMMAND_SIZES[IsPoweredOn] = {1, 1};
    RETURNED_COMMAND_SIZES[ReleaseAllServos] = {0, 0};
    RETURNED_COMMAND_SIZES[IsControllerConnected] = {1, 1};
    RETURNED_COMMAND_SIZES[ReadNextError] = {0, 0};
    RETURNED_COMMAND_SIZES[SetFreeMoveMode] = {0, 0};
    RETURNED_COMMAND_SIZES[IsFreeMoveMode] = {1, 1};
    RETURNED_COMMAND_SIZES[GetAngles] = {12, 12};
    RETURNED_COMMAND_SIZES[GetEncoders] = {12, 12};
    RETURNED_COMMAND_SIZES[SetEncoders] = {0, 0};
    RETURNED_COMMAND_SIZES[WriteAngle] = {0, 0};
    RETURNED_COMMAND_SIZES[WriteAngles] = {0, 0};
    RETURNED_COMMAND_SIZES[GetCoords] = {12, 12};
    RETURNED_COMMAND_SIZES[WriteCoord] = {0, 0};
    RETURNED_COMMAND_SIZES[WriteCoords] = {0, 0};
    RETURNED_COMMAND_SIZES[ProgramPause] = {0, 0};
    RETURNED_COMMAND_SIZES[IsProgramPaused] = {1, 1};
    RETURNED_COMMAND_SIZES[ProgramResume] = {0, 0};
    RETURNED_COMMAND_SIZES[TaskStop] = {0, 0};
    RETURNED_COMMAND_SIZES[IsInPosition] = {1, 1};
    RETURNED_COMMAND_SIZES[CheckRunning] = {1, 1};
    RETURNED_COMMAND_SIZES[JogAngle] = {0, 0};
    RETURNED_COMMAND_SIZES[JogAbsolute] = {0, 0};
    RETURNED_COMMAND_SIZES[JogCoord] = {0, 0};
    RETURNED_COMMAND_SIZES[SendJogIncrement] = {0, 0};
    RETURNED_COMMAND_SIZES[JogStop] = {0, 0};
    RETURNED_COMMAND_SIZES[SetEncoder] = {0, 0};
    RETURNED_COMMAND_SIZES[GetSpeed] = {1, 1};
    RETURNED_COMMAND_SIZES[SetSpeed] = {0, 0};
    RETURNED_COMMAND_SIZES[GetFeedOverride] = {0, 0};
    RETURNED_COMMAND_SIZES[SendFeedOverride] = {0, 0};
    RETURNED_COMMAND_SIZES[GetAcceleration] = {0, 0};
    RETURNED_COMMAND_SIZES[SetAcceleration] = {0, 0};
    RETURNED_COMMAND_SIZES[GetJointMin] = {3, 3};
    RETURNED_COMMAND_SIZES[GetJointMax] = {3, 3};
    RETURNED_COMMAND_SIZES[SetJointMin] = {0, 0};
    RETURNED_COMMAND_SIZES[SetJointMax] = {0, 0};
    RETURNED_COMMAND_SIZES[IsServoEnabled] = {2, 2};
    RETURNED_COMMAND_SIZES[IsAllServoEnabled] = {1, 1};
    RETURNED_COMMAND_SIZES[SetServoData] = {0, 0};
    RETURNED_COMMAND_SIZES[GetServoData] = {1, 1};
    RETURNED_COMMAND_SIZES[SetServoCalibration] = {0, 0};
    RETURNED_COMMAND_SIZES[JointBrake] = {0, 0};
    RETURNED_COMMAND_SIZES[SetDigitalOut] = {2, 2};
    RETURNED_COMMAND_SIZES[GetDigitalIn] = {2, 2};
    RETURNED_COMMAND_SIZES[GripperMode] = {0, 0};
    RETURNED_COMMAND_SIZES[SetLedRgb] = {0, 0};
    RETURNED_COMMAND_SIZES[SetBasicOut] = {2, 2};
    RETURNED_COMMAND_SIZES[GetBasicIn] = {2, 2};
}

/**
 * @brief This function ensures that complete command is returned or none.
 * It may be called several times to parse data. If there isn't enough data,
 * the function will save parsing state and continue on next call with more
 * data.
 * It will remove parsed data from data parameter.
 * Returned commands are in the std::vector<> with elements:
 * std::pair<unsigned char, QByteArray>, where unsigned char is command
 * id, and QByteArray is command parameters.
 * @param[in/out] data - data to parse
 * @return either {Command::Undefined, {}} or parsed command params in the form
 *         of std::pair<unsigned char, QByteArray>
 * @return vector with parsed command
 */
std::vector<std::pair<unsigned char, QByteArray>> Parse(QByteArray& data)
{
    static ParsingState parsing_state{ParsingState::NOT_STARTED};
    static unsigned char cmd_len{0};
    static unsigned char cmd{0};
    static QByteArray params{};
    std::vector<std::pair<unsigned char, QByteArray>> commands{};
    // GOT_FOOTER means we got complete command and need to execute it now (so don't need to wait for any data)
    char cur_token{0};
    while ((parsing_state == ParsingState::GOT_PARAMS) || (parsing_state == ParsingState::GOT_FOOTER) ||
            (!data.isEmpty())) {
        if (!data.isEmpty() && parsing_state != ParsingState::GOT_FOOTER) {
            cur_token = data.front();



            data.remove(0, 1);
        }
        switch (parsing_state) {
        case ParsingState::NOT_STARTED:
            // re-init (because finished or error)
            cmd_len = 0;
            cmd = 0;
            params.clear();
            if (cur_token == char(0xFE)) {
                parsing_state = ParsingState::GOT_FIRST_HEADER;
            }
            break;
        case ParsingState::GOT_FIRST_HEADER:
            if (cur_token == char(0xFE)) {
                parsing_state = ParsingState::GOT_FULL_HEADER;
            } else {
                parsing_state = ParsingState::NOT_STARTED;
            }
            break;
        case ParsingState::GOT_FULL_HEADER:
            cmd_len = static_cast<unsigned char>(cur_token);
            parsing_state = ParsingState::GOT_LENGTH;
            break;
        case ParsingState::GOT_LENGTH:
            cmd = static_cast<unsigned char>(cur_token);
            parsing_state = ParsingState::GOT_COMMAND;
            break;
        case ParsingState::GOT_COMMAND:
            // get params
            if (params.size() < (cmd_len - 2)) {
                params += cur_token;
                break;
            }
            parsing_state = ParsingState::GOT_PARAMS;
            [[fallthrough]];
        case ParsingState::GOT_PARAMS:
            if (cur_token == char(0xFA)) {
                parsing_state = ParsingState::GOT_FOOTER;
            } else {
                parsing_state = ParsingState::NOT_STARTED;
            }
            break;
        case ParsingState::GOT_FOOTER:
            // return whole command
            parsing_state = ParsingState::NOT_STARTED;
            commands.push_back(std::make_pair(cmd, params));
        default:
            break;
        }
    }
    return commands;
}

/**
 * @brief Make command to be expected size, so that during parsing there is
 * no need for size checks.
 * If returned command parameters have size bigger than max
 * allowed size for given command, shrink extra parameters.
 * If returned command parameters size is smaller that min size for given
 * command, add missing parameters (initialized to zero).
 * @arg[in,out] command the argument is modified in place, if needed.
 */
void FixupCommands(std::vector<std::pair<unsigned char, QByteArray>>& commands)
{
    for (auto& command : commands) {
        if (command.second.size() < RETURNED_COMMAND_SIZES[command.first].first) {
            command.second.append(RETURNED_COMMAND_SIZES[command.first].first - command.second.size(), 0);
            LogTrace << ": not enough data, appending";
        } else if (command.second.size() > RETURNED_COMMAND_SIZES[command.first].first) {
            command.second.resize(RETURNED_COMMAND_SIZES[command.first].first);
            LogTrace << ": too much data, removing excess data";
        }
    }
}

}
