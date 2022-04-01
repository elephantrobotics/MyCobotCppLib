#ifndef ROBOSIGNAL_LOG_LOG_HPP
#define ROBOSIGNAL_LOG_LOG_HPP

#include <QDate>
#include <QDateTime>
#include <QDir>
#include <QLoggingCategory>
#include <QMessageLogContext>
#include <QString>
#include <QFile>
#include <QObject>
#include <QStringList>
#include <QStandardPaths>
#include <QTextStream>
#include <QMutex>

#include <log/LogReader.hpp>
#include "robosignal_global.hpp"

// Undefine common define
#ifdef Log
#undef Log
#endif

#ifndef log_category
//FIXME:
//#error Please #define log_category ::rc::log::some_category before #include "log/Log.hpp"
//#warning Please #define log_category ::rc::log::some_category before #include "log/Log.hpp"
#endif
#define LogError qCCritical(log_category).nospace().noquote()
#define LogWarn qCWarning(log_category).nospace().noquote()
#define LogInfo qCInfo(log_category).nospace().noquote()
#define LogDebug qCDebug(log_category).nospace().noquote()
#define LogTrace qCDebug(log_category).nospace().noquote() << __func__

namespace rc {
namespace log {

constexpr const int MaxDisplayLines = 200;
enum class LogType {RoboFlow, Phoenix};
constexpr const char *const PhoenixLogFile = "/var/log/phoenix/phoenix.log";

ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(unspecified)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(cli)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(conf)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(core)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(db)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(ip)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(login)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(ui)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(ui_page_program)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(ui_page_quick_move)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(ui_widget_wait)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(ui_widget_waypoint)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(robot)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(python_worker)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(robot_test)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(robot_controller)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(log)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(elephant_script)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(signal_handler)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(ui_page_run)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(network)
ROBOSIGNALSHARED_EXPORT Q_DECLARE_LOGGING_CATEGORY(ui_popup)

namespace name {

constexpr const char *const Default = "default";

constexpr const char *const Unspecified = "unspecified";
constexpr const char *const Cli = "cli";
constexpr const char *const Conf = "conf";
constexpr const char *const Core = "core";
constexpr const char *const Db = "DB";
constexpr const char *const Ip = "IP"; // main program interpreter
constexpr const char *const Login = "login";
constexpr const char *const Ui = "UI";
constexpr const char *const UiPageProgram = "ui.page.program";
constexpr const char *const UiPageQuickMove = "ui.page.quick_move";
constexpr const char *const UiWidgetWait = "ui.widget.wait";
constexpr const char *const UiWidgetWaypoint = "ui.widget.waypoint";
constexpr const char *const Robot = "robot";
constexpr const char *const PythonWorker = "python_worker";
constexpr const char *const RobotTest = "robot_test";
constexpr const char *const RobotController = "robot_controller";
constexpr const char *const Log = "log";
constexpr const char *const ElephantScript = "ES";
constexpr const char *const SignalHandler = "signal_handler";
constexpr const char *const UiPageRun = "ui.page.run";
constexpr const char *const Network = "net";
constexpr const char *const UiPopup = "ui.popup";

}
constexpr const int DefaultLogsKeptDays = 4;
void ROBOSIGNALSHARED_EXPORT InitLogging(const QStringList &filter_rules = QStringList{});

void ROBOSIGNALSHARED_EXPORT RoboMessageHandler(QtMsgType type, const QMessageLogContext & context,
        const QString & msg);

/**
 * Global singleton class.
 * Created on first use or in InitLogging() function.
 * Cannot be created from the outside otherwise (constructor is private).
 */
class ROBOSIGNALSHARED_EXPORT Log : public QObject
{
    Q_OBJECT
public:
    void Reset();
    QStringList GetAllLines();
    QStringList GetMoreLines(LogType log_type = LogType::RoboFlow);
    QStringList GetLatestInsertLines(LogType log_type = LogType::RoboFlow);
    void ResetMessagePattern(bool format24h);
    void ClearOldLogs(int keep_days = DefaultLogsKeptDays);
    static Log& Instance();

private:
    explicit Log(QObject *parent = nullptr);
    ~Log();
    void RenameLogFileByDate();

private slots:
    void OnCheckCurrentDate();

private:
    QFile log_file{};
    QTextStream log_stream{};
    bool debug_output{false};
    bool trace_output{false};
    QMutex log_mutex{QMutex::RecursionMode::NonRecursive};
    QDate curr_date{};

    LogReader* roboflow_log_reader{nullptr};
    LogReader* phoenix_log_reader{nullptr};

    const QString LogPath{QStandardPaths::writableLocation(
            QStandardPaths::StandardLocation::AppLocalDataLocation) + "/logs"};

    friend void InitLogging();
    friend void ::rc::log::RoboMessageHandler(QtMsgType type,
            const QMessageLogContext & context, const QString & msg);
};

}
}
#endif
