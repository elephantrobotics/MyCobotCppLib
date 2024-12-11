#define log_category ::rc::log::log
#include "log/Log.hpp"

#include <iostream>

#include <QDir>
#include <QFileInfoList>
#include <QMessageLogContext>
#include <QString>
#include <QtGlobal>
#include <QLoggingCategory>
#include <QCoreApplication>
#include <QFileInfo>
#include <QMutexLocker>
#include <QRegularExpression>
#include <QTimer>
#include <QProcess>

namespace rc {
namespace log {

ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(unspecified, name::Unspecified)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(cli, name::Cli)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(conf, name::Conf)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(core, name::Core)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(db, name::Db)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(ip, name::Ip)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(login, name::Login)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(ui, name::Ui)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(ui_page_program, name::UiPageProgram)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(ui_page_quick_move, name::UiPageQuickMove)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(ui_widget_wait, name::UiWidgetWait)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(ui_widget_waypoint, name::UiWidgetWaypoint)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(robot, name::Robot)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(python_worker, name::PythonWorker)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(robot_test, name::RobotTest)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(robot_controller, name::RobotController)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(log, name::Log)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(elephant_script, name::ElephantScript)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(signal_handler, name::SignalHandler)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(ui_page_run, name::UiPageRun)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(network, name::Network)
ROBOSIGNALSHARED_EXPORT Q_LOGGING_CATEGORY(ui_popup, name::UiPopup)

void InitLogging(const QStringList &filter_rules)
{
    if (filter_rules.size() > 0) {
        QString categories{};
        for (auto &rule : filter_rules) {
            if (!categories.isEmpty()) {
                categories += "\n";
            }
            categories += rule;
        }
        QLoggingCategory::setFilterRules(categories);
    }
    Log::Instance();
}

void RoboMessageHandler(QtMsgType type, const QMessageLogContext & context,
        const QString & msg)
{
    QMutexLocker lock(&Log::Instance().log_mutex);
    const QString log_message = qFormatLogMessage(type, context, msg);
    Log::Instance().log_stream << log_message << '\n';
    Log::Instance().log_stream.flush();
    std::cerr << log_message.toStdString() << '\n';
}

void Log::ClearOldLogs(int keep_days)
{
    if (keep_days <= 0)
        keep_days = DefaultLogsKeptDays;
    QDir log_dir(LogPath);
    if (!log_dir.exists()) {
        return;
    }
#if QT_VERSION <= QT_VERSION_CHECK(5, 11, 0)
    QString cmd = QString("sh -c \"find %1 -type f -name '*.log*' -mtime +%2 | xargs rm -rf\"").
        arg(LogPath).arg(keep_days);
    QProcess::execute(cmd);
#else
    QString cmd = QString("find %1 -type f -name '*.log*' -mtime +%2 | xargs rm -rf")
        .arg(LogPath)
        .arg(keep_days);
    QProcess::execute("sh", QStringList() << "-c" << cmd);
#endif
}

Log::Log(QObject *parent) : QObject(parent)
{
    curr_date = QDateTime::currentDateTime().date();
    QString roboflow_log = LogPath + "/RoboFlow_" + curr_date.toString("yyyy_MM_dd") + ".log";
    roboflow_log_reader = new LogReader(roboflow_log);
    phoenix_log_reader = new LogReader(PhoenixLogFile);

    QDir log_dir(LogPath);
    if (!log_dir.exists()) {
        log_dir.mkpath(LogPath);
    }

    RenameLogFileByDate();

    ResetMessagePattern(true);
    ClearOldLogs(DefaultLogsKeptDays);
    qInstallMessageHandler(RoboMessageHandler);

    QTimer *date_timer = new QTimer(this);
    connect(date_timer, &QTimer::timeout,
            this, &Log::OnCheckCurrentDate);
    //FIXME: calculate timeout as ms to the end of current day
    date_timer->start(2000);
}

Log::~Log()
{
    delete roboflow_log_reader;
    roboflow_log_reader = nullptr;
    delete phoenix_log_reader;
    phoenix_log_reader = nullptr;
}

void Log::RenameLogFileByDate()
{
    if (log_file.isOpen()) {
        log_file.flush();
        log_file.close();
    }
    QString file_name = LogPath + "/RoboFlow_" + curr_date.toString("yyyy_MM_dd") + ".log";
    log_file.setFileName(file_name);
    roboflow_log_reader->SetLogFile(file_name);
    log_file.open(QFile::WriteOnly | QFile::Append | QFile::Text);
    log_stream.setDevice(&log_file);
}

void Log::OnCheckCurrentDate()
{
    if (QDateTime::currentDateTime().date() != curr_date) {
        curr_date = QDateTime::currentDateTime().date();
        RenameLogFileByDate();
        ClearOldLogs(DefaultLogsKeptDays);
    }
}

void Log::Reset()
{
    roboflow_log_reader->Reset();
    phoenix_log_reader->Reset();
}

QStringList Log::GetAllLines()
{
    QStringList all_lst;
    QFile file(log_file.fileName());
    file.open(QFile::ReadOnly | QFile::Text);
    QTextStream stream(&file);
    while (!stream.atEnd()) {
        all_lst << log::ReadLogLine(stream);
    }
    file.close();
    return all_lst;
}

QStringList Log::GetMoreLines(LogType log_type)
{
    if (log_type == LogType::RoboFlow) {
        QDir log_dir(LogPath);
        QFileInfoList file_list = log_dir.entryInfoList(QStringList{"*.log*"}, QDir::Files, QDir::Time);
        roboflow_log_reader->SetLogList(file_list);
        return roboflow_log_reader->GetMoreLines();
    } else {
        QFileInfo phoenix_info(PhoenixLogFile);
        phoenix_log_reader->SetLogList(QFileInfoList{} << phoenix_info);
        return phoenix_log_reader->GetMoreLines();
    }
}

QStringList Log::GetLatestInsertLines(LogType log_type)
{
    if (log_type == LogType::RoboFlow) {
        return roboflow_log_reader->GetLatestInsertLines();
    } else {
        return phoenix_log_reader->GetLatestInsertLines();
    }
}

void Log::ResetMessagePattern(bool format24h)
{
    QString message_format{};
    QString suffix = format24h ? "" : " AP";
    if (debug_output) {
        message_format = QString("%{time yyyy/MM/dd hh:mm:ss:zzz %1} %{file}:%{line}:"
                                 "%{function}: {%{category}} [%{type}]: %{message}").arg(suffix);
    } else if (trace_output) {
        message_format = QString("%{time yyyy/MM/dd hh:mm:ss:zzz %1} %{file}:%{line}:%{function}: {%{category}} "
                                 "[%{type}]: %{message}\n%{backtrace depth=10 separator=\"\n    \"}").arg(suffix);
    } else {
        message_format = QString("%{time yyyy/MM/dd hh:mm:ss:zzz %1} {%{category}} [%{type}]: %{message}").arg(suffix);
    }
    qSetMessagePattern(message_format);
}

Log& Log::Instance()
{
    static Log singleton{};
    return singleton;
}

}
}

