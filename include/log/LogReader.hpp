#ifndef ROBOSIGNAL_LOG_LOGREADER_HPP
#define ROBOSIGNAL_LOG_LOGREADER_HPP

#include <QFileInfoList>
#include <QString>
#include <QStringList>
#include <QTextStream>

#include "robosignal_global.hpp"

constexpr const int MaxDisplayLines = 200;

namespace rc {
namespace log {

QString ReadLogLine(QTextStream& stream);

class ROBOSIGNALSHARED_EXPORT LogReader
{
public:
    explicit LogReader(const QString& file_name);
    void Reset();
    QStringList GetMoreLines();
    QStringList GetLatestInsertLines();
    void SetLogFile(const QString& file_name);
    void SetLogList(const QFileInfoList& logs_info_list);

private:
    QStringList::iterator log_it{};
    QStringList log_lines{};
    int pre_line_numbers{0};
    qint64 pre_file_size{0};
    bool no_more_log_in_one_file{false};
    QString log_file_name{};
    QFileInfoList logs_list{};
    QString curr_file_name{};
};

}
}
#endif
