#include "log/LogReader.hpp"

#include <QDebug>
#include <QFile>
#include <QTextStream>

namespace rc {
namespace log {

QString ReadLogLine(QTextStream& stream)
{
    QString line = stream.readLine();
    if (line.contains("[warning]")) {
        line.replace("[warning]", "<font color=#B8860B>[warning]</font>");
    } else if ("[critical]") {
        line.replace("[critical]", "<font color=#FF0000>[critical]</font>");
    }
    return line;
}

LogReader::LogReader(const QString& file_name)
:
    log_file_name(file_name),
    curr_file_name(file_name)
{
}

void LogReader::Reset()
{
    log_lines.clear();
    pre_line_numbers = 0;
    curr_file_name = log_file_name;
    QFile file(log_file_name);
    file.open(QFile::ReadOnly | QFile::Text);
    pre_file_size = file.size();
    QTextStream stream(&file);
    while (!stream.atEnd()) {
        log_lines << ReadLogLine(stream);
    }

    log_it = log_lines.end();
    --log_it;
    no_more_log_in_one_file = false;
    file.close();
}

QStringList LogReader::GetMoreLines()
{
    QStringList log_lines_ret;
    if (no_more_log_in_one_file) {
        log_lines.clear();
        QFileInfoList::iterator it_file = logs_list.begin();
        for ( ; it_file != logs_list.end(); ++it_file) {
            if (it_file->filePath() == curr_file_name) {
                if (++it_file == logs_list.end()) {
                    //no more log
                    return log_lines_ret;
                } else {
                    QFile file(it_file->filePath());
                    curr_file_name = it_file->filePath();
                    file.open(QFile::ReadOnly | QFile::Text);
                    QTextStream stream(&file);
                    while (!stream.atEnd()) {
                        log_lines << ReadLogLine(stream);
                    }
                    log_it = log_lines.end();
                    --log_it;
                    file.close();
                }
                break;
            }
        }
    }

    if (log_lines.size() == 0) {
        return log_lines_ret;
    }
    for (int num = 0; log_it != log_lines.begin() && (num < MaxDisplayLines); --log_it) {  //show the latest log at the begining of ui
        log_lines_ret << *log_it;
        ++num;
    }
    log_lines_ret << *log_it;
     if (log_it == log_lines.begin()) {
         no_more_log_in_one_file = true;
     } else {
         no_more_log_in_one_file = false;
     }
    return log_lines_ret;
}

QStringList LogReader::GetLatestInsertLines()
{
    QStringList latest_log;
    QFile file(log_file_name);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        return latest_log;
    }
    qint64 file_size = file.size();

    qDebug() <<"file_size = " << file_size
                        << ", pre_file_size " << pre_file_size;
    if (file_size == pre_file_size) {
        return  latest_log;
    }
    QTextStream stream(&file);

    int line_num = 0;
    while (!stream.atEnd()) {
        ++line_num;
        if (line_num > pre_line_numbers) {
            latest_log << ReadLogLine(stream);
        } else {
            stream.readLine();
        }
    }
    qDebug() << "cuur_line_num = " << line_num << ", pre_line_numbers = " << pre_line_numbers;
    qDebug() <<"lattest lines = " << line_num - pre_line_numbers;
    pre_file_size = file_size;
    pre_line_numbers = line_num;

    file.close();
    return latest_log;
}

void LogReader::SetLogFile(const QString &file_name)
{
    log_file_name = file_name;
    curr_file_name = file_name;
}

void LogReader::SetLogList(const QFileInfoList& logs_info_list)
{
    logs_list = logs_info_list;
}

}
}
