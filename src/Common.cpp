#include "Common.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <QCoreApplication>
#include <QDebug>
#include <QStringList>
#include <QThread>
#define log_category ::rc::log::robot_controller
#include "log/Log.hpp"
#include "MyCobot.hpp"

namespace rc {
int current_robot = 0;

bool DoubleEqual(double a, double b, double epsilon)
{
    return std::abs(a - b) < epsilon;
}

bool CoordEqual(double a, double b, double epsilon)
{
    return std::abs(a - b) < epsilon;
}

bool CoordsEqual(const Coords& c1, const Coords& c2, double epsilon)
{
    return (CoordEqual(c1[X], c2[X], epsilon) &&
            CoordEqual(c1[Y], c2[Y], epsilon) &&
            CoordEqual(c1[Z], c2[Z], epsilon) &&
            CoordEqual(c1[RX], c2[RX], epsilon) &&
            CoordEqual(c1[RY], c2[RY], epsilon) &&
            CoordEqual(c1[RZ], c2[RZ], epsilon));
}

bool IsCoordsZero(const Coords& c1)
{
    return (CoordEqual(c1[0], 0.0) &&
            CoordEqual(c1[1], 0.0) &&
            CoordEqual(c1[2], 0.0) &&
            CoordEqual(c1[3], 0.0) &&
            CoordEqual(c1[4], 0.0) &&
            CoordEqual(c1[5], 0.0));
}

QDebug operator << (QDebug debug, const Coords& c)
{
    QDebugStateSaver saver(debug);
    debug.nospace().noquote() << "[" << c[X] << ", " << c[Y] << ", " << c[Z]
        << ", " << c[RX] << ", " << c[RY] << ", " << c[RZ] << "]";
    return debug;
}

std::string CoordsToString(const Coords& coords)
{
    std::string str{};
    str += '[';
    str += std::to_string(coords[X]) + ",";
    str += std::to_string(coords[Y]) + ",";
    str += std::to_string(coords[Z]) + ",";
    str += std::to_string(coords[RX]) + ",";
    str += std::to_string(coords[RY]) + ",";
    str += std::to_string(coords[RZ]);
    str += ']';
    return str;
}

std::string AnglesToString(const Angles& angles)
{
    return CoordsToString(angles);
}

Coords StringToCoords(const std::string& coords_str)
{
    std::string clean_str{coords_str};
    //clean_str.erase(std::remove(clean_str.begin(), clean_str.end(), ','),
    //        clean_str.end());
    std::replace(clean_str.begin(), clean_str.end(), ',', ' ');
    clean_str.erase(std::remove(clean_str.begin(), clean_str.end(), '['),
            clean_str.end());
    clean_str.erase(std::remove(clean_str.begin(), clean_str.end(), ']'),
            clean_str.end());
    std::istringstream iss(clean_str);
    std::vector<double> coords((std::istream_iterator<double>(iss)),
                                std::istream_iterator<double>());
    Coords c{};
    if (coords.size() < 6)
        return c;
    c[X] = coords[X];
    c[Y] = coords[Y];
    c[Z] = coords[Z];
    c[RX] = coords[RX];
    c[RY] = coords[RY];
    c[RZ] = coords[RZ];
    return c;
}

Angles StringToAngles(const std::string& angles_str)
{
    return StringToCoords(angles_str);
}

std::string CoordsToGcode(const Coords& coords)
{
    std::string gcode{};
    gcode += 'X' + std::to_string(coords[X]);
    gcode += 'Y' + std::to_string(coords[Y]);
    gcode += 'Z' + std::to_string(coords[Z]);
    gcode += 'A' + std::to_string(coords[RX]);
    gcode += 'B' + std::to_string(coords[RY]);
    gcode += 'C' + std::to_string(coords[RZ]);
    return gcode;
}

std::string AnglesToGcode(const Angles& angles)
{
    return CoordsToGcode(angles);
}

Coords InvalidCoords()
{
    return Coords{10000, 10000, 10000, 10000, 10000, 10000};
}

Angles InvalidAngles()
{
    return Angles{10000, 10000, 10000, 10000, 10000, 10000};
}

Angles GcodeToAngles(const QString &gcode)
{
#if QT_VERSION <= QT_VERSION_CHECK(5, 12, 10)
    QStringList list = gcode.split(QRegExp("X|Y|Z|A|B|C"), QString::SkipEmptyParts);
#else
    QStringList list = gcode.split(QRegExp("X|Y|Z|A|B|C"), Qt::SkipEmptyParts);
#endif
    if (list.size() != 6) {
        return InvalidAngles();
    }
    Coords c{};
    c[X] = list.at(0).toDouble();
    c[Y] = list.at(1).toDouble();
    c[Z] = list.at(2).toDouble();
    c[RX] = list.at(3).toDouble();
    c[RY] = list.at(4).toDouble();
    c[RZ] = list.at(5).toDouble();
    return c;
}

}
