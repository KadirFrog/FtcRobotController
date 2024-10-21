#pragma once

#ifndef FTCROBOTCONTROLLER_POSITION_H
#define FTCROBOTCONTROLLER_POSITION_H


#include <optional>
#include <string>

using namespace std;

class Position {

public:
    double x;
    double y;
    double rotation;

    Position(double x, double y, double rotation);

    void set(double x, double y, double rotation);

    Position setX(double x);

    Position setY(double y);

    Position setRotation(double rotation);

    std::string to_string();
};


#endif //FTCROBOTCONTROLLER_POSITION_H
