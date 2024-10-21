#include <iostream>
namespace graphicalDrive {

    float deviation = 10.0;
    const unsigned int field_size_mm = 1000, points = field_size_mm / deviation;
    unsigned int field[] = {};
    float new_deviation = field_size_mm / points;

    unsigned int getArrayIdViaRelativeCoordinates(unsigned int e_x, unsigned int e_y) {
        return e_y*points + e_x;
    }
    unsigned int getRelativeAxisPos(unsigned int a_x_or_y) {
        return a_x_or_y / new_deviation;
    }
    void setObstacle(unsigned int absoluteXRange[2], unsigned int absoluteYRange[2]) {
        int nX = sizeof(absoluteXRange) / sizeof(absoluteXRange[0]);
        std::sort(absoluteXRange, absoluteXRange + nX);
        int nY = sizeof(absoluteYRange) / sizeof(absoluteYRange[0]);
        std::sort(absoluteYRange, absoluteYRange + nY);
        for (int y = getRelativeAxisPos(absoluteYRange[0]); y <= getRelativeAxisPos(absoluteYRange[1]); y++) {
            for (int x = getRelativeAxisPos(absoluteXRange[0]); x <= getRelativeAxisPos(absoluteXRange[1]); x++) {
                field[getArrayIdViaRelativeCoordinates(x, y)] = 1; // the integer 1 stands for an obstacle
            }
        }
    }
    void setRobotLocation(int robotPos[2], int robotSize[2]) {
        int absoluteYRange[2] = {robotPos[1] - (robotSize[1] / 2), robotPos[1]  + (robotSize[1] / 2)};
        int absoluteXRange[2] = {robotPos[0] - (robotSize[0] / 2), robotPos[0] + (robotSize[0] / 2)};
        for (int y = getRelativeAxisPos(absoluteYRange[0]); y <= getRelativeAxisPos(absoluteYRange[1]); y++) {
            for (int x = getRelativeAxisPos(absoluteXRange[0]); x <= getRelativeAxisPos(absoluteXRange[1]); x++) {
                field[getArrayIdViaRelativeCoordinates(x, y)] = 2; // the integer 2 stands for the robot
            }
        }
    }
    void setTargetLocation(int absoluteTargetPos[2]) {
        field[getArrayIdViaRelativeCoordinates(getRelativeAxisPos(absoluteTargetPos[0]), getRelativeAxisPos(absoluteTargetPos[1]))] = 3; // the integer 3 stands for the wanted target location
    }



}
