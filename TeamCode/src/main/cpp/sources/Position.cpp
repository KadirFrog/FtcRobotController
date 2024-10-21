#include "../header/Position.h"

Position::Position(double x, double y, double rotation) {
    this->x = x;
    this->y = y;
    this->rotation = rotation;
}

void Position::set(double x, double y, double rotation) {
    this->x = x;
    this->y = y;
    this->rotation = rotation;
}

Position Position::setX(double x) {
    this->x = x;
    return *this;
}

Position Position::setY(double y) {
    this->y = y;
    return *this;
}

Position Position::setRotation(double rotation) {
    this->rotation = rotation;
    return *this;
}

std::string Position::to_string() {
    return (::to_string(x) + "/" + ::to_string(y) + "/" + ::to_string(rotation));
}