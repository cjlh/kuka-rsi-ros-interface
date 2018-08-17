// Class header
#include "KukaPose.h"


/*
 * TODO.
 */
KukaPose::KukaPose(double x, double y, double z, double a, double b, double c) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->a = a;
    this->b = b;
    this->c = c;
}


/*
 * Destructor..
 */
KukaPose::~KukaPose() {}


/*
 * Returns the X value of the position.
 */
double KukaPose::getX() {
    return this->x;
}


/*
 * Returns the Y value of the position.
 */
double KukaPose::getY() {
    return this->y;
}


/*
 * Returns the Z value of the position.
 */
double KukaPose::getZ() {
    return this->z;
}


/*
 * Returns the A value of the position.
 */
double KukaPose::getA() {
    return this->a;
}


/*
 * Returns the B value of the position.
 */
double KukaPose::getB() {
    return this->b;
}


/*
 * Returns the C value of the position.
 */
double KukaPose::getC() {
    return this->c;
}

/*
 * Returns true if the values of X, Y, Z, A, B, C are all 0, and
 * false otherwise.
 */
bool KukaPose::isZero() {
    return (this->x == 0 && this->y == 0 && this->z == 0 &&
            this->a == 0 && this->b == 0 && this->c == 0);
}