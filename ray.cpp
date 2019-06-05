#include "ray.h"

Ray::Ray(QObject *parent) : QObject(parent) {}

double Ray::detect_intersection(LineSeg l) {
    return -1.0;
}
