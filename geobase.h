#ifndef GEOBASE_H
#define GEOBASE_H

#include <math.h>
#include <cmath>

#include <iostream>

#define EPS 1e-8

#ifndef M_PI
#define M_PI 3.1415926
#endif

struct Vector {
  Vector(double ix, double iy) : x(ix), y(iy) {}
  double x;
  double y;
  Vector operator+(Vector v) { return {x + v.x, y + v.y}; }
  Vector operator-(Vector v) { return {x - v.x, y - v.y}; }
  Vector operator-(void) { return {-x, -y}; }
  double operator*(Vector v)  //µã³Ë
  {
    return x * v.x + y * v.y;
  }
  Vector operator*(double f)  //±¶Êý
  {
    return {x * f, y * f};
  }
  Vector operator/(double f) { return {x / f, y / f}; }
  // length of the vector.
  double len() const { return sqrt(x * x + y * y); }
  Vector reflect(const Vector normal) {
    double idotn2 = (normal.x * x + normal.y * y) * -2.;
    return {x + idotn2 * normal.x, y + idotn2 * normal.y};
  }
  Vector normalize() const {
    double length = len();
    if (length > EPS) {
      return {x / length, y / length};
    } else {
      std::cout << "some proble of vector:" << x << "," << y << std::endl;
      return {0., 0.};
    }
  }

  double cos(Vector v) {
    double cross = x * v.x + y * v.y;
    double n1 = sqrt(x * x + y * y);
    double n2 = sqrt(v.x * v.x + v.y * v.y);
    if (std::abs(n1) < EPS || std::abs(n2) < EPS) {
      std::cout << "n1 or n2 len equal to zero" << std::endl;
      return -1.0;
    } else {
      return cross / n1 / n2;
    }
  }
  std::string toString() {
    return std::string("{\"type\":\"Vector\",\"x\":" + std::to_string(x) +
                       ",\"y\":" + std::to_string(y) + "}");
  }
};

struct Point {
  Point(double ix, double iy) : x(ix), y(iy) {}
  double x;
  double y;
  Point operator+(Vector v) { return {x + v.x, y + v.y}; }
  Vector operator-(Point p) { return {x - p.x, y - p.y}; }
  Point operator-(Vector v) { return {x - v.x, y - v.y}; }
  bool isClose(Point p) { return Vector(p.x - x, p.y - y).len() < EPS; }

  std::string toString() {
    return std::string("{\"type\":\"Point\",\"x\":" + std::to_string(x) +
                       ",\"y\":" + std::to_string(y) + "}");
  }
};

struct LineSeg {
  LineSeg(Point sp, Vector sv) : start_point(sp.x, sp.y), ori_vec(sv.x, sv.y) {}
  LineSeg() : start_point(0, 0), ori_vec(0, 0) {}
  Point start_point;
  Vector ori_vec;

  Vector getNormalVector() {
    if (std::fabs(ori_vec.x) > EPS) {
      return Vector(-1.0 * ori_vec.y / ori_vec.x, 1.0);
    } else {
      return Vector(1.0, 0.0);
    }
  }

  std::string toString() {
    return std::string(
        "{\"type\":\"LineSeg\",\"start_point\":" + start_point.toString() +
        ",\"ori_vec\":" + ori_vec.toString() + "}");
  }
};

#endif  // GEOBASE_H
