#ifndef GEOBASE_H
#define GEOBASE_H

#include <math.h>
#include <cmath>

#define EPS 1e-8

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
  double len()  //ÏòÁ¿³¤¶È
  {
    return sqrt(x * x + y * y);
  }
  Vector reflect(const Vector normal) {
    double idotn2 = (normal.x * x + normal.y * y) * -2.;
    return {x + idotn2 * normal.x, y + idotn2 * normal.y};
  }
  Vector normalize() {
    double length = len();
    if (length > 0) return {x / length, y / length};
    return {0., 0.};
  }
};

struct Point {
  Point(double ix, double iy) : x(ix), y(iy) {}
  double x;
  double y;
  Point operator+(Vector v) { return {x + v.x, y + v.y}; }
  Vector operator-(Point p) { return {x - p.x, y - p.y}; }
  Point operator-(Vector v) { return {x - v.x, y - v.y}; }
  bool IsValid() { return x >= 0. && x <= 1. && y > 0. && y < 1.; }
};

struct LineSeg {
  LineSeg(Point sp, Vector sv) : start_point(sp.x, sp.y), ori_vec(sv.x, sv.y) {}
  Point start_point;
  Vector ori_vec;

  Vector getNormalVector() {
    if (std::fabs(ori_vec.x) > EPS) {
      return Vector(-1.0 * ori_vec.y / ori_vec.x, 1.0);
    }
  }
};

#endif  // GEOBASE_H
