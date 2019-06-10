#ifndef RAY_H
#define RAY_H

#include <geobase.h>
#include <vector>

#include <functional>

#include <iostream>

class Ray {
 public:
  Ray() {}

  bool Initial(Point s_p, Vector s_v) {
    start_point = s_p;
    cur_point = Point(s_p.x, s_p.y);
    cur_vec = Vector(s_v.x, s_v.y);
    return true;
  }

  Point start_point = Point(0, 0);
  Point cur_point = Point(0, 0);
  Vector cur_vec = Vector(1, 0);

  bool reached_flag = false;
  int beacon_id_ = 0;

  std::vector<LineSeg> line_list;

  /**
   * @brief detect_intersection
   * @param l
   * @return
   */
  double detect_intersection(const LineSeg &l, Point &intersect_point);

  /**
   * @brief reflection
   * @param p
   * @return
   */
  bool reflection(Point p, Vector norm_vec);

  bool reachedPoint(Point p, double max_dis);

  int beacon_id() const;
  void setBeacon_id(int beacon_id);
};

#endif  // RAY_H
