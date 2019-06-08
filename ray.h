#ifndef RAY_H
#define RAY_H

#include <geobase.h>
#include <vector>

#include <functional>

#include <iostream>

class Ray {
 public:
  Ray() {}

  //  ~Ray() {}

  //  Ray(const Ray &r)
  //      : start_point(r.start_point.x, r.start_point.y),
  //        cur_point(r.cur_point.x, r.cur_point.y),
  //        cur_vec(r.cur_vec.x, r.cur_vec.y),
  //        line_list() {
  //    for (LineSeg l : r.line_list) {
  //      line_list.push_back(LineSeg(Point(l.start_point.x, l.start_point.y),
  //                                  Vector(l.ori_vec.x, l.ori_vec.y)));
  //    }
  //  }

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
};

#endif  // RAY_H
