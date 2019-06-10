#ifndef RAY_H
#define RAY_H

#include <geobase.h>
#include <vector>

#include <functional>

#include <assert.h>
#include <iostream>

class Ray {
 public:
  Ray() {}

  bool Initial(Point s_p, Vector s_v) {
    start_point = s_p;
    start_vec = s_v.normalize();
    cur_point = Point(s_p.x, s_p.y);
    cur_vec = Vector(s_v.x, s_v.y);
    total_distance_ = 0.0;
    return true;
  }

  Point start_point =
      Point(0, 0);  // start point of the ray(position of the uwb beacon)
  Vector start_vec = Vector(1, 0);  // initial orientation.
  Point cur_point = Point(0, 0);    // current point of ray tracing.
  Vector cur_vec = Vector(1, 0);    // current result of

  bool reached_flag = false;
  int beacon_id_ = 0;
  double total_distance_ = 0.0;

  std::vector<LineSeg> line_list;

  /////// IMPORTANT PARAMETERS
  double pl_dis_threshold =
      0.01;  // upper bound of distance between target point to current ray.

  /**
   * @brief detect_intersection
   *  whether current ray will intersect with provided line. if so, return the
   * distance.
   * @param l: line segment.
   * @return -1.0:not intersection, >0: distance from current point to
   * intersected point.
   */
  double detect_intersection(const LineSeg &l, Point &intersect_point);

  /**
   * @brief reflection
   * @param p
   * @param norm_vec
   * Reflection at point p and normal vector is norm_vec.
   * @return
   */
  bool reflection(Point p, Vector norm_vec);

  /**
   * @brief reachedPoint
   * @param p
   * @param max_dis
   * check whether the ray is near cross the target point.
   * @return
   */
  bool reachedPoint(Point p, double max_dis);

  /**
   * @brief updateRay
   * @param new_p
   * @param new_ori
   * refresh ray to new point and new orientation.
   * @return
   */
  bool updateRay(const Point &new_p, const Vector &new_ori);

  int beacon_id() const;
  void setBeacon_id(int beacon_id);
};

#endif  // RAY_H
