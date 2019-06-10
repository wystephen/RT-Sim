#ifndef SCENE_H
#define SCENE_H

#include <QObject>

#include <QImage>
#include <QPainter>

#include <geobase.h>
#include <ray.h>

#include <omp.h>
#include <iostream>

class Scene : public QObject {
  Q_OBJECT
 public:
  explicit Scene(QObject *parent = nullptr);

  std::vector<LineSeg> line_list_;  // scene

  std::vector<Point> beacon_list_;  // valid beacons

  std::vector<Point> tra_list_;  // valid trajectory
  int trajectory_index_ = -1;

  std::vector<Ray> valid_ray_list_;
  double angle_resolution_ = 0.1 / 180.0 * M_PI;
  double reach_threshold_ = 0.1;

  /**
   * @brief loadDefult
   * Load defualt empty scene.
   * @return
   */
  bool loadDefult() {
    line_list_.push_back(LineSeg(Point(0, 0), Vector(10, 0)));
    line_list_.push_back(LineSeg(Point(0, 0), Vector(0, 10)));
    line_list_.push_back(LineSeg(Point(10, 0), Vector(0, 10)));
    line_list_.push_back(LineSeg(Point(0, 10), Vector(10, 0)));
    line_list_.push_back(LineSeg(Point(5, 2), Vector(0, 6)));
    line_list_.push_back(LineSeg(Point(3, 6), Vector(0, -3)));
    line_list_.push_back(LineSeg(Point(5, 7), Vector(-3, 0)));
    line_list_.push_back(LineSeg(Point(5, 3), Vector(3, 0)));
    line_list_.push_back(LineSeg(Point(7, 4), Vector(0, 3)));
    //    line_list.push_back(LineSeg(Point(10, 5), Vector(10, 0)));

    drawScene();
    return true;
  }

  /**
   * @brief loadScene
   * @param s_str
   *  each line of string represented a line segment in the scene.
   * format:
   * start_point_x,start_point_y,ori_vec_x,ori_vec_y
   * @return
   */
  bool loadScene(const QString s_str);

  /**
   * @brief loadBeacon
   * @param b_str
   * each line of string represented a beacon
   * beacon_x, beacon_y
   * @return
   */
  bool loadBeacon(const QString b_str);

  /**
   * @brief loadTrajectory
   * @param t_str
   * @return
   */
  bool loadTrajectory(const QString t_str);

  /**
   * @brief drawScene
   * draw the scene, including scene, beacon, trajectory, current pose, current
   * ray tracing result.
   * @return
   */
  bool drawScene();

  /**
   * @brief initalAxis
   * initial axis based on line segment !!! WARNING.
   * @return
   */
  bool initalAxis();

  /**
   * @brief toImage
   * @param v in image frame
   * @return
   */
  Vector toImage(const Vector &v);
  /**
   * @brief toImage
   * @param v in image frame
   * @return
   */
  Point toImage(const Point &v);

  // transform parameters
  double img_height_ = 1000;
  double img_width_ = 1000;
  double x_scale_ = 100.0;
  double y_scale_ = 100.0;
  double x_offset = 0;
  double y_offset = 0;

  bool calRayTracing(Point target_point);

  /**
   * @brief drawEnvironment
   *
   * @param painter
   * @return
   */
  bool drawEnvironment(QPainter &painter);

  /**
   * @brief drawBeacons
   * @param painter
   * Draw current Beacons.
   * @return
   */
  bool drawBeacons(QPainter &painter);

  bool drawTrajectory(QPainter &painter);

  /**
   * @brief drawRay
   * @param painter
   * @return
   */
  bool drawRay(QPainter &painter);

 signals:
  void newImage(QImage img);

 public slots:

  /**
   * @brief next_step
   * point to next road point.
   */
  void nextStep();
  /**
   * @brief prev_step
   * select previous road point.
   */
  void prevStep();
  /**
   * @brief cal_step
   * calculate ray tracing to current point.
   */
  void calStep();
};

inline void testRayIntersection() {
  Ray ray;
  ray.Initial(Point(0, 0), Vector(1, 0));

  for (int i = 0; i < 10; ++i) {
    Point ip(0, 0);
    LineSeg l(Point(2, i - 5), Vector(3, 3));
    double dis = ray.detect_intersection(l, ip);
    std::cout << "intersection checking:" << i << "ray. inter:" << dis
              << "ip:" << ip.x << "," << ip.y << std::endl;
  }
}
inline void testRayIntersectionY() {
  Ray ray;
  ray.Initial(Point(0, 0), Vector(0, 1));

  for (int i = 0; i < 10; ++i) {
    Point ip(0, 0);
    LineSeg l(Point(i - 5, 3), Vector(3, 0));
    double dis = ray.detect_intersection(l, ip);
    std::cout << "intersection checkingY:" << i << "ray. inter:" << dis
              << "ip:" << ip.x << "," << ip.y << std::endl;
  }
}
#endif  // SCENE_H
