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

  bool drawEnvironment(QPainter &painter) {
    if (line_list_.size() > 0) {
      //    painter.setPen(QColor(255, 0, 0));
      QPen line_pen;
      line_pen.setColor(QColor(255, 0, 0));
      line_pen.setWidth(10);
      painter.setPen(line_pen);

      //#pragma omp parallel for
      for (int i = 0; i < line_list_.size(); ++i) {
        int x1 = line_list_[i].start_point.x * x_scale_ + x_offset;
        int y1 = line_list_[i].start_point.y * y_scale_ + y_offset;
        int x2 =
            (line_list_[i].start_point.x + line_list_[i].ori_vec.x) * x_scale_ +
            x_offset;
        int y2 =
            (line_list_[i].start_point.y + line_list_[i].ori_vec.y) * y_scale_ +
            y_offset;
        painter.drawLine(x1, y1, x2, y2);
      }
    }
    return true;
  }

  bool drawBeacons(QPainter &painter) {
    if (beacon_list_.size() > 0) {
      //    painter.setPen(QColor(55, 55, 0));
      QPen beacon_pen;
      beacon_pen.setWidth(20);
      beacon_pen.setColor(QColor(55, 55, 0));
      painter.setPen(beacon_pen);

      //#pragma omp parallel for
      for (int i = 0; i < beacon_list_.size(); ++i) {
        int xx = beacon_list_[i].x * x_scale_ + x_offset;
        int yy = beacon_list_[i].y * y_scale_ + y_offset;
        painter.drawPoint(xx, yy);
      }
    }
    return true;
  }

  bool drawTrajectory(QPainter &painter) {
    if (tra_list_.size() > 0) {
      QPen tra_pen;
      tra_pen.setWidth(5);
      tra_pen.setColor(QColor(100, 0, 100));
      painter.setPen(tra_pen);
      //#pragma omp parallel for
      for (int i = 0; i < tra_list_.size() - 1; ++i) {
        Point p1 = toImage(tra_list_[i]);
        Point p2 = toImage(tra_list_[i + 1]);
        painter.drawLine(p1.x, p1.y, p2.x, p2.y);
      }

      QPen tra_p_pen;
      tra_p_pen.setWidth(20);
      tra_p_pen.setColor(QColor(0, 100, 200));
      painter.setPen(tra_p_pen);

      //#pragma omp parallel for
      for (int i = 0; i < tra_list_.size(); ++i) {
        Point p = toImage(tra_list_[i]);
        painter.drawPoint(p.x, p.y);
      }

      if (trajectory_index_ >= 0 && trajectory_index_ < tra_list_.size()) {
        tra_p_pen.setColor(QColor(0, 10, 250));
        tra_p_pen.setWidth(30);
        painter.setPen(tra_p_pen);
        Point p = toImage(tra_list_[trajectory_index_]);
        painter.drawPoint(p.x, p.y);
      }
    }
    return true;
  }

  bool drawRay(QPainter &painter) {
    if (valid_ray_list_.size() > 0) {
      QPen tracing_pen;
      tracing_pen.setWidth(2);
      tracing_pen.setColor(QColor(200, 200, 0));
      painter.setPen(tracing_pen);

      for (int i = 0; i < valid_ray_list_.size(); ++i) {
        Ray &ray = valid_ray_list_[i];

        for (int k = 0; k < ray.line_list.size(); ++k) {
          auto l = ray.line_list[k];
          Point sp = toImage(l.start_point);
          Point ep = toImage(Point(l.start_point.x + l.ori_vec.x,
                                   l.start_point.y + l.ori_vec.y));
          painter.drawLine(sp.x, sp.y, ep.x, ep.y);
          painter.drawLine(ep.x - 10, ep.y - 10, ep.x + 10, ep.y + 10);
          painter.drawLine(ep.x - 10, ep.y + 10, ep.x + 10, ep.y - 10);
        }
      }

      return true;
    } else {
      return false;
    }
  }

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
