#ifndef SCENE_H
#define SCENE_H

#include <QObject>

#include <QImage>
#include <QPainter>

#include <geobase.h>

#include <omp.h>
#include <iostream>

class Scene : public QObject {
  Q_OBJECT
 public:
  explicit Scene(QObject *parent = nullptr);

  std::vector<LineSeg> line_list;  // scene

  std::vector<Point> beacon_list;  // valid beacons

  std::vector<Point> tra_list;  // valid trajectory

  bool loadDefult() {
    line_list.push_back(LineSeg(Point(0, 0), Vector(10, 0)));
    line_list.push_back(LineSeg(Point(0, 0), Vector(0, 10)));
    line_list.push_back(LineSeg(Point(10, 0), Vector(0, 10)));
    line_list.push_back(LineSeg(Point(0, 10), Vector(10, 0)));
    line_list.push_back(LineSeg(Point(5, 2), Vector(0, 6)));
    line_list.push_back(LineSeg(Point(3, 6), Vector(0, -3)));
    line_list.push_back(LineSeg(Point(5, 7), Vector(-3, 0)));
    line_list.push_back(LineSeg(Point(5, 3), Vector(3, 0)));
    line_list.push_back(LineSeg(Point(7, 4), Vector(0, 3)));

    drawScene();
    return true;
  }

  bool loadScene(const QString s_str);

  bool loadBeacon(const QString b_str);

  bool loadTrajectory(const QString t_str);

  bool drawScene();

  Vector toImage(const Vector &v);
  Point toImage(const Point &v);

  // transform parameters
  double img_height = 1000;
  double img_width = 1000;
  double x_scale = 100.0;
  double y_scale = 100.0;
  double x_offset = 0;
  double y_offset = 0;

 signals:
  void newImage(QImage img);

 public slots:
};

#endif  // SCENE_H
