#ifndef RAY_H
#define RAY_H

#include <QObject>

#include <geobase.h>
#include <vector>

class Ray : public QObject {
  Q_OBJECT
 public:
  explicit Ray(QObject *parent = nullptr);

  bool Initial(Point s_p, Vector s_v) {
    start_point = s_p;
    cur_point = s_p;
    cur_vec = s_v;
  }

  Point start_point = Point(0, 0);
  Point cur_point = Point(0, 0);
  Vector cur_vec = Vector(1, 0);

  std::vector<LineSeg> line_list;

  /**
   * @brief detect_intersection
   * @param l
   * @return
   */
  double detect_intersection(LineSeg l);

 signals:

 public slots:
};

#endif  // RAY_H
