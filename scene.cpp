#include "scene.h"

Scene::Scene(QObject *parent) : QObject(parent) {}

bool Scene::drawScene() {
  initalAxis();
  QImage img(int(img_width), int(img_height), QImage::Format_RGB32);
  QRgb value = qRgb(255, 255, 255);
  img.fill(value);
  //  for (int i = 0; i < img_width; i++) {
  //    for (int j = 0; j < img_height; j++) {
  //      img.setPixel(i, j, value);
  //    }
  //  }
  QPainter painter(&img);

  if (line_list.size() > 0) {
    //    painter.setPen(QColor(255, 0, 0));
    QPen line_pen;
    line_pen.setColor(QColor(255, 0, 0));
    line_pen.setWidth(10);
    painter.setPen(line_pen);

#pragma omp parallel for num_threads(6)
    for (int i = 0; i < line_list.size(); ++i) {
      int x1 = line_list[i].start_point.x * x_scale + x_offset;
      int y1 = line_list[i].start_point.y * y_scale + y_offset;
      int x2 = (line_list[i].start_point.x + line_list[i].ori_vec.x) * x_scale +
               x_offset;
      int y2 = (line_list[i].start_point.y + line_list[i].ori_vec.y) * y_scale +
               y_offset;
      painter.drawLine(x1, y1, x2, y2);
    }
  }

  if (beacon_list.size() > 0) {
    //    painter.setPen(QColor(55, 55, 0));
    QPen beacon_pen;
    beacon_pen.setWidth(5);
    beacon_pen.setColor(QColor(55, 55, 0));
    painter.setPen(beacon_pen);

    //#pragma omp parallel for num_threads(6)
    for (int i = 0; i < beacon_list.size(); ++i) {
      Point img_beacon_vec = toImage(beacon_list[i]);
      painter.drawEllipse(img_beacon_vec.x, img_beacon_vec.y, 10, 10);
    }
  }

  emit newImage(img);
  return true;
}

bool Scene::initalAxis() {
  if (line_list.size() > 0) {
    // search contain box of all line segments.
    double x_min(NAN), y_min(NAN), x_max(NAN), y_max(NAN);
    for (int i = 0; i < line_list.size(); ++i) {
      double x1 = (line_list[i].start_point.x);
      double y1 = (line_list[i].start_point.y);
      double x2 = (x1 + line_list[i].ori_vec.x);
      double y2 = (y1 + line_list[i].ori_vec.y);
      if (std::min(x1, x2) < x_min || std::isnan(x_min)) {
        x_min = std::min(x1, x2);
      }
      if (std::max(x1, x2) > x_max || std::isnan(x_max)) {
        x_max = std::max(x1, x2);
      }
      if (std::min(y1, y2) < y_min || std::isnan(y_min)) {
        y_min = std::min(y1, y2);
      }
      if (std::max(y1, y2) > y_max || std::isnan(y_max)) {
        y_max = std::max(y1, y2);
      }
    }

    img_height = 1000;
    img_width =
        int(double(img_height) / double(y_max - y_min) * double(x_max - x_min));

    // calculate transformation
    x_scale = double(img_width) * 0.9 / (x_max - x_min);
    y_scale = double(img_height) * 0.9 / (y_max - y_min);
    //    std::cout << "x scale,y_scale:" << x_scale << "," << y_scale <<
    //    std::endl;
    x_offset = (-1.0 * x_min) * x_scale + 0.05 * img_width;
    y_offset = (-1.0 * y_min) * y_scale + 0.05 * img_height;
    //    std::cout << "x offset,y offset:" << x_offset << "," << y_offset
    //              << std::endl;

    return true;
  } else {
    img_height = 1000;
    img_width = 1000;
    x_scale = 100.0;
    y_scale = 100.0;
    x_offset = 0;
    y_offset = 0;

    return true;
  }
}

Vector Scene::toImage(const Vector &v) {
  return Vector(v.x * x_scale + x_offset, v.y * y_scale + y_offset);
}

Point Scene::toImage(const Point &v) {
  return Point(v.x * x_scale + x_offset, v.y * y_scale + y_offset);
}

bool Scene::loadScene(const QString s_str) {
  std::vector<LineSeg> tmp_scene_list;
  for (auto line_str : s_str.split("\n")) {
    auto num_strs = line_str.split(',');
    if (num_strs.size() == 4) {
      try {
        double px = num_strs[0].toDouble();
        double py = num_strs[1].toDouble();
        double vx = num_strs[2].toDouble();
        double vy = num_strs[3].toDouble();
        tmp_scene_list.push_back(LineSeg(Point(px, py), Vector(vx, vy)));

      } catch (std::exception &e) {
        std::cout << "convert string to int with some error"
                  << line_str.toStdString() << std::endl;
      }

    } else {
      std::cout << "current scene string with some error:"
                << line_str.toStdString() << std::endl;
    }
  }

  if (tmp_scene_list.size() > 0) {
    line_list.clear();
    for (auto ls : tmp_scene_list) {
      line_list.push_back(ls);
    }
    drawScene();
    return true;

  } else {
    return false;
  }
}

bool Scene::loadBeacon(const QString b_str) {
  std::vector<Point> tmp_beacon_list;
  for (auto line_str : b_str.split("\n")) {
    //    std::cout << line_str.toStdString() << std::endl;
    auto num_strs = line_str.split(',');
    if (num_strs.size() == 2) {
      try {
        double x = num_strs[0].toDouble();
        double y = num_strs[1].toDouble();
        tmp_beacon_list.push_back(Point(x, y));

      } catch (std::exception &e) {
        std::cout << "some error when trying to conver to int" << std::endl;
      }
    } else {
      std::cout << "There are some problem of this line of data" << std::endl;
    }
  }

  if (tmp_beacon_list.size() > 0) {
    beacon_list.clear();
    for (auto p : tmp_beacon_list) {
      beacon_list.push_back(p);
    }

    drawScene();
    return true;
  } else {
    return false;
  }
}

bool Scene::loadTrajectory(const QString t_str) { return true; }
