#include "scene.h"

Scene::Scene(QObject *parent) : QObject(parent) {}

bool Scene::drawScene() {
  initalAxis();
  QImage img(int(img_width_), int(img_height_), QImage::Format_RGB32);
  QRgb value = qRgb(255, 255, 255);
  img.fill(value);
  //  for (int i = 0; i < img_width; i++) {
  //    for (int j = 0; j < img_height; j++) {
  //      img.setPixel(i, j, value);
  //    }
  //  }
  QPainter painter(&img);

  if (line_list_.size() > 0) {
    //    painter.setPen(QColor(255, 0, 0));
    QPen line_pen;
    line_pen.setColor(QColor(255, 0, 0));
    line_pen.setWidth(10);
    painter.setPen(line_pen);

#pragma omp parallel for
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

  if (beacon_list_.size() > 0) {
    //    painter.setPen(QColor(55, 55, 0));
    QPen beacon_pen;
    beacon_pen.setWidth(20);
    beacon_pen.setColor(QColor(55, 55, 0));
    painter.setPen(beacon_pen);

#pragma omp parallel for
    for (int i = 0; i < beacon_list_.size(); ++i) {
      int xx = beacon_list_[i].x * x_scale_ + x_offset;
      int yy = beacon_list_[i].y * y_scale_ + y_offset;
      painter.drawPoint(xx, yy);
    }
  }

  if (tra_list_.size() > 0) {
    QPen tra_pen;
    tra_pen.setWidth(5);
    tra_pen.setColor(QColor(100, 0, 100));
    painter.setPen(tra_pen);
#pragma omp parallel for
    for (int i = 0; i < tra_list_.size() - 1; ++i) {
      Point p1 = toImage(tra_list_[i]);
      Point p2 = toImage(tra_list_[i + 1]);
      painter.drawLine(p1.x, p1.y, p2.x, p2.y);
    }

    QPen tra_p_pen;
    tra_p_pen.setWidth(20);
    tra_p_pen.setColor(QColor(0, 100, 200));
    painter.setPen(tra_p_pen);

#pragma omp parallel for
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

  emit newImage(img);
  return true;
}

bool Scene::initalAxis() {
  if (line_list_.size() > 0) {
    // search contain box of all line segments.
    double x_min(NAN), y_min(NAN), x_max(NAN), y_max(NAN);
    for (int i = 0; i < line_list_.size(); ++i) {
      double x1 = (line_list_[i].start_point.x);
      double y1 = (line_list_[i].start_point.y);
      double x2 = (x1 + line_list_[i].ori_vec.x);
      double y2 = (y1 + line_list_[i].ori_vec.y);
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

    img_height_ = 1000;
    img_width_ = int(double(img_height_) / double(y_max - y_min) *
                     double(x_max - x_min));

    // calculate transformation
    x_scale_ = double(img_width_) * 0.9 / (x_max - x_min);
    y_scale_ = double(img_height_) * 0.9 / (y_max - y_min);
    //    std::cout << "x scale,y_scale:" << x_scale << "," << y_scale <<
    //    std::endl;
    x_offset = (-1.0 * x_min) * x_scale_ + 0.05 * img_width_;
    y_offset = (-1.0 * y_min) * y_scale_ + 0.05 * img_height_;
    //    std::cout << "x offset,y offset:" << x_offset << "," << y_offset
    //              << std::endl;

    return true;
  } else {
    img_height_ = 1000;
    img_width_ = 1000;
    x_scale_ = 100.0;
    y_scale_ = 100.0;
    x_offset = 0;
    y_offset = 0;

    return true;
  }
}

Vector Scene::toImage(const Vector &v) {
  return Vector(v.x * x_scale_ + x_offset, v.y * y_scale_ + y_offset);
}

Point Scene::toImage(const Point &v) {
  return Point(v.x * x_scale_ + x_offset, v.y * y_scale_ + y_offset);
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
    line_list_.clear();
    for (auto ls : tmp_scene_list) {
      line_list_.push_back(ls);
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
    beacon_list_.clear();
    for (auto p : tmp_beacon_list) {
      beacon_list_.push_back(p);
    }

    drawScene();
    return true;
  } else {
    return false;
  }
}

bool Scene::loadTrajectory(const QString t_str) {
  std::vector<Point> tmp_tra_list;
  for (auto line_str : t_str.split("\n")) {
    auto unit_strs = line_str.split(",");
    if (unit_strs.size() > 0) {
      if (unit_strs[0] == "L" && unit_strs.size() == 6) {
        try {
          Point p(unit_strs[1].toDouble(), unit_strs[2].toDouble());
          Vector v_step(unit_strs[3].toDouble(), unit_strs[4].toDouble());
          int counter = unit_strs[5].toInt();
          for (int i = 0; i < counter + 1; ++i) {
            tmp_tra_list.push_back(Point(p.x, p.y));
            p = p + v_step;
          }
        } catch (std::exception &e) {
          std::cout << "some error at this line:[" << line_str.toStdString()
                    << "] with" << e.what() << std::endl;
        }
      } else if (unit_strs[0] == "P" && unit_strs.size() == 3) {
        try {
          Point p(unit_strs[1].toDouble(), unit_strs[2].toDouble());
          tmp_tra_list.push_back(p);

        } catch (std::exception &e) {
          std::cout << "some error at this line:[" << line_str.toStdString()
                    << "] with" << e.what() << std::endl;
        }
      } else {
        std::cout << "some error :" << line_str.toStdString() << std::endl;
      }
    }
  }
  if (tmp_tra_list.size() > 0) {
    tra_list_.clear();
    for (auto p : tmp_tra_list) {
      tra_list_.push_back(p);
    }
    trajectory_index_ = 0;
    drawScene();
    return true;
  } else {
    return false;
  }
}

void Scene::nextStep() {
  if (tra_list_.size() > 0) {
    trajectory_index_ += 1;
    if (trajectory_index_ < 0) {
      trajectory_index_ = 0;
    }
    if (trajectory_index_ > tra_list_.size() - 1) {
      trajectory_index_ = 0;
    }
    drawScene();
  }
}

void Scene::prevStep() {
  if (tra_list_.size() > 0) {
    trajectory_index_ -= 1;
    if (trajectory_index_ > tra_list_.size() - 1) {
      trajectory_index_ = tra_list_.size() - 1;
    }
    if (trajectory_index_ <= 0) {
      trajectory_index_ = tra_list_.size() - 1;
    }
    drawScene();
  }
}

void Scene::calStep() {
  if (line_list_.size() > 0 && beacon_list_.size() > 0 &&
      tra_list_.size() > trajectory_index_ && trajectory_index_ > 0) {
  }
}
