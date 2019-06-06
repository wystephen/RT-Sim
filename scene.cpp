#include "scene.h"

Scene::Scene(QObject *parent) : QObject(parent) {}

bool Scene::drawScene() {
  QImage img(1000, 1000, QImage::Format_RGB32);
  QRgb value = qRgb(255, 255, 255);
  img.fill(value);
  for (int i = 0; i < 1000; i++) {
    for (int j = 0; j < 1000; j++) {
      img.setPixel(i, j, value);
    }
  }
  QPainter painter(&img);
#pragma omp parallel for num_threads(6)
  for (int i = 0; i < line_list.size(); ++i) {
    int x1 = line_list[i].start_point.x * 100;
    int y1 = line_list[i].start_point.y * 100;
    int x2 = x1 + line_list[i].ori_vec.x * 100;
    int y2 = y1 + line_list[i].ori_vec.y * 100;
    painter.drawLine(x1, y1, x2, y2);
  }

  emit newImage(img);
  return true;
}

bool initialAxis() {}
