#include "mainwindow.h"
#include <iostream>
#include "ui_mainwindow.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  // initial draw
  initial_draw();

  connect(&cur_scene, SIGNAL(newImage(QImage)), this, SLOT(draw_image(QImage)));

  cur_scene.loadDefult();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::draw_image(QImage img) {
  int width = img_label->width();
  int height = img_label->height();
  std::cout << "drawing image:" << std::endl;

  //  if (img.width() / width * height > img.height()) {
  //    img_label->setPixmap(QPixmap::fromImage(img).scaledToWidth(width));
  //  } else {
  //    img_label->setPixmap(QPixmap::fromImage(img).scaledToHeight(width));
  //  }
  img_label->setPixmap(QPixmap::fromImage(img).scaled(
      width, height, Qt::AspectRatioMode::KeepAspectRatio,
      Qt::TransformationMode::SmoothTransformation));

  printf("draw image\n");
}

bool MainWindow::initial_draw() {
  QWidget *w = ui->img_widget;
  w->setAcceptDrops(true);
  QGridLayout *layer = new QGridLayout(w);
  layer->setSpacing(10);
  layer->setMargin(8);
  layer->setSizeConstraint(QLayout::SetNoConstraint);
  //    layout->setMargin(0);

  img_label = new QLabel(w);
  img_label->updatesEnabled();
  img_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  //  img_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  //  img_label->setFixedSize(QSize(600, 600));
  //  img_label->
  layer->addWidget(img_label, 0, 0);
  layer->setRowStretch(0, 1);
  layer->setColumnStretch(0, 1);

  return true;
}

void MainWindow::on_actionRefresh_Scene_triggered() { cur_scene.drawScene(); }
