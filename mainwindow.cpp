#include "mainwindow.h"
#include <iostream>
#include "ui_mainwindow.h"
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  // initial draw
  initial_draw();

  connect(&cur_scene, SIGNAL(newImage(QImage)), this, SLOT(draw_image(QImage)));

  connect(this, SIGNAL(next_step()), &cur_scene, SLOT(nextStep()));

  connect(this, SIGNAL(prev_step()), &cur_scene, SLOT(prevStep()));

  connect(this, SIGNAL(cal_step()), &cur_scene, SLOT(calStep()));

  //  cur_scene.loadDefult();
}

MainWindow::~MainWindow() { delete ui; }

/**
 * @brief MainWindow::draw_image
 * draw image of current scene.
 * @param img
 */
void MainWindow::draw_image(QImage img) {
  int width = img_label->width();
  int height = img_label->height();
  QPixmap q_pix = QPixmap::fromImage(img).scaled(
      width, height, Qt::AspectRatioMode::KeepAspectRatio,
      Qt::TransformationMode::SmoothTransformation);
  img_label->setPixmap(q_pix);
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

void MainWindow::on_btn_refresh_beacon_clicked() {
  QString beacon_str;
  beacon_str = ui->txt_beacon->toPlainText();
  cur_scene.loadBeacon(beacon_str);
}

void MainWindow::on_btn_refresh_scene_clicked() {
  QString scene_str = ui->txtScene->toPlainText();
  cur_scene.loadScene(scene_str);
}

void MainWindow::on_btn_refresh_trajectory_clicked() {
  QString tra_str = ui->txtTrajectory->toPlainText();
  cur_scene.loadTrajectory(tra_str);
}

void MainWindow::on_btn_pre_step_clicked() {
  emit prev_step();
  emit cal_step();
}

void MainWindow::on_btn_newx_step_clicked() {
  emit next_step();
  emit cal_step();
}

void MainWindow::on_btn_calculate_step_clicked() { emit cal_step(); }

void MainWindow::on_actionLoad_Trajectory_From_File_triggered() {}

void MainWindow::on_actionSave_Trajectory_To_File_triggered() {}

void MainWindow::on_btn_global_search_clicked() {
  // searching all result in this scene needn't any trajectory.
}
