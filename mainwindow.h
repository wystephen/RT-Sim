#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QLabel>

#include <scene.h>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  Scene cur_scene;

 private:
  Ui::MainWindow *ui;

  QLabel *img_label = nullptr;

  bool initial_draw();

 signals:
  void next_step();
  void prev_step();
  void cal_step();
  void cal_whole_tra();
  void cal_whole_scene();
  void stop_cal();

 public slots:
  void draw_image(QImage img);
 private slots:
  void on_actionRefresh_Scene_triggered();
  void on_btn_refresh_beacon_clicked();
  void on_btn_refresh_scene_clicked();
  void on_btn_refresh_trajectory_clicked();
  void on_btn_pre_step_clicked();
  void on_btn_newx_step_clicked();
  void on_btn_calculate_step_clicked();
  void on_actionLoad_Trajectory_From_File_triggered();
  void on_actionSave_Trajectory_To_File_triggered();
  void on_btn_global_search_clicked();
  void on_btn_cal_whole_tra_clicked();
  void on_btn_stop_sim_clicked();
};

#endif  // MAINWINDOW_H
