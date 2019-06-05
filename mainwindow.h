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

 public slots:
  void draw_image(QImage img);
 private slots:
  void on_actionRefresh_Scene_triggered();
};

#endif  // MAINWINDOW_H
