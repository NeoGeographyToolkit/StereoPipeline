#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <QMainWindow>
#include <string>

class QAction;
class QLabel;
class PreviewGLWidget;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv);

private slots:
  void about();
  void update_status_bar(std::string const& s);
  void tab_switch(int index);
  
private:
  void create_actions();
  void create_menus();
  void create_status_bar();

  QWidget *MainWindow::genInputTab();
  QWidget *MainWindow::genPreprocessTab();

  QMenu *file_menu;
  QMenu *edit_menu;
  QMenu *help_menu;

  QLabel *status_label;
  QAction *about_action;
  QAction *exit_action;

  
};

#endif // __MAINWINDOW_H__
