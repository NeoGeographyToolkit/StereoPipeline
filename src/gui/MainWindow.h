#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <QMainWindow>
#include <string>

class QAction;
class QLabel;
class QTabWidget;
class PreviewGLWidget;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow();

private slots:
  void about();
  void update_status_bar(std::string const& s);
  void tab_switch(int index);

protected:
  void keyPressEvent(QKeyEvent *event);
  
private:
  void create_actions();
  void create_menus();
  void create_status_bar();


  QWidget *genInputTab();
  QWidget *genAlignmentTab();
  QWidget *genPreprocessTab();
  QWidget *genCostFunctionTab();
  QWidget *genCorrelateTab();

  QMenu *file_menu;
  QMenu *edit_menu;
  QMenu *help_menu;

  QLabel *status_label;
  QAction *about_action;
  QAction *exit_action;

  QTabWidget* m_tab_widget;
  
};

#endif // __MAINWINDOW_H__
