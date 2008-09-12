#include <QApplication>
#include <QWidget>
#include <QGLFormat>
#include "MainWindow.h"

// Standard Library
#include <iostream>

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

  if (!QGLFormat::hasOpenGL()) {
    std::cerr << "This system has no OpenGL support.\nExiting\n\n";
    return 1;
  }

  MainWindow main_window(argc, argv);
  main_window.show();
  return app.exec(); 
}
