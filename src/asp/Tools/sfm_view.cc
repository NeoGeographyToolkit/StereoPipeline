// __BEGIN_LICENSE__
//  Copyright (c) 2006-2026, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

// Derived from MVE (Simon Fuhrmann, TU Darmstadt, BSD 3-Clause).

#include <asp/SfmView/GlCommon.h>
#include <asp/SfmView/SfmMainWindow.h>

#include <QApplication>
#include <QStyleFactory>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

void print_usage_and_exit() {
  std::cerr << "Usage: sfm_view [OPTIONS] cameras.tsai ...\n"
            << "Options:\n"
            << "  -h, --help      Print this help and exit\n"
            << "  --width VALUE   Window width in pixels\n"
            << "  --height VALUE  Window height in pixels\n";
  std::exit(EXIT_FAILURE);
}

int main(int argc, char** argv) {

  std::vector<std::string> cameras;
  int width = 1400, height = 1200; // default window size

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help")
      print_usage_and_exit();
    else if (arg == "--width" && i + 1 < argc)
      width = std::atoi(argv[++i]);
    else if (arg == "--height" && i + 1 < argc)
      height = std::atoi(argv[++i]);
    else
      cameras.push_back(arg);
  }

  if (cameras.empty()) {
    std::cerr << "No camera files specified.\n";
    print_usage_and_exit();
  }

  if (width < 10 || height < 10) {
    std::cerr << "Invalid width or height. Must be at least 10.\n";
    std::exit(EXIT_FAILURE);
  }

  // Set OpenGL version that Qt should use when creating a context.
  QSurfaceFormat fmt;
  fmt.setVersion(3, 3);
  fmt.setDepthBufferSize(24);
  fmt.setStencilBufferSize(8);
  fmt.setProfile(QSurfaceFormat::CoreProfile);
  QSurfaceFormat::setDefaultFormat(fmt);

  // Create application.
  QApplication app(argc, argv);
  QStyle* style = QStyleFactory::create("Cleanlooks");
  if (style != nullptr)
    QApplication::setStyle(style);

  // Create main window.
  SfmMainWindow win(width, height);

  // Load camera files
  win.load_scene(cameras);

  return app.exec();
}
