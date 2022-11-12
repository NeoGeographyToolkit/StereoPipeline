// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
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


/// \file chooseFilesDlg.h
///
/// A dialog for user to choose which files to show.
///
#ifndef __STEREO_GUI_CHOOSE_FILES_DLG_H__
#define __STEREO_GUI_CHOOSE_FILES_DLG_H__

// Qt
#include <QWidget>
#include <QTableWidget>

#include <map>
#include <string>

namespace vw { namespace gui {

  class imageData;
  
  /// Class to create a file list on the left side of the window
  class chooseFilesDlg: public QWidget{
    Q_OBJECT

  public:
    chooseFilesDlg(QWidget * parent);
    ~chooseFilesDlg();
    void chooseFiles(const std::vector<std::string> & image_files);

    QTableWidget * getFilesTable(){ return m_filesTable; }
    static QString selectFilesTag(){ return ""; }

    // Check if the given image is hidden (not shown) based on the table checkbox  
    bool isHidden(std::string const& image) const;
    // Hide the given image  
    void hide(std::string const& image);
    // Show the given image  
    void unhide(std::string const& image);

    // Show only first two images; this is the best default for pairwise stereo
    void showTwoImages();
    
    // Show all images
    void showAllImages();
    
  private:
    int imageRow(std::string const& image) const;
    QTableWidget * m_filesTable;
    void keyPressEvent(QKeyEvent *event);
    std::map<std::string, int> image_to_row;
  };
  
}} // namespace vw::gui

#endif  // __STEREO_GUI_CHOOSE_FILES_DLG_H__
