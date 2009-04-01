#ifndef __STEREO_GUI_SESSION_H__
#define __STEREO_GUI_SESSION_H__

#include <QObject>

#include <vw/Image.h>
#include <vw/Stereo.h>

#include "StereoSession.h"

class StereoGuiSession : public QObject {
  Q_OBJECT

  std::string m_output_prefix;
  std::string m_left_input_image;
  std::string m_right_input_image;
  
  std::string m_left_interest_point_file;
  std::string m_right_interest_point_file;
  std::string m_interest_point_match_file;
  
  std::string m_left_aligned_image;
  std::string m_right_aligned_image;
  
  StereoSession* m_session;
  
public:
  
  StereoGuiSession() {
    m_output_prefix = "results/out";
  }

  QObject const* qobject_ptr() const { return this; }
  
  std::string const& left_input_image() const { return m_left_input_image; }
  std::string const& right_input_image() const { return m_right_input_image; }

  std::string const& left_interest_point_file() const { return m_left_interest_point_file; }
  std::string const& right_interest_point_file() const { return m_right_interest_point_file; }
  std::string const& interest_point_match_file() const { return m_interest_point_match_file; }

  std::string const& left_aligned_image() const { return m_left_aligned_image; }
  std::string const& right_aligned_image() const { return m_right_aligned_image; }

  std::string const& output_prefix() const { return m_output_prefix; }

  StereoSession* session() const { return m_session; }
 

  void set_left_input_image(std::string const& filename) { 
    m_left_input_image = filename;
    emit left_input_image_changed(filename);
  }

  void set_right_input_image(std::string const& filename) { 
    m_right_input_image = filename;
    emit right_input_image_changed(filename);
  }

  void set_interest_point_filenames(std::string const& left, std::string const& right, std::string const& match) { 
    m_left_interest_point_file = left;
    m_right_interest_point_file = right;
    m_interest_point_match_file = match;
    emit interest_points_changed(left, right, match);
  }

  void set_left_aligned_image(std::string const& filename) { 
    m_left_aligned_image = filename;
    emit left_aligned_image_changed(filename);
  }

  void set_right_aligned_image(std::string const& filename) { 
    m_right_aligned_image = filename;
    emit right_aligned_image_changed(filename);
  }

  void set_output_prefix(std::string const& prefix) {
    m_output_prefix = prefix;
  }

  void set_session(StereoSession* session) {
    m_session = session;
  }

signals:

  void left_input_image_changed(std::string const& filename);
  void right_input_image_changed(std::string const& filename);
  void interest_points_changed(std::string const& left, std::string const& right, std::string const& match);
  void left_aligned_image_changed(std::string const& filename);
  void right_aligned_image_changed(std::string const& filename);      
};

                                   
  /// Return the singleton instance of the StereoGuiSession structure.
  /// The stereo gui session struct is created the first time this
  /// method is invoked.  You should *always* access the stereo gui
  /// session through this function.
  StereoGuiSession& stereo_gui_session();
#endif
