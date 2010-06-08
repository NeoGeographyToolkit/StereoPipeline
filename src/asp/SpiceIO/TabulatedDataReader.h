// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file TabulatedDataReader.h
///

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

class TabulatedDataReader {
 public:
  /* Constructor / Destructor */
  TabulatedDataReader( const std::string &filename, const std::string &delimeters);
  ~TabulatedDataReader() { close(); }

  void close() {
    if (m_file.is_open()) {
      m_file.close();
    }
  }

  /* Accessors */
  int find_line_with_text(std::string query,
                          std::vector<std::string> &result);

 private:
  std::string m_delimeters;

  std::ifstream m_file;
};
