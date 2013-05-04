// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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


/// \file DiskImageResourceDDD.h
///
/// Provides support for Malin Space Science System's DDD image file
/// format.
///
#ifndef __VW_FILEIO_DISK_IMAGE_RESOUCE_DDD_H__
#define __VW_FILEIO_DISK_IMAGE_RESOUCE_DDD_H__

#include <map>
#include <string>
#include <fstream>

#include <vw/Image/PixelTypes.h>
#include <vw/FileIO/DiskImageResource.h>

namespace vw
{
  class DiskImageResourceDDD : public DiskImageResource
  {
  public:

    DiskImageResourceDDD(std::string const& filename)
      : DiskImageResource(filename)
    {
      open(filename);
    }
    DiskImageResourceDDD(std::string const& filename, ImageFormat const& format)
      : DiskImageResource(filename)
    {
      create(filename, format);
    }
    virtual ~DiskImageResourceDDD() {}

    /// Returns the type of disk image resource.
    static std::string type_static() { return "DDD"; }
    virtual std::string type() { return type_static(); }

    virtual Vector2i block_size() const;

    virtual void read(ImageBuffer const& dest, BBox2i const& bbox) const;
    virtual void write(ImageBuffer const& dest, BBox2i const& bbox);
    virtual void flush() {}
    std::string query(std::string const& key) const;
    void open(std::string const& filename);
    void create(std::string const& filename, ImageFormat const& format);
    static DiskImageResource* construct_open(std::string const& filename);
    static DiskImageResource* construct_create(std::string const& filename,
                                               ImageFormat const& format);

  private:

    void parse_ddd_header(const char* label);
    std::string m_filename;
    std::map<std::string, std::string> m_header_entries;
    bool m_is_other_endian;
    int m_bytes_per_pixel;
  };

} // namespace vw

#endif // __VW_FILEIO_DISK_IMAGE_RESOUCE_DDD_H__
