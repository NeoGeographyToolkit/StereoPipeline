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


/// \file DiskImageResourceIsis.h
///
/// Provides support for ISIS image files.
///
#ifndef __VW_FILEIO_DISK_IMAGE_RESOUCE_ISIS_H__
#define __VW_FILEIO_DISK_IMAGE_RESOUCE_ISIS_H__

#include <vw/Image/PixelTypes.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>

namespace Isis {
  class Cube;
}

namespace vw {

  class DiskImageResourceIsis : public DiskImageResourceGDAL {
  public:

    DiskImageResourceIsis(std::string const& filename) : DiskImageResourceGDAL(filename) {
      open(filename);
    }

    DiskImageResourceIsis(std::string const& filename, ImageFormat const& format)
      : DiskImageResourceGDAL(filename) {
      create(filename, format);
    }

    virtual ~DiskImageResourceIsis() {}

    /// Returns the type of disk image resource.
    static std::string type_static() { return "ISIS"; }
    virtual std::string type() { return type_static(); }

    virtual bool has_block_write () const {return false;}
    virtual bool has_nodata_write() const {return false;}
    virtual bool has_block_read  () const {return true; }
    virtual bool has_nodata_read () const {return true; }

    virtual Vector2i block_read_size() const;

    virtual void read (ImageBuffer const& dest, BBox2i const& bbox) const;
    virtual void write(ImageBuffer const& dest, BBox2i const& bbox);
    virtual void flush() {}
    //    std::string query(std::string const& key) const;
    void open(std::string const& filename);
    void create(std::string const& filename, ImageFormat const& format);
    static DiskImageResource* construct_open  (std::string const& filename);
    static DiskImageResource* construct_create(std::string const& filename,
                                               ImageFormat const& format);

    // Info about special pixel types in ISIS
    // --------------------------------------
    // Note: Isis has many types of invalid pixels. If creating a mask
    // of valid pixels, please use only pixels within the valid range
    // provided in the functions below.
    double nodata_read  () const;
    double valid_minimum() const;
    double valid_maximum() const;

    // Additional cube informat
    bool is_map_projected() const;

  private:
    boost::shared_ptr<Isis::Cube> m_cube;
    std::string m_filename;
    int         m_bytes_per_pixel;
    Vector2i    m_native_block_size;
  };

} // namespace vw

#endif // __VW_FILEIO_DISK_IMAGE_RESOUCE_ISIS_H__
