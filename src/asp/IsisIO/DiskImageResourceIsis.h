// __BEGIN_LICENSE__
//
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
//
// Copyright 2006 Carnegie Mellon University. All rights reserved.
//
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
//
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file DiskImageResourceDDD.h
///
/// Provides support for Malin Space Science System's DDD image file
/// format.
///
#ifndef __VW_FILEIO_DISK_IMAGE_RESOUCE_ISIS_H__
#define __VW_FILEIO_DISK_IMAGE_RESOUCE_ISIS_H__

#include <vw/Image/PixelTypes.h>
#include <vw/FileIO/DiskImageResource.h>

namespace vw {

  class DiskImageResourceIsis : public DiskImageResource {
  public:

    DiskImageResourceIsis(std::string const& filename) : DiskImageResource(filename) {
      open(filename);
    }

    DiskImageResourceIsis(std::string const& filename, ImageFormat const& format)
      : DiskImageResource(filename) {
      create(filename, format);
    }

    virtual ~DiskImageResourceIsis() {}

    /// Returns the type of disk image resource.
    static std::string type_static() { return "ISIS"; }
    virtual std::string type() { return type_static(); }

    virtual Vector2i block_size() const;

    virtual void read(ImageBuffer const& dest, BBox2i const& bbox) const;
    virtual void write(ImageBuffer const& dest, BBox2i const& bbox);
    virtual void flush() {}
    //    std::string query(std::string const& key) const;
    void open(std::string const& filename);
    void create(std::string const& filename, ImageFormat const& format);
    static DiskImageResource* construct_open(std::string const& filename);
    static DiskImageResource* construct_create(std::string const& filename,
                                               ImageFormat const& format);

    // Info about special pixel types in ISIS
    // --------------------------------------
    // Note: Isis has many types of invalid pixels. If creating a mask
    // of valid pixels, please use only pixels within the valid range
    // provided in the functions below.
    bool has_nodata_value() const { return true; }
    double nodata_value() const;
    double valid_minimum() const;
    double valid_maximum() const;

    // Additional cube informat
    bool is_map_projected() const { return m_is_projected; }

  private:

    std::string m_filename;
    int m_bytes_per_pixel;
    Vector2i m_native_block_size;
    bool m_is_projected;
  };

} // namespace vw

#endif // __VW_FILEIO_DISK_IMAGE_RESOUCE_ISIS_H__
