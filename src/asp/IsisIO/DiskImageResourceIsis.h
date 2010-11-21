// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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

    virtual bool has_block_write()  const {return false;}
    virtual bool has_nodata_write() const {return false;}
    virtual bool has_block_read()   const {return true;}
    virtual bool has_nodata_read()  const {return true;}

    virtual Vector2i block_read_size() const;

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
    double nodata_read() const;
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
