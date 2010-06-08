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
