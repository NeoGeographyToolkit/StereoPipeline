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

/// \file DiskImageResourceDDD.cc
/// 
#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <vw/Core/Exception.h>
#include <vw/Image/PerPixelViews.h>

#include "Isis/DiskImageResourceIsis.h"

// Isis Includes
#include <Cube.h>
#include <Portal.h>

using namespace std;
using namespace boost;

namespace vw {
  

  // Set the default block size to be the width of the image by 10
  // scanlines.
  Vector2i DiskImageResourceIsis::native_block_size() const
  {
    return Vector2i(2048,2048);
  }

  /// Bind the resource to a file for writing.
  void DiskImageResourceIsis::create(std::string const& filename,
                                     ImageFormat const& format)
  {
    throw NoImplErr() << "The Isis driver does not yet support creation of Isis files";
  }

  /// Bind the resource to a file for reading.  Confirm that we can open
  /// the file and that it has a sane pixel format.  
  void DiskImageResourceIsis::open(std::string const& filename)
  {

    Isis::Cube cube;
    m_filename = filename;
    cube.Open(filename);

    if ( !(cube.IsOpen()) ) {
      vw_throw(IOErr() << "DiskImageResourceIsis: Could not open cube file: \"" << filename << "\".");
    }

    // Extract the dimensions of the image
    m_format.planes = cube.Bands();
    m_format.cols = cube.Samples();
    m_format.rows = cube.Lines();

    m_format.pixel_format = VW_PIXEL_SCALAR;

    Isis::PixelType isis_ptype = cube.PixelType();
    switch (isis_ptype) {
    case Isis::UnsignedByte: 
      m_bytes_per_pixel = 1;
      m_format.channel_type = VW_CHANNEL_UINT8;
      break;
    case Isis::SignedByte: 
      m_bytes_per_pixel = 1;
      m_format.channel_type = VW_CHANNEL_INT8;
      break;
    case Isis::UnsignedWord: 
      m_bytes_per_pixel = 2;
      m_format.channel_type = VW_CHANNEL_UINT16;
      break;
    case Isis::SignedWord: 
      m_bytes_per_pixel = 2;
      m_format.channel_type = VW_CHANNEL_INT16;
      break;
    case Isis::UnsignedInteger: 
      m_bytes_per_pixel = 4;
      m_format.channel_type = VW_CHANNEL_UINT32;
      break;
    case Isis::SignedInteger: 
      m_bytes_per_pixel = 4;
      m_format.channel_type = VW_CHANNEL_INT32;
      break;
    case Isis::Real: 
      m_bytes_per_pixel = 4;
      m_format.channel_type = VW_CHANNEL_FLOAT32;
      break;
    case Isis::Double: 
      m_bytes_per_pixel = 8;
      m_format.channel_type = VW_CHANNEL_FLOAT64;
      break;
    default:
      vw_throw(IOErr() << "DiskImageResourceIsis: Unknown pixel type.");
    }

    vw_out(0) << "Bytes per pixel: " << m_bytes_per_pixel; 
    vw_out(0) << "   channel_type " << m_format.channel_type << "\n";
   
    // Close the cube file
    cube.Close();
  }

  /// Read the disk image into the given buffer.
  void DiskImageResourceIsis::read(ImageBuffer const& dest, BBox2i const& bbox) const
  {
    Isis::Cube cube;
    cube.Open(m_filename);

    if ( !(cube.IsOpen()) ) {
      vw_throw(IOErr() << "DiskImageResourceIsis: Could not open cube file: \"" << m_filename << "\".");
    }

    VW_ASSERT(bbox.max().x() <= cube.Samples() || bbox.max().y() <= cube.Lines(), 
              IOErr() << "DiskImageResourceIsis: requested bbox " << bbox << " exceeds image dimensions [" << cube.Samples() << " " << cube.Lines() << "]");

    // Read in the requested tile from the cube file.  Note that ISIS
    // cube pixel indices appear to be 1-based.
    Isis::Portal buffer( bbox.width(), bbox.height(), cube.PixelType() );
    buffer.SetPosition(bbox.min().x()+1, bbox.min().y()+1, 1);
    cube.Read(buffer);

    // Create generic image buffer from the Isis data.
    ImageBuffer src;
    src.data = buffer.RawBuffer();
    src.format = m_format;
    src.format.cols = bbox.width();
    src.format.rows = bbox.height();
    src.cstride = m_bytes_per_pixel;
    src.rstride = m_bytes_per_pixel * bbox.width();
    src.pstride = m_bytes_per_pixel * bbox.width() * bbox.height();
    convert(dest, src);

    cube.Close();
  }

  // Write the given buffer into the disk image.
  void DiskImageResourceIsis::write(ImageBuffer const& src, BBox2i const& bbox)
  {
    throw NoImplErr() <<
      "The Isis driver does not yet support creation of Isis files";
  }

  // A FileIO hook to open a file for reading
  DiskImageResource*
  DiskImageResourceIsis::construct_open(std::string const& filename)
  {
    return new DiskImageResourceIsis(filename);
  }

  // A FileIO hook to open a file for writing
  DiskImageResource*
  DiskImageResourceIsis::construct_create(std::string const& filename,
					 ImageFormat const& format)
  {
    return new DiskImageResourceIsis(filename, format);
  }
}
