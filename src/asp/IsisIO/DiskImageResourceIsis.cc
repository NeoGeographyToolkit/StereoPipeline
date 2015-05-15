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


/// \file DiskImageResourceDDD.cc
///
#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <vw/Core/Exception.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Image/ImageResource.h>
#include <vw/Image/PixelTypeInfo.h>
#include <asp/IsisIO/DiskImageResourceIsis.h>

#include <string>

#include <Cube.h>
#include <Portal.h>
#include <SpecialPixel.h>

using namespace std;
using namespace boost;

namespace vw {


  // We use a fixed tile size of 2048x2048 pixels here.  Although this
  // may not be the native tile size of the ISIS cube, it seems to be
  // much faster to let the ISIS driver aggregate smaller blocks by
  // making a larger request rather than caching those blocks ourselves.
  Vector2i DiskImageResourceIsis::block_read_size() const
  {
    return Vector2i(2048,2048);
  }

  /// Bind the resource to a file for writing.
  void DiskImageResourceIsis::create(std::string const& /*filename*/,
                                     ImageFormat const& /*format*/)
  {
    throw NoImplErr() << "The Isis driver does not yet support creation of Isis files";
  }

  /// Bind the resource to a file for reading.  Confirm that we can open
  /// the file and that it has a sane pixel format.
  void DiskImageResourceIsis::open(std::string const& filename) {
    m_cube     = boost::shared_ptr<Isis::Cube>( new Isis::Cube() );
    m_filename = filename;
    m_cube->open( QString::fromStdString(m_filename) );
    VW_ASSERT(m_cube->isOpen(), IOErr() << "DiskImageResourceIsis: Could not open cube file: \"" << filename << "\".");

    // Extract the dimensions of the image
    m_format.planes = m_cube->bandCount();
    m_format.cols   = m_cube->sampleCount();
    m_format.rows   = m_cube->lineCount();

    m_format.pixel_format = VW_PIXEL_SCALAR;

    // Set member variables according to the specified pixel type
    Isis::PixelType isis_ptype = m_cube->pixelType();
    switch (isis_ptype) {
      case Isis::UnsignedByte:    m_bytes_per_pixel = 1;  m_format.channel_type = VW_CHANNEL_UINT8;    break;
      case Isis::SignedByte:      m_bytes_per_pixel = 1;  m_format.channel_type = VW_CHANNEL_INT8;     break;
      case Isis::UnsignedWord:    m_bytes_per_pixel = 2;  m_format.channel_type = VW_CHANNEL_UINT16;   break;
      case Isis::SignedWord:      m_bytes_per_pixel = 2;  m_format.channel_type = VW_CHANNEL_INT16;    break;
      case Isis::UnsignedInteger: m_bytes_per_pixel = 4;  m_format.channel_type = VW_CHANNEL_UINT32;   break;
      case Isis::SignedInteger:   m_bytes_per_pixel = 4;  m_format.channel_type = VW_CHANNEL_INT32;    break;
      case Isis::Real:            m_bytes_per_pixel = 4;  m_format.channel_type = VW_CHANNEL_FLOAT32;  break;
      case Isis::Double:          m_bytes_per_pixel = 8;  m_format.channel_type = VW_CHANNEL_FLOAT64;  break;
      default:
        vw_throw(IOErr() << "DiskImageResourceIsis: Unknown pixel type.");
    }
  }

  /// Read the disk image into the given buffer.
  void DiskImageResourceIsis::read(ImageBuffer const& dest, BBox2i const& bbox) const
  {
    VW_ASSERT(bbox.max().x() <= m_cube->sampleCount() ||
              bbox.max().y() <= m_cube->lineCount(),
              IOErr() << "DiskImageResourceIsis: requested bbox " << bbox
              << " exceeds image dimensions [" << m_cube->sampleCount()
              << " " << m_cube->lineCount() << "]");

    // Read in the requested tile from the cube file.  Note that ISIS
    // cube pixel indices appear to be 1-based.
    Isis::Portal buffer( bbox.width(), bbox.height(),
                         m_cube->pixelType() );
    buffer.SetPosition(bbox.min().x()+1, bbox.min().y()+1, 1);
    m_cube->read(buffer);

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
  }

  // Write the given buffer into the disk image.
  void DiskImageResourceIsis::write(ImageBuffer const& /*src*/, BBox2i const& /*bbox*/) {
    vw_throw ( NoImplErr() << "The Isis driver does not yet support creation of Isis files" );
  }

  // A FileIO hook to open a file for reading
  DiskImageResource* DiskImageResourceIsis::construct_open(std::string const& filename)
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

  /// Info about special pixel types in ISIS
  //  --------------------------------------
  double DiskImageResourceIsis::nodata_read() const {
    switch (m_format.channel_type) {
      case VW_CHANNEL_FLOAT64: return Isis::NULL8;
      case VW_CHANNEL_FLOAT32: return Isis::NULL4;
      case VW_CHANNEL_INT32:   return Isis::INULL4;
      case VW_CHANNEL_INT16:   return Isis::NULL2;
      default:                 return 0.0;
    }
  }
  double DiskImageResourceIsis::valid_minimum() const {
    switch (m_format.channel_type) {
      case VW_CHANNEL_FLOAT64: return Isis::ValidMinimum;
      case VW_CHANNEL_FLOAT32: return Isis::VALID_MIN4;
      case VW_CHANNEL_INT32:   return Isis::IVALID_MIN4;
      case VW_CHANNEL_INT16:   return Isis::VALID_MIN2;
      case VW_CHANNEL_UINT16:  return Isis::VALID_MINU2;
      default:                 return Isis::VALID_MIN1;
    }
  }
  double DiskImageResourceIsis::valid_maximum() const {
    switch (m_format.channel_type) {
      case VW_CHANNEL_FLOAT64: return Isis::ValidMaximum;
      case VW_CHANNEL_FLOAT32: return Isis::VALID_MAX4;
      case VW_CHANNEL_INT32:   return 2147483647;
      case VW_CHANNEL_INT16:   return Isis::VALID_MAX2;
      case VW_CHANNEL_UINT16:  return Isis::VALID_MAXU2;
      default:                 return Isis::VALID_MAX1;
    }
  }
  bool DiskImageResourceIsis::is_map_projected() const {
    // They used to have a HasProjection. I'm not sure if this fix is the correct method now.
    return m_cube->projection() != NULL;
  }
}
