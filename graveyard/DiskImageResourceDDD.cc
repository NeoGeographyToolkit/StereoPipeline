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

#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

#ifdef __APPLE__
static const int BITSPERBYTE = 8;
#else
#include <values.h>                        // for BITSPERBYTE
#endif

#include <boost/algorithm/string.hpp>

#include <vw/Core/Exception.h>
#include <asp/Core/DiskImageResourceDDD.h>

using namespace std;
using namespace boost;

namespace vw
{
  static const int IMAGE_HEADER_LENGTH = 1024;
  static const int IMAGE_LABEL_OFFSET = 24;
  static const int IMAGE_LABEL_LENGTH = IMAGE_HEADER_LENGTH-IMAGE_LABEL_OFFSET;
  static const uint32 MAGIC = 1659;

  struct DDDHeader
  {
    uint32 magic;                                  // always set to MAGIC
    uint32 numScanlines, bytesPerScanline;
    uint32 bitsPerElement; // the data size unless there's padding
    uint32 spare1, spare2;
    char label[IMAGE_LABEL_LENGTH];
  };

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // The label contains linefeed delimited strings, e.g.:
  //
  // decompressed-from ./4A_04_1001004F00_01.DAT
  // id 79 time 844381966:250
  // start 0 cross 5056 down 7168
  // cam ctx
  // mode 0x0
  // dac 195
  // offset 234 232
  // sram_base 0
  // start_addr 0
  // exposure 1.87 msec (19)
  // FPA-temp 17.9 C (2395)
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  std::string
  DiskImageResourceDDD::query(std::string const& key) const
  {
    std::map<std::string, std::string>::const_iterator
      entry = m_header_entries.find(key);

    if (entry != m_header_entries.end())
      return (*entry).second;
    else
      throw NotFoundErr() << "DiskImageResourceDDD::query(): "
                          << "no matching value found for \"" << key << "\"";
  }

  void
  DiskImageResourceDDD::parse_ddd_header(const char* header_label)
  {
    string header_label_string(header_label);
    istringstream label_stream(header_label_string);

    while (!label_stream.eof())
    {
      char line[IMAGE_LABEL_LENGTH+1];
      vector<string> tokens;

      label_stream.getline(line, sizeof(line));
      split(tokens, line, is_any_of(" "));

      // First handle any special cases... yes this is a bit hacky,
      // but their label format is a bit hacky...
      if (tokens[0] == string("id"))
      {
        m_header_entries[tokens[2]] = tokens[3];
      }
      else if (tokens[0] == string("start"))
      {
        m_header_entries[tokens[2]] = tokens[3];
        m_header_entries[tokens[4]] = tokens[5];
      }
      else if (tokens[0] == string("offset"))
      {
        tokens[1] = tokens[1] + tokens[2];
      }
      else if ((tokens[0] == string("exposure")) ||
               (tokens[0] == string("FPA-temp")))
      {
        tokens[1] = tokens[1] + tokens[2] + tokens[3];
      }
      else if (tokens[0] == string("@stereographic"))
      {
        std::cout << "Projection line: " << line << "\n";
        m_header_entries["projection"] = "stereographic";
        m_header_entries["projection_x_offset"] = tokens[1];
        m_header_entries["projection_y_offset"] = tokens[2];
        m_header_entries["projection_fullwidth"] = tokens[3];
        m_header_entries["projection_pole_flag"] = tokens[4];
        m_header_entries["projection_extent"] = tokens[5];
      }

      m_header_entries[tokens[0]] = tokens[1];
    }
  }

  inline uint32
  swap_long(uint32 data)
  {
    unsigned char *bytes = reinterpret_cast<unsigned char *>(&data);
    unsigned char bytes2 = bytes[2];
    unsigned char bytes3 = bytes[3];

    bytes[3] = bytes[0];
    bytes[2] = bytes[1];
    bytes[1] = bytes2;
    bytes[0] = bytes3;

    return data;
  }

  inline void
  swap_longs(void *buffer, size_t numLongs)
  {
    uint32 *longs = reinterpret_cast<uint32 *>(buffer);
    uint32 *endLong = longs + numLongs;

    while (longs != endLong)
    {
      uint8 *bytes = (uint8 *)(longs);
      uint8 bytes2 = bytes[2];
      uint8 bytes3 = bytes[3];

      bytes[3] = bytes[0];
      bytes[2] = bytes[1];
      bytes[1] = bytes2;
      bytes[0] = bytes3;

      longs++;
    }
  }

  inline void
  swap_shorts(void *buffer, size_t numShorts)
  {
    uint16 *shorts = reinterpret_cast<uint16 *>(buffer);
    uint16 *endShort = shorts + numShorts;

    while (shorts != endShort)
    {
      uint8 *bytes = (uint8 *)(shorts);
      uint8 bytes1 = bytes[1];
      bytes[1] = bytes[0];
      bytes[0] = bytes1;
      shorts++;
    }
  }

  // Set the default block size to be the width of the image by 10
  // scanlines.
  Vector2i DiskImageResourceDDD::block_size() const
  {
    return Vector2i(m_format.cols,10);
  }

  /// Bind the resource to a file for writing.
  void
  DiskImageResourceDDD::create(std::string const& filename,
                               ImageFormat const& format)
  {
    throw NoImplErr()
      << "The DDD driver does not yet support creation of DDD files";
  }

  /// Bind the resource to a file for reading.  Confirm that we can open
  /// the file and that it has a sane pixel format.
  void
  DiskImageResourceDDD::open(std::string const& filename)
  {
    m_filename = filename;
    ifstream image_file(m_filename.c_str(), ios::in | ios::binary);

    DDDHeader header;
    image_file.read((char *)(&header), sizeof(DDDHeader));

    if (image_file.bad())
      throw IOErr() << "DiskImageResourceDDD::open(): could not read "
                    << filename << " header.";

    if (header.magic == MAGIC)
    {
      m_is_other_endian = false;
    }
    else if (header.magic == swap_long(MAGIC))
    {
      swap_longs(&header, 4);
      m_is_other_endian = true;
    }
    else
    {
      throw IOErr()
        << "DiskImageResourceDDD::open(): " << filename
        << " has bad magic number (" << header.magic << " != "
        << MAGIC << ").";
    }

    m_bytes_per_pixel = header.bitsPerElement / BITSPERBYTE;
    m_format.planes = 1;
    m_format.pixel_format = VW_PIXEL_GRAY;
    m_format.cols = header.bytesPerScanline / m_bytes_per_pixel;
    m_format.rows = header.numScanlines;

    switch (header.bitsPerElement)
    {
    case BITSPERBYTE:
      m_format.channel_type = VW_CHANNEL_UINT8;
      break;
    case sizeof(short) * BITSPERBYTE:
      // If you think you want/need unsigned ints here talk to LJE before changing!
      //       m_format.channel_type = VW_CHANNEL_UINT16;
       m_format.channel_type = VW_CHANNEL_INT16;
      break;
    default:
      throw IOErr() << "DiskImageResourceDDD::open(): unsupported pixel size ("
                    << header.bitsPerElement << " bits) in " << filename
                    << ".";
    }

    // Put the data into an associative contain (std::map).  Key/value
    // pairs are located by searching for strings seperated by the
    // equals sign "=".
    parse_ddd_header(header.label);

    // Close the file
    image_file.close();
  }

  /// Read the disk image into the given buffer.
  void
  DiskImageResourceDDD::read(ImageBuffer const& dest, BBox2i const& bbox) const
  {
    ifstream image_file(m_filename.c_str(), ios::in | ios::binary);

    if (image_file.bad())
      throw IOErr() << "  DiskImageResourceDDD::read(): \"" << m_filename
                    << "\" is not yet open.";

    // Read the pixel data from the file.
    unsigned int total_pixels = (bbox.width() * bbox.height() *
                                 m_format.planes);
    uint8* image_data = new uint8[total_pixels * m_bytes_per_pixel];

    for (int line = 0; line < bbox.height(); ++line)
    {
      int file_offset =
        IMAGE_HEADER_LENGTH + (((bbox.min().y() + line) * m_format.cols +
                                bbox.min().x()) * m_bytes_per_pixel);
      int image_data_offset =
        (line * bbox.width() + bbox.min().x()) * m_bytes_per_pixel;

      // Set the file offset to the position of the first image
      // byte... the header length is always the same for DDD images.
      image_file.seekg(file_offset, ios::beg);

      // Read one scanline of the bounding box into memory.
      image_file.read((char *) (image_data + image_data_offset),
                      m_bytes_per_pixel * bbox.width());

      if (image_file.bad())
        throw IOErr() << "DiskImageResourceDDD::read():"
          " An unrecoverable error occured while reading the image data.";
    }


    // DDD images are always big-endian, swap bytes if this is a
    // little-endian system
    if (m_is_other_endian && m_bytes_per_pixel == sizeof(uint16))
      swap_shorts(image_data, total_pixels);

    // Create generic image buffer from the DDD data.
    ImageBuffer src;
    src.data = image_data;
    src.format = m_format;
    src.format.cols = bbox.width();
    src.format.rows = bbox.height();
    src.cstride = m_bytes_per_pixel;
    src.rstride = m_bytes_per_pixel * bbox.width();
    src.pstride = m_bytes_per_pixel * bbox.width() * bbox.height();

    convert(dest, src);

    delete[] image_data;
    image_file.close();
  }

  // Write the given buffer into the disk image.
  void
  DiskImageResourceDDD::write(ImageBuffer const& src, BBox2i const& bbox)
  {
    throw NoImplErr() <<
      "The DDD driver does not yet support creation of DDD files";
  }

  // A FileIO hook to open a file for reading
  DiskImageResource*
  DiskImageResourceDDD::construct_open(std::string const& filename)
  {
    return new DiskImageResourceDDD(filename);
  }

  // A FileIO hook to open a file for writing
  DiskImageResource*
  DiskImageResourceDDD::construct_create(std::string const& filename,
                                         ImageFormat const& format)
  {
    return new DiskImageResourceDDD(filename, format);
  }
}
