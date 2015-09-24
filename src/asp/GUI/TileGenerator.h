// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
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


#ifndef __STEREO_GUI_TILEGENERATOR_H__
#define __STEREO_GUI_TILEGENERATOR_H__

#include <vw/Image/PixelTypeInfo.h>
#include <vw/Image/ImageView.h>
#include <vw/Math/BBox.h>
#include <vw/Core/FundamentalTypes.h>

namespace vw {
namespace gui {

  /// Simple implementation of the VW SrcImageResource type using
  ///  a plain data buffer copy of the input data.
  /// - TODO: This should live in VW!
  class ConstantSrc : public SrcImageResource {
      ImageFormat m_fmt;
      size_t m_size;
      boost::shared_array<const uint8> m_data;
    public:
      /// Creates a new buffer with copy of the data.
      ConstantSrc(const uint8* data, size_t size, const ImageFormat& fmt);

      virtual void read( ImageBuffer const& buf, BBox2i const& bbox ) const;
      
      // Define virtual functions for a boring POD block of data
      virtual ImageFormat format         () const {return m_fmt; }
      virtual bool        has_block_read () const {return false; }
      virtual bool        has_nodata_read() const {return false; }
      virtual size_t      native_size    () const {return m_size;}      
      virtual boost::shared_array<const uint8> native_ptr() const {return m_data;}
  }; // End class ConstantSrc

  /// Make a ConstantSrc object wrapping a single pixel object.
  template <typename PixelT>
  SrcImageResource* make_point_src(const PixelT& px) {
    ImageFormat fmt;
    fmt.rows = fmt.cols = fmt.planes = 1;
    fmt.pixel_format = PixelFormatID<PixelT>::value;
    fmt.channel_type = ChannelTypeID<typename PixelChannelType<PixelT>::type>::value;
    return new ConstantSrc(reinterpret_cast<const uint8*>(&px), sizeof(PixelT), fmt);
  }

  /// Augmented pyramid tile index object.
  struct TileLocator {
    int  col;
    int  row;
    int  level; ///< Increasing level means increasing image resolution
    int  transaction_id;
    bool exact_transaction_id_match;

    bool is_valid() const {
      return col >= 0 && row >= 0 && col < (1 << level) && row < (1 << level);
    }
  };  // End struct TileLocator

  /// Nicely formatted write function for TileLocator
  std::ostream& operator<<(std::ostream& o, const TileLocator& l);

  /// Given a tile index, return the bounding box of that tile coverage
  /// in the bottom (i.e. highest resolution) level of the source image pyramid.
  BBox2i tile_to_bbox(Vector2i tile_size, int col, int row, int level, int max_level);

  /// Given a pixel bbox on a certain level, return a list of all intersecting tiles.
  std::list<TileLocator> bbox_to_tiles(Vector2i tile_size, BBox2i bbox, int level,
                                       int max_level, int transaction_id,
                                       bool exact_transaction_id_match);

  // --------------------------------------------------------------------------
  //                              TILE GENERATOR
  // --------------------------------------------------------------------------

  // TODO: Not sure why we need this base class with only one derived class planned

  // TODO: Is this class used at all?

  /// Vestigial class, description pasted from ImageTileGenerator:
  /// Wraps an image on disk and provides acces to tiles from a 
  /// 2x downsample image image pyramid of it.
  class TileGenerator {
  public:
    virtual ~TileGenerator() {}
    
    /// Generate an image tile at the specified tree location.
    virtual boost::shared_ptr<SrcImageResource> generate_tile(TileLocator const& tile_info) = 0;
    
    ///
    virtual Vector2 minmax() = 0;
    
    ///
    virtual PixelRGBA<float> sample(int x, int y, int level, int transaction_id) = 0;

    // 
    virtual int cols() const = 0;
    virtual int rows() const = 0;
    
    virtual PixelFormatEnum pixel_format() const = 0;
    virtual ChannelTypeEnum channel_type() const = 0;
    virtual Vector2i tile_size () const = 0;
    virtual int32    num_levels() const = 0;

    // Use this method to generate the correct type of TileGenerator
    // for a given filename.
    static boost::shared_ptr<TileGenerator> create(std::string filename);
  };

}} // namespace vw::gui

#endif
