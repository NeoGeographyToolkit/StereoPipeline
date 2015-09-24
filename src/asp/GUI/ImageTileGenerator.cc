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


#include <asp/GUI/ImageTileGenerator.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Image/ViewImageResource.h>
#include <vw/Core/Debugging.h>

namespace vw { namespace gui {

ImageTileGenerator::ImageTileGenerator(std::string filename) :
  m_rsrc( DiskImageResource::open(filename) ) {
  vw_out() << "\t--> Loading image: " << filename << ".\n";
}


/// Load, downsample, and float cast a base resolution tile bbox from disk.
/// - The bbox is specified at max resolution, but the output image is the
///   same region at a lower resolution according to the input level info.
/// - Helper function for the generate_tile function below.
template <class PixelT>
boost::shared_ptr<SrcImageResource> do_image_tilegen(boost::shared_ptr<SrcImageResource> rsrc,
                                                      BBox2i tile_bbox,
                                                      int level, int num_levels) {
  // Create a new image buffer of the bbox size
  ImageView<PixelT> tile(tile_bbox.width(), tile_bbox.height());
  // Read data from the disk into it.
  rsrc->read(tile.buffer(), tile_bbox);
  // Make another image view containing the appropriately subsampled tile
  //  and also cast it to float.
  ImageView<typename CompoundChannelCast<PixelT,float>::type> reduced_tile =
    channel_cast<float>(subsample(tile, (1 << ((num_levels-1) - level))));
  // Wrap the ImageView object as a VW resource type.
  return boost::shared_ptr<SrcImageResource>( new ViewImageResource(reduced_tile) );
}

boost::shared_ptr<SrcImageResource> ImageTileGenerator::generate_tile(TileLocator const& tile_info) {

  // Compute the bounding box of the image and the tile that is being
  // requested.  The bounding box of the tile depends on the pyramid
  // level we are looking at.
  // - The bounding box is computed for the max resolution (ie disk) level
  BBox2i image_bbox(0,0,m_rsrc->cols(),m_rsrc->rows());
  BBox2i tile_bbox = tile_to_bbox(this->tile_size(), tile_info.col,
                                  tile_info.row, tile_info.level, this->num_levels()-1);

  // Check to make sure the image intersects the bounding box.  Print
  // an error to screen and return an empty tile if it does not.
  if (!image_bbox.intersects(tile_bbox)) {
    vw_out(WarningMessage) << "ImageTileGenerator: No such tile!" << std::endl;
    return boost::shared_ptr<SrcImageResource>( make_point_src(PixelGray<uint8>(0)) );
  }

  // Make sure we don't access any pixels outside the image boundary
  // by cropping the tile to the image dimensions.
  tile_bbox.crop(image_bbox);

  /// Load the requested image area downsampled to the specified tile level, and
  ///  also converted from the disk type to float.
  switch (this->pixel_format()) {
  case VW_PIXEL_GRAY:
    if (this->channel_type() == VW_CHANNEL_UINT8) {
      return do_image_tilegen<PixelGray<uint8> >(m_rsrc, tile_bbox,
                                                 tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_INT16) {
      return do_image_tilegen<PixelGray<int16> >(m_rsrc, tile_bbox,
                                                 tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_UINT16) {
      return do_image_tilegen<PixelGray<uint16> >(m_rsrc, tile_bbox,
                                                  tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_FLOAT32) {
      return do_image_tilegen<PixelGray<float> >(m_rsrc, tile_bbox,
                                                  tile_info.level, this->num_levels());
    } else {
      std::cout << "This image has a channel type that is not yet support by stereo_gui.\n";
      std::cout << "Exiting...\n\n";
      exit(0);
  }
  break; // Done with VW_PIXEL_GRAY image

  case VW_PIXEL_GRAYA:
    if (this->channel_type() == VW_CHANNEL_UINT8) {
      return do_image_tilegen<PixelGrayA<uint8> >(m_rsrc, tile_bbox,
                                                  tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_INT16) {
      return do_image_tilegen<PixelGrayA<int16> >(m_rsrc, tile_bbox,
                                                  tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_UINT16) {
      return do_image_tilegen<PixelGrayA<uint16> >(m_rsrc, tile_bbox,
                                                   tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_FLOAT32) {
      return do_image_tilegen<PixelGrayA<float> >(m_rsrc, tile_bbox,
                                                  tile_info.level, this->num_levels());
    } else {
      std::cout << "This image has a channel type that is not yet support by stereo_gui.\n";
      std::cout << "Exiting...\n\n";
      exit(0);
    }

    break; // Done with VW_PIXEL_GRAYA image

  case VW_PIXEL_RGB:
    if (this->channel_type() == VW_CHANNEL_UINT8) {
      return do_image_tilegen<PixelRGB<uint8> >(m_rsrc, tile_bbox,
                                                tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_UINT16) {
      return do_image_tilegen<PixelRGB<uint16> >(m_rsrc, tile_bbox,
                                                 tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_INT32) {
      return do_image_tilegen<PixelRGB<int32> >(m_rsrc, tile_bbox,
                                                 tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_FLOAT32) {
      return do_image_tilegen<PixelRGB<float> >( m_rsrc, tile_bbox,
                                                 tile_info.level, this->num_levels());
    } else {
      std::cout << "This image has a channel type that is not yet support by stereo_gui.\n";
      std::cout << "Exiting...\n\n";
      exit(0);
    }

    break; // Done with VW_PIXEL_RGB image

  case VW_PIXEL_RGBA:
    if (this->channel_type() == VW_CHANNEL_UINT8) {
      return do_image_tilegen<PixelRGBA<uint8> >(m_rsrc, tile_bbox,
                                                 tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_UINT16) {
      return do_image_tilegen<PixelRGBA<uint16> >(m_rsrc, tile_bbox,
                                                  tile_info.level, this->num_levels());
    } else if (this->channel_type() == VW_CHANNEL_FLOAT32 ||
               this->channel_type() == VW_CHANNEL_FLOAT64 ) {
      return do_image_tilegen<PixelRGBA<float> >(m_rsrc, tile_bbox,
                                                 tile_info.level, this->num_levels());
    } else {
      std::cout << "This image has a channel type that is not yet support by stereo_gui.\n";
      std::cout << "Exiting...\n\n";
      exit(0);
    }
    break; // Done with VW_PIXEL_RGBA image

  default:
    std::cout << "This image has a pixel format that is not yet support by stereo_gui.\n";
    std::cout << "Exiting...\n\n";
    exit(0);
  }

  vw_throw(NoImplErr() << "Unsupported pixel format or channel type in TileGenerator.\n");
}

Vector2 ImageTileGenerator::minmax() {
  vw_throw(NoImplErr() << VW_CURRENT_FUNCTION << " not implemented.");
}

PixelRGBA<float32> ImageTileGenerator::sample(int x, int y, int level, int /*transaction_id*/) {
  ImageView<PixelRGBA<float32> > point(1,1);
  m_rsrc->read(point.buffer(), BBox2i(x,y,1,1));
  return point(0,0);
}

int ImageTileGenerator::cols() const {
  return m_rsrc->cols();
}

int ImageTileGenerator::rows() const {
  return m_rsrc->rows();
}

PixelFormatEnum ImageTileGenerator::pixel_format() const {
  return m_rsrc->pixel_format();
}

ChannelTypeEnum ImageTileGenerator::channel_type() const {
  return m_rsrc->channel_type();
}

Vector2i ImageTileGenerator::tile_size() const {
  return m_rsrc->block_read_size();
}

int32 ImageTileGenerator::num_levels() const {
  float tiles_x = float(this->cols()) / float(this->tile_size()[0]);
  float tiles_y = float(this->rows()) / float(this->tile_size()[1]);
  return 1 + boost::numeric_cast<int32>(ceil(log(std::max(tiles_x, tiles_y)) / log(2)));
}

}} // namespace vw::gui
