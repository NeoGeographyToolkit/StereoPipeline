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


#ifndef __STEREO_GUI_IMAGETILEGENERATOR_H__
#define __STEREO_GUI_IMAGETILEGENERATOR_H__

#include <asp/GUI/TileGenerator.h>

namespace vw {
namespace gui {

  /// Wraps an image on disk and provides acces to tiles from a 
  /// 2x downsample image image pyramid of it.
  class ImageTileGenerator : public TileGenerator {
    /// This points to a DiskImageResource for the input image file.
    boost::shared_ptr<SrcImageResource> m_rsrc; 

  public:
    /// Init with any type of ASP supported image file.
    ImageTileGenerator(std::string filename);
    virtual ~ImageTileGenerator() {}
    
    /// Read a single pixel from disk at the specified location.
    virtual PixelRGBA<float> sample(int x, int y, int level, int transaction_id);

    /// Generate the specified image from the disk resource.
    virtual boost::shared_ptr<SrcImageResource> generate_tile(TileLocator const& tile_info);
    
    /// TODO: Does nothing!!!!
    virtual Vector2 minmax();

    // These describe the image on disk
    virtual int cols() const;
    virtual int rows() const;   
    virtual PixelFormatEnum pixel_format() const;
    virtual ChannelTypeEnum channel_type() const;
    
    /// Return the block read size of the disk resource
    virtual Vector2i tile_size() const; 
    
    /// Return number of pyramid levels needed to represent the image 
    ///  with a 2x downsample image pyramid.
    virtual int32 num_levels() const; 
  };


}} // namespace vw::gui

#endif // __STEREO_GUI_IMAGETILEGENERATOR_H__
