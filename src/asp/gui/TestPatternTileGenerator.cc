// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


#include <vw/gui/TestPatternTileGenerator.h>
#include <vw/Image/PixelTypes.h>
#include <vw/Image/ViewImageResource.h>

namespace vw { namespace gui {

boost::shared_ptr<SrcImageResource> TestPatternTileGenerator::generate_tile(TileLocator const& /*tile_info*/) {
  ImageView<PixelRGBA<uint8> > tile(m_tile_size, m_tile_size);
  for (int j = 0; j < m_tile_size; ++j){
    for (int i = 0; i < m_tile_size; ++i){
      if (abs(i - j) < 10 || abs(i - (m_tile_size - j)) < 10)
        tile(i,j) = PixelRGBA<uint8>(255,0,0,255);
      else
        tile(i,j) = PixelRGBA<uint8>(0,0,0,255);
    }
  }
  boost::shared_ptr<SrcImageResource> result( new ViewImageResource(tile) );
  return result;
}

Vector2 TestPatternTileGenerator::minmax() { return Vector2(0.0, 1.0); }

PixelRGBA<float32> TestPatternTileGenerator::sample(int /*x*/, int /*y*/, int /*level*/, int /*transaction_id*/) {
  PixelRGBA<float32> result;
  return result;
}

int TestPatternTileGenerator::cols() const { return 2048; }
int TestPatternTileGenerator::rows() const { return 2048; }
PixelFormatEnum TestPatternTileGenerator::pixel_format() const { return VW_PIXEL_RGBA; }
ChannelTypeEnum TestPatternTileGenerator::channel_type() const { return VW_CHANNEL_UINT8; }
Vector2i TestPatternTileGenerator::tile_size() const {
  return Vector2i(m_tile_size, m_tile_size);
}
int32 TestPatternTileGenerator::num_levels() const {
  return 4;
}

}}
