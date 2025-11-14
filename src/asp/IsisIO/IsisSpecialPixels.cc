// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1

#include <asp/IsisIO/IsisSpecialPixels.h>

#include <vw/Core/Functors.h>
#include <vw/Image/PerPixelViews.h>
#include <vw/Image/PixelTypes.h>
#include <vw/Image/Filter.h>

// Isis headers
#include <isis/SpecialPixel.h>

namespace asp {

//  IsisSpecialPixelFunc
//
/// Replace ISIS missing data values with a pixel value of your choice.
template <class PixelT>
class IsisSpecialPixelFunc: public vw::UnaryReturnSameType {
  PixelT m_replacement_low;
  PixelT m_replacement_high;
  PixelT m_replacement_null;

  // Private
  IsisSpecialPixelFunc(): m_replacement_low(0), m_replacement_high(0), m_replacement_null(0) {}

public:
  IsisSpecialPixelFunc(PixelT const& pix_l, PixelT const& pix_h, PixelT const& pix_n):
    m_replacement_low(pix_l), m_replacement_high(pix_h), m_replacement_null(pix_n) {}

  // Helper to determine special across different channel types
  template <typename ChannelT, typename T = void>
  struct Helper {
    static inline bool IsSpecial(ChannelT const& arg) { return false; }
    static inline bool IsHighPixel(ChannelT const& arg) { return false; }
    static inline bool IsNull(ChannelT const& arg) { return false; }
  };
  template<typename T> struct Helper<double, T> {
    static inline bool IsSpecial(double const& arg) {
      return arg < Isis::VALID_MIN8;
    }
    static inline bool IsHighPixel(double const& arg) {
      return arg == Isis::HIGH_INSTR_SAT8 || arg == Isis::HIGH_REPR_SAT8;
    }
    static inline bool IsNull(double const& arg) {
      return arg == Isis::NULL8;
    }
  };
  template<typename T> struct Helper<float, T> {
    static inline bool IsSpecial(float const& arg) {
      return arg < Isis::VALID_MIN4;
    }
    static inline bool IsHighPixel(float const& arg) {
      return arg == Isis::HIGH_INSTR_SAT4 || arg == Isis::HIGH_REPR_SAT4;
    }
    static inline bool IsNull(float const& arg) {
      return arg == Isis::NULL4;
    }
  };
  template<typename T> struct Helper<short, T>{
    static inline bool IsSpecial(short const& arg) {
      return arg < Isis::VALID_MIN2;
    }
    static inline bool IsHighPixel(short const& arg) {
      return arg == Isis::HIGH_INSTR_SAT2 || arg == Isis::HIGH_REPR_SAT2;
    }
    static inline bool IsNull(short const& arg) {
      return arg == Isis::NULL2;
    }
  };
  template<typename T> struct Helper<unsigned short, T> {
    static inline bool IsSpecial(unsigned short const& arg) {
      return arg < Isis::VALID_MINU2 || arg > Isis::VALID_MAXU2;
    }
    static inline bool IsHighPixel(unsigned short const& arg) {
      return arg == Isis::HIGH_INSTR_SATU2 || arg == Isis::HIGH_REPR_SATU2;
    }
    static inline bool IsNull(unsigned short const& arg) {
      return arg == Isis::NULLU2;
    }
  };
  template<typename T> struct Helper<unsigned char, T> {
    static inline bool IsSpecial(unsigned char const& arg) {
      return arg < Isis::VALID_MIN1 || arg > Isis::VALID_MAX1;
    }
    static inline bool IsHighPixel(unsigned char const& arg) {
      return arg == Isis::HIGH_INSTR_SAT1 || arg == Isis::HIGH_REPR_SAT1;
    }
    static inline bool IsNull(unsigned char const& arg) {
      return arg == Isis::NULL1;
    }
  };

  PixelT operator() (PixelT const& pix) const {
    typedef typename vw::CompoundChannelType<PixelT>::type channel_type;
    typedef Helper<channel_type, void> help;
    for (size_t n = 0; n < vw::CompoundNumChannels<PixelT>::value; ++n) {
      if (help::IsSpecial(vw::compound_select_channel<const channel_type&>(pix,n))) {
        if (help::IsHighPixel(vw::compound_select_channel<const channel_type&>(pix,n)))
          return m_replacement_high;
        else if (help::IsNull(vw::compound_select_channel<const channel_type&>(pix,n)))
          return m_replacement_null;
        else
          return m_replacement_low;
      }
    }
    return pix;
  }
};

// Specialize this only for float pixels, as that's all that is needed
vw::ImageViewRef<float>
remove_isis_special_pixels(vw::ImageViewRef<float> const& image,
                           float r_low, float r_high, float r_null) {
  auto obj = IsisSpecialPixelFunc<float>(r_low, r_high, r_null);
  return vw::per_pixel_filter(image.impl(), obj);
}

} // end namespace asp

#endif  // ASP_HAVE_PKG_ISIS
