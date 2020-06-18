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


/// \file StereoSessionIsis.h
///

#ifndef __STEREO_SESSION_ISIS_H__
#define __STEREO_SESSION_ISIS_H__

#include <asp/Sessions/StereoSession.h>
#include <vw/Stereo/StereoModel.h>

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1

// Isis Headers
#include <SpecialPixel.h>

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
    IsisSpecialPixelFunc() : m_replacement_low(0), m_replacement_high(0), m_replacement_null(0) {}

  public:
    IsisSpecialPixelFunc(PixelT const& pix_l, PixelT const& pix_h, PixelT const& pix_n) : m_replacement_low(pix_l), m_replacement_high(pix_h), m_replacement_null(pix_n) {}

    // TODO: What is all this helper stuff??

    // Helper to determine special across different channel types
    template <typename ChannelT, typename T = void>
    struct Helper {
      static inline bool IsSpecial( ChannelT const& arg ) { return false; }
      static inline bool IsHighPixel( ChannelT const& arg ) { return false; }
      static inline bool IsNull( ChannelT const& arg ) { return false; }
    };
    template<typename T> struct Helper<double, T> {
      static inline bool IsSpecial( double const& arg ) {
        return arg < Isis::VALID_MIN8;
      }
      static inline bool IsHighPixel( double const& arg ) {
        return arg == Isis::HIGH_INSTR_SAT8 || arg == Isis::HIGH_REPR_SAT8;
      }
      static inline bool IsNull( double const& arg ) {
        return arg == Isis::NULL8;
      }
    };
    template<typename T> struct Helper<float, T> {
      static inline bool IsSpecial( float const& arg ) {
        return arg < Isis::VALID_MIN4;
      }
      static inline bool IsHighPixel( float const& arg ) {
        return arg == Isis::HIGH_INSTR_SAT4 || arg == Isis::HIGH_REPR_SAT4;
      }
      static inline bool IsNull( float const& arg ) {
        return arg == Isis::NULL4;
      }
    };
    template<typename T> struct Helper<short, T>{
      static inline bool IsSpecial( short const& arg ) {
        return arg < Isis::VALID_MIN2;
      }
      static inline bool IsHighPixel( short const& arg ) {
        return arg == Isis::HIGH_INSTR_SAT2 || arg == Isis::HIGH_REPR_SAT2;
      }
      static inline bool IsNull( short const& arg ) {
        return arg == Isis::NULL2;
      }
    };
    template<typename T> struct Helper<unsigned short, T> {
      static inline bool IsSpecial( unsigned short const& arg ) {
        return arg < Isis::VALID_MINU2 || arg > Isis::VALID_MAXU2;
      }
      static inline bool IsHighPixel( unsigned short const& arg ) {
        return arg == Isis::HIGH_INSTR_SATU2 || arg == Isis::HIGH_REPR_SATU2;
      }
      static inline bool IsNull( unsigned short const& arg ) {
        return arg == Isis::NULLU2;
      }
    };
    template<typename T> struct Helper<unsigned char, T> {
      static inline bool IsSpecial( unsigned char const& arg ) {
        return arg < Isis::VALID_MIN1 || arg > Isis::VALID_MAX1;
      }
      static inline bool IsHighPixel( unsigned char const& arg ) {
        return arg == Isis::HIGH_INSTR_SAT1 || arg == Isis::HIGH_REPR_SAT1;
      }
      static inline bool IsNull( unsigned char const& arg ) {
        return arg == Isis::NULL1;
      }
    };

    PixelT operator() (PixelT const& pix) const {
      using namespace vw;
      typedef typename CompoundChannelType<PixelT>::type channel_type;
      typedef Helper<channel_type, void> help;
      for (size_t n = 0; n < CompoundNumChannels<PixelT>::value; ++n) {
        if (help::IsSpecial(compound_select_channel<const channel_type&>(pix,n))) {
          if (help::IsHighPixel(compound_select_channel<const channel_type&>(pix,n)))
            return m_replacement_high;
          else if ( help::IsNull(compound_select_channel<const channel_type&>(pix,n)))
            return m_replacement_null;
          else
            return m_replacement_low;
        }
      }
      return pix;
    }
  };

  template <class ViewT>
  vw::UnaryPerPixelView<ViewT, IsisSpecialPixelFunc<typename ViewT::pixel_type> >
  remove_isis_special_pixels(vw::ImageViewBase<ViewT> &image,
                             typename ViewT::pixel_type r_low  = typename ViewT::pixel_type(),
                             typename ViewT::pixel_type r_high = typename ViewT::pixel_type(),
                             typename ViewT::pixel_type r_null = typename ViewT::pixel_type()) {
    return vw::per_pixel_filter(image.impl(),
                                IsisSpecialPixelFunc<typename ViewT::pixel_type>(r_low,r_high,r_null));
  }



  /// Derived StereoSession class for ISIS images.
  class StereoSessionIsis : public StereoSession {
  public:
    StereoSessionIsis() {}
    virtual ~StereoSessionIsis() {}

    virtual std::string name() const { return "isis"; }
    
    /// Only the alternative CSM sensor model for ISIS images supports multi threading.
    virtual bool supports_multi_threading() const;
    
    /// Returns the target datum to use for a given camera model
    virtual vw::cartography::Datum get_datum(const vw::camera::CameraModel* cam,
                                             bool use_sphere_for_datum) const;

    /// Stage 1: Preprocessing
    ///
    // Pre file is a pair of images.            ( ImageView<PixelT> )
    virtual void pre_preprocessing_hook(bool adjust_left_image_size,
                                        std::string const& left_input_file,
                                        std::string const& right_input_file,
                                        std::string      & left_output_file,
                                        std::string      & right_output_file);

    /// Stage 2: Correlation
    ///
    /// Pre file is a pair of grayscale images.  ( ImageView<PixelGray<float> > )
    /// Post file is a disparity map.            ( ImageView<PixelDisparity> > )
    virtual void pre_filtering_hook(std::string const& input_file,
                                    std::string      & output_file);

    /// Stage 4: Point cloud generation
    virtual vw::ImageViewRef<vw::PixelMask<vw::Vector2f> >
    pre_pointcloud_hook(std::string const& input_file);

    /// Simple factory function.
    static StereoSession* construct() { return new StereoSessionIsis; }

    
  protected:
    /// Function to load a camera model of the particular type.
    virtual boost::shared_ptr<vw::camera::CameraModel> load_camera_model(std::string const& image_file, 
                                                                         std::string const& camera_file,
                                                                         vw::Vector2 pixel_offset) const;
  };


} // end namespace asp

#include <asp/Sessions/StereoSessionIsis.tcc>

#endif  // ASP_HAVE_PKG_ISISIO

#endif // __STEREO_SESSION_ISIS_H__
