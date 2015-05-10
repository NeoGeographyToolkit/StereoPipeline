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


/// \file StereoSessionConcrete.h
///

#ifndef __STEREO_SESSION_CONCRETE_H__
#define __STEREO_SESSION_CONCRETE_H__
/*
#include <vw/Image/ImageViewBase.h>
#include <vw/Image/ImageViewRef.h>


#include <vw/Math/Functors.h>
#include <vw/Math/Geometry.h>
#include <vw/InterestPoint.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/operations.hpp>

#include <asp/Core/Common.h>
#include <asp/Core/InterestPointMatching.h>
*/

#include <vw/Cartography.h>
#include <vw/Geometry.h>
#include <vw/Math.h>
#include <vw/Camera.h>
#include <vw/Stereo/StereoModel.h>

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/RPC/RPCStereoModel.h>

namespace asp {



  /// List of disk transform options
  enum STEREOSESSION_DISKTRANSFORM_TYPE
  {
    DISKTRANSFORM_TYPE_MATRIX       = 0, // Covers Homography, Affine-Epipolar, and None.
    DISKTRANSFORM_TYPE_MATRIX_RIGHT = 1, // As Matrix, but only for the right image.
    DISKTRANSFORM_TYPE_MAP_PROJECT  = 2
  };
  /// List of stereo model options
  enum STEREOSESSION_STEREOMODEL_TYPE
  {
    STEREOMODEL_TYPE_PINHOLE = 0,
    STEREOMODEL_TYPE_ISIS    = 1,
    STEREOMODEL_TYPE_DG      = 2,
    STEREOMODEL_TYPE_RPC     = 3
  };

  template <int v>
  struct Int2Type
  {
    enum {value = v};
  };


  /// Unitily for converting DISKTRANSFORM_TYPE into the corresponding class
  template <STEREOSESSION_DISKTRANSFORM_TYPE T> struct DiskTransformType2Class { typedef vw::HomographyTransform       type; };
  template <> struct DiskTransformType2Class<DISKTRANSFORM_TYPE_MAP_PROJECT>   { typedef vw::cartography::Map2CamTrans type; };

  /// Unitily for converting STEREOMODEL_TYPE into the corresponding class
  template <STEREOSESSION_STEREOMODEL_TYPE T> struct StereoModelType2Class { typedef vw::stereo::StereoModel    type; };
  template <> struct StereoModelType2Class<STEREOMODEL_TYPE_RPC>           { typedef asp::RPCStereoModel type; };

  /// Define the currently used names
  template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE, // Transform from disk pixels to sensor pixels
            STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE  > struct NameFromTypes       { static std::string name(){return("StereoSessionConcrete");} };
  template <> struct NameFromTypes<DISKTRANSFORM_TYPE_MATRIX,       STEREOMODEL_TYPE_ISIS   > { static std::string name(){return("isis"        );} };
  template <> struct NameFromTypes<DISKTRANSFORM_TYPE_MATRIX_RIGHT, STEREOMODEL_TYPE_PINHOLE> { static std::string name(){return("pinhole"     );} };
  template <> struct NameFromTypes<DISKTRANSFORM_TYPE_MATRIX,       STEREOMODEL_TYPE_PINHOLE> { static std::string name(){return("nadirpinhole");} };
  template <> struct NameFromTypes<DISKTRANSFORM_TYPE_MATRIX,       STEREOMODEL_TYPE_DG     > { static std::string name(){return("dg"          );} };
  template <> struct NameFromTypes<DISKTRANSFORM_TYPE_MAP_PROJECT,  STEREOMODEL_TYPE_DG     > { static std::string name(){return("dgmaprpc"    );} };
  template <> struct NameFromTypes<DISKTRANSFORM_TYPE_MAP_PROJECT,  STEREOMODEL_TYPE_RPC    > { static std::string name(){return("rpcmaprpc"   );} };

  // TODO: Conversion functions from input strings


  /// Derived class of StereoSession with different configuration options
  /// - This keeps the base class template-free but makes it easier for
  ///   derived classes to share common code.
  /// - This class implements sensor model loading and transform types so derived classes do not have to.
  /// - For now the derived classes are still responsible for the various hooks.
  template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE, // Transform from disk pixels to sensor pixels
            STEREOSESSION_STEREOMODEL_TYPE    STEREOMODEL_TYPE>   // Sensor model type
  class StereoSessionConcrete : public StereoSession
  {
  public: // Definitions


  public: // Functions

    StereoSessionConcrete();
    virtual ~StereoSessionConcrete();

    /// Verify that the inputs needed by the template configuration are selected
    /// - Derived classes should call this before initializing their own behavior.
    virtual void initialize(BaseOptions const& options,
                            std::string const& left_image_file,
                            std::string const& right_image_file,
                            std::string const& left_camera_file,
                            std::string const& right_camera_file,
                            std::string const& out_prefix,
                            std::string const& input_dem);

/*
    /// This class guesses the name but derived classes may still need to override.
    virtual std::string name() const { return NameFromTypes<DISKTRANSFORM_TYPE, STEREOMODEL_TYPE>::name(); }
*/

    /// Method that produces a Camera Model from input files.
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model(std::string const& image_file,
                 std::string const& camera_file = "");

    /// Transforms from pixel coordinates on disk to original unwarped image coordinates.
    /// - For reversing our arithmetic applied in preprocessing.
    typedef typename DiskTransformType2Class<DISKTRANSFORM_TYPE>::type tx_type;
    tx_type tx_left () const;
    tx_type tx_right() const;

    /// The type of sensor models created by this lass
    typedef typename StereoModelType2Class<STEREOMODEL_TYPE>::type stereo_model_type;
/*
    /// Simple factory function
    static StereoSession* construct() { return new StereoSessionConcrete<DISKTRANSFORM_TYPE, STEREOMODEL_TYPE>; }
*/
  private: // Variables

    /// Storage for the camera models used to map project the input images.
    boost::shared_ptr<vw::camera::CameraModel> m_left_map_proj_model, m_right_map_proj_model;

  private: // Functions

    // Init calls for the chosen stereo model
    void init_sensor_model(Int2Type<STEREOMODEL_TYPE_PINHOLE>) {}
    void init_sensor_model(Int2Type<STEREOMODEL_TYPE_ISIS   >) {}
    void init_sensor_model(Int2Type<STEREOMODEL_TYPE_DG     >) {} // Currently being lazy and doing init in senser model load!
    void init_sensor_model(Int2Type<STEREOMODEL_TYPE_RPC    >) {} // ditto.

    // Init calls for the chosen disk transform
    void init_disk_transform(Int2Type<DISKTRANSFORM_TYPE_MATRIX      >) {}
    void init_disk_transform(Int2Type<DISKTRANSFORM_TYPE_MATRIX_RIGHT>) {}
    void init_disk_transform(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT >); // Needs to load the map_proj_models

    // Specializations of camera_model for each of the different model types
    boost::shared_ptr<vw::camera::CameraModel> load_camera_model(Int2Type<STEREOMODEL_TYPE_PINHOLE>, std::string const& image_file, std::string const& camera_file="");
    boost::shared_ptr<vw::camera::CameraModel> load_camera_model(Int2Type<STEREOMODEL_TYPE_ISIS   >, std::string const& image_file, std::string const& camera_file="");
    boost::shared_ptr<vw::camera::CameraModel> load_camera_model(Int2Type<STEREOMODEL_TYPE_DG     >, std::string const& image_file, std::string const& camera_file="");
    boost::shared_ptr<vw::camera::CameraModel> load_camera_model(Int2Type<STEREOMODEL_TYPE_RPC    >, std::string const& image_file, std::string const& camera_file="");

    // Specializations of disk transform method
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MATRIX      >) const;
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MATRIX_RIGHT>) const;
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT >) const;
    
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MATRIX      >) const;
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MATRIX_RIGHT>) const;
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT >) const;


  };


} // end namespace asp

#include <asp/Sessions/StereoSessionConcrete.tcc>


#endif // __STEREO_SESSION_CONCRETE_H__
