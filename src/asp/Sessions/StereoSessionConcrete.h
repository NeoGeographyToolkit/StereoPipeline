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


#include <vw/Cartography.h>
#include <vw/Geometry.h>
#include <vw/Math.h>
#include <vw/Camera.h>
#include <vw/Stereo/StereoModel.h>

#include <asp/Sessions/StereoSession.h>
#include <asp/Camera/RPCStereoModel.h>
#include <asp/Sessions/CameraModelLoader.h>

namespace asp {



  /// List of disk transform options
  /// - These describe if the image in disk has been transformed by a map projection
  ///   or some other type of alignment transform.
  enum STEREOSESSION_DISKTRANSFORM_TYPE
  {
    DISKTRANSFORM_TYPE_MATRIX              = 0,  // Covers Homography, Affine-Epipolar, and None.
    DISKTRANSFORM_TYPE_MATRIX_RIGHT        = 1,  // As Matrix, but only homography and only for the right image.
    DISKTRANSFORM_TYPE_MAP_PROJECT_RPC     = 2,  // RPC     map projected image
    DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS    = 3,  // ISIS    map projected image
    DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE = 4,  // Pinhole map projected image
    DISKTRANSFORM_TYPE_MAP_PROJECT_SPOT5   = 5,  // Spot5   map projected image
    DISKTRANSFORM_TYPE_MAP_PROJECT_ASTER   = 6   // ASTER   map projected image
  };
  /// List of stereo model options
  /// - This can be different from the model which was used to map project an image on disk.
  /// - This is what is used for triangulation and other operations.
  enum STEREOSESSION_STEREOMODEL_TYPE
  {
    STEREOMODEL_TYPE_PINHOLE = 0,
    STEREOMODEL_TYPE_ISIS    = 1,
    STEREOMODEL_TYPE_DG      = 2,
    STEREOMODEL_TYPE_RPC     = 3,
    STEREOMODEL_TYPE_SPOT5   = 4,
    STEREOMODEL_TYPE_ASTER   = 5
  };

  // TODO: Move this to vw somewhere?
  template <int v>
  struct Int2Type
  {
    enum {value = v};
  };


  /// Utility for converting DISKTRANSFORM_TYPE into the corresponding class
  template <STEREOSESSION_DISKTRANSFORM_TYPE T> struct DiskTransformType2Class       { typedef vw::HomographyTransform       type; };
  template <> struct DiskTransformType2Class<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC    > { typedef vw::cartography::Map2CamTrans type; };
  template <> struct DiskTransformType2Class<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS   > { typedef vw::cartography::Map2CamTrans type; };
  template <> struct DiskTransformType2Class<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE> { typedef vw::cartography::Map2CamTrans type; };
  template <> struct DiskTransformType2Class<DISKTRANSFORM_TYPE_MAP_PROJECT_SPOT5> { typedef vw::cartography::Map2CamTrans type; };
  template <> struct DiskTransformType2Class<DISKTRANSFORM_TYPE_MAP_PROJECT_ASTER> { typedef vw::cartography::Map2CamTrans type; };

  /// Utility for converting STEREOMODEL_TYPE into the corresponding class
  template <STEREOSESSION_STEREOMODEL_TYPE T> struct StereoModelType2Class { typedef vw::stereo::StereoModel type; };
  // Unfortunately there is still a problems here that the unit test does not catch.
  //template <> struct StereoModelType2Class<STEREOMODEL_TYPE_RPC>           { typedef asp::RPCStereoModel     type; };

  /// Utility returns true if our inputs are map projected.
  template <STEREOSESSION_DISKTRANSFORM_TYPE  DISKTRANSFORM_TYPE> struct IsTypeMapProjected { static const bool value=false; };
  template <> struct IsTypeMapProjected<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC    > { static const bool value=true; };
  template <> struct IsTypeMapProjected<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS   > { static const bool value=true; };
  template <> struct IsTypeMapProjected<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE> { static const bool value=true; };
  template <> struct IsTypeMapProjected<DISKTRANSFORM_TYPE_MAP_PROJECT_SPOT5> { static const bool value=true; };
  template <> struct IsTypeMapProjected<DISKTRANSFORM_TYPE_MAP_PROJECT_ASTER> { static const bool value=true; };

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

    StereoSessionConcrete() {}
    virtual ~StereoSessionConcrete() {}

    // Static copies of some of the characteristic functions defined in the base class.
    static bool isMapProjected() { return IsTypeMapProjected<DISKTRANSFORM_TYPE>::value; }

    // Override the base class functions according to the class paramaters
    virtual bool uses_map_projected_inputs() const {return  isMapProjected();}
    virtual bool requires_input_dem       () const {return  isMapProjected();}
    virtual bool supports_image_alignment () const {return !isMapProjected();}

    /// Verify that the inputs needed by the template configuration are selected
    /// - Derived classes should call this before initializing their own behavior.
    virtual void initialize(vw::cartography::GdalWriteOptions const& options,
                            std::string const& left_image_file,
                            std::string const& right_image_file,
                            std::string const& left_camera_file,
                            std::string const& right_camera_file,
                            std::string const& out_prefix,
                            std::string const& input_dem);


    /// Override the default ip_matching implementation so it just throws if we are using DISKTRANSFORM_TYPE_MAP_PROJECT
    inline virtual bool ip_matching(std::string  const& input_file1,
                                    std::string  const& input_file2,
                                    vw::Vector2  const& uncropped_image_size,
                                    Vector6f const& stats1,
                                    Vector6f const& stats2,
                                    int ip_per_tile,
                                    float nodata1, float nodata2,
                                    std::string const& match_filename,
                                    vw::camera::CameraModel* cam1,
                                    vw::camera::CameraModel* cam2);

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

    /// The type of sensor models created by this class
    typedef typename StereoModelType2Class<STEREOMODEL_TYPE>::type stereo_model_type;
/*
    /// Simple factory function
    static StereoSession* construct() { return new StereoSessionConcrete<DISKTRANSFORM_TYPE, STEREOMODEL_TYPE>; }
*/
  private: // Variables

    /// Object to handle camera model loading
    CameraModelLoader m_camera_loader;

    /// Storage for the camera models used to map project the input images.
    boost::shared_ptr<vw::camera::CameraModel> m_left_map_proj_model, m_right_map_proj_model;

  private: // Functions

    // Init calls for the chosen disk transform
    void init_disk_transform(STEREOSESSION_DISKTRANSFORM_TYPE disk_transform_type);

    /// Function to load a camera model of the specified type
    /// - The type is handled by a switch statement; there is no benefit to templatizing this.
    boost::shared_ptr<vw::camera::CameraModel> load_camera_model(STEREOSESSION_STEREOMODEL_TYPE model_type, 
                                                                 std::string const& image_file, 
                                                                 std::string const& camera_file);

    // Specializations of disk transform method
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MATRIX             >) const;
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MATRIX_RIGHT       >) const;
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC    >) const;
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS   >) const;
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE>) const;
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_SPOT5  >) const;
    tx_type tx_left (Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ASTER  >) const;

    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MATRIX             >) const;
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MATRIX_RIGHT       >) const;
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_RPC    >) const;
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ISIS   >) const;
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_PINHOLE>) const;
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_SPOT5  >) const;
    tx_type tx_right(Int2Type<DISKTRANSFORM_TYPE_MAP_PROJECT_ASTER  >) const;

  };


} // end namespace asp

#include <asp/Sessions/StereoSessionConcrete.tcc>


#endif // __STEREO_SESSION_CONCRETE_H__
