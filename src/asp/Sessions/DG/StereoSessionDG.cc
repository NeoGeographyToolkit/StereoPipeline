// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file StereoSessionDG.cc
///

// Ames Stereo Pipeline
#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Sessions/DG/LinescanDGModel.h>
#include <asp/Sessions/DG/StereoSessionDG.h>
#include <asp/Sessions/DG/XML.h>
#include <asp/Sessions/RPC/RPCModel.h>

// Vision Workbench
#include <vw/Camera/Extrinsics.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/PointImageManipulation.h>

// Std
#include <iostream>
#include <string>

// Other
#include <xercesc/dom/DOM.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>

// Boost
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace vw;
using namespace asp;
using namespace xercesc;
namespace pt = boost::posix_time;
namespace fs = boost::filesystem;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector2f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
}

// Helper class for converting to floating point seconds based on a
// given reference.
class SecondsFrom {
  pt::ptime m_reference;
public:
  SecondsFrom( pt::ptime const& time ) : m_reference(time) {}

  double operator()( pt::ptime const& time ) const {
    return double( (time - m_reference).total_microseconds() ) / 1e6;
  }
};

// RPCMapTransform. Used to test the validity of IP matching on map
// projected images. However, this could be used for performing an RPC
// map projection.
class RPCMapTransform : public TransformBase<RPCMapTransform> {
  RPCModel m_rpc;
  cartography::GeoReference m_image_georef, m_dem_georef;
  DiskImageView<float> m_dem;
  ImageViewRef<Vector3> m_point_cloud;
public:
  RPCMapTransform( RPCModel const& rpc,
                   cartography::GeoReference const& image_georef,
                   cartography::GeoReference const& dem_georef,
                   boost::shared_ptr<DiskImageResource> dem_rsrc ) : m_rpc(rpc), m_image_georef(image_georef), m_dem_georef(dem_georef), m_dem(dem_rsrc) {
    if ( dem_rsrc->has_nodata_read() )
      m_point_cloud =
        cartography::geo_transform(
          geodetic_to_cartesian(
            dem_to_geodetic( create_mask( m_dem, dem_rsrc->nodata_read()),
                             m_dem_georef ), m_dem_georef.datum() ),
          m_dem_georef, m_image_georef,
          ValueEdgeExtension<Vector3>( Vector3() ) );
    else
      m_point_cloud =
        cartography::geo_transform(
          geodetic_to_cartesian(
            dem_to_geodetic( m_dem, m_dem_georef ),
            m_dem_georef.datum() ),
          m_dem_georef, m_image_georef,
          ValueEdgeExtension<Vector3>( Vector3() ) );
  }

  // Convert Map Projected Coordinate to camera coordinate
  inline Vector2 reverse(const Vector2 &p) const {
    Vector3 point = m_point_cloud(p.x(),p.y());
    return m_rpc.point_to_pixel( point );
  }
};

// Helper functor that converts projected pixel indices and height
// value to unprojected pixel indices.
class OriginalCameraIndex : public ReturnFixedType<Vector2f> {
  RPCModel m_rpc;
  BBox2i m_image_boundaries;
public:
  OriginalCameraIndex( RPCModel const& rpc,
                       BBox2i const& bbox ) : m_rpc(rpc),
                                              m_image_boundaries(bbox) {}

  Vector2f operator()( Vector3 const& point ) const {
    if ( point == Vector3() )
      return Vector2f(-1,-1);
    Vector2f result = m_rpc.point_to_pixel( point );
    if ( m_image_boundaries.contains( result ) )
      return result;
    return Vector2f(-1,1);
  }
};

// Helper function to read RPC.
RPCModel*
read_rpc_model( std::string const& image_file,
                std::string const& camera_file ) {
  RPCModel* rpc_model = NULL;
  try {
    rpc_model = new RPCModel( image_file );
  } catch ( NotFoundErr const& err ) {}

  if ( !rpc_model ) {
    RPCXML rpc_xml;
    rpc_xml.read_from_file( camera_file );
    rpc_model = new RPCModel( *rpc_xml.rpc_ptr() ); // Copy the ptr

    // We don't catch an error here because the User will need to
    // know of a failure at this point. We previously opened the
    // XML safely before.
  }
  return rpc_model;
}

namespace asp {

  // Xerces-C initialize
  StereoSessionDG::StereoSessionDG() : m_rpc_map_projected(false) {
    XMLPlatformUtils::Initialize();
  }

  // Initializer to determine what kinda of input do we have?
  void StereoSessionDG::initialize(BaseOptions const& options,
                                   std::string const& left_image_file,
                                   std::string const& right_image_file,
                                   std::string const& left_camera_file,
                                   std::string const& right_camera_file,
                                   std::string const& out_prefix,
                                   std::string const& extra_argument1,
                                   std::string const& extra_argument2,
                                   std::string const& extra_argument3,
                                   std::string const& extra_argument4 ) {
    StereoSession::initialize( options, left_image_file,
                               right_image_file, left_camera_file,
                               right_camera_file, out_prefix,
                               extra_argument1, extra_argument2,
                               extra_argument3, extra_argument4 );

    // Is there a possible DEM?
    if ( !extra_argument1.empty() ) {
      boost::scoped_ptr<RPCModel> model1, model2;
      // Try and pull RPC Camera Models from left and right images.
      try {
        model1.reset( new RPCModel(left_image_file) );
        model2.reset( new RPCModel(right_image_file) );
      } catch ( NotFoundErr const& err ) {}

      // If the above failed to load the RPC Model. Let's try from the XML.
      if ( !model1.get() || !model2.get() ) {
        try {
          RPCXML rpc_xml;
          rpc_xml.read_from_file( left_camera_file );
          model1.reset( new RPCModel( *rpc_xml.rpc_ptr() ) ); // Copy the ptr
          rpc_xml.read_from_file( right_camera_file );
          model2.reset( new RPCModel( *rpc_xml.rpc_ptr() ) );
        } catch ( IOErr const& err ) {
          // Just give up if it is not there.
          vw_out(WarningMessage) << "Unknown extra argument \"" << extra_argument1 << "\". Ignoring.";
          return;
        }
      }

      // Double check that we can read the DEM and that it has
      // cartographic information.
      if ( !fs::exists( extra_argument1 ) )
        vw_throw( ArgumentErr() << "StereoSessionDG: DEM \"" << extra_argument1
                  << "\" doesn't exist." );

      // Verify that center of our lonlat boundaries from the RPC models
      // actually projects into the DEM. (?)

      m_rpc_map_projected = true;
    }
  }

  // Provide our camera model
  boost::shared_ptr<camera::CameraModel>
  StereoSessionDG::camera_model( std::string const& /*image_file*/,
                                 std::string const& camera_file ) {
    GeometricXML geo;
    AttitudeXML att;
    EphemerisXML eph;
    ImageXML img;
    read_xml( camera_file, geo, att, eph, img );

    // Convert measurements in millimeters to pixels.
    geo.principal_distance /= geo.detector_pixel_pitch;
    geo.detector_origin /= geo.detector_pixel_pitch;

    // Convert all time measurements to something that boost::date_time can read.
    boost::replace_all( eph.start_time, "T", " " );
    boost::replace_all( img.tlc_start_time, "T", " " );
    boost::replace_all( img.first_line_start_time, "T", " " );
    boost::replace_all( att.start_time, "T", " " );

    // Convert UTC time measurements to line measurements. Ephemeris
    // start time will be our reference frame to calculate seconds
    // against.
    SecondsFrom convert( pt::time_from_string( eph.start_time ) );

    // I'm going make the assumption that EPH and ATT are sampled at the
    // same rate and time.
    VW_ASSERT( eph.position_vec.size() == att.quat_vec.size(),
               MathErr() << "Ephemeris and Attitude don't have the same number of samples." );
    VW_ASSERT( eph.start_time == att.start_time && eph.time_interval == att.time_interval,
               MathErr() << "Ephemeris and Attitude don't seem to sample with the same t0 or dt." );

    // I also don't support optical distortion yet.
    VW_ASSERT( geo.optical_polyorder <= 0,
               NoImplErr() << "Cameras with optical distortion are not supported currently." );

    // Convert ephemeris to be position of camera. Change attitude to be
    // be the rotation from camera frame to world frame. We also add an
    // additional rotation to the camera frame so X is the horizontal
    // direction to the picture and +Y points down the image (in the
    // direction of flight).
    Quat sensor_coordinate = math::euler_xyz_to_quaternion(Vector3(0,0,geo.detector_rotation * M_PI/180.0 - M_PI/2));
    for ( size_t i = 0; i < eph.position_vec.size(); i++ ) {
      eph.position_vec[i] += att.quat_vec[i].rotate( geo.perspective_center );
      att.quat_vec[i] = att.quat_vec[i] * geo.camera_attitude * sensor_coordinate;
    }

    typedef LinescanDGModel<camera::PiecewiseAPositionInterpolation, camera::SLERPPoseInterpolation, camera::TLCTimeInterpolation> camera_type;
    typedef boost::shared_ptr<camera::CameraModel> result_type;

    return result_type( new camera_type( camera::PiecewiseAPositionInterpolation( eph.position_vec, eph.velocity_vec,
                                                                                  convert( pt::time_from_string( eph.start_time ) ),
                                                                                  eph.time_interval ),
                                         camera::SLERPPoseInterpolation( att.quat_vec,
                                                                         convert( pt::time_from_string( att.start_time ) ),
                                                                         att.time_interval ),
                                         camera::TLCTimeInterpolation( img.tlc_vec,
                                                                       convert( pt::time_from_string( img.tlc_start_time ) ) ),
                                         img.image_size, subvector(inverse(sensor_coordinate).rotate(Vector3(geo.detector_origin[0],
                                                                                                             geo.detector_origin[1], 0 ) ), 0, 2 ),
                                         geo.principal_distance ) );
  }

  // LUT image access
  bool StereoSessionDG::has_lut_images() const {
    return m_rpc_map_projected;
  }

  // The madness that happens below with the conversion from DEM
  // to cartesian and then geo_transforming ... is purely to
  // handle the problem of different datums between the DEM and
  // the project camera models. I hope GDAL noticed this.
  ImageViewRef<Vector2f>
  StereoSessionDG::generate_lut_image( std::string const& image_file,
                                       std::string const& camera_file ) const {
    boost::shared_ptr<DiskImageResource> dem_rsrc( DiskImageResource::open( m_extra_argument1 ) ),
      image_rsrc( DiskImageResource::open( image_file ) );

    ImageXML image_xml;
    RPCXML rpc_xml;

    { // Parsing XML
      boost::scoped_ptr<XercesDOMParser> parser( new XercesDOMParser() );
      parser->setValidationScheme(XercesDOMParser::Val_Always);
      parser->setDoNamespaces(true);
      boost::scoped_ptr<ErrorHandler> errHandler( new HandlerBase() );
      parser->setErrorHandler(errHandler.get());

      parser->parse( camera_file.c_str() );
      DOMDocument* xmlDoc = parser->getDocument();
      DOMElement* elementRoot = xmlDoc->getDocumentElement();

      DOMNodeList* children = elementRoot->getChildNodes();
      for ( XMLSize_t i = 0; i < children->getLength(); ++i ) {
        DOMNode* curr_node = children->item(i);
        if ( curr_node->getNodeType() == DOMNode::ELEMENT_NODE ) {
          DOMElement* curr_element =
            dynamic_cast<DOMElement*>( curr_node );
          std::string tag( XMLString::transcode(curr_element->getTagName()) );
          if ( tag == "RPB" ) {
            rpc_xml.parse( curr_element );
          } else if ( tag == "IMD" ) {
            image_xml.parse( curr_element );
          }
        }
      }
    }

    BBox2i map_image_bbox( 0, 0, image_rsrc->cols(),
                           image_rsrc->rows() ),
      org_image_bbox( Vector2i(),
                      image_xml.image_size );

    cartography::GeoReference dem_georef, image_georef;
    read_georeference( dem_georef, m_extra_argument1 );
    read_georeference( image_georef, image_file );

    boost::scoped_ptr<RPCModel> rpc_model( new RPCModel( *rpc_xml.rpc_ptr() ) );

    DiskImageView<float> dem( dem_rsrc );
    if ( dem_rsrc->has_nodata_read() ) {
      return crop(per_pixel_filter(
               cartography::geo_transform(
                 geodetic_to_cartesian(
                   dem_to_geodetic(
                     create_mask(dem, dem_rsrc->nodata_read()), dem_georef ), dem_georef.datum()),
                 dem_georef, image_georef,
                 image_rsrc->cols(),
                 image_rsrc->rows(),
                 ValueEdgeExtension<Vector3>( Vector3() ) ),
               OriginalCameraIndex( *rpc_model, org_image_bbox ) ), map_image_bbox );
    }
    return crop(per_pixel_filter(
             cartography::geo_transform(
               geodetic_to_cartesian(
                 dem_to_geodetic(dem, dem_georef ), dem_georef.datum()),
               dem_georef, image_georef,
               image_rsrc->cols(),
               image_rsrc->rows(),
               ValueEdgeExtension<Vector3>( Vector3() ) ),
             OriginalCameraIndex( *rpc_model, org_image_bbox ) ), map_image_bbox );
  }

  ImageViewRef<Vector2f> StereoSessionDG::lut_image_left() const {
    if ( !m_rpc_map_projected )
      vw_throw( LogicErr() << "StereoSessionDG: This is not a map projected session. LUT table shouldn't be used here" );
    return generate_lut_image( m_left_image_file, m_left_camera_file );
  }

  ImageViewRef<Vector2f> StereoSessionDG::lut_image_right() const {
    if ( !m_rpc_map_projected )
      vw_throw( LogicErr() << "StereoSessionDG: This is not a map projected session. LUT table shouldn't be used here" );
    return generate_lut_image( m_right_image_file, m_right_camera_file );
  }

  void StereoSessionDG::pre_preprocessing_hook(std::string const& input_file1,
                                               std::string const& input_file2,
                                               std::string &output_file1,
                                               std::string &output_file2) {

    // Load the unmodified images
    boost::shared_ptr<DiskImageResource>
      left_rsrc( DiskImageResource::open(m_left_image_file) ),
      righ_rsrc( DiskImageResource::open(m_right_image_file) );
    DiskImageView<PixelGray<float> >
      left_disk_image( left_rsrc ),
      righ_disk_image( righ_rsrc );

    // Normalized Images' filenames
    output_file1 = m_out_prefix + "-L.tif";
    output_file2 = m_out_prefix + "-R.tif";

    // See if these files already exist, if they do don't bother
    // writting them again.
    bool rebuild = false;
    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelGray<float> > testa(output_file1);
      DiskImageView<PixelGray<float> > testb(output_file2);
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
      rebuild = true;
    } catch (vw::ArgumentErr const& e ) {
      // Throws on a corrupted file.
      vw_settings().reload_config();
      rebuild = true;
    }

    // They don't exist or are corrupted.
    if (rebuild) {
      Vector4f left_stats = gather_stats( left_disk_image, "left" ),
        right_stats = gather_stats( righ_disk_image, "right" );

      ImageViewRef<PixelGray<float> > Limg, Rimg;
      std::string lcase_file = boost::to_lower_copy(m_left_camera_file);

      if ( stereo_settings().alignment_method == "homography" ) {
        std::string match_filename =
          m_out_prefix +
          fs::basename(input_file1) + "__" +
          fs::basename(input_file2) + ".match";

        if (!fs::exists(match_filename)) {
          bool inlier = false;

          boost::shared_ptr<camera::CameraModel> cam1, cam2;
          camera_models( cam1, cam2 );
          if ( m_rpc_map_projected ) {
            // We'll make the assumption that the user has map
            // projected the images to the same scale. If they
            // haven't, below would be the ideal but lossy method.
            //            inlier =
            //  homography_ip_matching( left_disk_image, righ_disk_image,
            //                          match_filename );
            boost::scoped_ptr<RPCModel>
              left_rpc( read_rpc_model( m_left_image_file, m_left_camera_file ) ),
              righ_rpc( read_rpc_model( m_right_image_file, m_right_camera_file ) );
            cartography::GeoReference left_georef, righ_georef, dem_georef;
            read_georeference( left_georef, m_left_image_file );
            read_georeference( righ_georef, m_right_image_file );
            read_georeference( dem_georef, m_extra_argument1 );
            boost::shared_ptr<DiskImageResource>
              dem_rsrc( DiskImageResource::open( m_extra_argument1 ) );
            TransformRef
              left_tx( RPCMapTransform( *left_rpc, left_georef,
                                        dem_georef, dem_rsrc ) ),
              righ_tx( RPCMapTransform( *righ_rpc, righ_georef,
                                        dem_georef, dem_rsrc ) );
            inlier =
              ip_matching( cam1.get(), cam2.get(),
                           left_disk_image, righ_disk_image,
                           cartography::Datum("WGS84"), match_filename,
                           left_rsrc->has_nodata_read() ?
                           left_rsrc->nodata_read() :
                           std::numeric_limits<double>::quiet_NaN(),
                           righ_rsrc->has_nodata_read() ?
                           righ_rsrc->nodata_read() :
                           std::numeric_limits<double>::quiet_NaN(),
                           left_tx, righ_tx, false );
          } else {
            inlier =
              ip_matching_w_alignment( cam1.get(), cam2.get(),
                                       left_disk_image, righ_disk_image,
                                       cartography::Datum("WGS84"), match_filename,
                                       left_rsrc->has_nodata_read() ?
                                       left_rsrc->nodata_read() :
                                       std::numeric_limits<double>::quiet_NaN(),
                                       righ_rsrc->has_nodata_read() ?
                                       righ_rsrc->nodata_read() :
                                       std::numeric_limits<double>::quiet_NaN() );
          }

          if ( !inlier ) {
            fs::remove( match_filename );
            vw_throw( IOErr() << "Unable to match left and right images." );
          }
        }

        std::vector<ip::InterestPoint> ip1, ip2;
        ip::read_binary_match_file( match_filename, ip1, ip2  );
        Matrix<double> align_matrix =
          homography_fit(ip2, ip1, bounding_box(left_disk_image) );
        write_matrix( m_out_prefix + "-align.exr", align_matrix );

        vw_out() << "\t--> Aligning right image to left using homography:\n"
                 << "\t      " << align_matrix << "\n";

        // Applying alignment transform
        Limg = left_disk_image;
        Rimg = transform(righ_disk_image,
                         HomographyTransform(align_matrix),
                         left_disk_image.cols(), left_disk_image.rows());
      } else if ( stereo_settings().alignment_method == "epipolar" ) {
        vw_throw( NoImplErr() << "StereoSessionDG doesn't support epipolar rectification" );
      } else {
        // Do nothing just provide the original files.
        Limg = left_disk_image;
        Rimg = righ_disk_image;
      }

      // Apply our normalization options
      if ( stereo_settings().force_max_min > 0 ) {
        if ( stereo_settings().individually_normalize > 0 ) {
          vw_out() << "\t--> Individually normalize images to their respective Min Max\n";
          Limg = normalize( Limg, left_stats[0], left_stats[1], 0, 1.0 );
          Rimg = normalize( Rimg, right_stats[0], right_stats[1], 0, 1.0 );
        } else {
          float low = std::min(left_stats[0], right_stats[0]);
          float hi  = std::max(left_stats[1], right_stats[1]);
          vw_out() << "\t--> Normalizing globally to: [" << low << " " << hi << "]\n";
          Limg = normalize( Limg, low, hi, 0, 1.0 );
          Rimg = normalize( Rimg, low, hi, 0, 1.0 );
        }
      } else {
        if ( stereo_settings().individually_normalize > 0 ) {
          vw_out() << "\t--> Individually normalize images to their respective 4 std dev window\n";
          Limg = normalize( Limg, left_stats[2] - 2*left_stats[3],
                            left_stats[2] + 2*left_stats[3], 0, 1.0 );
          Rimg = normalize( Rimg, right_stats[2] - 2*right_stats[3],
                            right_stats[2] + 2*right_stats[3], 0, 1.0 );
        } else {
          float low = std::min(left_stats[2] - 2*left_stats[3],
                               right_stats[2] - 2*right_stats[3]);
          float hi  = std::max(left_stats[2] + 2*left_stats[3],
                               right_stats[2] + 2*right_stats[3]);
          vw_out() << "\t--> Normalizing globally to: [" << low << " " << hi << "]\n";
          Limg = normalize( Limg, low, hi, 0, 1.0 );
          Rimg = normalize( Rimg, low, hi, 0, 1.0 );
        }
      }


      vw_out() << "\t--> Writing pre-aligned images.\n";
      block_write_gdal_image( output_file1, Limg, m_options,
                              TerminalProgressCallback("asp","\t  L:  ") );
      block_write_gdal_image( output_file2, crop(edge_extend(Rimg,ConstantEdgeExtension()),bounding_box(Limg)), m_options,
                              TerminalProgressCallback("asp","\t  R:  ") );

      // We could write the LUT images at this point, but I'm going to
      // let triangulation render them on the fly. This will save a lot
      // of storage and possibly make the triangulation faster since we
      // don't mutex on these massive files
    } else {
      vw_out() << "\t--> Using cached L and R files.\n";
    }
  }

  // Xerces-C terminate
  StereoSessionDG::~StereoSessionDG() {
    XMLPlatformUtils::Terminate();
  }

}
