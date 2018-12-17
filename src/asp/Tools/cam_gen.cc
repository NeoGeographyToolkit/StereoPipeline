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

// Create a pinhole camera model based on intrinsics, image corner coordiantes
// and, optionally, a DEM of the area. 

#include <vw/FileIO/DiskImageView.h>
#include <vw/Core/StringUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/PointUtils.h>

#include <limits>
#include <cstring>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace vw::camera;
using namespace std;
using namespace vw::cartography;

struct Options : public vw::cartography::GdalWriteOptions {
  double penalty_weight;
  string image_file, camera_file, output_rpc, stereo_session, bundle_adjust_prefix,
    datum_str, dem_file;
  bool no_crop, skip_computing_rpc, save_tif, has_output_nodata;
  BBox2 lon_lat_range;
  BBox2i image_crop_box;
  Vector2 height_range;
  float output_nodata_value;
  double gsd;
  int num_samples;
  Options(): penalty_weight(-1.0), no_crop(false),
             skip_computing_rpc(false), save_tif(false), has_output_nodata(false),
             output_nodata_value(-std::numeric_limits<float>::max()),
             gsd(-1.0), num_samples(-1) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("datum",            po::value(&opt.datum_str)->default_value(""),
     "Use this datum to interpret the heights. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("lon-lat-range", po::value(&opt.lon_lat_range)->default_value(BBox2i(0,0,0,0), "0 0 0 0"),
     "The longitude-latitude range in which to compute the RPC model. Specify in the format: lon_min lat_min lon_max lat_max.")
    ("height-range", po::value(&opt.height_range)->default_value(Vector2i(0,0),"0 0"),
     "Minimum and maximum heights above the datum in which to compute the RPC model.")
    ("num-samples",     po::value(&opt.num_samples)->default_value(40),
     "How many samples to use in each direction in the longitude-latitude-height range.")
    ("penalty-weight",     po::value(&opt.penalty_weight)->default_value(0.03), // check here!
     "A higher penalty weight will result in smaller higher-order RPC coefficients.")
    ("save-tif-image", po::bool_switch(&opt.save_tif)->default_value(false),
     "Save a TIF version of the input image that approximately corresponds to the input longitude-latitude-height range and which can be used for stereo together with the RPC model.")
    ("output-nodata-value", po::value(&opt.output_nodata_value)->default_value(-std::numeric_limits<float>::max()),
     "Set the image output nodata value.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the input camera model type. Normally this is auto-detected, but may need to be specified if the input camera model is in XML format. Options: pinhole isis rpc dg spot5 aster.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust with this output prefix.")
    ("image-crop-box", po::value(&opt.image_crop_box)->default_value(BBox2i(0,0,0,0), "0 0 0 0"),
     "The output image and RPC model should not exceed this box, specified in input image pixels as minx miny widx widy.")
    ("no-crop", po::bool_switch(&opt.no_crop)->default_value(false),
      "Try to create an RPC model over the entire input image, even if the input longitude-latitude-height box covers just a small portion of it. Not recommended.")
    ("skip-computing-rpc", po::bool_switch(&opt.skip_computing_rpc)->default_value(false),
     "Skip computing the RPC model.")
    ("dem-file",   po::value(&opt.dem_file)->default_value(""),
     "Instead of using a longitude-latitude-height box, sample the surface of this DEM.")
    ("gsd",     po::value(&opt.gsd)->default_value(-1),
     "Expected resolution on the ground, in meters. This is needed for SETSM.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_file))
    ("output-rpc" , po::value(&opt.output_rpc));

  po::positional_options_description positional_desc;
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);
  positional_desc.add("output-rpc", 1);

  string usage("[options] <camera-image> <camera-model> <output-rpc>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
			    positional, positional_desc, usage,
			    allow_unregistered, unregistered);

  opt.has_output_nodata = vm.count("output-nodata-value");
  
  if ( opt.image_file.empty() )
    vw_throw( ArgumentErr() << "Missing input image.\n" << usage << general_options );

  if (boost::iends_with(opt.image_file, ".cub") && opt.stereo_session == "" )
    opt.stereo_session = "isis";

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  // Swap min and max if need be
  if ( opt.lon_lat_range.min().x() > opt.lon_lat_range.max().x() )
    std::swap( opt.lon_lat_range.min().x(), opt.lon_lat_range.max().x() );
  if ( opt.lon_lat_range.min().y() > opt.lon_lat_range.max().y() )
    std::swap( opt.lon_lat_range.min().y(), opt.lon_lat_range.max().y() );
  
  // If we cannot read the data from a DEM, must specify a lot of things.
  if (opt.dem_file.empty()) {
    
    if (opt.datum_str.empty() )
      vw_throw( ArgumentErr() << "Missing input datum.\n" << usage << general_options );

    if (opt.height_range[0] >= opt.height_range[1])
      vw_throw( ArgumentErr() << "Must specify a valid range of heights.\n"
                << usage << general_options );
    
    if (opt.lon_lat_range.empty() )
      vw_throw( ArgumentErr() << "Must specify a valid range of longitude and latitude.\n"
                << usage << general_options );
  }
  
  vw_out() << "Height range is  " << opt.height_range[0] << ' ' << opt.height_range[1] << std::endl;
  vw_out() << "Lon-lat range is " << opt.lon_lat_range.min() << ' ' << opt.lon_lat_range.max()
           << std::endl;

  if (!opt.dem_file.empty()) {
    opt.num_samples *= 5;
    vw_out() << "Since an input DEM was specified, increasing the number of samples "
             << "on its surface to " << opt.num_samples << "^2.\n";
  }

  // Convert from width and height to min and max
  if (!opt.image_crop_box.empty()) {
    BBox2 b = opt.image_crop_box; // make a copy
    opt.image_crop_box = BBox2i(b.min().x(), b.min().y(), b.max().x(), b.max().y());
  }
}

// Pack a pinhole model's rotation and camera center into a vector
void camera_to_vector(PinholeModel const& P, Vector<double> & C){

  Vector3 ctr = P.camera_center();
  Vector3 axis_angle = P.camera_pose().axis_angle();
  C.set_size(6);
  subvector(C, 0, 3) = ctr;
  subvector(C, 3, 3) = axis_angle;
}

// Pack a vector into a pinhole model. We assume the model
// already has set its optical center, focal length, etc. 
void vector_to_camera(PinholeModel & P, Vector<double> const& C){

  if (C.size() != 6) 
    vw_throw( LogicErr() << "Expecting a vector of size 6.\n" );

  Vector3 ctr      = subvector(C, 0, 3);
  Quat    rotation = axis_angle_to_quaternion(subvector(C, 3, 3));
  P.set_camera_center(ctr);
  P.set_camera_pose(rotation);
}

/// Find the camera model that best projects given xyz points into given pixels
class CameraSolveLMA : public vw::math::LeastSquaresModelBase<CameraSolveLMA> {
  std::vector<vw::Vector3> const& m_xyz;
  PinholeModel m_camera_model;
  mutable size_t m_iter_count;
  
public:

  typedef vw::Vector<double>    result_type;   // pixel residuals
  typedef vw::Vector<double, 6> domain_type;   // camera parameters (camera center and axis angle)
  typedef vw::Matrix<double> jacobian_type;

  /// Instantiate the solver with a set of xyz to pixel pairs and a pinhole model
  CameraSolveLMA(std::vector<vw::Vector3> const& xyz,
              PinholeModel const& camera_model):
    m_xyz(xyz),
    m_camera_model(camera_model), m_iter_count(0){
    std::cout << "Init model with " << m_xyz.size() << " points.\n";
  }

  /// Given the camera, project xyz into it
  inline result_type operator()( domain_type const& C ) const {

    // Create the camera model
    PinholeModel camera_model = m_camera_model; // make a copy local to this function
    vector_to_camera(camera_model, C);          // update its parameters
    
    const size_t result_size = m_xyz.size() * 2;
    result_type result;
    result.set_size(result_size);
    for (size_t i = 0; i < m_xyz.size(); i++) {
      Vector2 pixel = camera_model.point_to_pixel(m_xyz[i]);
      result[2*i  ] = pixel[0];
      result[2*i+1] = pixel[1];
    }
  
    ++m_iter_count;
    
    return result;
  }

}; // End class CameraSolveLMA


int main( int argc, char *argv[] ) {
  Options opt;
  try {

    handle_arguments(argc, argv, opt);

  //double v[] = {-122.389,37.6273,-122.354,37.626,-122.358,37.6125,-122.393,37.6138};
  //double v[] = {-122.386,37.626,-122.358,37.6223,-122.361,37.6128,-122.389,37.6165}; // img1
  //double v[] = {-122.387,37.6263,-122.357,37.6231,-122.36,37.6129,-122.39,37.6161};  // img2
  //double v[] = {-122.387,37.6268,-122.356,37.6241,-122.359,37.6131,-122.39,37.6157}; // img3
  //double  v[] = {-122.388,37.627,-122.355,37.625,-122.359,37.6128,-122.392,37.6149}; // img4
  //double v[] = {-122.389,37.6273,-122.354,37.626,-122.358,37.6125,-122.393,37.6138}; // img5
  double v1[] = {-106.101,39.4792,-106.064,39.4809,-106.068,39.4680,-106.105,39.4665}; // v1
  double v2[] = {-106.103,39.4786,-106.061,39.4844,-106.066,39.4683,-106.107,39.4641}; // v2
  double v3[] = {-106.105,39.478,-106.0570,39.4885,-106.064,39.4683,-106.111,39.4596}; // v3
  double v4[] = {-106.107,39.4788,-106.053,39.4941,-106.061,39.4687,-106.114,39.4558}; // v4
  double v5[] = {-106.109,39.4808,-106.048,39.5016,-106.058,39.4689,-106.117,39.4526}; // v5

  double * v;
  std::string num = argv[1];
  std::cout << "--num is " << num << std::endl;
  if (num == "1") v = v1;
  if (num == "2") v = v2;
  if (num == "3") v = v3;
  if (num == "4") v = v4;
  if (num == "5") v = v5;

  vw::cartography::Datum datum("WGS_1984");
  std::cout << "datum is " << datum << std::endl;

  std::cout.precision(18);

  std::string dem_file = "n39w107.hgt";
  DiskImageView<float> dem(dem_file);
  GeoReference geo;
  bool ans = read_georeference(geo, dem_file);
  std::cout << "ans is " << ans << std::endl;
  std::cout << "geo is " << geo << std::endl;

  // Prepare the DEM for interpolation
  ImageViewRef<float> interp_dem
    = interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());

  double m = 2560, n = 1080; // to do: get these from image!

  Vector2 pix;
  double height;

  // change the height to 0
  Vector3 llh(0, 0, 4000), xyz, mean_xyz, xyz1, xyz2, xyz3, xyz4;
  
  llh[0] = v[0]; llh[1] = v[1];
  pix = geo.lonlat_to_pixel(subvector(llh, 0, 2)); height = interp_dem(pix[0], pix[1]); llh[2] = height;
  xyz = datum.geodetic_to_cartesian(llh);
  xyz1 = xyz; // pixel 0, 0
  mean_xyz += xyz;
  std::cout << "llh is      " << llh << std::endl;
  std::cout << "xyz ecef is " << xyz << std::endl;
  std::cout << 1 << ' ' << llh[1] << ' ' << llh[0] << ' ' << llh[2]  << ' ' << 1000 << ' ' << 1000 << ' ' << 1000 << ' ' << "v" << num << ".tif"  << ' ' << 0 << ' ' << 0 << ' ' << 1 << ' ' << 1 << std::endl;
  
  llh[0] = v[2]; llh[1] = v[3];
  pix = geo.lonlat_to_pixel(subvector(llh, 0, 2)); height = interp_dem(pix[0], pix[1]); llh[2] = height;
  xyz = datum.geodetic_to_cartesian(llh);
  mean_xyz += xyz;
  xyz2 = xyz; // pixel m, 0
  std::cout << "xyz is " << xyz << std::endl;
  std::cout << 2 << ' ' << llh[1] << ' ' << llh[0] << ' ' << llh[2]  << ' ' << 1000 << ' ' << 1000 << ' ' << 1000 << ' ' << "v" << num << ".tif"  << ' ' << m << ' ' << 0 << ' ' << 1 << ' ' << 1 << std::endl;
  
  llh[0] = v[4]; llh[1] = v[5];
  pix = geo.lonlat_to_pixel(subvector(llh, 0, 2)); height = interp_dem(pix[0], pix[1]); llh[2] = height;
  xyz = datum.geodetic_to_cartesian(llh);
  mean_xyz += xyz;
  xyz3 = xyz; // pixel m, n
  std::cout << "xyz is " << xyz << std::endl;
  std::cout << 3 << ' ' << llh[1] << ' ' << llh[0] << ' ' << llh[2]  << ' ' << 1000 << ' ' << 1000 << ' ' << 1000 << ' ' << "v" << num << ".tif"  << ' ' << m << ' ' << n << ' ' << 1 << ' ' << 1 << std::endl;
  
  llh[0] = v[6]; llh[1] = v[7];
  pix = geo.lonlat_to_pixel(subvector(llh, 0, 2)); height = interp_dem(pix[0], pix[1]); llh[2] = height;
  xyz = datum.geodetic_to_cartesian(llh);
  mean_xyz += xyz;
  xyz4 = xyz; // pixel 0, n
  std::cout << "xyz is " << xyz << std::endl;
  std::cout << 4 << ' ' << llh[1] << ' ' << llh[0] << ' ' << llh[2]  << ' ' << 1000 << ' ' << 1000 << ' ' << 1000 << ' ' << "v" << num << ".tif"  << ' ' << 0 << ' ' << n << ' ' << 1 << ' ' << 1 << std::endl;

  mean_xyz /= 4;
  std::cout << "mean xyz " << mean_xyz << std::endl;

    
//   xyz = Vector3(-5454.41,-1488.23,3909.04)*1000;
//   std::cout << "norm is " << norm_2(xyz) << std::endl;
//   std::cout << "llh is " << datum.cartesian_to_geodetic(xyz) << std::endl;

//   Vector3 eci = Vector3(-5318.61,-1488.2,4091.32)*1000;
//   std::cout << "ECI norm is " << norm_2(eci) << std::endl;
//   std::cout << "ECI llh is " << datum.cartesian_to_geodetic(eci) << std::endl;

//   Matrix<double,3,3> E; // eci2ecr
//   E(0,0)=0.741342042047181904;
//   E(0,1)=-0.671127392298447534;
//   E(0,2)=0;
  
//   E(1,0)=0.671127392298447534;
//   E(1,1)=0.741342042047181904;
//   E(1,2)=0;
  
//   E(2,0)=0;
//   E(2,1)=0;
//   E(2,2)=1;

//   Vector3 ecr =  E*eci; 
//   std::cout << "eci is " << eci << std::endl;
//   std::cout << "ecr is " << ecr << std::endl;

//   std::cout << "mean_xyz is " << mean_xyz << std::endl;
//   std::cout << "error is " << norm_2(mean_xyz-ecr) << std::endl;

//   Vector3 X = xyz2 - xyz1;
//   Vector3 Y = xyz4 - xyz1;
//   Vector3 Z = mean_xyz - ecr;
  
//   X /= norm_2(X);
//   Y /= norm_2(Y);
//   Z /= norm_2(Z);

//   Vector3 Z2 = cross_prod(X, Y);

//   Vector3 ctr5 = norm_2(eci)*mean_xyz / norm_2(mean_xyz);
//   std::cout << "-ctr5 is " << ctr5 << std::endl;

  
  
//   vw::Matrix<double, 3, 3> in, out;
//   for (int row = 0; row < in.rows(); row++) {
//     for (int col = 0; col < in.cols(); col++) {
//       in(row, col) = ( row == col );
//       if (col == 0) out(row, col) = X[row];
//       if (col == 1) out(row, col) = Y[row];
//       if (col == 2) out(row, col) = Z2[row];
//     }
//   }

//   std::cout << "X = " << X << std::endl;
//   std::cout << "Y = " << Y << std::endl;
//   std::cout << "Z = " << Z << std::endl;
//   std::cout << "--Z2 is " << Z2 << std::endl;
  
//   std::cout << "out is " << out << std::endl;
//   std::cout << "in is " << in << std::endl;
  
//   vw::Matrix3x3 rotation3;
//   vw::Vector3   translation3;
//   double        scale3;
//   asp::find_3D_affine_transform(in, out, rotation3, translation3, scale3);

//   std::cout << "rotation 3 is " << rotation3 << std::endl;
//   std::cout << "translation3 is " << translation3 << std::endl;
//   std::cout << "scale3 is " << scale3 << std::endl;

//   Quaternion<double> q5(rotation3);
//   std::cout << "q5 is " << q5 << std::endl;

//   // Quaternion 0.691192,0.0197885,0.72239,-0.00394691;
//   // is this w, x, y, z or x, y, z, w?
//   Quaternion<double> q(0.0197885,0.72239,-0.00394691,0.691192);

  
  double c_u = 2560/2.0, c_v = 1080/2.0;
  double pixel_pitch = 1.0; 

  // We know that the satellite is at 502.689 km above ground.
  // The swath length is 2.5 km which is 2560 pixels.
  // This implies a focal length in pixels as below.
  // double fu = 502.689 * 2560 / 2.5; // = 514753.536

  // If we divide focal length = 3.6 m by pixel pitch of
  // of 6.5 * 1e-6 m, we get 553846.153846.
  double f_u = 553846.153846;
  double f_v = f_u;

//   PinholeModel P5(ctr5, rotation3, f_u, f_v, c_u, c_v, NULL, pixel_pitch);
//   P5.write("P5.tsai");

//   std::cout << "point to pixel1 " << P5.point_to_pixel(xyz1) << std::endl;
//   std::cout << "point to pixel2 " << P5.point_to_pixel(xyz2) << std::endl;
//   std::cout << "point to pixel3 " << P5.point_to_pixel(xyz3) << std::endl;
//   std::cout << "point to pixel4 " << P5.point_to_pixel(xyz4) << std::endl;

  Vector3 ctr0(0, 0, 0);
  Matrix<double, 3, 3> rotation0;
  for (int row = 0; row < rotation0.rows(); row++) {
    for (int col = 0; col < rotation0.cols(); col++) {
      rotation0(row, col) = ( row == col );
    }
  }
  PinholeModel cam(ctr0, rotation0, f_u, f_v, c_u, c_v, NULL, pixel_pitch);

  Vector3 u_vec, v_vec, w_vec;
  cam.coordinate_frame(u_vec, v_vec, w_vec);
  cam.set_coordinate_frame(u_vec, v_vec, w_vec);
  
//   std::cout << "cam is " << cam << std::endl;
  double ht = 500000; 
  Vector3 a0 = cam.camera_center(Vector2(0, 0)) + ht*cam.pixel_to_vector(Vector2(0, 0));
  Vector3 a1 = cam.camera_center(Vector2(m, 0)) + ht*cam.pixel_to_vector(Vector2(m, 0));
  Vector3 a2 = cam.camera_center(Vector2(m, n)) + ht*cam.pixel_to_vector(Vector2(m, n));
  Vector3 a3 = cam.camera_center(Vector2(0, n)) + ht*cam.pixel_to_vector(Vector2(0, n));
//   std::cout << "centers are " << cam.camera_center(Vector2(0, 0)) << std::endl;
//   std::cout << "centers are " << cam.camera_center(Vector2(m, n)) << std::endl;
//   std::cout << "vector0 " <<  ht*cam.pixel_to_vector(Vector2(0, 0)) << std::endl;
//   std::cout << "vector3 " <<  ht*cam.pixel_to_vector(Vector2(m, n)) << std::endl;

  vw::Matrix<double, 3, 4> in0, out0;
  for (int row = 0; row < in0.rows(); row++) {
    for (int col = 0; col < in0.cols(); col++) {
      if (col == 0) in0(row, col) = a0[row];
      if (col == 1) in0(row, col) = a1[row];
      if (col == 2) in0(row, col) = a2[row];
      if (col == 3) in0(row, col) = a3[row];
      if (col == 0) out0(row, col) = xyz1[row];
      if (col == 1) out0(row, col) = xyz2[row];
      if (col == 2) out0(row, col) = xyz3[row];
      if (col == 3) out0(row, col) = xyz4[row];
    }
  }
//   std::cout << "apoint to pixel1 " << cam.point_to_pixel(a0) << std::endl;
//   std::cout << "apoint to pixel2 " << cam.point_to_pixel(a1) << std::endl;
//   std::cout << "apoint to pixel3 " << cam.point_to_pixel(a2) << std::endl;
//   std::cout << "apoint to pixel4 " << cam.point_to_pixel(a3) << std::endl;


//   std::cout << "in0=\n" << in0 << std::endl;
//   std::cout << "out0=\n" << out0 << std::endl;

//   std::cout << "diag0 " << norm_2(a2-a0) << std::endl;
//   std::cout << "diag 1 " << norm_2(xyz3-xyz1) << std::endl;

//   std::cout << "length " << norm_2(a1-a0) << ' ' << norm_2(xyz2-xyz1) << std::endl;
//   std::cout << "width " << norm_2(a2-a1) << ' ' << norm_2(xyz3-xyz2) << std::endl;

  vw::Matrix3x3 rotation1;
  vw::Vector3   translation1;
  double        scale1;
  asp::find_3D_affine_transform(in0, out0, rotation1, translation1, scale1);
  cam.apply_transform(rotation1, translation1, scale1);
  std::cout << "--scale1 is " << scale1 << std::endl;
  std::cout << "rotation is " << rotation1 << std::endl;
  std::cout << "translation is " << translation1 << std::endl;
  std::cout << "determinant is " << det(rotation1) << std::endl;
  
  std::cout << "trans cam is " << cam << std::endl;
  std::cout << "0point to pixel1 " << cam.point_to_pixel(xyz1) << std::endl;
  std::cout << "0point to pixel2 " << cam.point_to_pixel(xyz2) << std::endl;
  std::cout << "0point to pixel3 " << cam.point_to_pixel(xyz3) << std::endl;
  std::cout << "0point to pixel4 " << cam.point_to_pixel(xyz4) << std::endl;

  std::cout << "ans0 " << norm_2(scale1*rotation1*a0 + translation1  - xyz1) << std::endl;
  std::cout << "ans1 " << norm_2(scale1*rotation1*a1 + translation1  - xyz2) << std::endl;
  std::cout << "ans2 " << norm_2(scale1*rotation1*a2 + translation1  - xyz3) << std::endl;
  std::cout << "ans3 " << norm_2(scale1*rotation1*a3 + translation1  - xyz4) << std::endl;

  llh = datum.cartesian_to_geodetic(cam.camera_center());
  std::cout << "camera center llh is " << llh << std::endl;
  
  std::vector<vw::Vector3> xyz_vec;
  xyz_vec.push_back(xyz1);
  xyz_vec.push_back(xyz2);
  xyz_vec.push_back(xyz3);
  xyz_vec.push_back(xyz4);
  
  Vector<double, 8> pixel_vec;
  pixel_vec[0]=0; pixel_vec[1]=0;
  pixel_vec[2]=m; pixel_vec[3]=0;
  pixel_vec[4]=m; pixel_vec[5]=n;
  pixel_vec[6]=0; pixel_vec[7]=n;
  
  // Set up the optimizer model
  CameraSolveLMA lma_model(xyz_vec, cam);

  Vector<double> seed;
  camera_to_vector(cam, seed);
  std::cout << "--seed size is " << seed.size() << std::endl;
  
  // Use the L-M solver to optimize the camera position.
  const double abs_tolerance  = 1e-24;
  const double rel_tolerance  = 1e-24;
  const int    max_iterations = 2000;

  // Run the optimization
  int status = 0;
  Vector<double> final_params;
  final_params = math::levenberg_marquardt(lma_model, seed, pixel_vec,
                                           status, abs_tolerance, rel_tolerance,
                                           max_iterations);
  
  if (status < 1) { // This means the solver failed to converge!
    VW_OUT(DebugMessage, "asp") << "WARNING: Levenberg-Marquardt solver status = " 
                                << status << std::endl;
  }
  
  vector_to_camera(cam, final_params);
  
  std::cout << "1point to pixel1 " << cam.point_to_pixel(xyz1) << std::endl;
  std::cout << "1point to pixel2 " << cam.point_to_pixel(xyz2) << std::endl;
  std::cout << "1point to pixel3 " << cam.point_to_pixel(xyz3) << std::endl;
  std::cout << "1point to pixel4 " << cam.point_to_pixel(xyz4) << std::endl;

  llh = datum.cartesian_to_geodetic(cam.camera_center());
  std::cout << "camera center llh2 is " << llh << std::endl;

  std::string out_file = "v" + num + "_rot9.tsai";
  std::cout << "Writing: " << out_file << std::endl;
  cam.write(out_file);
  
  
//   std::cout << "q is " << q << std::endl;
//   Vector3 camera_center = ecr;
//   Matrix<double,3,3> rotation = q.rotation_matrix();

//   std::cout << "E is " << E << std::endl;
  
//   std::cout << "q with rotation applied " << Quaternion<double>(E*rotation) << std::endl;
  
//   PinholeModel P1(camera_center, E*rotation, f_u, f_v, c_u, c_v, NULL, pixel_pitch);
//   P1.write("P1.tsai");
  
//   PinholeModel P2(camera_center, inverse(E*rotation), f_u, f_v, c_u, c_v, NULL, pixel_pitch);
//   P2.write("P2.tsai");

//   Quaternion<double> q2(0.691192, 0.0197885, 0.72239, -0.00394691);
//   Matrix<double,3,3> rotation2 = q2.rotation_matrix();

//   PinholeModel P3(camera_center, E*rotation2, f_u, f_v, c_u, c_v, NULL, pixel_pitch);
//   P1.write("P3.tsai");
  
//   PinholeModel P4(camera_center, inverse(E*rotation2), f_u, f_v, c_u, c_v, NULL, pixel_pitch);
//   P2.write("P4.tsai");

//   PinholeModel P8;
//   P8.read("ba/run-P8.tsai");
//   std::cout << "center is " << P8.camera_center() << std::endl;
//   std::cout << "pose1 is " << P8.camera_pose() << std::endl;
//   std::cout << "pose1 inv is " << inverse(P8.camera_pose()) << std::endl;

//   std::cout << "8point to pixel1 " << P8.point_to_pixel(xyz1) << std::endl;
//   std::cout << "8point to pixel2 " << P8.point_to_pixel(xyz2) << std::endl;
//   std::cout << "8point to pixel3 " << P8.point_to_pixel(xyz3) << std::endl;
//   std::cout << "8point to pixel4 " << P8.point_to_pixel(xyz4) << std::endl;
  
//   rotation = P8.camera_pose().rotation_matrix();
  
//   std::cout << "q1 " << Quaternion<double>(E*rotation) << std::endl;
//   std::cout << "q2 " << Quaternion<double>(inverse(E)*rotation) << std::endl;
//   std::cout << "q3 " << Quaternion<double>(inverse(E)*inverse(rotation)) << std::endl;
//   std::cout << "q4 " << Quaternion<double>(E*inverse(rotation)) << std::endl;

//   std::cout << "q5 " << inverse(Quaternion<double>(E*rotation)) << std::endl;
//   std::cout << "q6 " << inverse(Quaternion<double>(inverse(E)*rotation)) << std::endl;
//   std::cout << "q7 " << inverse(Quaternion<double>(inverse(E)*inverse(rotation))) << std::endl;
//   std::cout << "q8 " << inverse(Quaternion<double>(E*inverse(rotation))) << std::endl;

//   //   std::cout << "q5 " << Quaternion<double>(E*rotation2) << std::endl;
// //   std::cout << "q6 " << Quaternion<double>(inverse(E)*rotation2) << std::endl;
// //   std::cout << "q7 " << Quaternion<double>(inverse(E)*inverse(rotation2)) << std::endl;
// //   std::cout << "q8 " << Quaternion<double>(E*inverse(rotation2)) << std::endl;

//   std::cout << "orig position: " << P8.camera_center() << std::endl;
//   std::cout << "orig quaternion: " << P8.camera_pose() << std::endl;
  
//   Matrix<double,3,3> invE = inverse(E);
//   Vector3 translation(0, 0, 0);
//   double scale = 1.0;
//   P8.apply_transform(invE, translation, scale);

//   std::cout << "transformed position: " << P8.camera_center() << std::endl;
//   std::cout << "transformed quaternion: " << P8.camera_pose() << std::endl;
  
  
    if ( opt.camera_file.empty() )
      vw_throw( ArgumentErr() << "Missing input camera.\n" );

    if ( opt.output_rpc.empty() )
      vw_throw( ArgumentErr() << "Missing output RPC file.\n" );

    // Create the output directory
    vw::create_out_dir(opt.output_rpc);
    
    boost::shared_ptr<CameraModel> cam = session->camera_model(opt.image_file, opt.camera_file);

    // The input nodata value
    float input_nodata_value = -std::numeric_limits<float>::max(); 
    vw::read_nodata_val(opt.image_file, input_nodata_value);

    // If the output nodata value was not specified, use the input one
    if (!opt.has_output_nodata) {
      opt.output_nodata_value = input_nodata_value;
      opt.has_output_nodata = true;
    }

    // Read the image as float and mask it right away
    ImageViewRef< PixelMask<float> > input_img
      = create_mask(DiskImageView<float>(opt.image_file), input_nodata_value);

  } ASP_STANDARD_CATCHES;

  return 0;
}
