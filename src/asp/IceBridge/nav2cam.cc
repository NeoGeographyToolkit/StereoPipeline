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


/// \file nav2cam.cc
///

// Start with an IceBridge nav file, from https://nsidc.org/data/IPAPP1B/versions/1,
// for example, sbet_20111012.out. Convert it to text using the Perl reader
// from http://nsidc.org/data/docs/daac/icebridge/ipapp1b/index.html
// The obtained text file, with a name like sbet_20111012.txt, will have
// the format:
//
// GPS seconds since start of the week, latitude (degrees), longitude  (degrees),
// Altitude (meters, above WGS84), x velocity, y velocity, z velocity,
// roll (radians), pitch (radians), heading (radians), etc. These
// are relative to the camera center, with the camera looking down.
// 
// In addition to that nav file in text format, take an input a raw
// image, for example, 2011_10_12_09765.JPG, the corresponding L1B
// orthoimage, DMS_1281706_09765_20111012_18060740.tif.  in the format
// DMS_fffffff_FFFFF_YYYYMMDD_HHmmsshh.tif, where hh is hundreds of a
// second, and the time is GPS time (not UTC!).
//
// Convert this time stamp into GPS seconds, and look-up and
// interpolate the camera position/orientation in the sbet nav
// file. Combine with the camera intrinsics tsai file passed on input,
// and write down the camera intrinsics + extrinsics tsai file.

#include <asp/Core/Macros.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <ctime>

// Turn off warnings from eigen
#if defined(__GNUC__) || defined(__GNUG__)
#define LOCAL_GCC_VERSION (__GNUC__ * 10000                    \
                           + __GNUC_MINOR__ * 100              \
                           + __GNUC_PATCHLEVEL__)
#if LOCAL_GCC_VERSION >= 40600
#pragma GCC diagnostic push
#endif
#if LOCAL_GCC_VERSION >= 40202
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif
#endif

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#if defined(__GNUC__) || defined(__GNUG__)
#if LOCAL_GCC_VERSION >= 40600
#pragma GCC diagnostic pop
#endif
#undef LOCAL_GCC_VERSION
#endif


namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

// From DMS_1281706_09765_20111012_18060740.tif
// extract the date 2011/10/12 and time 18:06:07.40.
// Then find how many seconds elapsed since the beginning of the week.
double gps_seconds(std::string const& orthoimage){

  std::string label = "DMS";
  std::size_t it = orthoimage.find(label);
  if (it == std::string::npos) 
    vw_throw( ArgumentErr() << "Could not find the text " << label << " in " << orthoimage << "\n" );

  it += 18;

  std::string date = orthoimage.substr(it, 8);
  std::string time = orthoimage.substr(it+9, 8);

  int year  = atof(date.substr(0, 4).c_str());
  int month = atof(date.substr(4, 2).c_str());
  int day   = atof(date.substr(6, 2).c_str());

  int hour  = atof(time.substr(0, 2).c_str());
  int min   = atof(time.substr(2, 2).c_str());
  int sec   = atof(time.substr(4, 2).c_str());
  int fsec  = atof(time.substr(6, 2).c_str()); // first two digit of fractional part of second

  std::tm time_in = {sec, min, hour, // second, minute, hour
                     day,            // 1-based day
                     month-1,        // 0-based month
                     year - 1900};     // year since 1900

  std::time_t time_val = std::mktime( & time_in );

  // the return value from localtime is a static global - do not call
  // this function from more than one thread!
  std::tm const *time_out = std::localtime(&time_val);

  int weekday = time_out->tm_wday; // Sunday is 0

  double seconds = weekday*24*3600.0 + hour*3600.0 + min*60 + sec + fsec/100.0;

  return seconds; 
}

void scan_line(std::string const& line,
               double& seconds, double& lat, double& lon, double& alt,
               double& roll, double& pitch, double& heading){
  double xv, yv, zv;
  if (sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
             &seconds, &lat, &lon, &alt, &xv, &yv, &zv, &roll, &pitch, &heading) != 10) 
    vw_throw( ArgumentErr() << "Could not scan 10 values from line: " << line << "\n" );
}

// TODO: It is not clear what the right order should be! 
Matrix3x3 get_look_rotation_matrix(double yaw, double pitch, double roll, int index) {

  // These calculations are copied from the SPOT 123-4-58 Geometry Handbook (GAEL-P135-DOC-001)
  Matrix3x3 Mp, Mr, My;
  Mp(0,0) = 1.0;         Mp(0,1) = 0.0;           Mp(0,2) = 0.0;
  Mp(1,0) = 0.0;         Mp(1,1) = cos(pitch);    Mp(1,2) = sin(pitch);
  Mp(2,0) = 0.0;         Mp(2,1) = -sin(pitch);   Mp(2,2) = cos(pitch); 
  
  Mr(0,0) = cos(roll);   Mr(0,1) = 0.0;           Mr(0,2) = -sin(roll);
  Mr(1,0) = 0.0;         Mr(1,1) = 1.0;           Mr(1,2) = 0.0;
  Mr(2,0) = sin(roll);   Mr(2,1) = 0.0;           Mr(2,2) = cos(roll);
  
  My(0,0) = cos(yaw);    My(0,1) = -sin(yaw);     My(0,2) = 0.0;
  My(1,0) = sin(yaw);    My(1,1) = cos(yaw);      My(1,2) = 0.0;
  My(2,0) = 0.0;         My(2,1) = 0.0;           My(2,2) = 1.0; 

  if (index == 1) return Mp*Mr*My;
  if (index == 2) return Mp*My*Mr;
  if (index == 3) return My*Mp*Mr;
  if (index == 4) return My*Mr*Mp;
  if (index == 5) return Mr*Mp*My;
  if (index == 6) return Mr*My*Mp;
  
  Matrix3x3 out = Mp*Mr*My;
  return out;
  
  
  /* Optimized version
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);
  double cy = cos(yaw);
  double sy = sin(yaw);

  Matrix3x3 M;
  M(0,0) = (cr*cy);            M(0,1) = (-cr*sy);           M(0,2) = (-sr);
  M(1,0) = (cp*sy+sp*sr*cy);   M(1,1) = (cp*cy-sp*sr*sy);   M(1,2) = (sp*cr);
  M(2,0) = (-sp*sy+cp*sr*cy);  M(2,1) = (-sp*cy-cp*sr*sy);  M(2,2) = cp*cr; 
  return M;
  */
}


void parse_camera_pose(std::string const& line, Vector3 & xyz, Quat & rot, Quat & look, Quat & ned,
                       double & lon, double & lat, double & alt,
                       double & roll, double & pitch, double & heading,
                       int index){

  double seconds;
  //alt, roll, pitch, heading;
  scan_line(line, seconds, lat, lon, alt, roll, pitch, heading);
  
  Datum D("WGS84");
  xyz = D.geodetic_to_cartesian(Vector3(lon, lat, alt));

  //std::cout << "--xyz is " << xyz << std::endl;
  ned = Quat(D.lonlat_to_ned_matrix(Vector2(lon, lat)));

  //std::cout << "ned matrix " << Quat(ned) << std::endl;


    //std::cout << "---fix here!" << std::endl;
  look = Quat(get_look_rotation_matrix(-heading, -pitch, -roll, index));

    //std::cout << "--ned " << Quat(ned) << std::endl;
    //std::cout << "--look is " << Quat(look) << std::endl;
    //std::cout << "--nedlook is " << Quat(ned*look) << std::endl;
    //std::cout << "--inv_look *ned is " << inverse(Quat(look))*Quat(ned) << std::endl;
    
    //Matrix3x3 rot_mat = look*ned;
    
    //std::cout << "rot mat is " << rot_mat << std::endl;
    
    //Matrix3x3 orig((0.180419,-0.939935,-0.289778),(0.981781,0.154235,0.110988),(-0.0596276,-0.304523,0.950637));
    
    //std::cout << "--quat1 " << Quat(look*ned) << std::endl;
    //std::cout << "--quat " << Quat(ned*look) << std::endl;
    //std::cout << "--quat3 " << Quat(inverse(ned)*look) << std::endl;
    //std::cout << "--quat3 " << Quat(inverse(look)*ned) << std::endl;
    //std::cout << "--quat3 " << Quat(ned*inverse(look)) << std::endl;
    //std::cout << "--quat3 " << Quat(look*inverse(ned)) << std::endl;
  
}

            
struct Options : public vw::cartography::GdalWriteOptions {
  std::string nav_file, input_cam;
  std::vector<std::string> image_files;
  
#if 0
    , orthoimage, input_cam, output_cam, reference_dem;
  double camera_height, orthoimage_height;
  int ip_per_tile;
  int  ip_detect_method;
  bool individually_normalize;

  // Make sure all values are initialized, even though they will be
  // over-written later.
  Options(): camera_height(-1), orthoimage_height(0), ip_per_tile(0),
             ip_detect_method(0), individually_normalize(false){}
#endif
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("input-cam",             po::value(&opt.input_cam)->default_value(""), "The input camera file from where to read the intrinsics.")
    ("nav-file",             po::value(&opt.nav_file)->default_value(""), "The nav file, in text format.");
#if 0
    ("camera-height",   po::value(&opt.camera_height)->default_value(-1.0),
     "The approximate height above the datum, in meters, at which the camera should be. If not specified, it will be read from the orthoimage metadata.")
    ("orthoimage-height",   po::value(&opt.orthoimage_height)->default_value(0.0),
     "The approximate height above the datum, in meters, at which the orthoimage is positioned. We assume flat ground.")
    ("ip-per-tile",             po::value(&opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic determination).")
    ("ip-detect-method",po::value(&opt.ip_detect_method)->default_value(1),
     "Interest point detection algorithm (0: Integral OBALoG (default), 1: OpenCV SIFT, 2: OpenCV ORB.")
    ("individually-normalize",   po::bool_switch(&opt.individually_normalize)->default_value(false)->implicit_value(true),
     "Individually normalize the input images instead of using common values.")
     "If provided, extract from this DEM the heights above the ground rather than assuming the value in --orthoimage-height.");
#endif
  
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );
  
  po::options_description positional("");
  positional.add_options()
    ("image-files", po::value(&opt.image_files));
  
#if 0
  positional.add_options()
    ("nav-file",  po::value(&opt.nav_file))
    ("orthoimage", po::value(&opt.orthoimage))
    ("camera", po::value(&opt.camera));
    ("input-cam",  po::value(&opt.input_cam))
    ("output-cam", po::value(&opt.output_cam));
#endif
  
  po::positional_options_description positional_desc;
  positional_desc.add("image-files", -1);

#if 0
  po::positional_options_description positional_desc;
  positional_desc.add("nav-file",  1);
  positional_desc.add("orthoimage", 1);
  positional_desc.add("camera", 1);
  positional_desc.add("input-cam",  1);
  positional_desc.add("output-cam", 1);
#endif
  
  std::string usage("<nav file> <ortho image> <input pinhole cam> <output pinhole cam> [options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  
  if ( opt.nav_file.empty() )
    vw_throw( ArgumentErr() << "Missing input nav file.\n" << usage << general_options );

  if ( opt.input_cam.empty() )
    vw_throw( ArgumentErr() << "Missing input pinhole camera.\n" << usage << general_options );

#if 0
  if ( opt.orthoimage.empty() )
    vw_throw( ArgumentErr() << "Missing input ortho image.\n" << usage << general_options );


  if ( opt.output_cam.empty() )
    vw_throw( ArgumentErr() << "Missing output pinhole camera.\n" << usage << general_options );
  
  // Create the output directory
  vw::create_out_dir(opt.output_cam);
#endif
}

// ================================================================================

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    //std::cout << "--nav file is " << opt.nav_file << std::endl;

    PinholeModel Q;
    Q.read(opt.input_cam);

    double big  = 1000000;
    double lon0 = big;
    Datum D("WGS84");
    
    std::ifstream ifs(opt.nav_file.c_str());
    
    for (size_t i = 0; i < opt.image_files.size(); i++) {

      std::string orthoimage = opt.image_files[i];

      //std::cout << "--ortho image is " << orthoimage << std::endl;

      std::string camera = orthoimage;
      camera = fs::path(camera).replace_extension("tsai").string();

      std::string camera_out = orthoimage;
      camera_out = fs::path(camera_out).replace_extension("out.tsai").string();

      //std::cout << "--camera " << camera_out << std::endl;
      
      PinholeModel P;
      P.read(camera);
      
      // Find how many seconds elapsed since the beginning of the week
      double curr_seconds = gps_seconds(orthoimage);
      
      std::cout.precision(18);
      //std::cout << "--seconds " << curr_seconds << std::endl;
      
      // Now locate where we are in the huge nav file (it is too big to open it in memory)
      std::string prev_line, curr_line;
      double seconds, lat, lon, alt, roll, pitch, heading;
      double prev_seconds = -1; 
      int attempts = 0;
      while (getline(ifs, curr_line)){
        
        scan_line(curr_line, seconds, lat, lon, alt, roll, pitch, heading);
        
        //         std::cout << "--prev line " << prev_line << std::endl;
        //         std::cout << "--curr_line " << curr_line << std::endl;
        //         std::cout << "--curr_seconds " << curr_seconds << std::endl;
        //         std::cout << "--parsed " << seconds << ' ' << lat << ' ' << lon << ' ' << alt << ' ' << roll << ' ' << pitch << ' ' << heading << std::endl;
//         break;
        
        if (seconds >= curr_seconds) {
          
          // Found where we should be.
          
          if (prev_seconds == -1 && seconds > curr_seconds && attempts == 0) {
            // We just started, and already overshot.
            // Need to rewind. This code is not tested!
            attempts++;
            
            vw_out() << "Rewinding to the beginning of: " << opt.nav_file << ".\n";
            ifs.clear();
            ifs.seekg(0);
            continue;
          }
          
          //std::cout << "--prev line " << prev_line << std::endl;
          //std::cout << "--curr_line " << curr_line << std::endl;
          //std::cout << "--curr_seconds " << curr_seconds << std::endl;
          //std::cout << "--parsed " << seconds << ' ' << lat << ' ' << lon << ' ' << alt << ' ' << roll << ' ' << pitch << ' ' << heading << std::endl;
          break;
        }
        
        prev_line    = curr_line;
        prev_seconds = seconds;
      }
      
      if (prev_seconds == -1 || seconds < curr_seconds)
        vw_throw( ArgumentErr() << "Could not find the time stamp in seconds " << curr_seconds << " in file: " << opt.nav_file << "\n" );
      
      
      Vector3 xyz;
      Quat rot, look, ned;
      
      for (int index = 5; index <= 5; index++) {
        parse_camera_pose(curr_line, xyz, rot, look, ned, lon, lat, alt, roll, pitch, heading, index);

        // This is highly confusing, but apparently we need to keep
        // the original longitude when computing the ned matrix that
        // transforms from the coordinate system with orgin at the
        // center of the Earth to the North-East-Down coordinate
        // system that was obtained from the starting image, rather
        // than modifying it as we move over new points over the Earth
        // surface.
        if (lon0 == big){
          // First value for these variables.
          lon0 = lon;
        }
        std::cout << "--temporary!!!" << std::endl;
        lon0 = -65;
        ned = Quat(D.lonlat_to_ned_matrix(Vector2(lon0, lat)));

        Quat diff = inverse(P.camera_pose())*inverse(ned)*inverse(look);
        if (diff[0] < 0) diff = -diff;
          
        std::cout << "inv_p * inv_ned * inv_look index " << index << " "
                  << orthoimage << " " 
                  << diff << ' ' << norm_2(diff - Quat(1, 0, 0, 0)) << ' ' 
                  << alt << ' ' << roll << ' ' << pitch << ' ' << heading << std::endl;

#if 0
        std::cout << "inv_p2 * inv_ned * inv_look index " << index << " "
                  << orthoimage << " " 
                  << inverse(P.camera_pose())*inverse(look)*inverse(ned)
                  << std::endl;

        std::cout << "inv_p3 * inv_ned * inv_look index " << index << " "
                  << orthoimage << " " 
                  << inverse(look)*inverse(P.camera_pose())*inverse(ned)
                  << std::endl;

        std::cout << "inv_p4 * inv_ned * inv_look index " << index << " "
                  << orthoimage << " " 
                  << inverse(look)*inverse(ned)*inverse(P.camera_pose())
                  << std::endl;

        std::cout << "inv_p5 * inv_ned * inv_look index " << index << " "
                  << orthoimage << " " 
                  << inverse(ned)*inverse(look)*inverse(P.camera_pose())
                  << std::endl;

        std::cout << "inv_p6 * inv_ned * inv_look index " << index << " "
                  << orthoimage << " " 
                  << inverse(ned)*inverse(P.camera_pose())*inverse(look)
                  << std::endl;
#endif
        
        Q.set_camera_center(xyz);
        Q.set_camera_pose(inverse(ned)*inverse(look));
        vw_out() << "Writing: " << camera_out << std::endl;
        Q.write(camera_out);
        
      }
    }
    
    return 0;
#if 0
    
//     Quat q1(0.145877240812620657,-0.152860522745347588,0.0446281066841296148,0.976402490417095148);
//     Quat q2(0.955898256513265432,0.1116164494592609,0.153534813290112621,0.224114596831571866);
//     Quat q3(0.755859,-0.13743,-0.0761222,0.635607);
//     Quat q4(0.696057,-0.0272992,-0.192871,-0.691057);

    //Quat p1(0.756247,-0.137381,-0.0762064,0.635145);
    //Quat p2(-0.115443,-0.127551,0.0725423,0.982416);
    //Quat p3(0.696057,-0.0272992,-0.192871,-0.691057);

    Quat p1(0.756247,-0.137381,-0.0762064,0.635145);
    Quat p2(-0.115443,-0.127551,0.0725423,0.982416);
    Quat p3(0.696057,-0.0272992,-0.192871,-0.691057);

    Quat look1(-0.320883494733396446,0.00792666922468665547,0.0354974906797180106,0.946420033007611239);
    Quat look2(0.614615631887660996,-0.0119952417800386936,0.0174059918283487151,0.788543448810806513);
    Quat look3(0.973914145234334616,-0.0127410037246634914,-0.00258822025729765629,0.226544047929896658);

    //Quat q1(-0.135931876658998196,0.122964798977559353,-0.109658191144168513,-0.976922342991171644);
    //Quat q2(0.753003560573910602,0.139526553754940053,0.0461585133512844928,-0.641394862939735355);
    //Quat q3(0.952786485993646459,0.136258577000613501,0.146959041977113319,0.228110833330676804);

    Quat q1(0.486006654155486828,0.164297902347748881,-0.0123069068710277771,-0.85828449330276646);
    Quat q2(-0.43808556751133787,0.0788344960980872644,-0.124029793930698559,-0.886838636946281289);
    Quat q3(0.754376451992149755,0.0574351653291488434,0.192001462517024601,0.625102238721533143);

    Quat ned1(0.971115061057905571,0.0285695556686058998,0.154482352453917116,0.179595438293132764);

    Quat ned2(0.969445696508651089,0.0291606341778107084,0.143805581113631648,0.196582435064211386);

    Quat ned3(0.878371517944454339,0.0866816020544148025,0.17443439904365754,0.436488736119860443);

    std::cout << "diff1 " << (p1*inverse(ned1))*look1 << std::endl;
    std::cout << "diff2 " << (p2*inverse(ned2))*look2 << std::endl;
    std::cout << "diff3 " << (p3*inverse(ned3))*look3 << std::endl;
    std::cout << "diffx1 " << norm_2( (p1*inverse(ned1))*look1 - (p2*inverse(ned2))*look2) << std::endl;
    std::cout << std::endl;
    std::cout << "diff1 " << (p1*inverse(ned1))*inverse(look1) << std::endl;
    std::cout << "diff2 " << (p2*inverse(ned2))*inverse(look2) << std::endl;
    std::cout << "diff3 " << (p3*inverse(ned3))*inverse(look3) << std::endl;
    std::cout << "difx2 " << norm_2( (p1*inverse(ned1))*inverse(look1) - (p2*inverse(ned2))*inverse(look2) ) << std::endl;
    
    std::cout << std::endl;
    std::cout << "diff1 " << (p1*(ned1))*look1 << std::endl;
    std::cout << "diff2 " << (p2*(ned2))*look2 << std::endl;
    std::cout << "diff3 " << (p3*(ned3))*look3 << std::endl;
    std::cout << "difx " << norm_2( (p1*(ned1))*look1 -  (p2*(ned2))*look2  )<< std::endl;
    std::cout << std::endl;
    std::cout << "diff1 " << (p1*(ned1))*inverse(look1) << std::endl;
    std::cout << "diff2 " << (p2*(ned2))*inverse(look2) << std::endl;
    std::cout << "diff3 " << (p3*(ned3))*inverse(look3) << std::endl;
    std::cout << "difx " << norm_2((p1*(ned1))*look1 - (p2*(ned2))*look2) << std::endl;
    std::cout << std::endl;
    
    std::cout << "diff1 " << (inverse(p1)*inverse(ned1))*look1 << std::endl;
    std::cout << "diff2 " << (inverse(p2)*inverse(ned2))*look2 << std::endl;
    std::cout << "diff3 " << (inverse(p3)*inverse(ned3))*look3 << std::endl;
    std::cout << "difx " << norm_2( (inverse(p1)*inverse(ned1))*look1 -  (inverse(p2)*inverse(ned2))*look2) << std::endl;
    std::cout << std::endl;
    
    std::cout << "diff1 " << (inverse(p1)*inverse(ned1))*inverse(look1) << std::endl;
    std::cout << "diff2 " << (inverse(p2)*inverse(ned2))*inverse(look2) << std::endl;
    std::cout << "diff3 " << (inverse(p3)*inverse(ned3))*inverse(look3) << std::endl;
    std::cout << "difx " << norm_2( (inverse(p1)*inverse(ned1))*inverse(look1) - (inverse(p2)*inverse(ned2))*inverse(look2)) << std::endl;
    std::cout << std::endl;
    
    std::cout << "diff1 " << (inverse(p1)*(ned1))*look1 << std::endl;
    std::cout << "diff2 " << (inverse(p2)*(ned2))*look2 << std::endl;
    std::cout << "diff3 " << (inverse(p3)*(ned3))*look3 << std::endl;
    std::cout << "difx " << norm_2((inverse(p1)*(ned1))*look1 - (inverse(p2)*(ned2))*look2) << std::endl;
    std::cout << std::endl;
    std::cout << "diff1 " << (inverse(p1)*(ned1))*inverse(look1) << std::endl;
    std::cout << "diff2 " << (inverse(p2)*(ned2))*inverse(look2) << std::endl;
    std::cout << "diff3 " << (inverse(p3)*(ned3))*inverse(look3) << std::endl;
    std::cout << "difx " << norm_2((inverse(p1)*(ned1))*inverse(look1) - (inverse(p2)*(ned2))*inverse(look2)) << std::endl;
    std::cout << std::endl;

    //std::cout << "diff1 " << inverse(q1)*p1 << std::endl;
    //std::cout << "diff2 " << inverse(q2)*p2 << std::endl;
    //std::cout << "diff3 " << inverse(q3)*p3 << std::endl;
    
    //Quat q1(0.320617120012335721,-0.0152876300423172486,-0.0354974906797180106,0.94642003300761135);
    //Quat q2(-0.614574379576642627,0.0139496506771764374,-0.0174059918283487151,0.788543448810806513);
    
    //Quat q3(0.756247,-0.137381,-0.0762064,0.635145);
    //Quat q4(-0.115443,-0.127551,0.0725423,0.982416);
    
    //std::cout << "val1 " << q1*inverse(q2) << std::endl;
    //std::cout << "val2 " << inverse(q2)*q1 << std::endl;
    //std::cout << "val3 " << q3*inverse(q4) << std::endl;
    //std::cout << "val4 " << inverse(q4)*q3 << std::endl;
#endif
    
  } ASP_STANDARD_CATCHES;
}

//324367.399674238  -71.9220810155697  -20.9611504030847  414.989719506697  0.00357475554033786  0.0718573372998217  2.49070238721921
//324367.400000000023
//324367.404674378  -71.9220822024238  -20.9611363143516  414.989579265466  0.00355075687109858  0.0717698496654862  2.4906864299986 

//09765
//quat1 [Q](0.145877240812620657,-0.152860522745347588,0.0446281066841296148,0.976402490417095148)

// 20976
//quat1 [Q](0.955898256513265432,0.1116164494592609,0.153534813290112621,0.224114596831571866)

  
// -camera pose [Q](0.755859,-0.13743,-0.0761222,0.635607)   09765
//--camera pose [Q](0.696057,-0.0272992,-0.192871,-0.691057) 20976

//--look is [Q](0.320617120012335721,-0.0152876300423172486,-0.0354974906797180106,0.94642003300761135)
//--look is [Q](-0.614574379576642627,0.0139496506771764374,-0.0174059918283487151,0.788543448810806513)


//--camera pose [Q](0.756247,-0.137381,-0.0762064,0.635145)
//--camera pose [Q](-0.115443,-0.127551,0.0725423,0.982416)

//--nedlook is [Q](0.136705573998491842,0.134605806783797632,0.0517923457801958836,0.980056332940311359)
//  --nedlook is [Q](0.753507407418762876,-0.105881353731662603,0.0932348742688930143,-0.642123807467896879)
  
