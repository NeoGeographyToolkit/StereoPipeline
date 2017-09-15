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
// for example, sbet_20111012.out. See the User Guide there for more info.

// Convert the nav file to text using the Perl reader
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
// orthoimage_path, DMS_1281706_09765_20111012_18060740.tif.  in the format
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
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/Extrinsics.h>
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
using namespace vw::math;

// From DMS_1281706_09765_20111012_18060740.tif
// extract the date 2011/10/12 and time 18:06:07.40.
// Then find how many seconds elapsed since the beginning of the week.
double gps_seconds(std::string const& orthoimage_path){

  std::string label = "DMS";
  std::size_t it = orthoimage_path.find(label);
  if (it == std::string::npos) 
    vw_throw( ArgumentErr() << "Could not find the text " << label << " in " << orthoimage_path << "\n" );

  it += 18;

  std::string date = orthoimage_path.substr(it, 8);
  std::string time = orthoimage_path.substr(it+9, 8);

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

/// Extract the parameters from a line of the nav file
void scan_line(std::string const& line,
               double& seconds, double& lat, double& lon, double& alt,
               double& roll, double& pitch, double& heading){
  double xv, yv, zv;
  if (sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
             &seconds, &lat, &lon, &alt, &xv, &yv, &zv, &roll, &pitch, &heading) != 10) 
    vw_throw( ArgumentErr() << "Could not scan 10 values from line: " << line << "\n" );
}

// TODO: It is not clear what the right order should be! 
Matrix3x3 get_look_rotation_matrix(double yaw, double pitch, double roll, int rot_order) {

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

  if (rot_order == 1) return Mp*Mr*My;
  if (rot_order == 2) return Mp*My*Mr;
  if (rot_order == 3) return My*Mp*Mr;
  if (rot_order == 4) return My*Mr*Mp;
  if (rot_order == 5) return Mr*Mp*My;
  if (rot_order == 6) return Mr*My*Mp;
  
  Matrix3x3 out = Mp*Mr*My;
  return out;
  
}

/// Generate the pose and position from nav information
void parse_camera_pose(std::string const& line, Vector3 & xyz, Quat & look, Quat & ned,
                       double& lon, double& lat, double& alt,
                       double &roll, double &pitch, double &heading,
                       int rot_order){

  // Parse the line of text
  double seconds;
  scan_line(line, seconds, lat, lon, alt, roll, pitch, heading);
  
  // Get cartesian coordinates - this is easy.
  Datum datum_wgs84("WGS84");
  xyz = datum_wgs84.geodetic_to_cartesian(Vector3(lon, lat, alt));

  // Generate the NED frame at this location.
  // - This is relative to the GCC coordinate frame.
  //std::cout << "--xyz is " << xyz << std::endl;
  ned = Quat(datum_wgs84.lonlat_to_ned_matrix(Vector2(lon, lat)));

  //std::cout << "ned matrix " << Quat(ned) << std::endl;

  // Get the local rotation matrix at this coordinate.
  look = Quat(get_look_rotation_matrix(-heading, -pitch, -roll, rot_order));

    //std::cout << "--ned " << Quat(ned) << std::endl;
    //std::cout << "--look is " << Quat(look) << std::endl;
    //std::cout << "--nedlook is " << Quat(ned*look) << std::endl;
    //std::cout << "--inv_look *ned is " << inverse(Quat(look))*Quat(ned) << std::endl;
    
    //Matrix3x3 rot_mat = look*ned;
    
    //std::cout << "rot mat is " << rot_mat << std::endl;
    
    //std::cout << "--quat1 " << Quat(look*ned) << std::endl;
    //std::cout << "--quat " << Quat(ned*look) << std::endl;
    //std::cout << "--quat3 " << Quat(inverse(ned)*look) << std::endl;
    //std::cout << "--quat3 " << Quat(inverse(look)*ned) << std::endl;
    //std::cout << "--quat3 " << Quat(ned*inverse(look)) << std::endl;
    //std::cout << "--quat3 " << Quat(look*inverse(ned)) << std::endl;
  
}

            
struct Options : public vw::cartography::GdalWriteOptions {
  std::string nav_file, input_cam, output_folder;
  std::vector<std::string> image_files, camera_files;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  std::string cam_list_path;
  po::options_description general_options("");
  general_options.add_options()
    ("input-cam",      po::value(&opt.input_cam)->default_value(""), 
                       "The input camera file from where to read the intrinsics.")
    ("nav-file",       po::value(&opt.nav_file)->default_value(""), "The nav file, in text format.")
    ("output-folder",  po::value(&opt.output_folder)->default_value(""), 
                       "Output folder where the camera files are written.")
    ("cam-list",       po::value(&cam_list_path)->default_value(""), 
                       "A sorted list of input ortho and camera files.");
  
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );
  
  po::options_description positional("");
  
  po::positional_options_description positional_desc;
  
  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Check mandatory inputs  
  if ( opt.nav_file.empty() )
    vw_throw( ArgumentErr() << "Missing input nav file.\n" << usage << general_options );

  if ( opt.input_cam.empty() )
    vw_throw( ArgumentErr() << "Missing input pinhole camera.\n" << usage << general_options );

  if ( cam_list_path.empty() )
    vw_throw( ArgumentErr() << "Missing input file list file.\n" << usage << general_options );

  if ( !cam_list_path.empty() ) {
    // Load all the names from the camera list file into opt.image_files
    vw_out() << "Reading input files from " << cam_list_path << "...\n";
    std::ifstream input_stream(cam_list_path.c_str());
    std::string line;
    while (getline(input_stream, line)){ // Loop through lines in the file
      // The line should be in the form "orthoFile, cameraFile"
      size_t comma       = line.find(",");
      std::string ortho  = line.substr(0, comma);
      std::string camera = line.substr(comma+2);
      boost::trim(ortho);
      boost::trim(camera);
      opt.image_files.push_back(ortho);
      opt.camera_files.push_back(camera);
    }
    input_stream.close();
    vw_out() << "Read " << opt.image_files.size() << " files from " << cam_list_path << std::endl;
  }

}



/**
  Class which loads an Icebridge nav file in chunks and provides an interpolator
   for each chunk.
*/
class ScrollingNavInterpolator {
public:

  typedef vw::camera::LagrangianInterpolationVarTime InterpType;

  // Open the file
  ScrollingNavInterpolator(std::string const& path, Datum const& datum_in)
    : m_datum(datum_in) {
    m_input_stream.open(path.c_str());
    
    // Load the nav file in chunks of this size
    const size_t INTERPOLATE_CHUNK_LENGTH  = 10000;
    const size_t INTERPOLATE_CHUNK_OVERLAP = 1000; // Overlap between chunks
    m_chunk_length  = INTERPOLATE_CHUNK_LENGTH;
    m_chunk_overlap = INTERPOLATE_CHUNK_OVERLAP;
    
    // Init vectors to a fixed size
    m_time_vector.resize(INTERPOLATE_CHUNK_LENGTH);
    m_loc_vector.resize (INTERPOLATE_CHUNK_LENGTH);
    
    m_first_chunk = true;
  }
  
  ~ScrollingNavInterpolator() {
    m_input_stream.close();
  }

  /// Set up the next interpolator
  /// - Returns false if the end of the file was reached
  bool load_next_chunk(boost::shared_ptr<InterpType> &interpolator_ptr) {

    //vw_out() << "Loading next nav chunk\n";
  
    size_t index = 0;
    
    if (!m_first_chunk) {
      // Copy N values from the end of the vectors to the beginning
      const size_t end_index = m_chunk_length - m_chunk_overlap;
      for (index=0; index<m_chunk_overlap; ++index) {
        m_time_vector[index] = m_time_vector[end_index+index];
        m_loc_vector [index] = m_loc_vector [end_index+index];
      }
    }
    else
      m_first_chunk = false;
    
    // Populate rest of the vectors by reading from the file
    double  time;
    Vector3 loc;
    while (read_next_line(time, loc)){
      m_time_vector[index] = time;
      m_loc_vector [index] = loc;
      ++index;
      if (index == m_chunk_length)
        break;
    }
    
    // If we hit the end of the file just return false, hopefully no camera
    //  data is right at the end of the file!
    if (index < m_chunk_length)
      return false;
  
    // Set up the interpolator
    const int INTERP_RADIUS = 4;  
    interpolator_ptr = boost::shared_ptr<InterpType>(
          new InterpType(m_loc_vector, m_time_vector, INTERP_RADIUS));
    
    return true;
  }
  
  /// Return the time boundaries of the current interpolator
  void get_time_boundaries(double &start, double &end) const {
    start = m_time_vector.front();
    end   = m_time_vector.back ();
  }


private:

  bool m_first_chunk;
  const Datum m_datum;
  std::vector<double > m_time_vector;
  std::vector<Vector3> m_loc_vector;
  size_t m_chunk_length;
  size_t m_chunk_overlap;
  std::ifstream m_input_stream;

  /// Read and parse the next line in the file
  bool read_next_line(double &time, Vector3& loc) {
    // Try to read the line
    std::string line;
    if (!getline(m_input_stream, line))
      return false;

    // Parse the line
    double lat, lon, alt, roll, pitch, heading;
    scan_line(line, time, lat, lon, alt, roll, pitch, heading);
    Vector3 llh(lon, lat, alt);
    loc = m_datum.geodetic_to_cartesian(llh);
    
    return true;
  }

}; // End class ScrollingNavInterpolator




// ================================================================================

int main(int argc, char* argv[]) {

  Options opt;
  //try {
  handle_arguments(argc, argv, opt);

  //std::cout << "--nav file is " << opt.nav_file << std::endl;

  //const double BIG_NUMBER  = 1000000;
  //double lon0 = BIG_NUMBER;
  Datum datum_wgs84("WGS84");
  
  // Print progress every N files
  const size_t PRINT_INTERVAL = 200;
  
  // Initialize the na v interpolator
  std::cout << "Opening input stream: " << opt.nav_file << std::endl;
  ScrollingNavInterpolator interpLoader(opt.nav_file, datum_wgs84);

  boost::shared_ptr<ScrollingNavInterpolator::InterpType> interpolator_ptr;
  double start, end;
  
  const double POSE_TIME_DELTA     = 0.1; // Look this far ahead/behind to determine direction
  const double CHUNK_TIME_BOUNDARY = 1.0; // Require this much interpolation time
  size_t file_index = 0;
  const size_t num_files = opt.image_files.size();

  const boost::filesystem::path output_dir(opt.output_folder);

  // Keep loading chunks until the nav data catches up with the images
  while (interpLoader.load_next_chunk(interpolator_ptr)) {

    // Get the time boundaries of the current chunk
    interpLoader.get_time_boundaries(start, end);  

    //std::cout << "Time range: " << start << " ---- " << end << std::endl;

    // Loop through ortho files until we need to advance the nav chunk
    while (file_index < num_files) {

      // Get the next input and output paths
      std::string             orthoimage_path = opt.image_files [file_index];
      boost::filesystem::path camera_file(opt.camera_files[file_index]);
      boost::filesystem::path output_camera_path = output_dir / camera_file;
      
      // Get time for this frame
      double ortho_time = gps_seconds(orthoimage_path);
      //std::cout << "Ortho time  = " << ortho_time << std::endl;

      // If this ortho is too far ahead in time, move on to the next nav chunk.
      if (ortho_time > end - CHUNK_TIME_BOUNDARY)
        break; // Don't advance file_index

      // If this ortho is too early in time, move on to the next ortho file.
      if (ortho_time < start + CHUNK_TIME_BOUNDARY) {
        vw_out() << "Too early to interpolate position for file " << orthoimage_path << std::endl;
        ++file_index;
        continue;
      }

      // Try to interpolate this ortho position
      Vector3 gcc_interp;
      try{
        gcc_interp = interpolator_ptr->operator()(ortho_time);
      } catch(...){
        vw_out() << "Failed to interpolate position for file " << orthoimage_path << std::endl;
        ++file_index;
        continue;
      }
      Vector3 llh_interp = datum_wgs84.cartesian_to_geodetic(gcc_interp);
      
      //vw_out() << "For file " << orthoimage_path << " computed LLH " << llh_interp << std::endl;
      
      // Now estimate the rotation information
      // - TODO: Attempt to use the rotation information contained in the file!
      
      // Get a point ahead of and behind the frame location
      Vector3 gcc_interp_forward  = interpolator_ptr->operator()(ortho_time+POSE_TIME_DELTA);
      Vector3 gcc_interp_backward = interpolator_ptr->operator()(ortho_time-POSE_TIME_DELTA);
      
      if (gcc_interp_forward == gcc_interp_backward) {
        vw_out() << "Failed to estimate pose for file " << orthoimage_path << std::endl;
        ++file_index;
        continue;
      }
      
      // From these points get two flight direction vectors and take the mean.
      Vector3 dir1 = gcc_interp_forward - gcc_interp;
      Vector3 dir2 = gcc_interp - gcc_interp_backward;
      Vector3 xDir = (dir1 + dir2) / 2.0;
      
      // The Z vector is straight down from the camera to the ground.
      Vector3 llh_ground = llh_interp;
      llh_ground[2] = 0;
      Vector3 gcc_ground = datum_wgs84.geodetic_to_cartesian(llh_ground);
      Vector3 zDir = gcc_ground - gcc_interp;
      
      // Normalize the vectors
      xDir = xDir / norm_2(xDir);
      zDir = zDir / norm_2(zDir);
      
      // The Y vector is the cross product of the two established vectors
      // - Negate this vector to make it consistent with VW conventions.
      Vector3 yDir = -1.0 * cross_prod(xDir, zDir);
      
      //std::cout << "gcc = " << gcc_interp << std::endl;
      //std::cout << "xDir = " << xDir << std::endl;
      //std::cout << "yDir = " << yDir << std::endl;
      //std::cout << "zDir = " << zDir << std::endl;
      
      // Pack into a rotation matrix
      Matrix3x3 rotation_matrix_gcc(xDir[0], yDir[0], zDir[0],
                                    xDir[1], yDir[1], zDir[1],
                                    xDir[2], yDir[2], zDir[2]);
      
      // Load the reference pinhole model, update it, and write it out to disk.
      PinholeModel camera_model(opt.input_cam);
      camera_model.set_camera_center(gcc_interp);
      camera_model.set_camera_pose(rotation_matrix_gcc);
      //vw_out() << "Writing: " << output_camera_path.string() << std::endl;
      camera_model.write(output_camera_path.string());


      // Update progress
      if (file_index % PRINT_INTERVAL == 0)
        vw_out() << file_index << " files processed.\n";

      ++file_index;

    } // End loop through ortho files
    
  } // End loop through nav batches
  
  vw_out() << "Finished looping through the nav file.\n";
/*
    
    
    // TODO: The camera position needs to be interpolated from the several nearest lines!
    
    Vector3 xyz;
    Quat rot, look, ned;
    
    // Try out different rotation orders
    for (int rot_order = 5; rot_order <= 5; rot_order++) {
    
      // Process the navigation information from this line
      parse_camera_pose(curr_line, xyz, look, ned, 
                        lon, lat, alt, roll, pitch, heading, rot_order);

      vw_out() << "Got GCC coord: " << xyz << std::endl;

      // This is highly confusing, but apparently we need to keep
      // the original longitude when computing the ned matrix that
      // transforms from the coordinate system with orgin at the
      // center of the Earth to the North-East-Down coordinate
      // system that was obtained from the starting image, rather
      // than modifying it as we move over new points over the Earth surface.
      if (lon0 == BIG_NUMBER){
        // First value for these variables.
        lon0 = lon;
      }
      //std::cout << "--temporary!!!" << std::endl;
      //lon0 = -65;
      //ned = Quat(datum_wgs84.lonlat_to_ned_matrix(Vector2(lon0, lat)));

      Quat rotation_matrix_gcc = inverse(ned)*inverse(look);
      
      // Update the input pinhole model and write it out to disk.
      input_pinhole_model.set_camera_center(xyz);
      input_pinhole_model.set_camera_pose(rotation_matrix_gcc);
      vw_out() << "Writing: " << output_camera_path << std::endl;
      input_pinhole_model.write(output_camera_path);
      
    } // End rot_order loop
  
    // Update progress
    if (i % PRINT_INTERVAL == 0)
      vw_out() << i << " files completed.\n";
    
  } // End loop through input files
  
  */
  
  return 0;
    
  //} ASP_STANDARD_CATCHES;
}

  
