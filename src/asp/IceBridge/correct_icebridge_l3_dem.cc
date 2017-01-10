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


#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/GeoReferenceUtils.h>

/**
  A simple tool to fix the Icebridge L3 dems located here: http://nsidc.org/data/iodms3
  
  The files are missing a tag indicating that they are type float and are incorrectly
  read as type uint32.  They are also missing the projection information.  This tool
  just rewrites the file so that this information is added.
*/

using namespace vw;
using namespace vw::cartography;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw::cartography;
int main( int argc, char *argv[] ) {

  // Simple input parsing
  if (argc < 4) {
    vw_out() << "Usage: <input path> <output path> <Pole: North=1, South=0>\n"; 
    return -1;
  }
  std::string input_path  = argv[1];
  std::string output_path = argv[2];
  int pole = atoi(argv[3]);  // Specify which polar projection to use

  // Load the existing georeference info, then add the projection info.
  DiskImageResourceGDAL in_rsrc(input_path);
  GeoReference georef;
  bool has_georef = read_georeference(georef, in_rsrc);
  if (!has_georef) {
    vw_throw( ArgumentErr() << "Could not read the georeference from: " << input_path  
	      << ". Check if the corresponding .tfw file is present.\n");
  }
  
  //vw_out() << "Georef = " << georef << std::endl;
  if (pole) // EPSG:3413
    georef.set_proj4_projection_str("+proj=stere +lat_0=90 +lon_0=-45 +lat_ts=70 +ellps=WGS84 +datum=WGS84 +units=m");
  else // EPSG:3031
    georef.set_proj4_projection_str("+proj=stere +lat_0=-90 +lon_0=0 +lat_ts=-71 +ellps=WGS84 +datum=WGS84 +units=m");
  //vw_out() << "Output Georef = " << georef << std::endl;

  // Set nodata if it is not already set.  This should be constant across files.
  double nodata_val = -32767; // Somehow -32767 is hard-coded in those files.
  if ( in_rsrc.has_nodata_read() ) {
    nodata_val = in_rsrc.nodata_read();
    vw_out() << "\tFound input nodata value: " << nodata_val << std::endl;
  }
  
  // Rasterize the input data using the incorrect uint32 type.
  DiskImageView<uint32> input_dem(in_rsrc); 
  ImageView<uint32>     data_in = input_dem;
  
  // Make a float casted copy of the image data.
  ImageView<float> data_out(data_in.cols(), data_in.rows());
  size_t num_bytes = data_in.cols()*data_in.rows()*sizeof(float);
  memcpy(data_out.data(), data_in.data(), num_bytes);

  // Write the output file.  
  GdalWriteOptions opt;
  vw_out() << "Writing: " << output_path << std::endl;
  block_write_gdal_image(output_path, data_out, true, georef, true, nodata_val, opt,
                         TerminalProgressCallback("vw",""));
           
  return 0;
}
