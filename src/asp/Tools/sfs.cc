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


/// \file sfs.cc

#include <asp/Core/Macros.h>
#include <asp/Sessions.h>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

// TODO: Move this out of here.
bool readNoDEMDataVal(std::string DEMFile, double & noDEMDataValue){

  boost::scoped_ptr<SrcImageResource> rsrc( DiskImageResource::open(DEMFile) );
  if ( rsrc->has_nodata_read() ){
    noDEMDataValue = rsrc->nodata_read();
    return true;
  }

  return false;
}

struct GlobalParams{

  std::string drgDir;
  std::string demDir;
  std::string sunPosFile;
  std::string spacecraftPosFile;

  int initialSetup;
  double tileSize;        // in degrees
  int pixelPadding;      // in pixels
  vw::Vector4 simulationBox; // lonMin, lonMax, latMin, latMax (If not present the entire albedo will be simulated)

  int reflectanceType;
  int initDEM;
  int initExposure;
  int initAlbedo;
  int shadowType;

  double shadowThresh;

  //double exposureInitRefValue;//this will be removed
  //int exposureInitRefIndex;//this will be removed
  double TRConst;
  int updateAlbedo, updateExposure, updateHeight;

  // Two parameters used in the formula for the reflectance
  double phaseCoeffC1, phaseCoeffC2;
  // Update the components of the coefficients phaseCoeffC1 and
  // phaseCoeffC2 for each tile.
  int updateTilePhaseCoeffs;
  // Update the phase coefficients by combining the results from all tiles
  int updatePhaseCoeffs;

  int useWeights;
  int saveWeights, computeWeightsSum, useNormalizedCostFun;
  int maxNumIter;
  int computeErrors;
  int noDEMDataValue;
  int forceMosaic; // see the description in reconstruct.cc
};

struct ModelParams {

  double   exposureTime;

  vw::Vector2 cameraParams; //currently not used
  vw::Vector3 sunPosition; //relative to the center of the Moon
  vw::Vector3 spacecraftPosition;//relative to the center of the planet

  std::vector<int> hCenterLine;
  std::vector<int> hMaxDistArray;
  std::vector<int> vCenterLine;
  std::vector<int> vMaxDistArray;

  int *hCenterLineDEM;
  int *hMaxDistArrayDEM;
  int *vCenterLineDEM;
  int *vMaxDistArrayDEM;

  vw::Vector4 corners; // cached bounds to quickly calculate overlap

  /*
    vector<int> hCenterLine;
    vector<int> hMaxDistArray;
    vector<int> hCenterLineDEM;
    vector<int> hMaxDistArrayDEM;
    vector<int> vCenterLine;
    vector<int> vMaxDistArray;
    vector<int> vCenterLineDEM;
    vector<int> vMaxDistArrayDEM;
  */
  std::string infoFilename, DEMFilename, meanDEMFilename,
    var2DEMFilename, reliefFilename, shadowFilename,
    errorFilename, inputFilename, outputFilename,
    sfsDEMFilename, errorHeightFilename, weightFilename, exposureFilename;

  ModelParams(){
    hCenterLineDEM   = NULL;
    hMaxDistArrayDEM = NULL;
    vCenterLineDEM   = NULL;
    vMaxDistArrayDEM = NULL;
  }

  ~ModelParams(){
    // Need to deal with copying of these structures properly before deleting
    // delete[] hCenterLine;       hCenterLine       = NULL;
    // delete[] hMaxDistArray;     hMaxDistArray     = NULL;
    // delete[] hCenterLineDEM;    hCenterLineDEM    = NULL;
    // delete[] hMaxDistArrayDEM;  hMaxDistArrayDEM  = NULL;
    // delete[] vCenterLine;       vCenterLine       = NULL;
    // delete[] vMaxDistArray;     vMaxDistArray     = NULL;
    // delete[] vCenterLineDEM;    vCenterLineDEM    = NULL;
    // delete[] vMaxDistArrayDEM;  vMaxDistArrayDEM  = NULL;
  }

};

enum {NO_REFL = 0, LAMBERT, LUNAR_LAMBERT};

void
ReadSunOrSpacecraftPosition(std::string const& filename,
                            std::map<std::string, Vector3> & records){

  records.clear();

  std::ifstream infile( filename.c_str() );
  if ( !infile.is_open() ) {
    std::cerr << "ERROR: Could not read file: " << filename << std::endl;
    exit(1);
  }

  while ( infile.good() ){
    std::string line, key;
    Vector3 val;
    getline (infile, line);
    std::istringstream lh(line);
    if (line == "") continue;
    if (! ( lh >> key >> val[0] >> val[1] >> val[2] ) ){
      std::cerr << "ERROR: Unable to read from file: " << filename << " the line: '" << line << "'" << std::endl;
      exit(1);
    }

    if (records.find(key) != records.end()){
      std::cerr << "ERROR: Duplicate key: " << key << " in file: " << filename << std::endl;
      exit(1);
    }

    records[key] = val;
  }

  infile.close();

  return;
}

//computes the Lambertian reflectance model (cosine of the light direction and the normal to the Moon)
//Vector3  sunpos: the 3D coordinates of the Sun relative to the center of the Moon
//Vector2 lon_lat is a 2D vector. First element is the longitude and the second the latitude
//author Ara Nefian
double
computeLambertianReflectanceFromNormal(Vector3 sunPos, Vector3 xyz,
                                       Vector3 normal) {
  double reflectance;

  //Vector3 xyz = get_normal(lon_lat);
  //printf("xyz[0] = %f, xyz[1] = %f, xyz[2] = %f\n", xyz[0], xyz[1], xyz[2]);
  // sun coordinates relative to the xyz point on the Moon surface
  Vector3 sunDirection = normalize(sunPos-xyz);

  reflectance = sunDirection[0]*normal[0] + sunDirection[1]*normal[1] + sunDirection[2]*normal[2];

  return reflectance;
}


double
computeLunarLambertianReflectanceFromNormal(Vector3 const& sunPos, Vector3 const& viewPos, Vector3 const& xyz,
                                                        Vector3 const& normal, double phaseCoeffC1, double phaseCoeffC2,
                                                        double & alpha // output
                                                        ) {

  double reflectance;
  double L;

  double len = dot_prod(normal, normal);
  if (abs(len - 1.0) > 1.0e-4){
    std::cerr << "Error: Expecting unit normal in the reflectance computation, in "
              << __FILE__ << " at line " << __LINE__ << std::endl;
    exit(1);
  }

  //compute /mu_0 = cosine of the angle between the light direction and the surface normal.
  //sun coordinates relative to the xyz point on the Moon surface
  //Vector3 sunDirection = -normalize(sunPos-xyz);
  Vector3 sunDirection = normalize(sunPos-xyz);
  double mu_0 = dot_prod(sunDirection,normal);

  double tol = 0.3;
  if (mu_0 < tol){
    // Sun is too low, reflectance is too close to 0, the albedo will be inaccurate
    return 0.0;
  }

  //compute  /mu = cosine of the angle between the viewer direction and the surface normal.
  //viewer coordinates relative to the xyz point on the Moon surface
  Vector3 viewDirection = normalize(viewPos-xyz);
  double mu = dot_prod(viewDirection,normal);

  //compute the phase angle (alpha) between the viewing direction and the light source direction
  double deg_alpha;
  double cos_alpha;

  cos_alpha = dot_prod(sunDirection,viewDirection);
  if ((cos_alpha > 1)||(cos_alpha< -1)){
    printf("cos_alpha error\n");
  }

  alpha     = acos(cos_alpha);  // phase angle in radians
  deg_alpha = alpha*180.0/M_PI; // phase angle in degrees

  //printf("deg_alpha = %f\n", deg_alpha);

  //Bob Gaskell's model
  //L = exp(-deg_alpha/60.0);

  //Alfred McEwen's model
  double A = -0.019;
  double B =  0.000242;//0.242*1e-3;
  double C = -0.00000146;//-1.46*1e-6;

  L = 1.0 + A*deg_alpha + B*deg_alpha*deg_alpha + C*deg_alpha*deg_alpha*deg_alpha;

  //printf(" deg_alpha = %f, L = %f\n", deg_alpha, L);

  if (mu_0 < 0.0){
    return 0.0;
  }

  if (mu < 0.0){ //emission angle is > 90
    mu = 0.0;
  }

  if (mu_0 + mu == 0){
    //printf("negative reflectance\n");
    return 0.0;
  }
  else{
    reflectance = 2*L*mu_0/(mu_0+mu) + (1-L)*mu_0;
  }
  if (reflectance <= 0){
    //printf("negative reflectance\n");
    return 0.0;
  }

  // Attempt to compensate for points on the terrain being too bright
  // if the sun is behind the spacecraft as seen from those points.

  //reflectance *= std::max(0.4, exp(-alpha*alpha));
  reflectance *= ( exp(-phaseCoeffC1*alpha) + phaseCoeffC2 );

  return reflectance;
}

double
ComputeReflectance(Vector3 const& normal, Vector3 const& xyz,
                               ModelParams const& input_img_params,
                               GlobalParams const& globalParams,
                               double & phaseAngle) {
  double input_img_reflectance;

  switch ( globalParams.reflectanceType )
    {
    case LUNAR_LAMBERT:
      //printf("Lunar Lambert\n");
      input_img_reflectance
        = computeLunarLambertianReflectanceFromNormal(input_img_params.sunPosition,
                                                      input_img_params.spacecraftPosition,
                                                      xyz,  normal,
                                                      globalParams.phaseCoeffC1,
                                                      globalParams.phaseCoeffC2,
                                                      phaseAngle // output
                                                      );
      break;
    case LAMBERT:
      //printf("Lambert\n");
      input_img_reflectance
        = computeLambertianReflectanceFromNormal(input_img_params.sunPosition,
                                                 xyz,  normal);
      break;

    default:
      //printf("No reflectance model\n");
      input_img_reflectance = 1;
    }

  return input_img_reflectance;
}

void computeReflectanceAtPixel(int x, int y,
                               ImageView<PixelGray<double> > const& DEMTile,
                               cartography::GeoReference const& DEMGeo,
                               double noDEMDataValue,
                               ModelParams const& input_img_params,
                               GlobalParams const& globalParams,
                               bool savePhaseAngle,
                               double &outputReflectance,
                               double & phaseAngle) {

  // We assume 1<= x and 1 <= y

  outputReflectance = 0.0;
  if (savePhaseAngle) phaseAngle = std::numeric_limits<double>::quiet_NaN();

  if (x <= 0 || y <= 0) return;

  Vector2 lonlat = DEMGeo.pixel_to_lonlat(Vector2(x, y));
  double   h      = DEMTile(x, y);
  if (h == noDEMDataValue) return;;
  Vector3 lonlat3(lonlat(0), lonlat(1), h);
  Vector3 base = DEMGeo.datum().geodetic_to_cartesian(lonlat3);

  lonlat = DEMGeo.pixel_to_lonlat(Vector2(x-1, y));
  h      = DEMTile(x-1, y);
  if (h == noDEMDataValue) return;;
  lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 x1 = DEMGeo.datum().geodetic_to_cartesian(lonlat3);

  lonlat = DEMGeo.pixel_to_lonlat(Vector2(x, y-1));
  h      = DEMTile(x, y-1);
  if (h == noDEMDataValue) return;;
  lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 y1 = DEMGeo.datum().geodetic_to_cartesian(lonlat3);

  Vector3 dx = base - x1;
  Vector3 dy = base - y1;
  Vector3 normal = -normalize(cross_prod(dx, dy));

  outputReflectance = ComputeReflectance(normal, base, input_img_params, globalParams, phaseAngle);
}

void computeReflectanceAux(ImageView<PixelGray<double> > const& DEMTile,
                           cartography::GeoReference const& DEMGeo,
                           double noDEMDataValue,
                           ModelParams const& input_img_params,
                           GlobalParams const& globalParams,
                           ImageView<PixelMask<PixelGray<double> > >& outputReflectance,
                           bool savePhaseAngle,
                           ImageView<PixelMask<PixelGray<double> > >& phaseAngle) {

  //std::cout << "---sun        " << input_img_params.sunPosition << std::endl;
  //std::cout << "---spacecraft " << input_img_params.spacecraftPosition << std::endl;
  outputReflectance.set_size(DEMTile.cols(), DEMTile.rows());
  if (savePhaseAngle) phaseAngle.set_size(DEMTile.cols(), DEMTile.rows());

  for (int y = 1; y < (int)outputReflectance.rows(); y++) {
    for (int x = 1; x < (int)outputReflectance.cols(); x++) {

      double outputReflectanceVal, phaseAngleVal;
      computeReflectanceAtPixel(x, y, DEMTile, DEMGeo, noDEMDataValue, input_img_params, globalParams, savePhaseAngle, outputReflectanceVal, phaseAngleVal);

      outputReflectance(x, y) = outputReflectanceVal;
      if (savePhaseAngle) phaseAngle(x, y) = phaseAngleVal;
    }
  }

  return;
}

std::string getFirstElevenCharsFromFileName(std::string fileName){

  // Out of path/to/AS15-M-1723_1724-DEM.tif extract the 11 characters
  // "AS15-M-1723".
  int index = fileName.rfind("/");
  if (index != -1) fileName.erase(0, index + 1);

  return fileName.substr(0, 11);
}

struct Options : public asp::BaseOptions {
  std::string input_dem, meta_dir, out_prefix;
  std::vector<std::string> input_images;
  int max_iterations;
  Options():max_iterations(0) {};

};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("input-dem,i",  po::value(&opt.input_dem),
     "The input DEM to refine using SfS.")
    ("meta-dir,m",  po::value(&opt.meta_dir),
     "The directory containing sun and spacecraft position.")
    ("output-prefix,o", po::value(&opt.out_prefix),
     "Prefix for output filenames.")
    ("max-iterations,n", po::value(&opt.max_iterations)->default_value(100),
     "Set the maximum number of iterations.");

  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-images", po::value(&opt.input_images));

  po::positional_options_description positional_desc;
  positional_desc.add("input-images", -1);

  std::string usage("-i <input DEM> -m <meta dir> -n <max iterations> -o <output prefix> <images> [other options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if (opt.input_dem.empty())
    vw_throw( ArgumentErr() << "Missing input DEM.\n"
              << usage << general_options );

  if (opt.meta_dir.empty())
    vw_throw( ArgumentErr() << "Missing meta directory.\n"
              << usage << general_options );

  if (opt.out_prefix.empty())
    vw_throw( ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options );

  if (opt.max_iterations <= 0)
    vw_throw( ArgumentErr() << "The number of iterations must be positive.\n"
              << usage << general_options );

  vw_throw( ArgumentErr() << "Missing input images.\n"
            << usage << general_options );
}

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    std::cout << "input dem " << opt.input_dem << std::endl;
    std::cout << "meta dir " << opt.meta_dir << std::endl;
    std::cout << "out prefix " << opt.out_prefix << std::endl;
    std::cout << "max iter " << opt.max_iterations << std::endl;
    std::cout << "input images ";
    for (size_t i = 0; i < opt.input_images.size(); i++)
      std::cout << opt.input_images[i] << " ";
    std::cout << std::endl;

    ImageView<PixelGray<double> > DEMTile
      = copy(DiskImageView<PixelGray<float> >(opt.input_dem));
    GeoReference DEMGeo;
    if (!read_georeference(DEMGeo, opt.input_dem))
      vw_throw( ArgumentErr() << "The input DEM has no georeference.\n" );

    double DEMnodata = -32768;
    if (readNoDEMDataVal(opt.input_dem, DEMnodata)){
      std::cout << "Found DEM nodata value: " << DEMnodata << std::endl;
    }

    GlobalParams globalParams;
    globalParams.sunPosFile = opt.meta_dir + "/sunpos.txt";
    globalParams.spacecraftPosFile = opt.meta_dir + "/spacecraftpos.txt";
    globalParams.reflectanceType = LUNAR_LAMBERT;
    globalParams.phaseCoeffC1    = 1.383488;
    globalParams.phaseCoeffC2    = 0.501149;
    //PrintGlobalParams(globalParams);


    std::map<std::string, Vector3> sunPositions;
    std::map<std::string, Vector3> spacecraftPositions;
    ReadSunOrSpacecraftPosition(globalParams.sunPosFile, // Input
                              sunPositions             // Output
                                );
    ReadSunOrSpacecraftPosition(globalParams.spacecraftPosFile, // Input
                                spacecraftPositions             // Output
                                );

    std::vector<ModelParams> modelParamsArray;
    std::cout.precision(18);
    modelParamsArray.resize(opt.input_images.size());
    for (int k = 0; k < (int)opt.input_images.size(); k++){
      std::string prefix = getFirstElevenCharsFromFileName(opt.input_images[k]);

      modelParamsArray[k].inputFilename = opt.input_images[k];

      if ( sunPositions.find(prefix) == sunPositions.end()){
        std::cerr << "Could not find the sun position for the image file: "
                  << opt.input_images[k] << std::endl;
        exit(1);
      }
      // Go from kilometers to meters
      modelParamsArray[k].sunPosition = 1000*sunPositions[prefix];
      std::cout << "sun position: " <<  modelParamsArray[k].inputFilename
                << ' ' <<  modelParamsArray[k].sunPosition << std::endl;

      if (spacecraftPositions.find(prefix) == spacecraftPositions.end()){
        std::cerr << "Could not find the spacecraft position for the DRG file: "
                  << opt.input_images[k] << std::endl;
        exit(1);
      }
      // Go from kilometers to meters
      modelParamsArray[k].spacecraftPosition = 1000*spacecraftPositions[prefix];
      std::cout << "spacecraft position: " <<  modelParamsArray[k].inputFilename
                << ' ' <<  modelParamsArray[k].spacecraftPosition << std::endl;
    }

  } ASP_STANDARD_CATCHES;
}
