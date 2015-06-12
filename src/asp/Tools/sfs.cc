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

// Turn off warnings from boost and other packages
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

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions.h>
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

typedef InterpolationView<ImageViewRef<float>, BilinearInterpolation> BilinearInterpT;

// TODO: Find a good automatic value for the smoothness weight.
// TODO: Investigate the sign of the normal.
// TODO: Loop over all images when doing sfs.
// TODO: Shadow threshold needs detection.
// TODO: Check that we are within image boundaries when interpolating.
// TODO: Radiometric calibration of images.
// TODO: Handle the case when the DEM has no-data values.
// TODO: Add various kind of loss function.
// TODO: Study the normal computation formula.
// TODO: Move some code to Core.
// TODO: Make it work with non-ISIS cameras.
// TODO: Clean up some of the classes, not all members are needed.

bool readNoDEMDataVal(std::string DEMFile, double & nodata_val){

  boost::scoped_ptr<SrcImageResource> rsrc( DiskImageResource::open(DEMFile) );
  if ( rsrc->has_nodata_read() ){
    nodata_val = rsrc->nodata_read();
    return true;
  }

  return false;
}

struct GlobalParams{

  std::string drgDir;
  std::string demDir;

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
  int nodata_val;
  int forceMosaic; // see the description in reconstruct.cc
};

struct ModelParams {

  double   exposureTime;

  vw::Vector2 cameraParams; //currently not used
  vw::Vector3 sunPosition; //relative to the center of the Moon
  vw::Vector3 cameraPosition;//relative to the center of the planet

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
                   GlobalParams const& global_params,
                   double & phase_angle) {
  double input_img_reflectance;

  switch ( global_params.reflectanceType )
    {
    case LUNAR_LAMBERT:
      //printf("Lunar Lambert\n");
      input_img_reflectance
        = computeLunarLambertianReflectanceFromNormal(input_img_params.sunPosition,
                                                      input_img_params.cameraPosition,
                                                      xyz,  normal,
                                                      global_params.phaseCoeffC1,
                                                      global_params.phaseCoeffC2,
                                                      phase_angle // output
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
                               ImageView<double> const& dem,
                               cartography::GeoReference const& geo,
                               double nodata_val,
                               ModelParams const& input_img_params,
                               GlobalParams const& global_params,
                               bool savePhaseAngle,
                               double &outputReflectance,
                               double & phase_angle) {

  // We assume 1<= x and 1 <= y

  outputReflectance = 0.0;
  if (savePhaseAngle) phase_angle = std::numeric_limits<double>::quiet_NaN();

  if (x <= 0 || y <= 0) return;

  Vector2 lonlat = geo.pixel_to_lonlat(Vector2(x, y));
  double   h      = dem(x, y);
  if (h == nodata_val) return;;
  Vector3 lonlat3(lonlat(0), lonlat(1), h);
  Vector3 base = geo.datum().geodetic_to_cartesian(lonlat3);

  lonlat = geo.pixel_to_lonlat(Vector2(x-1, y));
  h      = dem(x-1, y);
  if (h == nodata_val) return;;
  lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 x1 = geo.datum().geodetic_to_cartesian(lonlat3);

  lonlat = geo.pixel_to_lonlat(Vector2(x, y-1));
  h      = dem(x, y-1);
  if (h == nodata_val) return;;
  lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 y1 = geo.datum().geodetic_to_cartesian(lonlat3);

  Vector3 dx = base - x1;
  Vector3 dy = base - y1;
  Vector3 normal = -normalize(cross_prod(dx, dy));

  outputReflectance = ComputeReflectance(normal, base, input_img_params,
                                         global_params, phase_angle);
}

void computeReflectanceAux(ImageView<double> const& dem,
                           cartography::GeoReference const& geo,
                           double nodata_val,
                           ModelParams const& input_img_params,
                           GlobalParams const& global_params,
                           ImageView<PixelMask<double> >& outputReflectance,
                           bool savePhaseAngle,
                           ImageView<PixelMask<double> >& phase_angle) {

  outputReflectance.set_size(dem.cols(), dem.rows());
  if (savePhaseAngle) phase_angle.set_size(dem.cols(), dem.rows());

  for (int y = 1; y < (int)outputReflectance.rows(); y++) {
    for (int x = 1; x < (int)outputReflectance.cols(); x++) {

      double outputReflectanceVal, phase_angleVal;
      computeReflectanceAtPixel(x, y, dem, geo, nodata_val, input_img_params, global_params, savePhaseAngle, outputReflectanceVal, phase_angleVal);

      outputReflectance(x, y) = outputReflectanceVal;
      if (savePhaseAngle) phase_angle(x, y) = phase_angleVal;
    }
  }

  return;
}

// Discrepancy between scaled intensity and reflectance.
// sum | (I + A[1])/A[0] - R |^2.
struct IntensityError {
  IntensityError(int col, int row,
                 ImageView<double> const& dem,
                 cartography::GeoReference const& geo,
                 GlobalParams const& global_params,
                 std::vector<ModelParams> & model_params, // const?
                 std::vector<BilinearInterpT> const& images,
                 std::vector<boost::shared_ptr<CameraModel> > const& cameras,
                 double grid_size, double nodata_val):
    m_col(col), m_row(row), m_dem(dem), m_geo(geo),
    m_global_params(global_params),
    m_model_params(model_params),
    m_images(images), m_cameras(cameras),
    m_grid_size(grid_size), m_nodata_val(nodata_val) {}

  // See SmoothnessError() for the definitions of tl, top, tr, etc.
  template <typename T>
  bool operator()(const T* const A,
                  const T* const tl,   const T* const top,    const T* const tr,
                  const T* const left, const T* const center, const T* const right,
                  const T* const bl,   const T* const bottom, const T* const br,
                  T* residuals) const {

    // Default residuals
    residuals[0] = T(1e+20);

    try{

      // TODO: Investigate various ways of finding the normal.

      // The xyz position at the center grid point
      Vector2 lonlat = m_geo.pixel_to_lonlat(Vector2(m_col, m_row));
      double h = center[0];
      if (h == m_nodata_val) return false;
      Vector3 lonlat3 = Vector3(lonlat(0), lonlat(1), h);
      Vector3 base = m_geo.datum().geodetic_to_cartesian(lonlat3);

      // The xyz position at the right grid point
      lonlat = m_geo.pixel_to_lonlat(Vector2(m_col+1, m_row));
      h = right[0];
      if (h == m_nodata_val) return false;
      lonlat3 = Vector3(lonlat(0), lonlat(1), h);
      Vector3 right = m_geo.datum().geodetic_to_cartesian(lonlat3);

      // The xyz position at the top grid point
      lonlat = m_geo.pixel_to_lonlat(Vector2(m_col, m_row+1));
      h = top[0];
      if (h == m_nodata_val) return false;
      lonlat3 = Vector3(lonlat(0), lonlat(1), h);
      Vector3 top = m_geo.datum().geodetic_to_cartesian(lonlat3);

      // TODO: Study the sign of the normal.
      Vector3 dx = right - base;
      Vector3 dy = top - base;
      Vector3 normal = -normalize(cross_prod(dx, dy));

      // TODO: Must iterate over all images below.
      double phase_angle;
      double output_reflectance
        = ComputeReflectance(normal, base, m_model_params[0],
                             m_global_params, phase_angle);

      Vector2 pix = m_cameras[0]->point_to_pixel(base);
      residuals[0] = ( m_images[0](pix[0], pix[1]) + A[1] ) / A[0] - output_reflectance;

    } catch (const camera::PointToPixelErr& e) {
      return false;
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(int col, int row,
                                     ImageView<double> const& dem,
                                     vw::cartography::GeoReference const& geo,
                                     GlobalParams const& global_params,
                                     std::vector<ModelParams> & model_params,
                                     std::vector<BilinearInterpT> & images,
                                     std::vector<boost::shared_ptr<CameraModel> > & cameras,
                                     double grid_size, double nodata_val){
    return (new ceres::NumericDiffCostFunction<IntensityError,
            ceres::CENTRAL, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1>
            (new IntensityError(col, row, dem, geo, global_params, model_params,
                                images, cameras, grid_size, nodata_val)));
  }

  int m_col, m_row;
  ImageView<double>                            const & m_dem;           // alias
  cartography::GeoReference                    const & m_geo;           // alias
  GlobalParams                                 const&  m_global_params; // alias
  std::vector<ModelParams>                     const & m_model_params;  // alias
  std::vector<BilinearInterpT>                 const & m_images;        // alias
  std::vector<boost::shared_ptr<CameraModel> > const & m_cameras;       // alias
  double m_grid_size, m_nodata_val;
};


// The smoothness error is the sum of squares of
// the 4 second order partial derivatives, with a weight:
// error = smoothness_weight * ( u_xx^2 + u_xy^2 + u_yx^2 + u_yy^2 )

// We will use finite differences to compute these.
// Consider a grid point and its neighbors, 9 points in all.
//
// tl   = u(c-1, r+1)  top    = u(c, r+1) tr    = u(c+1,r+1)
// left = u(c-1, r  )  center = u(c, r  ) right = u(c+1,r  )
// bl   = u(c-1, r-1)  bottom = u(c, r-1) br    = u(c+1,r-1)
//
// See https://en.wikipedia.org/wiki/Finite_difference
// for the obtained formulas.

struct SmoothnessError {
  SmoothnessError(double smoothness_weight, double grid_size):
    m_smoothness_weight(smoothness_weight),
    m_grid_size(grid_size) {}

  template <typename T>
  bool operator()(const T* const tl,   const T* const top,    const T* const tr,
                  const T* const left, const T* const center, const T* const right,
                  const T* const bl,   const T* const bottom, const T* const br,
                  T* residuals) const {
    try{

      T gs = m_grid_size * m_grid_size;
      residuals[0] = (left[0] + right[0] - 2*center[0])/gs;     // u_xx
      residuals[1] = (tr[0] + bl[0] - tl[0] - br[0] ) /4.0/gs;  // u_xy
      residuals[2] = residuals[1];                              // u_yx
      residuals[3] = (top[0] + bottom[0] - 2*center[0])/gs;     // u_yy

      for (int i = 0; i < 4; i++)
        residuals[i] *= m_smoothness_weight;

    } catch (const camera::PointToPixelErr& e) {
      // Failed to compute the residuals
      residuals[0] = T(1e+20);
      residuals[1] = T(1e+20);
      residuals[2] = T(1e+20);
      residuals[3] = T(1e+20);
      return false;
    }

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double smoothness_weight, double grid_size){
    return (new ceres::NumericDiffCostFunction<SmoothnessError,
            ceres::CENTRAL, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1>
            (new SmoothnessError(smoothness_weight, grid_size)));
  }

  double m_smoothness_weight, m_grid_size;
};

struct Options : public asp::BaseOptions {
  std::string input_dem, out_prefix, stereo_session_string;
  std::vector<std::string> input_images;
  int max_iterations;
  double smoothness_weight;
  Options():max_iterations(0) {};

};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("input-dem,i",  po::value(&opt.input_dem),
     "The input DEM to refine using SfS.")
    ("output-prefix,o", po::value(&opt.out_prefix),
     "Prefix for output filenames.")
    ("max-iterations,n", po::value(&opt.max_iterations)->default_value(100),
     "Set the maximum number of iterations.")
    ("smoothness-weight", po::value(&opt.smoothness_weight)->default_value(1.0),
     "A larger value will result in a smoother solution.");

  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-images", po::value(&opt.input_images));

  po::positional_options_description positional_desc;
  positional_desc.add("input-images", -1);

  std::string usage("-i <input DEM> -n <max iterations> -o <output prefix> <images> [other options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if (opt.input_dem.empty())
    vw_throw( ArgumentErr() << "Missing input DEM.\n"
              << usage << general_options );

  if (opt.out_prefix.empty())
    vw_throw( ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options );

  if (opt.max_iterations < 0)
    vw_throw( ArgumentErr() << "The number of iterations must be non-negative.\n"
              << usage << general_options );

  if (opt.input_images.empty())
  vw_throw( ArgumentErr() << "Missing input images.\n"
            << usage << general_options );

  // Create the output directory
  asp::create_out_dir(opt.out_prefix);
}

int main(int argc, char* argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    ImageView<double> dem
      = copy(DiskImageView< PixelGray<float> >(opt.input_dem) );
    GeoReference geo;
    if (!read_georeference(geo, opt.input_dem))
      vw_throw( ArgumentErr() << "The input DEM has no georeference.\n" );

    double nodata_val = -32768;
    if (readNoDEMDataVal(opt.input_dem, nodata_val)){
      std::cout << "Found DEM nodata value: " << nodata_val << std::endl;
    }

    GlobalParams global_params;
    global_params.reflectanceType = LUNAR_LAMBERT;
    global_params.phaseCoeffC1    = 1.383488;
    global_params.phaseCoeffC2    = 0.501149;
    //PrintGlobalParams(global_params);

    int num_images = opt.input_images.size();

    std::vector<boost::shared_ptr<CameraModel> > cameras;
    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session(asp::StereoSessionFactory::create
                       (opt.stereo_session_string, opt,
                        opt.input_images[0], opt.input_images[0],
                        opt.input_images[0], opt.input_images[0],
                        opt.out_prefix));

    // Read in the camera models for the input images.
    for (int i = 0; i < num_images; i++){
      vw_out(DebugMessage,"asp") << "Loading: " << opt.input_images[i] << ' '
                                 << opt.input_images[i] << "\n";
      cameras.push_back(session->camera_model(opt.input_images[i],
                                              opt.input_images[i]));
    }

    // Get the sun and camera positions from the ISIS cube
    std::vector<ModelParams> model_params;
    model_params.resize(num_images);
    for (int i = 0; i < num_images; i++){
      IsisCameraModel* icam = dynamic_cast<IsisCameraModel*>
        (vw::camera::unadjusted_model(cameras[i].get()));
      if (icam == NULL)
        vw_throw( ArgumentErr() << "ISIS camera model expected." );
      model_params[i].sunPosition    = icam->sun_position();
      model_params[i].cameraPosition = icam->camera_center();
      std::cout << "sun position: " << model_params[i].sunPosition << std::endl;
      std::cout << "camera position: " << model_params[i].cameraPosition << std::endl;
    }

    // Images with bilinear interpolation
    std::vector<BilinearInterpT> interp_images;
    for (int i = 0; i < num_images; i++){
      interp_images.push_back(BilinearInterpT(DiskImageView<float>(opt.input_images[i])));
    }

    Vector2 ul = geo.pixel_to_point(Vector2(0, 0));
    int ncols = dem.cols(), nrows = dem.rows();
    Vector2 lr = geo.pixel_to_point(Vector2(ncols-1, nrows-1));
    double grid_size = norm_2(ul - lr)/norm_2(Vector2(ncols-1, nrows-1));

    // Intensity error is
    // sum | (I + A[1])/A[0] - R |^2.
    // TODO: What are good values here.
    double A[2];
    double mn =  -32752.000, mx = 32767.000;
    A[1] = -mn; A[0] = (mx - mn);

    // Add a residual block for every grid point not at the boundary
    ceres::Problem problem;
    for (int col = 1; col < ncols-1; col++) {
      for (int row = 1; row < nrows-1; row++) {

        // Intensity error
        ceres::CostFunction* cost_function1 =
          IntensityError::Create(col, row, dem, geo,
                                 global_params, model_params, interp_images,
                                 cameras, grid_size, nodata_val);
        ceres::LossFunction* loss_function1 = NULL;
        problem.AddResidualBlock(cost_function1, loss_function1,
                                 A,
                                 &dem(col-1, row+1), &dem(col, row+1), // tl, top
                                 &dem(col+1, row+1),                   // tr
                                 &dem(col-1, row  ), &dem(col, row  ), // left, ctr
                                 &dem(col+1, row  ),                   // right
                                 &dem(col-1, row-1), &dem(col, row-1), // bl, bot
                                 &dem(col+1, row-1));                  // br

        // Smoothness penalty
        ceres::LossFunction* loss_function2 = NULL;
        ceres::CostFunction* cost_function2 =
          SmoothnessError::Create(opt.smoothness_weight, grid_size);
        problem.AddResidualBlock(cost_function2, loss_function2,
                                 &dem(col-1, row+1), &dem(col, row+1),
                                 &dem(col+1, row+1),
                                 &dem(col-1, row  ), &dem(col, row  ),
                                 &dem(col+1, row  ),
                                 &dem(col-1, row-1), &dem(col, row-1),
                                 &dem(col+1, row-1));

        // Variables at the boundary must be fixed
        if (col==1) {
          // left boundary
          problem.SetParameterBlockConstant(&dem(col-1, row-1));
          problem.SetParameterBlockConstant(&dem(col-1, row));
          problem.SetParameterBlockConstant(&dem(col-1, row+1));
        }
        if (row==1) {
          // bottom boundary
          problem.SetParameterBlockConstant(&dem(col-1, row-1));
          problem.SetParameterBlockConstant(&dem(col,   row-1));
          problem.SetParameterBlockConstant(&dem(col+1, row-1));
        }
        if (col==ncols-2) {
          // right boundary
          problem.SetParameterBlockConstant(&dem(col+1, row-1));
          problem.SetParameterBlockConstant(&dem(col+1, row));
          problem.SetParameterBlockConstant(&dem(col+1, row+1));
        }
        if (row==nrows-2) {
          // top boundary
          problem.SetParameterBlockConstant(&dem(col-1, row+1));
          problem.SetParameterBlockConstant(&dem(col,   row+1));
          problem.SetParameterBlockConstant(&dem(col+1, row+1));
        }

      }
    }

    // Temporarily fix the scale and shift coefficients
    problem.SetParameterBlockConstant(A);

    ceres::Solver::Options options;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.max_num_iterations = opt.max_iterations;
    options.minimizer_progress_to_stdout = 1;

    options.num_threads = opt.num_threads;

    options.linear_solver_type = ceres::SPARSE_SCHUR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    vw_out() << summary.FullReport() << "\n";

    std::string out_dem_file = opt.out_prefix + "-final-DEM.tif";
    vw_out() << "Writing: " << out_dem_file << std::endl;
    TerminalProgressCallback tpc("asp", ": ");
    block_write_gdal_image(out_dem_file, dem, geo, nodata_val, opt, tpc);

  } ASP_STANDARD_CATCHES;
}
