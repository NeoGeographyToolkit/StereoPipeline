// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// To do:
// Pass blankTilesList as input to reconstruct.cc, to avoid the duplication
// in reconstruct.sh.
// Copy a lot of the code from reconstruct.cc to photometry.
// Wipe the network machines code. Create a copy of reconstruct.sh for use
// on the supercomputer.
// Rm blank tiles altogether.
// Add tool to StereoPipeline to extract the sun and spacecraft
// position from cube. Then wipe reconstruct_aux.cc and everything
// having to do with ISIS from reconstruct.cc and reconstruct.sh.
// Add a blurb in the documentation about how to get images and
// sun/spacecraft position from isis cubes.
// Fix the bug in orthoproject with images going beyond 180 degrees. They show up
// with a huge black band.
// Simplify the script.
// More work in shape from shading: Strip padding at the last iteration.
// Bug: There is a shift in the AS17 mosaic.
// Validate the list of machines, the number of CPUs
// Validate the  sim box
// Check if  reconstruct, orthoproject, image2qtree, gdal, exist.
// To do: How to find the number of processors on a Mac
// Validate in advance if we have sun/spacecraft position for each image.
// Should images not having sun/spacecraft info be ignored?
// To do: Must regenerate the index files all the time, as they are too fragile
// Bad image: DIM_input_10mpp/AS17-M-0473.tif
// To do: Find the sun position and spacecraft position for all cubes. It is missing for some.
// To do: Fix the logic for extracting sun/spacecraft position from cubes
// in both the .cc file and in the shell script.
// To do: Enable the flow of doing all the flow w/o the shell script.
// To do: Support DEMs which are not tiles, and which are uint16.
// To do: The above is not simple at all, when it comes to the part
// when we read portions of DEM images and we concatenate them to save
// on memory. 
// To do: Remove the file reconstruct_aux.cc which mostly duplicates orthoproject.cc.
// To do: Test this big image: AS17-M-0305
// To do: Test this as well. DIM_input_sub64_isis/AS17-M-0281.tif, has a lot of black.
// Note: We assume a certain convention about the isis cube file and corresponding
// isis adjust file.
// Note: The isis adjust file may not exist.
// Urgent: To do: Make sure that the flow projecting to DEM also works when
// we don't have tiles!
// To do: Move readDEMTilesIntersectingBox to Image/ and remove the DEM-specific language.
// To do: Note that the image extracted from cubes is uint16, not uint8.
// To do: Also note that those images are a bit darker than regular
// images we already have, even after converting them to uint8.
// To: The function ComputeGeoBoundary looks wrong. Use instead
// georef.bounding_box(img), which I think is accurate. Compare
// with gdalinfo.
// To do: Check the effect of pixel padding on final albedo. 
// To do: Implement the logic where one checks if a pixel is in the shadow,
// in one (inline) function, and call it wherever it is needed.
// To do: Copy the images from supercomp.
// To do: Fix the memory leaks where the weighs are read.
// Read the data from cubes, and remove all logic having to do with
// filters from reconstruct.sh.
// To do: Unit tests
// To do: Reorg the code which computes the reflectance and its
// derivative in ShapeFromShading.cc to only compute the
// derivative. Move all that code to Reflectance.cc. Convert all to
// double.
// Rename dem_out.tif to dem_mean.tif, and modelParams.outputFile to modelParams.albedoFile,
// inputFile to drgFile.
// The file name of the blankTilesList is repeated in the shell script and the code.
// Remove a lot of duplicate code related to overlaps
// There is only one image, rm the vector of images
// No need to initialize the albedo tiles on disk, create
//   them on the fly.
// Merge the imageRecord and modelParams classes
// See if to change the order of values in the corners vector
#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <boost/tokenizer.hpp>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem/convenience.hpp>
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <vw/Photometry.h>

using namespace vw;
using namespace vw::math;
using namespace vw::cartography;
using namespace vw::photometry;
using namespace std;

namespace fs = boost::filesystem; // comment to reach 6 character requirement

void resampleImage(std::string initFilename, std::string outputFilename, int factor){

    DiskImageView<float>  initImg(initFilename);
    GeoReference initGeo;
    read_georeference(initGeo, initFilename);

    InterpolationView<EdgeExtensionView<EdgeExtensionView<DiskImageView<float>, ConstantEdgeExtension>,
                                        ConstantEdgeExtension>, BilinearInterpolation> interpInitImg
    = interpolate(edge_extend(initImg.impl(),ConstantEdgeExtension()), BilinearInterpolation());

    ImageView<float> outImg(initImg.rows()/factor, initImg.cols()/factor);
    
    //create the outputGeo - START
    GeoReference outputGeo = initGeo;

    Matrix<double> init_H;
    init_H = initGeo.transform();
    cout<<"init_H="<<init_H<<endl;
    
    Matrix<double> output_H;
    output_H = initGeo.transform();
    //lon = H(0,0)*i + 0*j + H(0,2)
    //lat = 0*i + H(1,1)*j + H(1,2)
   
    output_H(0,2) = init_H(0,2);
    output_H(1,2) = init_H(1,2);
    output_H(0,0) = factor*init_H(0,0);
    output_H(1,1) = factor*init_H(1,1);

    outputGeo.set_transform(output_H);
    //create the outputGeo - END

    for (int j = 0; j < outImg.rows(); j++){
      for (int i = 0; i < outImg.cols(); i++){

         Vector2 outputPix;
         outputPix(0) = i;
         outputPix(1) = j;
         Vector2 outLonLat = outputGeo.pixel_to_lonlat(outputPix);
	 
         Vector2 initPix;
         initPix = initGeo.lonlat_to_pixel(outLonLat);
       
         outImg.impl()(i,j) = interpInitImg.impl()(initPix(0), initPix(1));
      }
    }
       
 

    //write the corrected file
    write_georeferenced_image(outputFilename,
                              outImg,
                              outputGeo, TerminalProgressCallback("photometry","Processing:"));
    

}  

bool boxesOverlap(const Vector4 & box1Corners, const Vector4 & box2Corners){

  int lonOverlap = 0;
  int latOverlap = 0; 
  
  if (box1Corners(0) > box1Corners(1) || box2Corners(0) > box2Corners(1))
    {
      std::cout << "ERROR: Must never happen: " << __FILE__ << " at line " << __LINE__ << std::endl;
      exit(1);
    }
  
  if ( std::max(box1Corners(0), box2Corners(0)) < std::min(box1Corners(1), box2Corners(1)) )
    {
      lonOverlap = 1;
    }
  
  if (box1Corners(2) > box1Corners(3) || box2Corners(2) > box2Corners(3))
    {
      std::cout << "ERROR: Must never happen: " << __FILE__ << " at line " << __LINE__ << std::endl;
      exit(1);
    }
  
  if ( std::max(box1Corners(2), box2Corners(2)) < std::min(box1Corners(3), box2Corners(3)) )
    {
      latOverlap = 1;
    }
  
  return (lonOverlap == 1 && latOverlap == 1);
         
}
      

Vector4 ComputeGeoBoundary(GeoReference Geo, int width, int height)
{

  // Get the lonlat coordinates of the four pixels corners of the image.
  
  Vector4 corners;
  Vector2 leftTopPixel(0,0);
  Vector2 leftTopLonLat = Geo.pixel_to_lonlat(leftTopPixel);

  Vector2 rightBottomPixel(width-1, height-1);
  Vector2 rightBottomLonLat = Geo.pixel_to_lonlat(rightBottomPixel);
  
  float minLon = leftTopLonLat(0);
  float minLat = leftTopLonLat(1);
  float maxLon = rightBottomLonLat(0);
  float maxLat = rightBottomLonLat(1);

  if (maxLat<minLat){
     float temp = minLat;
     minLat = maxLat;
     maxLat = temp;    
  }

  if (maxLon<minLon){
     float temp = minLon;
     minLon = maxLon;
     maxLon = temp;    
  }

  corners(0) = minLon;
  corners(1) = maxLon;
  corners(2) = minLat;
  corners(3) = maxLat;

  return corners;
}

Vector4 getImageCorners(std::string imageFile){

  // Get the four corners of an image, that is the lon-lat coordinates of
  // the pixels in the image corners.

  // Note: Below we assume that the image is uint8. In fact, for the
  // purpose of calculation of corners the type of the image being
  // read does not matter.
  DiskImageView<PixelMask<PixelGray<uint8> > >  image(imageFile);
  
  GeoReference imageGeo;
  read_georeference(imageGeo, imageFile);
  Vector4 imageCorners = ComputeGeoBoundary(imageGeo, image.cols(), image.rows());
  return imageCorners;
}

void listTifsInDirOverlappingWithBox(const std::string & dirName,
                                     Vector4 & boxCorners,
                                     const std::string & outputListName){
  
  std::vector<std::string> tifsInDir;
  listTifsInDir(dirName, tifsInDir);

  ofstream fh(outputListName.c_str());
  if (!fh) {
    std::cerr << "ERROR: listTifsInDirOverlappingWithBox: can't open " << outputListName
              << " for writing" << std::endl;
    exit(1);
  }

  fh.precision(20);
  for (int fileIter = 0; fileIter < (int)tifsInDir.size(); fileIter++){
    const std::string & currFile = tifsInDir[fileIter];
    Vector4 currCorners = getImageCorners(currFile);
    if (!boxesOverlap(currCorners, boxCorners)) continue;
    fh << 1 << " " << currFile << " " << currCorners(0) << " " << currCorners(1) << " " <<
      currCorners(2) << " " << currCorners(3) << std::endl;
  }
  fh.close();

  return;
}

void createAlbedoTilesOverlappingWithDRG(double tileSize, int pixelPadding,
                                         std::string imageFile, Vector4 const& simulationBox,
                                         std::vector<ImageRecord> const& drgRecords,
                                         std::string blankTilesList,  std::string blankTilesDir,
                                         std::string DEMTilesList,    std::string meanDEMDir,
                                         std::string albedoTilesList, std::string albedoDir
                                         ){

  // Create all the tiles which overlap with all DRG images which in
  // turn overlap with the simulation box.

  // The georeference of tiles will be obtained from the georeference
  // of an input image (any one of those images would work as well as
  // any other).

  //  Write to disk both the tiles themselves and their list.

  // Note that the tiles have a padding, so they are a few pixels larger than what
  // they should be. We need that in order to be able to compute the normals
  // for  DEM, and also for SfS. The padding will be removed when at the end
  // of all computations we save the final albedo.

  if (tileSize <= 0.0 || pixelPadding < 0){
    std::cout << "ERROR: Must have positive tile size and non-negative pixel padding." << std::endl;
    exit(1);
  }

  // To do: it is more intuitive if one iterates from north to south than from south to north.

  // Find all tiles overlapping with given DRGs. Use a set to avoid duplicates.
  std::set< std::pair<double, double> > Tiles;
  for (int j = 0; j < (int)drgRecords.size(); j++){
    const ImageRecord& rec = drgRecords[j];
    Vector4 currCorners = Vector4(rec.west, rec.east, rec.south, rec.north);
    for (double min_y = tileSize*floor(rec.south/tileSize); min_y < rec.north; min_y += tileSize){
      for (double min_x = tileSize*floor(rec.west/tileSize); min_x < rec.east; min_x += tileSize){
        Tiles.insert(std::make_pair(min_y, min_x));
      }
    }
  }

  // The input DRG must be uint8
  enforceUint8Img(imageFile);
  
  GeoReference geo; read_georeference(geo, imageFile);
  ofstream fht(blankTilesList.c_str());  fht.precision(20);
  ofstream fhd(DEMTilesList.c_str());    fhd.precision(20);
  ofstream fha(albedoTilesList.c_str()); fha.precision(20);

  for (std::set< std::pair<double, double> >::iterator it = Tiles.begin(); it != Tiles.end(); it++){
    
    std::pair<double, double> Tile = *it;

    // Tile corners coordinates without padding
    double min_y = Tile.first;
    double min_x = Tile.second;
    double max_y = min_y + tileSize;
    double max_x = min_x + tileSize;
    
    // Tile corners coordinates with padding
    double min_x_padded, max_x_padded, min_y_padded, max_y_padded;
    applyPaddingToTileCorners(// Inputs
                               geo, pixelPadding, min_x, max_x,  min_y, max_y,  
                               // Outputs
                               min_x_padded, max_x_padded, min_y_padded, max_y_padded
                               );
  
    // Set the upper-left corner in the tile
    Matrix3x3 T = geo.transform();
    T(0,2) = min_x_padded;
    T(1,2) = max_y_padded;
    geo.set_transform(T);

    // Determine the size of the tile
    Vector2 pixUL = geo.lonlat_to_pixel(Vector2(min_x_padded, max_y_padded));
    // Note: The value we get for  pixUL is (-0.5, -0.5).
    // I was expecting (0, 0). (Oleg)
    Vector2 pixLR = geo.lonlat_to_pixel(Vector2(max_x_padded, min_y_padded));

    // To do: Below nrows and ncols may need to be interchanged.
    int nrows = (int)round(pixLR(0) - pixUL(0));
    int ncols = (int)round(pixLR(1) - pixUL(1));

    double uE = min_x, uN = max_y; // uppper-left corner without padding
    std::string sN = "N", sE = "E";
    if (uE < 0){ uE = -uE; sE = "W";}
    if (uN < 0){ uN = -uN; sN = "S";}
    ostringstream os;
    os << blankTilesDir << "/tile_" << uE << sE << uN << sN << ".tif";
    std::string blankTileFile = os.str();
    std::string DEMTileFile    = meanDEMDir + sufix_from_filename(blankTileFile);
    std::string albedoTileFile = albedoDir  + sufix_from_filename(blankTileFile);
    
    // The blank tiles themselves have no information, they are just
    // templates which we will later cycle through and create DRG
    // and tiles at each pixel.
    ImageView<PixelMask<PixelGray<float> > > blankTile(nrows, ncols);
      
    std::cout << "Writing " << blankTileFile << std::endl;
    write_georeferenced_image(blankTileFile,
                              channel_cast<uint8>(clamp(blankTile, 0.0,255.0)),
                              geo, TerminalProgressCallback("{Core}","Processing:"));

    // The actual corners of the tile may differ slightly than what we intended
    // due to rounding. Compute the actual corners now that the tile was created.
    Vector4 C = ComputeGeoBoundary(geo, blankTile.cols(), blankTile.rows());

    fht << 1 << " " << blankTileFile << " "
        << C(0) << " " << C(1) << " " << C(2) << " " << C(3) << std::endl;

    fha << 1 << " " << albedoTileFile << " "
        << C(0) << " " << C(1) << " " << C(2) << " " << C(3) << std::endl;

    fhd << 1 << " " << DEMTileFile << " "
        << C(0) << " " << C(1) << " " << C(2) << " " << C(3) << std::endl;

  } // End iterating over tiles
    
  fht.close();
  fhd.close();
  fha.close();
  
}

std::vector<int> GetInputIndices( std::vector<std::string> inputFiles, std::vector<std::string> DRGFiles)
{

  std::vector<int>  inputIndices;
  for (int j = 0; j < (int)inputFiles.size(); j++){
    for (int i = 0; i < (int)DRGFiles.size(); i++){
      if (DRGFiles[i].compare(inputFiles[j])==0){
       inputIndices.push_back(i);
      }
    }
  }

  return inputIndices;
}

void SaveOverlapIndices(string filename, std::vector<vector<int> > overlapIndicesArray)
{

}

std::vector<vector<int> > ReadOverlapIndices(string filename)
{
  std::vector<vector<int> > overlapIndicesArray;
  return overlapIndicesArray;
}

std::vector<int> ReadOverlapIndices(string filename, int i)
{
  std::vector<int>  overlapIndices;
  return overlapIndices;
}

       
//this function determines the image overlap for the general case
//it takes into consideration any set of overlapping images.
std::vector<int> makeOverlapList(const std::vector<ModelParams>& drgFiles,
                                 const std::string& currFile) {

  std::vector<int> overlapIndices; overlapIndices.clear();
  Vector4 currCorners = getImageCorners(currFile);

  //std::cout << "file " << currFile << " overlaps with ";
  for (unsigned int i = 0; i < drgFiles.size(); i++){

       const ModelParams& params = drgFiles[i];
       //std::cout << params.inputFilename << " ";

       Vector4 corners;
       if (params.corners(3) == ImageRecord::defaultCoord) {
         std::cerr << "ERROR: Missing the bounding box information for image: " << params.inputFilename
                   << std::endl;
         cerr << "This should have been specified in the list of images." << endl;
         exit(1);
         corners = getImageCorners(params.inputFilename);
         //std::cout << "Reading from disk: " << corners << std::endl;
       } else {
         corners = params.corners;
         //std::cout << "Cached from list: " << corners << std::endl;
       }

       if (boxesOverlap(corners, currCorners) && currFile != params.inputFilename){
         overlapIndices.push_back(i);
         //std::cout << params.inputFilename << " ";
       }
  }
  
  //std::cout << std::endl;
  return overlapIndices;
}

std::vector<int> makeOverlapList(const std::vector<ImageRecord>& drgRecords,
                                 const std::string& currFile) {

  // To do: Merge this function with the one above it and together with other
  // overlap logic seen in this file.
  std::vector<int> overlapIndices;
  Vector4 corners = getImageCorners(currFile);

  for (int j = 0; j < (int)drgRecords.size(); j++){
    const ImageRecord& rec = drgRecords[j];
    Vector4 currCorners = Vector4(rec.west, rec.east, rec.south, rec.north);
    if (! boxesOverlap(currCorners, corners)){
      continue;
    }
    overlapIndices.push_back(j);
  }

  return overlapIndices;
}

std::vector<vector<int> > makeOverlapList(const std::vector<std::string>& inputFiles,
                                          const std::vector<ModelParams>& DRGFiles) 
{
  std::vector<vector<int> > overlapIndices;
  overlapIndices.resize(inputFiles.size());
  
  for (unsigned int i = 0; i < inputFiles.size(); ++i) {  
      overlapIndices[i] = makeOverlapList(DRGFiles, inputFiles[i]);
  }
  return overlapIndices;  
}

void printOverlapList(std::vector<vector<int> > overlapIndices)
{
  
  for (int i=0; i < (int)overlapIndices.size(); i++){
    printf("%d: ", i);
    for (int j = 0; j < (int)overlapIndices[i].size(); j++){
      printf("%d ", overlapIndices[i][j]);
    }
    printf("\n");
  }
}



Vector4 parseSimBox(std::string simulationBoxStr){

  // Parse the string "13:49:-12:28" to extract the vector of
  // numbers 13, 49, -12, 28 (lonMin, lonMax, latMin, latMax).
  
  typedef boost::tokenizer<boost::char_separator<char> >  tokenizer;
  boost::char_separator<char> colon(":");
  tokenizer tokens(simulationBoxStr, colon);

  Vector4 simulationBox;
  int count = 0;
  for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter){
    std::string tok = *tok_iter;
    if (tok == "") continue;
    simulationBox(count) = atoi(tok.c_str());
    count++;
    if (count >= 4) break;
  }

  // If parsing did not succeed, then fail
  if (count < 4){
    cerr << "ERROR: Could not extract the simulation box from the string: " << simulationBoxStr << endl;
    exit(1);
  }

  if (simulationBox(0) >= simulationBox(1) || simulationBox(2) >= simulationBox(3)){
    std::cerr << "ERROR: Invalid simulationBox: " << simulationBox << std::endl;
    simulationBox = Vector4(0, 0, 0, 0);
  }

  // If we simulate the full sphere, we need to go beyond [-180, 180], since
  // the images can have pixels outside of this range.
  if (simulationBox(0) <= -180.0) simulationBox(0) = std::min(-360.0, simulationBox(0));
  if (simulationBox(1) >=  180.0) simulationBox(1) = std::max( 360.0, simulationBox(1));
  if (simulationBox(2) <= -180.0) simulationBox(2) = std::min(-360.0, simulationBox(2));
  if (simulationBox(3) >=  180.0) simulationBox(3) = std::max( 360.0, simulationBox(3));
  
  return simulationBox;
}

void extractSimBox(char * line, Vector4 & simulationBox){

  // Out of the string "SIMULATION_BOX            6 : 10 : -10 : -9 "
  // extract the value  "6 : 10 : -10 : -9", then parse it to extract
  // the individual numbers in a vector.
  
  istringstream is(line);
  std::string token, boxStr;

  // First token
  if ( !(is >> token) || token != "SIMULATION_BOX"){
    return;
  }

  // Subsequent tokens
  boxStr = "";
  while(is >> token){
    boxStr += token + " ";
  }

  simulationBox = parseSimBox(boxStr);
  
  return;
}

int ReadConfigFile(char *config_filename, struct GlobalParams *settings)
{
  int MAX_LENGTH = 5000;
  char line[MAX_LENGTH];
  char inName[MAX_LENGTH];
  char inVal[MAX_LENGTH];
  char *commentPos;
  ifstream configFile(config_filename);
  int ret;

  settings->tileSize     = 0;
  settings->useTiles     = 0;
  settings->pixelPadding = 0;
  settings->TRConst      = 1.24; //this will go into config file

  // Default simulation box, simulate the full sphere. We need to go
  // beyond [-180, 180], since the images can have pixels
  // outside of this range.
  settings->simulationBox = Vector4(-360.0, 360.0, -360.0, 360.0);

#define CHECK_VAR(name, fmt, assignTo)             \
  if (0 == strcmp(inName, name)) {                 \
        sscanf(inVal, fmt, &(settings->assignTo)); \
    }
  
#define CHECK_STR(name, fmt, assignTo)          \
  if (0 == strcmp(inName, name)) {              \
    sscanf(inVal, fmt, settings->assignTo);     \
  }
  
  if (!configFile.is_open()){
    std::cout << "ERROR: Config file " << config_filename << " not found."<< std::endl;
    exit(1);
  }
  
  printf("CONFIG FILE FOUND\n");
  
  while (!configFile.eof()) {
    configFile.getline(line, MAX_LENGTH);
    
    // truncate comments
    commentPos = strchr(line, '#');
    if (NULL != commentPos) {
      *commentPos = '\0';
    }

    // The simulation box must be handled separately and before
    // other items.
    extractSimBox(line, settings->simulationBox);
    
    ret = sscanf(line, "%s %s\n", inName, inVal);
    if (2 != ret) continue;

    // Files/directories
    CHECK_STR("DRG_DIR",                  "%s", drgDir);
    CHECK_STR("DEM_DIR",                  "%s", demDir);
    CHECK_STR("SUN_POSITION_FILE",        "%s", sunPosFile);
    CHECK_STR("SPACECRAFT_POSITION_FILE", "%s", spacecraftPosFile);

    // Constants
    CHECK_VAR("USE_TILES",                "%d", useTiles);
    CHECK_VAR("TILE_SIZE",                "%f", tileSize);
    CHECK_VAR("PIXEL_PADDING",            "%d", pixelPadding);
    CHECK_VAR("REFLECTANCE_TYPE",         "%d", reflectanceType);
    CHECK_VAR("SHADOW_THRESH",            "%f", shadowThresh);
    CHECK_VAR("MAX_NUM_ITER",             "%d", maxNumIter);
    CHECK_VAR("USE_WEIGHTS",              "%d", useWeights);
    //CHECK_VAR("COMPUTE_ERRORS",         "%d", computeErrors); // handled via cmd-line option

    // Parameters controlling the flow
    CHECK_VAR("INITIAL_SETUP",            "%d", initialSetup);
    CHECK_VAR("SAVE_WEIGHTS",             "%d", saveWeights);
    CHECK_VAR("SHADOW_INIT_TYPE",         "%d", shadowInitType);
    CHECK_VAR("DEM_INIT_TYPE",            "%d", DEMInitType);
    CHECK_VAR("EXPOSURE_INIT_TYPE",       "%d", exposureInitType);
    CHECK_VAR("ALBEDO_INIT_TYPE",         "%d", albedoInitType);
    CHECK_VAR("UPDATE_EXPOSURE",          "%d", updateExposure);
    CHECK_VAR("UPDATE_ALBEDO",            "%d", updateAlbedo);
    CHECK_VAR("UPDATE_HEIGHT",            "%d", updateHeight);
    
    // Potentially obsolete
    CHECK_VAR("NO_DEM_DATA_VAL",          "%d", noDEMDataValue);
    CHECK_VAR("SAVE_REFLECTANCE",         "%d", saveReflectance);
    CHECK_VAR("SLOPE_TYPE",               "%d", slopeType);
  }

  configFile.close();

  // Validation
  if ( !fs::exists(settings->drgDir) ){
    std::cerr << "ERROR: Directory " << settings->drgDir << " does not exist." << std::endl;
    exit(1);
  }
  if ( !fs::exists(settings->demDir) ){
    std::cerr << "ERROR: Directory " << settings->demDir << " does not exist." << std::endl;
    exit(1);
  }
  if ( !fs::exists(settings->sunPosFile) ){
    std::cerr << "ERROR: File " << settings->sunPosFile << " does not exist." << std::endl;
    exit(1);
  }
  if ( !fs::exists(settings->spacecraftPosFile) ){
    std::cerr << "ERROR: File " << settings->spacecraftPosFile << " does not exist." << std::endl;
    exit(1);
  }
  if (settings->useTiles != 0 && settings->tileSize <= 0.0){
    std::cerr << "ERROR: The tile size must be positive." << std::endl;
    exit(1);
  }
  if (settings->useTiles != 0 && settings->pixelPadding < 0){
    std::cerr << "ERROR: The pixel padding must be non-negative." << std::endl;
    exit(1);
  }
  if (settings->shadowThresh < 0.0 || settings->shadowThresh > 255.0){
    std::cerr << "ERROR: The shadow threshold must be between 0 and 255." << std::endl;
    exit(1);
  }
    
  return 0;
}

void PrintGlobalParams(struct GlobalParams *settings)
{
  // Files/directories
  printf("DRG_DIR                  %s\n", settings->drgDir);
  printf("DEM_DIR                  %s\n", settings->demDir);
  printf("SUN_POSITION_FILE        %s\n", settings->sunPosFile);
  printf("SPACECRAFT_POSITION_FILE %s\n", settings->spacecraftPosFile);

  // Constants
  printf("USE_TILES                %d\n", settings->useTiles);
  printf("TILE_SIZE                %f\n", settings->tileSize);
  printf("PIXEL_PADDING            %d\n", settings->pixelPadding);
  std::cout << "SIMULATION_BOX           " << settings->simulationBox << std::endl;
  printf("REFLECTANCE_TYPE         %d\n", settings->reflectanceType);
  printf("SHADOW_THRESH            %f\n", settings->shadowThresh);
  printf("MAX_NUM_ITER             %d\n", settings->maxNumIter);
  printf("USE_WEIGHTS              %d\n", settings->useWeights);
  printf("TR_CONST                 %f\n", settings->TRConst);
  //printf("COMPUTE_ERRORS         %d\n", settings->computeErrors); // handled via cmd-line option

  // Parameters controlling the flow
  printf("INITIAL_SETUP            %d\n", settings->initialSetup);
  printf("SAVE_WEIGHTS             %d\n", settings->saveWeights);
  printf("SHADOW_INIT_TYPE         %d\n", settings->shadowInitType);
  printf("DEM_INIT_TYPE            %d\n", settings->DEMInitType);
  printf("EXPOSURE_INIT_TYPE       %d\n", settings->exposureInitType);
  printf("ALBEDO_INIT_TYPE         %d\n", settings->albedoInitType);
  printf("UPDATE_EXPOSURE          %d\n", settings->updateExposure);
  printf("UPDATE_ALBEDO            %d\n", settings->updateAlbedo);
  printf("UPDATE_HEIGHT            %d\n", settings->updateHeight);

  // Potentially obsolete
  printf("NO_DEM_DATA_VAL          %d\n", settings->noDEMDataValue);
  printf("SAVE_REFLECTANCE         %d\n", settings->saveReflectance);
  printf("SLOPE_TYPE               %d\n", settings->slopeType);
}
bool readImagesFile(std::vector<ImageRecord>& images,
                    const std::string& imagesListName)
{
  std::ifstream imagesList(imagesListName.c_str());
  if (!imagesList) {
    std::cerr << "ERROR: readImagesFile: can't open " << imagesListName
              << " for reading: " << strerror(errno) << std::endl;
    return false;
  }

  images.clear();

  std::string line;
  int lineNo = 1;
  while (std::getline(imagesList, line)) {
    char path[1024];
    int useImage;
    ImageRecord rec;

    // ignore blank lines
    if (line.size() == 0) continue;
    // ignore comment lines
    if ('#' == line[0]) continue;

    if (6 != sscanf(line.c_str(), "%d %1023s %lf %lf %lf %lf",
                    &useImage,
                    path,
                    &rec.west,
                    &rec.east,
                    &rec.south,
                    &rec.north
                    )
        ) {
      std::cerr << "ERROR: readImagesFile: " << imagesListName
                << ": line " << lineNo
                << ": expected '%d %s %f %f %f %f' format" << std::endl;
      return false;
    }
    rec.useImage = useImage;
    rec.path = path;
    
    images.push_back(rec);

    lineNo++;
  }

  return true;
}

void list_DRG_in_box_and_all_DEM(bool useTiles,
                                 std::string allDRGIndex, std::string allDEMIndex,
                                 Vector4 simulationBox, 
                                 std::string DRGDir,  std::string DEMDir, 
                                 std::string DRGInBoxList
                                 ){

  // Create the lists of ALL DRG and DEM images in DRGDir and
  // DEMDir, if these lists don't exist already.
  
  // Create the list of all DRG files intersecting the current simulationBox.

  Vector4 bigBox = Vector4(-360.0, 360.0, -360.0, 360.0);

  // Create the index of all DRG images if it does not exist already.
  std::vector<ImageRecord> imageRecords;
  if (!readImagesFile(imageRecords, allDRGIndex)){
    std::cout << "WILL create the file " << allDRGIndex << std::endl;
    listTifsInDirOverlappingWithBox(DRGDir, bigBox, allDRGIndex);
    if (!readImagesFile(imageRecords, allDRGIndex)) exit(1); // Second attempt at reading
  }

  // Create the list of all DRG files intersecting the current box.
  ofstream fh(DRGInBoxList.c_str());
  if (!fh){
    std::cerr << "ERROR: list_DRG_in_box_and_all_DEM: can't open " << DRGInBoxList
              << " for writing" << std::endl;
    exit(1);
  }
  fh.precision(20);
  for (int j = 0; j < (int)imageRecords.size(); j++){
    const ImageRecord& rec = imageRecords[j];
    Vector4 currCorners = Vector4(rec.west, rec.east, rec.south, rec.north);
    if (! boxesOverlap(currCorners, simulationBox)) continue;
    fh << rec.useImage << " " << rec.path << " "
       << currCorners(0) << " " << currCorners(1) << " "
       << currCorners(2) << " " << currCorners(3) << std::endl;
  }
  fh.close();

  if (!useTiles){
    return;
  }
  
  // Create the index of all DEM tiles if it does not exist already.
  std::vector<ImageRecord> DEMTilesRecords;
  if (!readImagesFile(DEMTilesRecords, allDEMIndex)){
    listTifsInDirOverlappingWithBox(DEMDir, bigBox, allDEMIndex);
  }

  return;
}

int main( int argc, char *argv[] ) {

  for (int s = 0; s < argc; s++) std::cout << argv[s] << " ";
  std::cout << std::endl;

  std::vector<std::string> inputDRGFiles;
  std::vector<std::string> DRGFiles;
  std::vector<std::string> imageFiles;
  std::string resDir              = "results";
  std::string configFilename      = "photometry_settings.txt";
  std::string DRGInBoxList        = "";
  
  po::options_description general_options("Options");
  general_options.add_options()
    ("image-files,i", po::value<std::vector<std::string> >(&imageFiles), "Image files.")
    ("res-directory,r", po::value<std::string>(&resDir)->default_value("results"), "Results directory.")
    ("images-list,f", po::value<std::string>(&DRGInBoxList)->default_value(DRGInBoxList), "Path to file listing images to use.")
    ("config-filename,c", po::value<std::string>(&configFilename)->default_value("photometry_settings.txt"), "Configuration filename.")
    ("initial-setup", "Initial setup")
    ("save-weights", "Save the weights")
    ("compute-shadow", "Compute the shadow")
    ("init-dem",  "Initialize the DEM")
    ("init-exposure", "Initialize the exposure")
    ("init-albedo", "Initialize the albedo")
    ("update-exposure", "Update the exposure")
    ("update-albedo", "Update the albedo")
    ("update-height", "Update the height (shape from shading)")
    ("compute-errors", "Compute the errors in albedo")
    ("is-last-iter", "Is this the last iteration")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");

  hidden_options.add_options()
    ("inputDRGFiles", po::value<std::vector<std::string> >(&inputDRGFiles));
 
  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("inputDRGFiles", -1);

  std::ostringstream usage;
  usage << "Description: main code for mosaic, albedo and shape reconstruction from multiple images" << std::endl << std::endl;
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch ( po::error const& e ) {
    std::cout << "An error occured while parsing command line arguments.\n";
    std::cout << "\t" << e.what() << "\n\n";
    std::cout << usage.str();
    return 1;
  }

  if( vm.count("help") ) {
    std::cerr << usage.str() << std::endl;
    return 1;
  }

  // Read the global parameters settings. Apply the command-line overrides.
  GlobalParams globalParams;
  ReadConfigFile((char*)configFilename.c_str(), &globalParams);
  if ( vm.count("initial-setup"  ) ) globalParams.initialSetup     = true;
  if ( vm.count("save-weights"   ) ) globalParams.saveWeights      = true;
  if ( vm.count("compute-shadow" ) ) globalParams.shadowInitType   = true;
  if ( vm.count("init-dem"       ) ) globalParams.DEMInitType      = true;
  if ( vm.count("init-exposure"  ) ) globalParams.exposureInitType = true;
  if ( vm.count("init-albedo"    ) ) globalParams.albedoInitType   = true;
  if ( vm.count("update-exposure") ) globalParams.updateExposure   = true;
  if ( vm.count("update-albedo"  ) ) globalParams.updateAlbedo     = true;
  if ( vm.count("update-height"  ) ) globalParams.updateHeight     = true;
  PrintGlobalParams(&globalParams);

  bool computeErrors = vm.count("compute-errors");
  bool isLastIter    = vm.count("is-last-iter");

  // Validation
  // To do: Make this stronger
  if ( (int)globalParams.albedoInitType  + (int)globalParams.updateAlbedo + (int)computeErrors >= 2 ){
    std::cerr << "ERROR: Cannot do more than one of the following three operations at a time: "
              << "Initialize the albedo, update the albedo, compute the errors." << std::endl;
    exit(1);
  }
  
  // Double check to make sure all folders exist  
  if ( !fs::exists(resDir) )
    fs::create_directories(resDir);
  if ( !fs::exists(resDir+"/info") )
    fs::create_directories(resDir+"/info");
  if ( !fs::exists(resDir+"/reflectance") )
    fs::create_directories(resDir+"/reflectance");
  if ( !fs::exists(resDir+"/shadow") )
    fs::create_directories(resDir+"/shadow");
  if ( !fs::exists(resDir+"/exposure") )
    fs::create_directories(resDir+"/exposure");
  if ( !fs::exists(resDir+"/weight") )
    fs::create_directories(resDir+"/weight");
  std::string albedoDir  = resDir + "/albedo";
  std::string meanDEMDir = resDir + "/DEM";
  std::string costFunDir = resDir + "/costFun";
  std::string errorDir   = resDir + "/error";
  std::string sfsDir     = resDir + "/DEM_sfs";
  if ( !fs::exists(albedoDir)  ) fs::create_directories(albedoDir);
  if ( !fs::exists(meanDEMDir) ) fs::create_directories(meanDEMDir);
  if ( !fs::exists(costFunDir) ) fs::create_directories(costFunDir);
  if ( !fs::exists(errorDir)   ) fs::create_directories(errorDir);
  if ( !fs::exists(sfsDir)     ) fs::create_directories(sfsDir);
  
  // blankTilesDir is used to create a tile with identical dimensions
  // as the subsequent DEM and albedo tiles.
  std::string blankTilesDir  = resDir + "/blank_tiles";
  if (globalParams.useTiles){
    if ( !fs::exists(blankTilesDir)  ) fs::create_directories(blankTilesDir);
  }
  
  // The names of the files listing all DRGs and DEMs and the coordinates
  // of their corners. 
  std::string allDRGIndex     = std::string(globalParams.drgDir) + "/index.txt";
  std::string allDEMIndex     = std::string(globalParams.demDir) + "/index.txt";
  std::string blankTilesList  = resDir + "/blankTilesList.txt";
  std::string DEMTilesList    = resDir + "/DEMTilesList.txt";
  std::string albedoTilesList = resDir + "/albedoTilesList.txt";

  if (globalParams.initialSetup == 1) {

    // This block of code creates the list of images (DRGInBoxList). As such, it must be above
    // any code which reads the list of images.
    // Create the DRGInBoxList used in subsequent iterations.
    // Create the list of all DEM if not there yet.
    list_DRG_in_box_and_all_DEM(globalParams.useTiles,
                                allDRGIndex, allDEMIndex,
                                globalParams.simulationBox, globalParams.drgDir, globalParams.demDir, DRGInBoxList
                                );
    
    if (globalParams.useTiles){
      vw_out( VerboseDebugMessage, "photometry" ) << "Initializing the albedo tiles ... ";
      if (imageFiles.size() == 0){
        std::cerr << "ERROR: Expecting an image file as input, the -i option" << std::endl;
        return 1;
      }
      std::vector<ImageRecord> drgRecords;
      if (!readImagesFile(drgRecords, DRGInBoxList)) exit(1);
      std::string imageFile = imageFiles[0]; // an image whose georef we will use
      createAlbedoTilesOverlappingWithDRG(globalParams.tileSize, globalParams.pixelPadding,
                                          imageFile, globalParams.simulationBox,
                                          drgRecords,
                                          blankTilesList,  blankTilesDir,
                                          DEMTilesList,    meanDEMDir,
                                          albedoTilesList, albedoDir
                                          );
    }
    
  }


  std::vector<ImageRecord> drgRecords;
  if( DRGInBoxList.size() == 0 ) {
    if ( vm.count("inputDRGFiles") < 1 ) {
      std::cerr << "ERROR: Must specify either the -f option or at least one orthoprojected image file!" << std::endl << std::endl;
      std::cerr << usage.str();
      return 1;
    }

    for (int i=0; i < (int)inputDRGFiles.size(); i++) {
      drgRecords.push_back(ImageRecord(inputDRGFiles[i]));
    }
  } else {
    if (!readImagesFile(drgRecords, DRGInBoxList)) exit(1);
  }

  std::map<std::string, Vector3> sunPositions;
  ReadSunOrSpacecraftPosition(globalParams.sunPosFile, // Input
                              sunPositions             // Output
                              );
  
  std::map<std::string, Vector3> spacecraftPositions;
  ReadSunOrSpacecraftPosition(globalParams.spacecraftPosFile, // Input
                              spacecraftPositions             // Output
                              );

  std::vector<ModelParams> modelParamsArray;

  // In order to find the corresponding DEM for a given DRG, we take advantage
  // of the fact that the first 11 characters of these files are always the same.
  std::map<std::string, std::string> DEMFilesIndex;
  if (!globalParams.useTiles) indexFilesByKey(globalParams.demDir, DEMFilesIndex);
  
  //this will contain all the DRG files
  int i = 0;
  for (unsigned int j = 0; j < drgRecords.size(); ++j) {
    if (!drgRecords[j].useImage) continue;

    DRGFiles.push_back(drgRecords[j].path);
    modelParamsArray.push_back(ModelParams());

    std::string temp = sufix_from_filename(DRGFiles[i]);
    modelParamsArray[i].exposureTime     = 1.0;
    modelParamsArray[i].hCenterLineDEM   = NULL;
    modelParamsArray[i].hCenterLine      = NULL;
    modelParamsArray[i].hMaxDistArray    = NULL;
    modelParamsArray[i].hMaxDistArrayDEM = NULL;
    modelParamsArray[i].inputFilename    = DRGFiles[i];//these filenames have full path

    std::string prefix = getFirstElevenCharsFromFileName(DRGFiles[i]);

    if ( sunPositions.find(prefix) == sunPositions.end()){
      std::cerr << "Could not find the sun position for the DRG file: " << DRGFiles[i] << std::endl;
      exit(1);
    }
    modelParamsArray[i].sunPosition = 1000*sunPositions[prefix];
    
    if (spacecraftPositions.find(prefix) == spacecraftPositions.end()){
      std::cerr << "Could not find the spacecraft position for the DRG file: " << DRGFiles[i] << std::endl;
      exit(1);
    }
    modelParamsArray[i].spacecraftPosition = 1000*spacecraftPositions[prefix];
  
    // Find the corresponding DEM for the given DRG. See the earlier note.
    if (!globalParams.useTiles){
      if (DEMFilesIndex.find(prefix) == DEMFilesIndex.end()){
        std::cerr << "Could not find a DEM for the DRG file: " << DRGFiles[i] << std::endl;
        exit(1);
      }
      modelParamsArray[i].DEMFilename     = DEMFilesIndex[prefix];
      modelParamsArray[i].meanDEMFilename = resDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM_out.tif";   
      modelParamsArray[i].var2DEMFilename = resDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM_var2.tif";
    }
    
    modelParamsArray[i].infoFilename        = resDir + "/info/" + prefix_less3_from_filename(temp)+"info.txt";
    modelParamsArray[i].reliefFilename      = resDir + "/reflectance" + prefix_from_filename(temp) + "_reflectance.tif";
    modelParamsArray[i].shadowFilename      = resDir + "/shadow/" + prefix_from_filename(temp) + "_shadow.tif";
    modelParamsArray[i].errorFilename       = resDir + "/error" + prefix_from_filename(temp) + "_err.tif";
    modelParamsArray[i].outputFilename      = resDir + "/albedo" + prefix_from_filename(temp) + "_albedo.tif";
    modelParamsArray[i].sfsDEMFilename      = resDir + "/DEM_sfs" + prefix_less3_from_filename(temp) + "DEM_sfs.tif";
    modelParamsArray[i].errorHeightFilename = resDir + "/error" + prefix_from_filename(temp) + "_height_err.tif";
    modelParamsArray[i].weightFilename      = resDir + "/weight" + prefix_from_filename(temp) + "_weight.txt";
    modelParamsArray[i].exposureFilename    = resDir + "/exposure" + prefix_from_filename(temp) + "_exposure.txt";
  
    const ImageRecord& rec = drgRecords[j];
    modelParamsArray[i].corners = Vector4(rec.west, rec.east, rec.south, rec.north);

    modelParamsArray[i].hCenterLine      = NULL;
    modelParamsArray[i].hMaxDistArray    = NULL;
    modelParamsArray[i].hCenterLineDEM   = NULL;
    modelParamsArray[i].hMaxDistArrayDEM = NULL;
    modelParamsArray[i].vCenterLine      = NULL;
    modelParamsArray[i].vMaxDistArray    = NULL;
    modelParamsArray[i].vCenterLineDEM   = NULL;
    modelParamsArray[i].vMaxDistArrayDEM = NULL;

    vw_out( VerboseDebugMessage, "photometry" ) << modelParamsArray[i] << "\n";

    i++;
  }

  if (globalParams.initialSetup == 1){
    // We performed all tasks, including validation, if we are in the initial setup.
    return 0;
  }
    
    
  vw_out() << "Number of Files = " << DRGFiles.size() << "\n";
  if (imageFiles.size() == 0){
    imageFiles = DRGFiles;
  }
  
  std::vector<vector<int> > overlapIndicesArray = makeOverlapList(imageFiles, modelParamsArray);
  printOverlapList(overlapIndicesArray);

  std::vector<int> inputIndices = GetInputIndices(imageFiles, DRGFiles);
  
  // set up weights only for images that we want to process or that
  // overlap one of the files we want to process
  std::vector<int> relevantIndices;
  for (unsigned int i=0; i < inputIndices.size(); i++) {
    relevantIndices.push_back(inputIndices[i]);
  }
  for (unsigned int i=0; i < overlapIndicesArray.size(); i++) {
    const std::vector<int>& ov = overlapIndicesArray[i];
    for (unsigned int j=0; j < ov.size(); j++) {
      relevantIndices.push_back(ov[j]);
    }
  }

  // Read the exposure information for the relevant indices
  for (unsigned int i=0; i < relevantIndices.size(); i++) {
    int j = relevantIndices[i];
    ReadExposureInfoFromFile(&(modelParamsArray[j]));
  }
  
  if (globalParams.useWeights == 1) {
    vw_out( VerboseDebugMessage, "photometry" ) << "Computing weights ... ";
    
    for (unsigned int i=0; i < relevantIndices.size(); i++) {
      int j = relevantIndices[i];

      // If we are not in weight saving mode, the weights should be on
      // disk already, so read them.
      if (globalParams.saveWeights != 1){
        if  (!globalParams.useTiles){
          ReadWeightsParamsFromFile(globalParams.useTiles, &modelParamsArray[j]);
          if (modelParamsArray[j].hCenterLine == NULL){
            cerr << "ERROR: Weights not found on disk for image: " << modelParamsArray[j].inputFilename << endl;
            exit(1);
          }
        }
        continue;
      }
      
      // We have globalParams.saveWeights == 1. Build the weights.
      if ((int)inputIndices.size() == 0){
        cerr << "Error: Could not find the image to process: " << imageFiles[0] << " in the list of input images." << endl;
        exit(1);
      }
      
      if (j == inputIndices[0]){
        // Compute and save the weights only for the current image,
        // not for all images overlapping with it.

        ComputeImageCenterLines(modelParamsArray[j]);

        if (!globalParams.useTiles){
          modelParamsArray[j].hCenterLineDEM = ComputeDEMHCenterLine(modelParamsArray[j].DEMFilename,
                                                                     globalParams.noDEMDataValue,
                                                                     &(modelParamsArray[j].hMaxDistArrayDEM));
          modelParamsArray[j].vCenterLineDEM = ComputeDEMVCenterLine(modelParamsArray[j].DEMFilename,
                                                                     globalParams.noDEMDataValue,
                                                                     &(modelParamsArray[j].vMaxDistArrayDEM));
        }
        
        if (globalParams.saveWeights == 1){
          SaveWeightsParamsToFile(globalParams.useTiles, modelParamsArray[j]);
        }
      }
      
    }
    
    vw_out( VerboseDebugMessage, "photometry" ) << "Done.\n";
  }
    
  if (globalParams.shadowInitType == 1 && (!globalParams.useTiles)){
    TerminalProgressCallback callback("photometry","Init Shadow:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); i++){ 
      callback.report_progress(float(i)/float(imageFiles.size()));
      ComputeSaveShadowMap (modelParamsArray[inputIndices[i]], globalParams);
    }
    callback.report_finished();
  }

  if ( (globalParams.DEMInitType == 1) && (globalParams.reflectanceType != NO_REFL) ){

    TerminalProgressCallback callback("photometry","Init DEM:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {
      callback.report_progress(float(i)/float(imageFiles.size()));

      if (!globalParams.useTiles){
        std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
        for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
        }
        if ((globalParams.useWeights == 1) && (modelParamsArray[inputIndices[i]].hCenterLineDEM == NULL)){
          ReadWeightsParamsFromFile(globalParams.useTiles, &modelParamsArray[inputIndices[i]]);
        }
        InitDEM(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
        
      }else{
        std::string blankTileFile = imageFiles[i];
        std::string DEMTileFile   = meanDEMDir + sufix_from_filename(blankTileFile);
        std::vector<ImageRecord> DEMImages;
        if (!readImagesFile(DEMImages, allDEMIndex)) exit(1);
        std::vector<int> overlap = makeOverlapList(DEMImages, blankTileFile);

        InitMeanDEMTile(blankTileFile, DEMTileFile, 
                        DEMImages, overlap, globalParams
                        );
      }
      
    }
    callback.report_finished();
  }
  
  std::vector<float> avgReflectanceArray(DRGFiles.size());
  if ( (globalParams.reflectanceType != NO_REFL) && (globalParams.exposureInitType == 1) ){ //compute reflectance
    
    TerminalProgressCallback callback("photometry","Init Exposure Time:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {
      
      callback.report_progress(float(i)/float(imageFiles.size()));
      vw_out(VerboseDebugMessage,"photometry") << modelParamsArray[inputIndices[i]].reliefFilename << "\n";

      //TO DO: check to see that file exists
      //TO DO: if file does not exist compute.

      if (globalParams.useTiles){
        // Compute the average reflectance image.
        // The same code is used below to update the exposure.
        std::string curDRG = imageFiles[i];
        std::vector<ImageRecord> DEMTiles, albedoTiles;
        std::vector<int> overlap;
        if (!readImagesFile(DEMTiles,    DEMTilesList))    exit(1);
        if (!readImagesFile(albedoTiles, albedoTilesList)) exit(1);
        overlap = makeOverlapList(DEMTiles, curDRG);
        bool compAvgRefl    = true;
        bool useReflectance = true;
        avgReflectanceArray[i]
          = computeAvgReflectanceOverTilesOrUpdateExposure(compAvgRefl, useReflectance,
                                                           globalParams.pixelPadding, globalParams.tileSize,
                                                           DEMTiles, albedoTiles, 
                                                           overlap,
                                                           modelParamsArray[inputIndices[i]],
                                                           globalParams);
        
      }else{
        //compute and save the reflectance image.
        avgReflectanceArray[i] = computeImageReflectance(modelParamsArray[inputIndices[i]],
                                                         globalParams);
      }
      
      modelParamsArray[inputIndices[i]].exposureTime = globalParams.TRConst/avgReflectanceArray[i];
     
      vw_out(VerboseDebugMessage,"photometry") << "\tExposure Time = "
        << modelParamsArray[inputIndices[i]].exposureTime
        << "\n";
      
      SaveExposureInfoToFile(modelParamsArray[inputIndices[i]]);
    }
    callback.report_finished();
  }

  if (globalParams.albedoInitType == 1){

    TerminalProgressCallback callback("photometry","Init Albedo:");
    callback.report_progress(0);

    for (unsigned int i = 0; i < imageFiles.size(); ++i){

      callback.report_progress(float(i)/float(imageFiles.size()));
           
      std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
      for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
        overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
      }

      bool useReflectance = (globalParams.reflectanceType != NO_REFL); 

      if (!globalParams.useTiles){
        
        if (!useReflectance) {
          InitImageMosaicByBlocks(modelParamsArray[inputIndices[i]],
                                  overlapParamsArray, globalParams);
        } else {
          
          InitAlbedoMosaic(modelParamsArray[inputIndices[i]],
                           overlapParamsArray, globalParams);
        }
        
      }else{
        
        // This code is repeated below where we update the albedo
        std::string blankTileFile  = imageFiles[i];
        Vector4 tileCorners = getImageCorners(blankTileFile);
        if ( isLastIter && !boxesOverlap(tileCorners, globalParams.simulationBox)){
          // If this is not the last albedo iteration, we must init/update the exposure
          // and albedo for all tiles. Otherwise it is enough to do it only
          // for the tiles which overlap with the sim box.
          std::cout << "Skipping tile: "
                    << blankTileFile << " as it does not overlap with the simulation box." << std::endl;
          continue;
        }
        
        std::string DEMTileFile    = meanDEMDir + sufix_from_filename(blankTileFile);
        std::string albedoTileFile = albedoDir  + sufix_from_filename(blankTileFile);
        std::string errorTileFile  = errorDir   + sufix_from_filename(blankTileFile);
        bool initTile = true; // init, rather than update
        InitOrUpdateAlbedoOrComputeErrors(initTile, isLastIter, computeErrors, useReflectance,
                                          globalParams.pixelPadding, globalParams.tileSize,
                                          blankTileFile, DEMTileFile, albedoTileFile, errorTileFile,
                                          overlapParamsArray, globalParams);
        
      }
      callback.report_finished();
    }
  }


  if (globalParams.updateExposure == 1){ //re-estimate the exposure time
    
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {   

      bool useReflectance = (globalParams.reflectanceType != NO_REFL); 
        
      if (!globalParams.useTiles){
          
        if (!useReflectance){
          //no use of reflectance map
          ComputeExposure(&modelParamsArray[inputIndices[i]], globalParams);
        }else{
          //use reflectance map
          ComputeExposureAlbedo(&modelParamsArray[inputIndices[i]], globalParams);
        }
          
      } else{
          
        // Update the exposure based on tiles
        std::string curDRG = imageFiles[i];
        std::vector<ImageRecord> DEMTiles, albedoTiles;
        std::vector<int> overlap;
        if (!readImagesFile(DEMTiles,    DEMTilesList))    exit(1);
        if (!readImagesFile(albedoTiles, albedoTilesList)) exit(1);
        overlap = makeOverlapList(DEMTiles, curDRG);
        bool compAvgRefl = false; // set this flag to false to update the exposure
        double exposure = computeAvgReflectanceOverTilesOrUpdateExposure(compAvgRefl,
                                                                         useReflectance,
                                                                         globalParams.pixelPadding,
                                                                         globalParams.tileSize,
                                                                         DEMTiles, albedoTiles, 
                                                                         overlap,
                                                                         modelParamsArray[inputIndices[i]],
                                                                         globalParams);
        modelParamsArray[inputIndices[i]].exposureTime = exposure;
          
      }
        
      //create the exposureInfoFilename
      SaveExposureInfoToFile(modelParamsArray[inputIndices[i]]);
    }
  }

  if (globalParams.updateAlbedo || computeErrors){

    for (unsigned int i = 0; i < imageFiles.size(); ++i) {

      std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
      for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
        overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
      }

      bool useReflectance = (globalParams.reflectanceType != NO_REFL); 
        
      if (!globalParams.useTiles){
        if (!useReflectance){
          //no use of the reflectance map
          UpdateImageMosaic( modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
        }else{
          //use reflectance
          UpdateAlbedoMosaic(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
        }
        
      }else{
        // We use the same logic as for initializing the albedo above. What is different now
        // is that we work with the updated exposure.
        std::string blankTileFile  = imageFiles[i];
        Vector4 tileCorners = getImageCorners(blankTileFile);
        if (isLastIter && !boxesOverlap(tileCorners, globalParams.simulationBox)){
          // If this is not the last albedo iteration, we must init/update the exposure
          // and albedo for all tiles. Otherwise it is enough to do it only
          // for the tiles which overlap with the sim box.
          std::cout << "Skipping tile: "
                    << blankTileFile << " as it does not overlap with the simulation box." << std::endl;
          continue;
        }
        std::string DEMTileFile    = meanDEMDir + sufix_from_filename(blankTileFile);
        std::string albedoTileFile = albedoDir  + sufix_from_filename(blankTileFile);
        std::string errorTileFile  = errorDir   + sufix_from_filename(blankTileFile);
        bool initTile = false; // will update or compute error, not init
        double costFunVal = InitOrUpdateAlbedoOrComputeErrors(initTile, isLastIter,
                                                              computeErrors,
                                                              useReflectance,
                                                              globalParams.pixelPadding,
                                                              globalParams.tileSize,
                                                              blankTileFile, DEMTileFile,
                                                              albedoTileFile, errorTileFile,
                                                              overlapParamsArray, globalParams);
        std::string costFunFile = costFunDir
          + prefix_from_filename(sufix_from_filename(albedoTileFile)) + ".txt";
        //std::cout << "cost fun file is " << costFunFile << std::endl;
        AppendCostFunToFile(costFunVal, costFunFile);
      }
    }
  }
        
  //re-estimate the height map  - shape from shading
  if ((globalParams.reflectanceType != NO_REFL) && (globalParams.updateHeight == 1)){

    for (unsigned int i = 0; i < imageFiles.size(); ++i) {

      std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
      for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
        overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
      }

      if (globalParams.useTiles){      
        std::string blankTileFile  = imageFiles[i];
        std::string DEMTileFile    = meanDEMDir + sufix_from_filename(blankTileFile);
        std::string albedoTileFile = albedoDir  + sufix_from_filename(blankTileFile);
        std::string sfsTileFile    = sfsDir  + sufix_from_filename(blankTileFile);
        UpdateHeightMapTiles(DEMTileFile, albedoTileFile, sfsTileFile,
                             overlapParamsArray, globalParams);
      }else{
        UpdateHeightMap(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
      }
      
    }
  }
    
    
  return 0;
}

