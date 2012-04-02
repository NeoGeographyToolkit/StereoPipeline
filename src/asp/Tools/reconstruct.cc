// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// To do: Remove the file reconstruct_aux.cc which mostly duplicates orthoproject.cc.
// Study the bug below.
// ~/StereoPipeline/src/asp/Tools/reconstruct --drg-directory ./albedo_all/./DIM_input_sub64 --dem-tiles-directory ./DEM_tiles_sub64 -d ./DEM_input_sub64 --isis-adjust-directory ./isis_adjust_sol_20110919 --cube-to-drg-scale-factor 16.0000000000000000 -s ./albedo_all/cubes -e ./albedo_all/exposure -r ./albedo_all -b -300:300:-40:40 -t 1 --tile-size 4 --pixel-padding 5 -f ./albedo_all/imagesList.txt -c photometry_init_cubes_settings.txt --is-last-iter 0 -i ./apollo_metric/cubes/a17/sub4_cubes/AS17-M-0185.lev1.cub
// Remove the file unique.pl.
// To do: rename cubes to meta?
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
// Fix the bug with orbit ends not showing up (beyond 180 degrees).
// Fix the non-plastic bug.
// Read the data from cubes, and remove all logic having to do with
// filters from reconstruct.sh.
// To do: Unit tests
// To do: Reorg the code which computes the reflectance and its
// derivative in ShapeFromShading.cc to only compute the
// derivative. Move all that code to Reflectance.cc. Convert all to
// double.
// Rename dem_out.tif to dem_mean.tif, and modelParams.outputFile to modelParams.albedoFile,
// inputFile to drgFile.
// Save the simBox in resDir. Remove this logic from the shell script.
// The file name of the blankTilesList is repeated in the shell script and the code.
// Rm blank_tiles.
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
    exit(EXIT_FAILURE);
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

void getDEMAlbedoTileFiles(// Inputs
                           std::string blankTilesDir, std::string DEMTilesDir, std::string albedoTilesDir,
                           std::string blankTileFile,
                           // Outputs
                           std::string & DEMTileFile, std::string & albedoTileFile
                           ){
  DEMTileFile    = DEMTilesDir    + blankTileFile.substr(blankTilesDir.size());
  albedoTileFile = albedoTilesDir + blankTileFile.substr(blankTilesDir.size());
  return;
}

void createAlbedoTilesOverlappingWithDRG(double tileSize, int pixelPadding,
                                         std::string imageFile, Vector4 const& simBox,
                                         std::vector<ImageRecord> const& drgRecords,
                                         std::string blankTilesList,  std::string blankTilesDir,
                                         std::string DEMTilesList,    std::string DEMTilesDir,
                                         std::string albedoTilesList, std::string albedoTilesDir
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
    std::cout << "ERROR: Must have positive tile size and non-negative pixel padding!" << std::endl;
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

    std::string DEMTileFile, albedoTileFile;
    getDEMAlbedoTileFiles(blankTilesDir, DEMTilesDir, albedoTilesDir, blankTileFile, // inputs
                          DEMTileFile, albedoTileFile                                // outputs
                          );

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
      //std::cout << "\nwww No overlap among " << currFile << " " << rec.path << std::endl;
      continue;
    }
    //std::cout << "\nwww Yes overlap among " << currFile << " " << rec.path << std::endl;
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

int ReadConfigFile(char *config_filename, struct GlobalParams *settings)
{
  int MAX_LENGTH = 5000;
  char line[MAX_LENGTH];
  char inName[MAX_LENGTH];
  char inVal[MAX_LENGTH];
  char *commentPos;
  ifstream configFile(config_filename);
  int ret;
  
 

#define CHECK_VAR(name, fmt, assignTo) \
    if (0 == strcmp(inName, name)) { \
        sscanf(inVal, fmt, &(settings->assignTo)); \
    }

  if (configFile.is_open()){
    printf("CONFIG FILE FOUND\n");

    while (!configFile.eof()) {
        configFile.getline(line, MAX_LENGTH);

        // truncate comments
        commentPos = strchr(line, '#');
        if (NULL != commentPos) {
            *commentPos = '\0';
        }
        ret = sscanf(line, "%s %s\n", inName, inVal);
        if (2 != ret) continue;
        
        CHECK_VAR("EXTRACT_DRG_FROM_CUBES", "%d", extractDrgFromCubes);
        CHECK_VAR("REFLECTANCE_TYPE", "%d", reflectanceType);
        CHECK_VAR("SAVE_REFLECTANCE", "%d", saveReflectance);
        CHECK_VAR("SHADOW_THRESH", "%f", shadowThresh);
        CHECK_VAR("SLOPE_TYPE", "%d", slopeType);
        CHECK_VAR("ALBEDO_INIT_TYPE", "%d", albedoInitType);
        CHECK_VAR("EXPOSURE_INIT_TYPE", "%d", exposureInitType);
        CHECK_VAR("DEM_INIT_TYPE", "%d", DEMInitType);
        CHECK_VAR("SHADOW_INIT_TYPE", "%d", shadowInitType);
        CHECK_VAR("UPDATE_EXPOSURE", "%d", updateExposure);
        CHECK_VAR("UPDATE_ALBEDO", "%d", updateAlbedo);
        CHECK_VAR("UPDATE_HEIGHT", "%d", updateHeight);
        CHECK_VAR("INIT_ALBEDO_TILES", "%d", initAlbedoTiles);
        CHECK_VAR("COMPUTE_ERRORS", "%d", computeErrors);
        CHECK_VAR("USE_WEIGHTS", "%d", useWeights);
        CHECK_VAR("SAVE_WEIGHTS", "%d", saveWeights);
        CHECK_VAR("MAX_NUM_ITER", "%d", maxNumIter);
        CHECK_VAR("NO_DEM_DATA_VAL", "%d", noDEMDataValue);
    }
    configFile.close();
    settings->TRConst = 1.24; //this will go into config file
    //settings->saveWeights=1;

    return(1);
  }
  else{
    printf("configFile NOT FOUND\n");
    settings->reflectanceType = LUNAR_LAMBERT;
    settings->saveReflectance = 1;
    settings->slopeType = 1;
    settings->shadowThresh = 40;

    settings->extractDrgFromCubes = 0;
    settings->albedoInitType = 0;//1;
    settings->exposureInitType = 0;//1;
    settings->DEMInitType = 0;//1;
    settings->shadowInitType = 0;//1;
    settings->updateExposure = 0;//1;
    settings->updateAlbedo = 0;//1;
    settings->updateHeight = 1;
    settings->initAlbedoTiles = 0;
    settings->computeErrors = 0;//1;
    settings->useWeights = 1;//0;
    settings->saveWeights = 1;//0;
    settings->maxNumIter = 10;
    settings->noDEMDataValue = -32767;
    settings->TRConst = 1.24;

    return(0);
  }
}

void PrintGlobalParams(struct GlobalParams *settings)
{
  printf("EXTRACT_DRG_FROM_CUBES %d\n", settings->extractDrgFromCubes);
  printf("REFLECTANCE_TYPE %d\n", settings->reflectanceType);
  printf("SAVE_REFLECTANCE %d\n", settings->saveReflectance);
  printf("SHADOW_THRESH %f\n", settings->shadowThresh);
  printf("SLOPE_TYPE %d\n", settings->slopeType);
  printf("ALBEDO_INIT_TYPE %d\n", settings->albedoInitType);
  printf("EXPOSURE_INIT_TYPE %d\n", settings->exposureInitType);
  printf("DEM_INIT_TYPE %d\n", settings->DEMInitType);
  printf("SHADOW_INIT_TYPE %d\n", settings->shadowInitType);
  printf("UPDATE_EXPOSURE %d\n", settings->updateExposure);
  printf("UPDATE_ALBEDO %d\n", settings->updateAlbedo);
  printf("UPDATE_HEIGHT %d\n", settings->updateHeight);
  printf("INIT_ALBEDO_TILES %d\n", settings->initAlbedoTiles);
  printf("COMPUTE_ERRORS %d\n", settings->computeErrors);
  printf("USE_WEIGHTS %d\n", settings->useWeights);
  printf("SAVE_WEIGHTS %d\n", settings->saveWeights);
  printf("MAX_NUM_ITER  %d\n", settings->maxNumIter);
  printf("NO_DEM_DATA_VAL %d\n", settings->noDEMDataValue);
  printf("TR_CONST %f\n", settings->TRConst);
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

Vector4 parseSimBox(std::string simBoxStr){

  // Parse the string "13W:49E:-12S:28N" to extract the vector of
  // numbers 13, 49, -12, 28.
  
  typedef boost::tokenizer<boost::char_separator<char> >  tokenizer;
  boost::char_separator<char> colon(":");
  tokenizer tokens(simBoxStr, colon);
  
  Vector4 simBox;
  int count = 0;
  for (tokenizer::iterator tok_iter = tokens.begin(); tok_iter != tokens.end(); ++tok_iter){
    std::string tok = *tok_iter;
    if (tok == "") continue;
    simBox(count) = atoi(tok.c_str());
    count++;
    if (count >= 4) break;
  }

  // If parsing did not succeed, use a huge box containing the entire moon.
  if (count < 4){
    double b = 180;
    simBox = Vector4(-b, b, -b, b);
  }

  if (simBox(0) >= simBox(1) || simBox(2) >= simBox(3)){
    std::cerr << "ERROR: Invalid simBox: " << simBox << std::endl;
    simBox = Vector4(0, 0, 0, 0);
  }
  
  return simBox;
}

void list_DRG_in_box_and_all_DEM(bool useTiles,
                                 std::string allDRGIndex, std::string allDEMIndex,
                                 Vector4 simBox, 
                                 std::string DRGDir,  std::string inputDEMTilesDir, 
                                 std::string DRGInBoxList
                                 ){

  // Create the lists of ALL DRG and DEM images in DRGDir and
  // inputDEMTilesDir, if these lists don't exist already.
  
  // Create the list of all DRG files intersecting the current simBox.

  Vector4 bigBox = Vector4(-1000, 1000, -1000, 1000);

  // Create the index of all DRG images if it does not exist already.
  std::vector<ImageRecord> imageRecords;
  if (!readImagesFile(imageRecords, allDRGIndex)){
    std::cout << "WILL create that missing file..." << std::endl;
    listTifsInDirOverlappingWithBox(DRGDir, bigBox, allDRGIndex);
    if (!readImagesFile(imageRecords, allDRGIndex)) exit(1); // Second attempt at reading
  }

  // Create the list of all DRG files intersecting the current box.
  ofstream fh(DRGInBoxList.c_str());
  if (!fh){
    std::cerr << "ERROR: list_DRG_in_box_and_all_DEM: can't open " << DRGInBoxList
              << " for writing" << std::endl;
    exit(EXIT_FAILURE);
  }
  fh.precision(20);
  for (int j = 0; j < (int)imageRecords.size(); j++){
    const ImageRecord& rec = imageRecords[j];
    Vector4 currCorners = Vector4(rec.west, rec.east, rec.south, rec.north);
    if (! boxesOverlap(currCorners, simBox)) continue;
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
    listTifsInDirOverlappingWithBox(inputDEMTilesDir, bigBox, allDEMIndex);
  }

  return;
}

int extractDRGFromCube(bool useDEMTiles, double cubeToDrgScaleFactor, std::string DEMTilesDir, std::string DEMFile,
                      std::string cubeFile, std::string isis_adjust_file, std::string outputDrgFile,
                      Vector3 & sunPosition, Vector3 & spacecraftPosition
                      );

int main( int argc, char *argv[] ) {

  for (int s = 0; s < argc; s++) std::cout << argv[s] << " ";
  std::cout << std::endl;
  
  std::vector<std::string> inputDRGFiles;
  std::vector<std::string> DRGFiles;
  std::vector<std::string> imageFiles;
  std::string cubDir                  = "data/cub";
  std::string simBoxStr               = "";
  std::string DRGDir                  = "data/DRG";
  std::string DEMDir                  = "data/DEM";
  std::string inputDEMTilesDir        = "data/DEMTiles";
  std::string isisAdjustDir           = "isis_adjust";
  std::string cubeToDrgScaleFactorStr = "1.0";
  std::string exposureDir             = "data/exposure";
  std::string resDir                  = "results";
  std::string configFilename          = "photometry_settings.txt";
  std::string DRGInBoxList            = "";
  bool useFeb13                       = false;
  std::string useTilesStr             = "0"; // Don't use tiles by default
  std::string tileSizeStr             = "";
  std::string pixelPaddingStr         = "0";
  std::string isLastIterStr           = "0";
  
  po::options_description general_options("Options");
  general_options.add_options()
  
    ("simulation-box,b", po::value<std::string>(&simBoxStr)->default_value(""), "Simulation box.")
    ("drg-directory", po::value<std::string>(&DRGDir)->default_value("data/DRG"), "DRG directory.")
    ("dem-tiles-directory", po::value<std::string>(&inputDEMTilesDir)->default_value("data/inputDEMTiles"), "Input DEM tiles directory.")
    ("isis-adjust-directory", po::value<std::string>(&isisAdjustDir)->default_value("isis_adjust"), "ISIS adjust directory.")
    ("cube-to-drg-scale-factor", po::value<std::string>(&cubeToDrgScaleFactorStr)->default_value("1.0"), "Cube to DRG scale factor.")
    ("dem-directory,d", po::value<std::string>(&DEMDir)->default_value("data/DEM"), "DEM directory.")
    ("image-files,i", po::value<std::vector<std::string> >(&imageFiles), "image files.")
    ("space info-directory,s", po::value<std::string>(&cubDir)->default_value("data/cub"), "space info directory.")
    ("exposure-directory,e", po::value<std::string>(&exposureDir)->default_value("data/exposure"), "exposure time directory.")
    ("res-directory,r", po::value<std::string>(&resDir)->default_value("results"), "results directory.")
    ("images-list,f", po::value<std::string>(&DRGInBoxList)->default_value(DRGInBoxList), "path to file listing images to use")
    ("config-filename,c", po::value<std::string>(&configFilename)->default_value("photometry_settings.txt"), "configuration filename.")
    ("use-tiles,t", po::value<std::string>(&useTilesStr)->default_value("0"), "use tiles")
    ("tile-size", po::value<std::string>(&tileSizeStr)->default_value("0"), "tile size")
    ("pixel-padding", po::value<std::string>(&pixelPaddingStr)->default_value("0"), "pixel padding")
    ("is-last-iter", po::value<std::string>(&isLastIterStr)->default_value("0"), "is last iteration")
    ("feb13", po::bool_switch(&useFeb13), "Use Feb 13 version of InitAlbedoMosaic")
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

  bool   useTiles     = atoi(useTilesStr.c_str());
  double tileSize     = atof(tileSizeStr.c_str());     // tile size in degrees
  int    pixelPadding = atoi(pixelPaddingStr.c_str()); // the pad for each tile in pixels
  bool   isLastIter   = atoi(isLastIterStr.c_str());
  double cubeToDrgScaleFactor = atof(cubeToDrgScaleFactorStr.c_str()); // used when extracting DRG from ISIS cubes
  std::cout << "ISIS adjust dir is      " << isisAdjustDir        << std::endl;
  std::cout << "tile size is            " << tileSize             << std::endl;
  std::cout << "pixel padding is        " << pixelPadding         << std::endl;
  std::cout << "is last iter is         " << isLastIter           << std::endl;
  std::cout << "cubeToDrgScaleFactor is " << cubeToDrgScaleFactor << std::endl;
  std::cout << "cube dir is             " << cubDir               << std::endl;
  
  // Double check to make sure all folders exist  
  if ( !fs::exists(resDir) )
    fs::create_directory(resDir);
  if ( !fs::exists(cubDir) )
    fs::create_directory(cubDir);
  if ( !fs::exists(resDir+"/info") )
    fs::create_directory(resDir+"/info");
  if ( !fs::exists(resDir+"/reflectance") )
    fs::create_directory(resDir+"/reflectance");
  if ( !fs::exists(resDir+"/shadow") )
    fs::create_directory(resDir+"/shadow");
  if ( !fs::exists(resDir+"/error") )
    fs::create_directory(resDir+"/error");
  if ( !fs::exists(resDir+"/exposure") )
    fs::create_directory(resDir+"/exposure");
  if ( !fs::exists(resDir+"/weight") )
    fs::create_directory(resDir+"/weight");
  if ( !fs::exists(resDir+"/DEM_sfs") )
    fs::create_directory(resDir+"/DEM_sfs");
  std::string albedoTilesDir = resDir + "/albedo";
  std::string DEMTilesDir    = resDir + "/DEM";
  std::string costFunDir     = resDir + "/costFun";
  if ( !fs::exists(albedoTilesDir) ) fs::create_directory(albedoTilesDir);
  if ( !fs::exists(DEMTilesDir)    ) fs::create_directory(DEMTilesDir);
  if ( !fs::exists(costFunDir)     ) fs::create_directory(costFunDir);

  // blankTilesDir is used to create a tile with identical dimensions
  // as the subsequent DEM and albedo tiles.
  std::string blankTilesDir  = resDir + "/blank_tiles";
  if (useTiles){
    if ( !fs::exists(blankTilesDir)  ) fs::create_directory(blankTilesDir);
  }
  
  GlobalParams globalParams;
  ReadConfigFile((char*)configFilename.c_str(), &globalParams);
  PrintGlobalParams(&globalParams);

  if (globalParams.extractDrgFromCubes == 1){

    // Extract the DRG image from the current cube by projecting on the DEM
    
    if ( !fs::exists(DRGDir) ) fs::create_directory(DRGDir);

    std::string cubeFile = imageFiles[0];
    std::string prefix   = getFirstElevenCharsFromFileName(cubeFile);

    sufix_from_filename(cubeFile).substr(0,12);
    if (prefix.size() >= 1 && prefix[0] == '/') prefix = prefix.substr(1); // strip '/'

    std::string DEMFile, DEMDirLoc; 
    if (!useTiles){
      // The DEM to project to will have the same prefix as the current cube
      std::map<std::string, std::string> DEMFilesIndex;
      indexFilesByKey(DEMDir, DEMFilesIndex);
      std::map<std::string, std::string>::iterator it = DEMFilesIndex.find(prefix);
      if (it == DEMFilesIndex.end()){
        std::cerr << "Could not find a DEM for the cube file: " << cubeFile << std::endl;
        exit(1);
      }
      DEMFile   = it->second;
      DEMDirLoc = DEMDir;
    }else{
      // We will project instead on the set of DEM tiles intersecting the current cube
      DEMFile    = "";
      DEMDirLoc  = inputDEMTilesDir;
    }
    
    std::string isisAdjustFile = isisAdjustDir + "/" + prefix + ".lev2.isis_adjust";
    std::string outputDrgFile  = DRGDir        + "/" + prefix + ".tif";
    Vector3 sunPosition, spacecraftPosition;
    std::cout << "cubeFile is         " << cubeFile       << std::endl;
    std::cout << "isis adjust file is " << isisAdjustFile << std::endl;
    std::cout << "outputDrgFile is    " << outputDrgFile  << std::endl;
    std::cout << "DEMFile is          " << DEMFile        << std::endl;
    std::cout << "DEMDirLoc  is       " << DEMDirLoc      << std::endl;
    extractDRGFromCube(useTiles, cubeToDrgScaleFactor, DEMDirLoc, DEMFile,
                      cubeFile, isisAdjustFile, outputDrgFile,
                      sunPosition, spacecraftPosition // outputs
                      );

    // Write the sun and spacecraft position to disk
    std::string sunDir        = cubDir + "/sunpos";
    std::string spacecraftDir = cubDir + "/spacecraftpos";
    if ( !fs::exists(cubDir) )        fs::create_directory(cubDir);
    if ( !fs::exists(sunDir) )        fs::create_directory(sunDir);
    if ( !fs::exists(spacecraftDir) ) fs::create_directory(spacecraftDir);
    std::string sunFile        = sunDir        + "/" + prefix + "_sun.txt";
    std::string spacecraftFile = spacecraftDir + "/" + prefix + "_spacecraft.txt";
    writeSunAndSpacecraftPosition(prefix, sunFile, spacecraftFile, sunPosition, spacecraftPosition);
    
    return 0;
  }
  
  // The names of the files listing all DRGs and DEMs and the coordinates
  // of their corners. 
  Vector4     simBox          = parseSimBox(simBoxStr);
  std::string allDRGIndex     = DRGDir           + "/index.txt";
  std::string allDEMIndex     = inputDEMTilesDir + "/index.txt";
  std::string blankTilesList  = resDir           + "/blankTilesList.txt";
  std::string DEMTilesList    = resDir           + "/DEMTilesList.txt";
  std::string albedoTilesList = resDir           + "/albedoTilesList.txt";
  if (globalParams.initAlbedoTiles == 1) {

    // Create the DRGInBoxList used in subsequent iterations.
    // Create the list of all DEM if not there yet.
    list_DRG_in_box_and_all_DEM(useTiles,
                                allDRGIndex, allDEMIndex,
                                simBox, DRGDir, inputDEMTilesDir, DRGInBoxList
                                );
    
    if (useTiles){
      vw_out( VerboseDebugMessage, "photometry" ) << "Initializing the albedo tiles ... ";
      if (imageFiles.size() == 0){
        std::cerr << "ERROR: Expecting an image file as input, the -i option" << std::endl;
        return 1;
      }
      std::vector<ImageRecord> drgRecords;
      if (!readImagesFile(drgRecords, DRGInBoxList)) exit(1);
      std::string imageFile = imageFiles[0]; // an image whose georef we will use
      createAlbedoTilesOverlappingWithDRG(tileSize, pixelPadding, imageFile, simBox,
                                          drgRecords,
                                          blankTilesList,  blankTilesDir,
                                          DEMTilesList,    DEMTilesDir,
                                          albedoTilesList, albedoTilesDir
                                          );
    }
    
    return 0;
  }

  // int factor = 4;
  // std::string inFile = imageFiles[0];
  // std::string str = "4";
  // std::string outFile = "up/" + sufix_from_filename(inFile);
  // //std::string outFile = inFile; outFile.replace(outFile.find(str), str.length(), "1");
  // std::cout << " --- Will upsample!" << std::endl;
  // upsample_uint8_image(outFile, inFile, 4);
  // exit(0);
    
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

  std::string sunPosFilename        = cubDir      + "/sunpos.txt";
  std::string spacecraftPosFilename = cubDir      + "/spacecraftpos.txt";
  std::string initExpTimeFile       = exposureDir + "/exposureTime.txt";
  std::string exposureInfoFilename  = resDir      + "/exposure/exposureInfo.txt";

  std::vector<Vector3> sunPositions;
  sunPositions = ReadSunPosition((char*)sunPosFilename.c_str(), drgRecords.size());

  std::vector<Vector3> spacecraftPositions;
  spacecraftPositions = ReadSpacecraftPosition((char*)spacecraftPosFilename.c_str(), drgRecords.size());

  std::vector<ModelParams> modelParamsArray;

  // In order to find the corresponding DEM for a given DRG, we take advantage
  // of the fact that the first 11 characters of these files are always the same.
  std::map<std::string, std::string> DEMFilesIndex;
  indexFilesByKey(DEMDir, DEMFilesIndex);
  
  //this will contain all the DRG files
  int i = 0;
  for (unsigned int j = 0; j < drgRecords.size(); ++j) {
    if (!drgRecords[j].useImage) continue;

    DRGFiles.push_back(drgRecords[j].path);
    modelParamsArray.push_back(ModelParams());

    std::string temp = sufix_from_filename(DRGFiles[i]);
    modelParamsArray[i].exposureTime        = 1.0;
    modelParamsArray[i].sunPosition         = 1000*sunPositions[j];
    modelParamsArray[i].spacecraftPosition  = 1000*spacecraftPositions[j];
  
    modelParamsArray[i].hCenterLineDEM      = NULL;
    modelParamsArray[i].hCenterLine         = NULL;
    modelParamsArray[i].hMaxDistArray       = NULL;
    modelParamsArray[i].hMaxDistArrayDEM    = NULL;

    modelParamsArray[i].inputFilename       = DRGFiles[i];//these filenames have full path


    // Find the corresponding DEM for the given DRG. See the earlier note.
    std::string prefix = getFirstElevenCharsFromFileName(DRGFiles[i]);
    std::map<std::string, std::string>::iterator it = DEMFilesIndex.find(prefix);
    if (it == DEMFilesIndex.end()){
      std::cerr << "Could not find a DEM for the DRG file: " << DRGFiles[i] << std::endl;
      exit(1);
    }
    modelParamsArray[i].DEMFilename = it->second;
    
    modelParamsArray[i].infoFilename        = resDir + "/info/" + prefix_less3_from_filename(temp)+"info.txt";
    modelParamsArray[i].meanDEMFilename     = resDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM_out.tif";   
    modelParamsArray[i].var2DEMFilename     = resDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM_var2.tif";
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
        if  (!useTiles){
          ReadWeightsParamsFromFile(useTiles, &modelParamsArray[j]);
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

        ComputeImageCenterLines(modelParamsArray[j].inputFilename,  
                                &modelParamsArray[j].hMaxDistArray,  
                                &modelParamsArray[j].hCenterLine,  
                                &modelParamsArray[j].vMaxDistArray,  
                                &modelParamsArray[j].vCenterLine
                                );

        if (!useTiles){
          modelParamsArray[j].hCenterLineDEM = ComputeDEMHCenterLine(modelParamsArray[j].DEMFilename,
                                                                     globalParams.noDEMDataValue,
                                                                     &(modelParamsArray[j].hMaxDistArrayDEM));
          modelParamsArray[j].vCenterLineDEM = ComputeDEMVCenterLine(modelParamsArray[j].DEMFilename,
                                                                     globalParams.noDEMDataValue,
                                                                     &(modelParamsArray[j].vMaxDistArrayDEM));
        }
        
        if (globalParams.saveWeights == 1){
          SaveWeightsParamsToFile(useTiles, modelParamsArray[j]);
        }
      }
      
    }
    
    vw_out( VerboseDebugMessage, "photometry" ) << "Done.\n";
  }
    
  if (globalParams.shadowInitType == 1 && (!useTiles)){
    TerminalProgressCallback callback("photometry","Init Shadow:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); i++){ 
      callback.report_progress(float(i)/float(imageFiles.size()));
      ComputeSaveShadowMap (modelParamsArray[inputIndices[i]], globalParams);
    }
    callback.report_finished();
  }

  if (globalParams.DEMInitType == 1){

    TerminalProgressCallback callback("photometry","Init DEM:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {
      callback.report_progress(float(i)/float(imageFiles.size()));

      if (!useTiles){
        std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
        for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
        }
        if ((globalParams.useWeights == 1) && (modelParamsArray[inputIndices[i]].hCenterLineDEM == NULL)){
          ReadWeightsParamsFromFile(useTiles, &modelParamsArray[inputIndices[i]]);
        }
        InitDEM(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
        
      }else{
        std::string blankTileFile = imageFiles[i];
        std::string DEMTileFile, albedoTileFile;
        getDEMAlbedoTileFiles(blankTilesDir, DEMTilesDir, albedoTilesDir, blankTileFile, // inputs
                              DEMTileFile, albedoTileFile                                // outputs
                              );

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
  if ((globalParams.reflectanceType != NO_REFL) && (globalParams.saveReflectance == 1)){ //compute reflectance

    TerminalProgressCallback callback("photometry","Init Reflectance:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {
      
      callback.report_progress(float(i)/float(imageFiles.size()));
      vw_out(VerboseDebugMessage,"photometry") << modelParamsArray[inputIndices[i]].reliefFilename << "\n";

      //TO DO: check to see that file exists
      //TO DO: if file does not exist compute.

      if (useTiles){
        // Compute the average reflectance image.
        // The same code is used below to update the exposure.
        std::string curDRG = imageFiles[i];
        std::vector<ImageRecord> DEMTiles, albedoTiles;
        std::vector<int> overlap;
        if (!readImagesFile(DEMTiles,    DEMTilesList))    exit(1);
        if (!readImagesFile(albedoTiles, albedoTilesList)) exit(1);
        overlap = makeOverlapList(DEMTiles, curDRG);
        bool compAvgRefl = true;
        avgReflectanceArray[i]
          = computeAvgReflectanceOverTilesOrUpdateExposure(compAvgRefl,
                                                           pixelPadding, tileSize,
                                                           DEMTiles, albedoTiles, 
                                                           overlap,
                                                           modelParamsArray[inputIndices[i]],
                                                           globalParams);
        
      }else{
        //compute and save the reflectance image.
        avgReflectanceArray[i] = computeImageReflectance(modelParamsArray[inputIndices[i]],
                                                         globalParams);
      }
    }
    callback.report_finished();
  }

  //compute exposure time from the reflectance images assuming average equal albedo
  if (globalParams.exposureInitType == 1){

    TerminalProgressCallback callback("photometry","Init Exposure Time:");
    callback.report_progress(0);
  
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {
      callback.report_progress(float(i)/float(imageFiles.size()));
 
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

    for (unsigned int i = 0; i < imageFiles.size(); ++i) {

      callback.report_progress(float(i)/float(imageFiles.size()));
           
      std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
      for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
        overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
      }
      
      if (globalParams.reflectanceType == NO_REFL) {
        InitImageMosaicByBlocks(modelParamsArray[inputIndices[i]],
				overlapParamsArray, globalParams);
      } else {
        if (useFeb13) {
          InitAlbedoMosaicFeb13(modelParamsArray[inputIndices[i]],
                                overlapParamsArray, globalParams);
        } else {
          
          if (!useTiles){
            InitAlbedoMosaic(modelParamsArray[inputIndices[i]],
                             overlapParamsArray, globalParams);
          }else{
            // This code is repeated below where we update the albedo
            std::string blankTileFile  = imageFiles[i];
            Vector4 tileCorners = getImageCorners(blankTileFile);
            if (isLastIter && !boxesOverlap(tileCorners, simBox)){
              // If this is not the last iteration, we must init/update the exposure
              // and albedo for all tiles. Otherwise it is enough to do it only
              // for the tiles which overlap with the sim box.
              std::cout << "Skipping tile: "
                        << blankTileFile << " as it does not overlap with the simulation box." << std::endl;
              continue;
            }

            std::string DEMTileFile, albedoTileFile;
            getDEMAlbedoTileFiles(blankTilesDir, DEMTilesDir, albedoTilesDir, blankTileFile, // inputs
                                  DEMTileFile, albedoTileFile                                // outputs
                                  );


            bool initTile = true; // init, rather than update
            InitOrUpdateAlbedoTile(isLastIter, initTile, pixelPadding, tileSize,
                                   blankTileFile, DEMTileFile, albedoTileFile,
                                   overlapParamsArray, globalParams);
            
          }
          
        }
      }      
    }
    callback.report_finished();
  }


  //-----------------------------------------------------------


  //re-estimate the parameters of the image formation model
  float overallError;
  for (int iter = 1; iter <= globalParams.maxNumIter; iter++){

    overallError = 0.0;

    if (globalParams.updateExposure == 1){ //re-estimate the exposure time

      for (unsigned int i = 0; i < imageFiles.size(); ++i) {   

        if (globalParams.reflectanceType == NO_REFL){
          //no use of reflectance map
          ComputeExposure(&modelParamsArray[inputIndices[i]], globalParams);
        }else if (!useTiles){
          //use reflectance map
          ComputeExposureAlbedo(&modelParamsArray[inputIndices[i]], globalParams);
        }else{

          // Update the exposure based on tiles
          std::string curDRG = imageFiles[i];
          std::vector<ImageRecord> DEMTiles, albedoTiles;
          std::vector<int> overlap;
          if (!readImagesFile(DEMTiles,    DEMTilesList))    exit(1);
          if (!readImagesFile(albedoTiles, albedoTilesList)) exit(1);
          overlap = makeOverlapList(DEMTiles, curDRG);
          bool compAvgRefl = false; // set this flag to false to update the exposure
          double exposure = computeAvgReflectanceOverTilesOrUpdateExposure(compAvgRefl,
                                                                           pixelPadding, tileSize,
                                                                           DEMTiles, albedoTiles, 
                                                                           overlap,
                                                                           modelParamsArray[inputIndices[i]],
                                                                           globalParams);
          modelParamsArray[inputIndices[i]].exposureTime = exposure;
          
        }
        
        //create the exposureInfoFilename
        SaveExposureInfoToFile(modelParamsArray[inputIndices[i]]);
      }
      if (useTiles) break; // Do just one iteration. We control the iterations from the script.
    }

    if (globalParams.updateAlbedo == 1){

      for (unsigned int i = 0; i < imageFiles.size(); ++i) {

        std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
        for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
        }
        
        if (globalParams.reflectanceType == NO_REFL){
          //no use of the reflectance map
          UpdateImageMosaic( modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
        }else{
          if (!useTiles){
            //use reflectance
            UpdateAlbedoMosaic(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
          }else{
            // We use the same logic as for initializing the albedo above. What is different now
            // is that we work with the updated exposure.
            std::string blankTileFile  = imageFiles[i];
            Vector4 tileCorners = getImageCorners(blankTileFile);
            if (isLastIter && !boxesOverlap(tileCorners, simBox)){
              // If this is not the last iteration, we must init/update the exposure
              // and albedo for all tiles. Otherwise it is enough to do it only
              // for the tiles which overlap with the sim box.
              std::cout << "Skipping tile: "
                        << blankTileFile << " as it does not overlap with the simulation box." << std::endl;
              continue;
            }
            std::string DEMTileFile, albedoTileFile;
            getDEMAlbedoTileFiles(blankTilesDir, DEMTilesDir, albedoTilesDir, blankTileFile, // inputs
                                  DEMTileFile, albedoTileFile                                // outputs
                                  );
            bool initTile = false; // will update, not init
            double costFunVal = InitOrUpdateAlbedoTile(isLastIter, initTile, pixelPadding, tileSize,
                                                       blankTileFile, DEMTileFile, albedoTileFile,
                                                       overlapParamsArray, globalParams);
            std::string costFunFile = costFunDir
              + prefix_from_filename(sufix_from_filename(albedoTileFile)) + ".txt";
            std::cout << "cost fun file is " << costFunFile << std::endl;
            AppendCostFunToFile(costFunVal, costFunFile);
          }
        }
      }
      if (useTiles) break; // Do just one iteration. We control the iterations from the script.      
    }
        
    //re-estimate the height map  - shape from shading
    if ((globalParams.reflectanceType != NO_REFL) && (globalParams.updateHeight == 1)){

      for (unsigned int i = 0; i < imageFiles.size(); ++i) {
        std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
        for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
        }
        UpdateHeightMap(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
      }
    }

    if (globalParams.computeErrors == 1){

      //compute the errors
      float overallAvgError = 0;
      int overallNumSamples = 0;

      for (unsigned int i = 0; i < imageFiles.size(); ++i) {

        std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
        for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
        }
	
        //use reflectance
        float avgError;
        int numSamples;

        ComputeReconstructionErrorMap(modelParamsArray[inputIndices[i]],
				      overlapParamsArray,
				      globalParams,
				      &avgError, &numSamples);

        if (i ==0){
          overallAvgError = avgError;
          overallNumSamples = numSamples;
        }
        else{
          overallAvgError = (overallAvgError*overallNumSamples + avgError*numSamples)/(overallNumSamples + numSamples);
          overallNumSamples = overallNumSamples + numSamples;
        }
      }
      printf("iter = %d, overallAvgError = %f\n", iter, overallAvgError);
    }
    
  }

}

