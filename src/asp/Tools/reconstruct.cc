// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// To do:
// Urgent: The final albedo tiles must have no padding.
// Save the simBox in resDir. Remove this logic from the shell script.
// The file name of the blankTilesList is repeated in the shell script and the code.
// Rename albedo_tiles to albedo, DEM_tiles to DEM. Rm blank_tiles.
// Remove a lot of duplicate code related to overlaps
// There is only one image, rm the vector of images
// No need to initialize the albedo tiles on disk, create
//   them on the fly.
// Merge the imageRecord and modelParams classes
// See if to change the order of values in the corners vector
// Reorg the code
#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <iostream>
#include <fstream>
#include <string>
#include <fstream>
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
      

//this will be used to compute the makeOverlapList in a more general way.
//it takes into consideration any set of overlapping images.
Vector4 ComputeGeoBoundary(GeoReference Geo, int width, int height)
{
 
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

void listTifsInDir(const std::string & dirName,
                   std::vector<std::string> & tifsInDir
                   ){
  
  tifsInDir.clear();
  
  fs::path dir(dirName);
  if ( !fs::exists(dirName) || !fs::is_directory(dirName ) ) return;

  fs::directory_iterator end_iter; // default construction yields past-the-end
  for  ( fs::directory_iterator dir_iter(dirName); dir_iter != end_iter; ++dir_iter)
    {
      if (! fs::is_regular_file(dir_iter->status()) ) continue;
      std::string fileName = (*dir_iter).string();
      int len = fileName.size();
      if (len >= 4 && fileName.substr(len - 4, 4) == ".tif"){
        //std::cout << "Now adding " << fileName << std::endl;
        tifsInDir.push_back( fileName );
      }
    }
  
  return;
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
    DiskImageView<PixelMask<PixelGray<uint8> > >  currImage(currFile);
    GeoReference currGeo;
    read_georeference(currGeo, currFile);
    Vector4 currCorners = ComputeGeoBoundary(currGeo, currImage.cols(), currImage.rows());
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

void createAlbedoTilesOverlappingWithBox(double tileSize, int pixelPadding,
                                         std::string imageFile, const Vector4 & boxCorners,
                                         std::string blankTilesList, std::string blankTilesDir,
                                         std::string DEMTilesList, std::string DEMTilesDir,
                                         std::string albedoTilesList, std::string albedoTilesDir
                                         ){

  // Read the georeference of an image, having info about which planet
  // we are, the pixel size, etc. (Any one of those images would work
  // as well as any other). Use it to create the list of tiles
  // overlapping with the current box. Write to disk both the tiles
  // themselves and their list.

  GeoReference geo;
  read_georeference(geo, imageFile);

  ofstream fht(blankTilesList.c_str());
  fht.precision(20);

  ofstream fhd(DEMTilesList.c_str());
  fhd.precision(20);

  ofstream fha(albedoTilesList.c_str());
  fha.precision(20);
  
  if (tileSize <= 0.0 || pixelPadding < 0){
    std::cout << "ERROR: Must have positive tile size and non-negative pixel padding!" << std::endl;
    exit(1);
  }
  
  for (double iBegE = tileSize*floor(boxCorners(0)/tileSize); iBegE < boxCorners(1); iBegE += tileSize){
    for (double iBegN = tileSize*floor(boxCorners(2)/tileSize); iBegN < boxCorners(3); iBegN += tileSize){

      // Upper left corner lon lat
      Vector2 A = Vector2(iBegE, iBegN + tileSize);

      // Right and down by pixelPadding
      Vector2 B = geo.pixel_to_lonlat(geo.lonlat_to_pixel(A) + Vector2(pixelPadding, pixelPadding));

      Vector2 D = B - A;

      // Careful with the signs below
      double begE = iBegE - D(0), endE = iBegE + tileSize + D(0);
      double begN = iBegN + D(1), endN = iBegN + tileSize - D(1);
      
      // Set the upper-left corner in the albedo tile
      Matrix3x3 T = geo.transform();
      T(0,2) = begE;
      T(1,2) = endN;
      geo.set_transform(T);

      // Determine the size of the albedo tile
      Vector2 pixUL = geo.lonlat_to_pixel(Vector2(begE, endN));
      Vector2 pixLR = geo.lonlat_to_pixel(Vector2(endE, begN));
      int nrows = (int)round(pixLR(0) - pixUL(0));
      int ncols = (int)round(pixLR(1) - pixUL(1));

      double uE = iBegE, uN = iBegN + tileSize; // uppper-left corner
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
      
      ImageView<PixelMask<PixelGray<float> > > blankTile(nrows, ncols);
      
      std::cout << "Writing " << blankTileFile << std::endl;
      write_georeferenced_image(blankTileFile,
                                channel_cast<uint8>(clamp(blankTile, 0.0,255.0)),
                                geo, TerminalProgressCallback("{Core}","Processing:"));

      fht << 1 << " " << blankTileFile << " "
         << begE << " " << endE << " " << begN << " " << endN << std::endl;

      fha << 1 << " " << albedoTileFile << " " << begE << " " << endE << " "
          << begN << " " << endN << std::endl;

      fhd << 1 << " " << DEMTileFile << " " << begE << " " << endE << " "
          << begN << " " << endN << std::endl;
    }
    
  }
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

  std::vector<int> overlapIndices;
  
  DiskImageView<PixelMask<PixelGray<uint8> > >  currImage(currFile);
  GeoReference currGeo;
  read_georeference(currGeo, currFile);
  Vector4 currCorners = ComputeGeoBoundary(currGeo, currImage.cols(), currImage.rows());

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
         DiskImageView<PixelMask<PixelGray<uint8> > >  image(params.inputFilename);
         GeoReference geo;
         read_georeference(geo, params.inputFilename);
         int width = image.cols();
         int height = image.rows();
         corners = ComputeGeoBoundary(geo, width, height);
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
  
  DiskImageView<PixelMask<PixelGray<uint8> > >  currImage(currFile);
  GeoReference currGeo;
  read_georeference(currGeo, currFile);
  Vector4 corners = ComputeGeoBoundary(currGeo, currImage.cols(), currImage.rows());

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
    double b = -180;
    simBox = Vector4(-b, b, -b, b);
  }

  if (simBox(0) >= simBox(1) || simBox(2) >= simBox(3)){
    std::cerr << "ERROR: Invalid simBox: " << simBox << std::endl;
    simBox = Vector4(0, 0, 0, 0);
  }
  
  return simBox;
}

void list_DRG_and_DEM_in_box(Vector4 simBox, 
                             std::string DRGDir,  std::string inputDEMTilesDir, 
                             std::string DRGList, std::string inputDEMList
                             ){

  // Create the lists of all DRG and DEM files intersecting the
  // current simBox. These lists are obtained from the cached lists of
  // ALL available DRG and DEM files. If the cached lists do not
  // exist, first create and cache them in the DRG and DEM
  // directories. Ideally this process will happen only the very first
  // time this executable is run for a freshly created set of DRG/DEM
  // directories, as it is time-consuming.
  std::vector<ImageRecord> imageRecords;

  std::string DRGIndex = DRGDir + "/index.txt";
  std::string DEMIndex = inputDEMTilesDir + "/index.txt";
  
  Vector4 bigBox = Vector4(-1000, 1000, -1000, 1000);

  // Create the index of all DRG images if it does not exist already.
  if (!readImagesFile(imageRecords, DRGIndex)){
    listTifsInDirOverlappingWithBox(DRGDir, bigBox, DRGIndex);
    readImagesFile(imageRecords, DRGIndex); // Second attempt at reading
  }
    
  ofstream fh(DRGList.c_str());
  if (!fh){
    std::cerr << "ERROR: list_DRG_and_DEM_in_box: can't open " << DRGList
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

  // Create the index of DEM tiles if it does not exist already.
  if (!readImagesFile(imageRecords, DEMIndex)){
    listTifsInDirOverlappingWithBox(inputDEMTilesDir, bigBox, DEMIndex);
    readImagesFile(imageRecords, DEMIndex); // Second attempt at reading
  }

  ofstream fh2(inputDEMList.c_str());
  if (!fh2) {
    std::cerr << "ERROR: list_DRG_and_DEM_in_box: can't open " << inputDEMList
              << " for writing" << std::endl;
    exit(EXIT_FAILURE);
  }
  fh2.precision(20);
  for (int j = 0; j < (int)imageRecords.size(); j++){
    const ImageRecord& rec = imageRecords[j];
    Vector4 currCorners = Vector4(rec.west, rec.east, rec.south, rec.north);
    if (! boxesOverlap(currCorners, simBox)) continue;
    fh2 << rec.useImage << " " << rec.path << " "
       << currCorners(0) << " " << currCorners(1) << " "
       << currCorners(2) << " " << currCorners(3) << std::endl;
  }
  fh2.close();

  return;
}

int main( int argc, char *argv[] ) {

  for (int s = 0; s < argc; s++) std::cout << argv[s] << " ";
  std::cout << std::endl;

  std::vector<std::string> inputDRGFiles;
  std::vector<std::string> DRGFiles;
  std::vector<std::string> imageFiles;
  std::string cubDir = "../data/cub";
  std::string simBoxStr = "";
  std::string DRGDir = "../data/DRG";
  std::string DEMDir = "../data/DEM";
  std::string inputDEMTilesDir = "../data/DEMTiles";
  std::string exposureDir = "../data/exposure";
  std::string resDir = "../results";
  std::string configFilename = "photometry_settings.txt";
  std::string imagesList = "";
  bool useFeb13 = false;
  std::string useTilesStr     = "";
  std::string tileSizeStr     = "";
  std::string pixelPaddingStr = "";
  
  po::options_description general_options("Options");
  general_options.add_options()
  
    ("simulation-box,b", po::value<std::string>(&simBoxStr)->default_value(""), "Simulation box.")
    ("drg-directory", po::value<std::string>(&DRGDir)->default_value("../data/DRG"), "DRG directory.")
    ("dem-tiles-directory", po::value<std::string>(&inputDEMTilesDir)->default_value("../data/inputDEMTiles"), "Input DEM tiles directory.")
    ("dem-directory,d", po::value<std::string>(&DEMDir)->default_value("../data/DEM"), "DEM directory.")
    ("image-files,i", po::value<std::vector<std::string> >(&imageFiles), "image files.")
    ("space info-directory,s", po::value<std::string>(&cubDir)->default_value("../data/cub"), "space info directory.")
    ("exposure-directory,e", po::value<std::string>(&exposureDir)->default_value("../data/exposure"), "exposure time directory.")
    ("res-directory,r", po::value<std::string>(&resDir)->default_value("../results"), "results directory.")
    ("images-list,f", po::value<std::string>(&imagesList)->default_value(imagesList), "path to file listing images to use")
    ("config-filename,c", po::value<std::string>(&configFilename)->default_value("photometry_settings.txt"), "configuration filename.")
    ("use-tiles,t", po::value<std::string>(&useTilesStr)->default_value("0"), "use tiles")
    ("tile-size", po::value<std::string>(&tileSizeStr)->default_value("0"), "tile size")
    ("pixel-padding", po::value<std::string>(&pixelPaddingStr)->default_value("0"), "pixel padding")
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

  bool useTiles    = atoi(useTilesStr.c_str());
  double tileSize  = atof(tileSizeStr.c_str());     // tile size in degrees
  int pixelPadding = atoi(pixelPaddingStr.c_str()); // the pad for each tile in pixels
  std::cout << "tile size is "     << tileSize << std::endl;
  std::cout << "pixel padding is " << pixelPadding << std::endl;

  // Double check to make sure all folders exist  
  if ( !fs::exists(resDir) )
    fs::create_directory(resDir);
  if ( !fs::exists(resDir+"/DEM") )
    fs::create_directory(resDir+"/DEM");
  if ( !fs::exists(resDir+"/info") )
    fs::create_directory(resDir+"/info");
  if ( !fs::exists(resDir+"/reflectance") )
    fs::create_directory(resDir+"/reflectance");
  if ( !fs::exists(resDir+"/shadow") )
    fs::create_directory(resDir+"/shadow");
  if ( !fs::exists(resDir+"/error") )
    fs::create_directory(resDir+"/error");
  if ( !fs::exists(resDir+"/albedo") )
    fs::create_directory(resDir+"/albedo");
  if ( !fs::exists(resDir+"/exposure") )
    fs::create_directory(resDir+"/exposure");
  if ( !fs::exists(resDir+"/weight") )
    fs::create_directory(resDir+"/weight");

  // blankTilesDir is used to create a tile with identical dimensions
  // as the subsequent DEM and albedo tiles.
  std::string blankTilesDir  = resDir + "/blank_tiles";
  std::string albedoTilesDir = resDir + "/albedo_tiles";
  std::string DEMTilesDir    = resDir + "/DEM_tiles";
  if (useTiles){
    if ( !fs::exists(blankTilesDir)  ) fs::create_directory(blankTilesDir);
    if ( !fs::exists(albedoTilesDir) ) fs::create_directory(albedoTilesDir);
    if ( !fs::exists(DEMTilesDir)    ) fs::create_directory(DEMTilesDir);
  }
  
  GlobalParams globalParams;
  ReadConfigFile((char*)configFilename.c_str(), &globalParams);
  PrintGlobalParams(&globalParams);

  Vector4 simBox = parseSimBox(simBoxStr);
  std::string inputDEMList    = resDir + "/inputDEMList.txt";
  std::string blankTilesList  = resDir + "/blankTilesList.txt";
  std::string DEMTilesList    = resDir + "/DEMTilesList.txt";
  std::string albedoTilesList = resDir + "/albedoTilesList.txt";
  
  if (globalParams.initAlbedoTiles == 1) {

    // Create the imagesList used in subsequent iterations
    
    //list_DRG_and_DEM_in_box(simBox, "DIM_input_sub32", "DEM_input_sub32", imagesList, inputDEMList);
    list_DRG_and_DEM_in_box(simBox, DRGDir, inputDEMTilesDir, imagesList, inputDEMList);
    
    if (useTiles){
      vw_out( VerboseDebugMessage, "photometry" ) << "Initializing the albedo tiles ... ";
      if (imageFiles.size() == 0){
        std::cerr << "ERROR: Expecting an image file as input, the -i option" << std::endl;
        return 1;
      }
      std::string imageFile = imageFiles[0];
      createAlbedoTilesOverlappingWithBox(tileSize, pixelPadding, imageFile, simBox,
                                          blankTilesList, blankTilesDir,
                                          DEMTilesList, DEMTilesDir,
                                          albedoTilesList, albedoTilesDir
                                          );
    }
    
    return 0;
  }
  
  std::vector<ImageRecord> drgRecords;
  if( imagesList.size() == 0 ) {
    if ( vm.count("inputDRGFiles") < 1 ) {
      std::cerr << "ERROR: Must specify either the -f option or at least one orthoprojected image file!" << std::endl << std::endl;
      std::cerr << usage.str();
      return 1;
    }

    for (int i=0; i < (int)inputDRGFiles.size(); i++) {
      drgRecords.push_back(ImageRecord(inputDRGFiles[i]));
    }
  } else {
    readImagesFile(drgRecords, imagesList);
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

    modelParamsArray[i].DEMFilename = DEMDir + prefix_less3_from_filename(temp) + "DEM.tif";

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

    // To do: Reading this only for overlap images should be enough. But it does not work!
    ReadExposureInfoFromFile(&(modelParamsArray[i]));
    
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

  if (globalParams.useWeights == 1) {
    vw_out( VerboseDebugMessage, "photometry" ) << "Computing weights ... ";
    
    for (unsigned int i=0; i < relevantIndices.size(); i++) {
      int j = relevantIndices[i];

      if (modelParamsArray[j].hCenterLine != NULL) continue;

      // first try to read the weight files
      ReadWeightsParamsFromFile(&modelParamsArray[j]);

      if (globalParams.saveWeights != 1 && modelParamsArray[j].hCenterLine == NULL){
        cerr << "ERROR: Weights not found on disk for image: " << modelParamsArray[j].inputFilename << endl;
        exit(1);
      }
      
      if (modelParamsArray[j].hCenterLine != NULL) continue;
        
      // weights not written yet, build them

      //std::cout << "size of input indices: " << inputIndices.size() << std::endl;
      if (j == inputIndices[0]){
        //std::cout << "Compute weights for  image with index: " << j << std::endl;
        modelParamsArray[j].hCenterLine = ComputeImageHCenterLine(modelParamsArray[j].inputFilename,
                                                                  &(modelParamsArray[j].hMaxDistArray));
        modelParamsArray[j].hCenterLineDEM = ComputeDEMHCenterLine( modelParamsArray[j].DEMFilename,
                                                                    globalParams.noDEMDataValue,
                                                                    &(modelParamsArray[j].hMaxDistArrayDEM));
        modelParamsArray[j].vCenterLine = ComputeImageVCenterLine(modelParamsArray[j].inputFilename,
                                                                  &(modelParamsArray[j].vMaxDistArray));
        modelParamsArray[j].vCenterLineDEM = ComputeDEMVCenterLine(modelParamsArray[j].DEMFilename,
                                                                   globalParams.noDEMDataValue,
                                                                   &(modelParamsArray[j].vMaxDistArrayDEM));
        
        if (globalParams.saveWeights == 1){
          SaveWeightsParamsToFile(modelParamsArray[j]);
        }
      }
      
    }
    
    vw_out( VerboseDebugMessage, "photometry" ) << "Done.\n";
  }
    
  //TO DO: save the overlapIndicesArray list 
 
 if (globalParams.shadowInitType == 1){
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
          ReadWeightsParamsFromFile(&modelParamsArray[inputIndices[i]]);
        }
        InitDEM(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams);
        
      }else{
        std::string blankTileFile  = imageFiles[i];
        std::string DEMTileFile, albedoTileFile;
        getDEMAlbedoTileFiles(blankTilesDir, DEMTilesDir, albedoTilesDir, blankTileFile, // inputs
                              DEMTileFile, albedoTileFile                                // outputs
                              );

        std::vector<ImageRecord> DEMImages;
        readImagesFile(DEMImages, inputDEMList);
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
      
      std::string curDRG = imageFiles[i];
      std::vector<ImageRecord> DEMTiles;
      std::vector<int> overlap;
      if (useTiles){
        readImagesFile(DEMTiles, DEMTilesList);
        overlap = makeOverlapList(DEMTiles, curDRG);
      }
      callback.report_progress(float(i)/float(imageFiles.size()));
      vw_out(VerboseDebugMessage,"photometry") << modelParamsArray[inputIndices[i]].reliefFilename << "\n";
      
      //TO DO: check to see that file exists
      //TO DO: if file does not exist compute.
      //compute and save the refectance image.
      avgReflectanceArray[i] = computeImageReflectance(useTiles, DEMTiles, overlap,
                                                       modelParamsArray[inputIndices[i]],
                                                         globalParams);
    }
    callback.report_finished();
  }

  //compute exposure time from the reflectance images assuming average equal albedo
  if (globalParams.exposureInitType == 1){

    TerminalProgressCallback callback("photometry","Init Exposure Time:");
    callback.report_progress(0);
  
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {
      callback.report_progress(float(i)/float(DRGFiles.size()));
 
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
            std::string blankTileFile  = imageFiles[i];
            std::string DEMTileFile, albedoTileFile;
            getDEMAlbedoTileFiles(blankTilesDir, DEMTilesDir, albedoTilesDir, blankTileFile, // inputs
                                  DEMTileFile, albedoTileFile                                // outputs
                                  );
            InitAlbedoTile(blankTileFile, albedoTileFile,
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
  for (int iter = 1; iter < globalParams.maxNumIter; iter++){

    overallError = 0.0;

    if (globalParams.updateExposure == 1){ //re-estimate the exposure time

      for (unsigned int i = 0; i < DRGFiles.size(); ++i) {   

        if (globalParams.reflectanceType == NO_REFL){
          //no use of reflectance map
          ComputeExposure(&modelParamsArray[i], globalParams);
        }else{
          //use reflectance map
          ComputeExposureAlbedo(&modelParamsArray[i], globalParams);
        }

        //create the exposureInfoFilename
        SaveExposureInfoToFile(modelParamsArray[i]);
      }
    }

    if (globalParams.updateAlbedo == 1){

      for (unsigned int i = 0; i < DRGFiles.size(); ++i) {

        std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
        for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
        }
        
        if (globalParams.reflectanceType == NO_REFL){
          //no use of the reflectance map
          UpdateImageMosaic( modelParamsArray[i], overlapParamsArray, globalParams);
        }else{
          //use reflectance
          UpdateAlbedoMosaic(modelParamsArray[i], overlapParamsArray, globalParams);
        }
      }
    }
        
    //re-estimate the height map  - shape from shading
    
    if ((globalParams.reflectanceType != NO_REFL) && (globalParams.updateHeight == 1)){

      for (unsigned int i = 0; i < DRGFiles.size(); ++i) {
   
        std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
        for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
        }

        UpdateHeightMap(modelParamsArray[i], overlapParamsArray, globalParams);
      }
    }

    if (globalParams.computeErrors == 1){

      //compute the errors
      float overallAvgError = 0;
      int overallNumSamples = 0;

      for (unsigned int i = 0; i < DRGFiles.size(); ++i) {

        std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
        for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
        }
	
        //use reflectance
        float avgError;
        int numSamples;

        ComputeReconstructionErrorMap(modelParamsArray[i],
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
