// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


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

std::vector<int> GetInputIndices(std::vector<std::string> inputFiles, std::vector<std::string> DRGFiles)
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
/*
//saves exposure info to file. it will be move to exposure.cc
void SaveExposureInfoToFile(struct ModelParams modelParams)
{
  FILE *fp;
  fp = fopen((char*)(modelParams.exposureFilename).c_str(), "w");
  fprintf(fp, "%f", modelParams.exposureTime);
  fclose(fp);
}

void ReadExposureInfoFromFile(struct ModelParams *modelParams)
{
  FILE *fp;
  fp = fopen((char*)(modelParams->exposureFilename).c_str(), "r");
  if (fp == NULL){
    modelParams->exposureTime = 1;
  }
  else{
    fscanf(fp, "%f", &(modelParams->exposureTime));
    fclose(fp);
  }
}
*/
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

struct ImageRecord {
  bool useImage;
  std::string path;
  double north, west, south, east;
  
  ImageRecord(void) {}
  ImageRecord(const std::string& path_) :
    useImage(true),
    path(path_),
    north(-999.0)
  {}
};

//this function determines the image overlap for the general case
//it takes into consideration any set of overlapping images.
std::vector<int> makeOverlapList(const std::vector<ModelParams>& inputFiles,
                                 const std::string& currFile) {
  
  std::vector<int> overlapIndices;
  
  DiskImageView<PixelMask<PixelGray<uint8> > >  currImage(currFile);
  GeoReference currGeo;
  read_georeference(currGeo, currFile);
  Vector4 currCorners = ComputeGeoBoundary(currGeo, currImage.cols(), currImage.rows());

  for (unsigned int i = 0; i < inputFiles.size(); i++){

       int lonOverlap = 0;
       int latOverlap = 0; 
       const ModelParams& params = inputFiles[i];

       Vector4 corners;
       if (params.corners(3) == -999.0) {
         DiskImageView<PixelMask<PixelGray<uint8> > >  image(params.inputFilename);
         GeoReference geo;
         read_georeference(geo, params.inputFilename);
         int width = image.cols();
         int height = image.rows();
         corners = ComputeGeoBoundary(geo, width, height);
       } else {
         corners = params.corners;
       }
       
       if (corners(0) > corners(1) || currCorners(0) > currCorners(1))
         {
           std::cout << "Must never happen: " << __FILE__ << " at line " << __LINE__ << std::endl;
           exit(1);
         }
       
       if ( std::max(corners(0), currCorners(0)) < std::min(corners(1), currCorners(1)) )
         {
           lonOverlap = 1;
         }
       
       if (corners(2) > corners(3) || currCorners(2) > currCorners(3))
         {
           std::cout << "Must never happen: " << __FILE__ << " at line " << __LINE__ << std::endl;
           exit(1);
         }
       
       if ( std::max(corners(2), currCorners(2)) < std::min(corners(3), currCorners(3)) )
         {
           latOverlap = 1;
         }
       
       if (lonOverlap == 1 && latOverlap == 1 && currFile != params.inputFilename){
         overlapIndices.push_back(i);
       }
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
  printf("COMPUTE_ERRORS %d\n", settings->computeErrors);
  printf("USE_WEIGHTS %d\n", settings->useWeights);
  printf("SAVE_WEIGHTS %d\n", settings->saveWeights);
  printf("MAX_NUM_ITER  %d\n", settings->maxNumIter);
  printf("NO_DEM_DATA_VAL %d\n", settings->noDEMDataValue);
  printf("TR_CONST %f\n", settings->TRConst);
}
void readImagesFile(std::vector<ImageRecord>& images,
                    const std::string& imagesFileName)
{
  std::ifstream imagesFile(imagesFileName.c_str());
  if (!imagesFile) {
    std::cerr << "ERROR: readImagesFile: can't open " << imagesFileName
              << " for reading: " << strerror(errno) << std::endl;
    exit(EXIT_FAILURE);
  }

  images.clear();

  std::string line;
  int lineNo = 1;
  while (std::getline(imagesFile, line)) {
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
                    &rec.north,
                    &rec.west,
                    &rec.south,
                    &rec.east)) {
      std::cerr << "ERROR: readImagesFile: " << imagesFileName
                << ": line " << lineNo
                << ": expected '%d %s %f %f %f %f' format" << std::endl;
      exit(EXIT_FAILURE);
    }
    rec.useImage = useImage;
    rec.path = path;
    
    images.push_back(rec);

    lineNo++;
  }
}

int main( int argc, char *argv[] ) {

  std::vector<std::string> inputDRGFiles;
  std::vector<std::string> DRGFiles;
  std::vector<std::string> imageFiles;
  std::string cubDir = "../data/cub";
  std::string DEMDir = "../data/DEM";
  std::string exposureDir = "../data/exposure";
  std::string resDir = "../results";
  std::string configFilename = "photometry_settings.txt";
  std::string imagesFile = "";
  bool useFeb13 = false;

  po::options_description general_options("Options");
  general_options.add_options()
  
    ("dem-directory,d", po::value<std::string>(&DEMDir)->default_value("../data/DEM"), "DEM directory.")
    ("image-files,i", po::value<std::vector<std::string> >(&imageFiles), "image files.")
    ("space info-directory,s", po::value<std::string>(&cubDir)->default_value("../data/cub"), "space info directory.")
    ("exposure-directory,e", po::value<std::string>(&exposureDir)->default_value("../data/exposure"), "exposure time directory.")
    ("res-directory,r", po::value<std::string>(&resDir)->default_value("../results"), "results directory.")
    ("images-file,f", po::value<std::string>(&imagesFile)->default_value(imagesFile), "path to file listing images to use")
    ("config-filename,c", po::value<std::string>(&configFilename)->default_value("photometry_settings.txt"), "configuration filename.")
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
  usage << "Description: main code for mosaic,albedo and shape reconstruction from multiple images" << std::endl << std::endl;
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

  std::vector<ImageRecord> drgRecords;
  if( imagesFile.size() == 0 ) {
    if ( vm.count("inputDRGFiles") < 1 ) {
      std::cerr << "Error: Must specify either -f option or at least one orthoprojected image file!" << std::endl << std::endl;
      std::cerr << usage.str();
      return 1;
    }

    for (int i=0; i < (int)inputDRGFiles.size(); i++) {
      drgRecords.push_back(ImageRecord(inputDRGFiles[i]));
    }
  } else {
    readImagesFile(drgRecords, imagesFile);
  }

  GlobalParams globalParams;
  ReadConfigFile((char*)configFilename.c_str(), &globalParams);
  PrintGlobalParams(&globalParams);
  
  std::string sunPosFilename = cubDir + "/sunpos.txt";
  std::string spacecraftPosFilename = cubDir + "/spacecraftpos.txt";
  std::string initExpTimeFile = exposureDir + "/exposureTime.txt";
  std::string exposureInfoFilename = resDir + "/exposure/exposureInfo.txt";

  std::vector<Vector3> sunPositions;
  sunPositions = ReadSunPosition((char*)sunPosFilename.c_str(), drgRecords.size());

  std::vector<Vector3> spacecraftPositions;
  spacecraftPositions = ReadSpacecraftPosition((char*)spacecraftPosFilename.c_str(), drgRecords.size());

  std::vector<ModelParams> modelParamsArray;

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
    modelParamsArray[i].DEMFilename         = DEMDir + prefix_less3_from_filename(temp) + "DEM.tif";
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

    ReadExposureInfoFromFile(&(modelParamsArray[i]));
    
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

  if (globalParams.useWeights == 1) {
    vw_out( VerboseDebugMessage, "photometry" ) << "Computing weights ... ";
    
    for (unsigned int i=0; i < relevantIndices.size(); i++) {
      int j = relevantIndices[i];

      if (modelParamsArray[j].hCenterLineDEM != NULL) continue;

      // first try to read the weight files
      ReadWeightsParamsFromFile(&modelParamsArray[j]);
      
      if (modelParamsArray[j].hCenterLineDEM != NULL) continue;
        
      // weights not written yet, build them
      
      // Horizontal
      modelParamsArray[j].hCenterLineDEM = ComputeDEMHCenterLine(modelParamsArray[j].DEMFilename,
                                                               globalParams.noDEMDataValue,
                                                               &(modelParamsArray[j].hMaxDistArrayDEM));
      modelParamsArray[j].hCenterLine = ComputeImageHCenterLine(modelParamsArray[j].inputFilename,
                                                              &(modelParamsArray[j].hMaxDistArray));
      // Vertical
      modelParamsArray[j].vCenterLineDEM = ComputeDEMVCenterLine(modelParamsArray[j].DEMFilename,
                                                               globalParams.noDEMDataValue,
                                                               &(modelParamsArray[j].vMaxDistArrayDEM));
      modelParamsArray[j].vCenterLine = ComputeImageVCenterLine(modelParamsArray[j].inputFilename,
                                                              &(modelParamsArray[j].vMaxDistArray));
      
      if (globalParams.saveWeights == 1){
        SaveWeightsParamsToFile(modelParamsArray[j]);
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

      std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
      for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
        overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
      }
      
      if ((globalParams.useWeights == 1) && (modelParamsArray[inputIndices[i]].hCenterLineDEM == NULL)){
          ReadWeightsParamsFromFile(&modelParamsArray[inputIndices[i]]);
      }

      InitDEM(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams); 
    }
    callback.report_finished();
  }
  /*
  if (globalParams.shadowInitType == 1){
    TerminalProgressCallback callback("photometry","Init Shadow:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); i++){ 
      callback.report_progress(float(i)/float(imageFiles.size()));
      ComputeSaveShadowMap (modelParamsArray[inputIndices[i]], globalParams);
    }
    callback.report_finished();
  }
  */
  std::vector<float> avgReflectanceArray(DRGFiles.size());
  if ((globalParams.reflectanceType != NO_REFL) && (globalParams.saveReflectance == 1)){ //compute reflectance

    TerminalProgressCallback callback("photometry","Init Reflectance:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {

      callback.report_progress(float(i)/float(imageFiles.size()));
      vw_out(VerboseDebugMessage,"photometry") << modelParamsArray[inputIndices[i]].reliefFilename << "\n";
   
      //TO DO: check to see that file exists
      //TO DO: if file does not exist compute.
      //compute and save the refectance image.
      avgReflectanceArray[i] = computeImageReflectance(modelParamsArray[inputIndices[i]],
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
          InitAlbedoMosaic(modelParamsArray[inputIndices[i]],
                           overlapParamsArray, globalParams);
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
