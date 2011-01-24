// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
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
  for (int j = 0; j < inputFiles.size(); j++){
    for (int i = 0; i < DRGFiles.size(); i++){
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

//saves weights to file. it will be move to weights.cc
void SaveWeightsParamsToFile(struct ModelParams modelParams)
{
  FILE *fp;
  fp = fopen((char*)(modelParams.infoFilename).c_str(), "w");
  /*
  for (int i = 0; i < modelParams.centerLineDEM.size(); i++){
      fprintf(fp, "%f", modelParams.centerLineDEM[i]);
  }
  for (int i = 0; i < modelParams.centerLine.size(); i++){
      fprintf(fp, "%f", modelParams.centerLine[i]);
  }
  */
  fclose(fp);
}

//saves exposure info to file. it will be move to exposure.cc
void SaveExposureInfoToFile(struct ModelParams modelParams)
{
  FILE *fp;
  fp = fopen((char*)(modelParams.infoFilename).c_str(), "w");
  fprintf(fp, "%f", modelParams.exposureTime);
  fclose(fp);
}

void ReadExposureInfoFromFile(struct ModelParams *modelParams)
{
  FILE *fp;
  fp = fopen((char*)(modelParams->infoFilename).c_str(), "r");
  if (fp == NULL){
    modelParams->exposureTime = 1;
  }
  else{
    fscanf(fp, "%f", &(modelParams->exposureTime));
    fclose(fp);
  }
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

//this function determines the image overlap for the general case
//it takes into consideration any set of overlapping images.
std::vector<int> makeOverlapList(std::vector<std::string> inputFiles, string currFile) {
  
  std::vector<int> overlapIndices;
  
  DiskImageView<PixelMask<PixelGray<uint8> > >  currImage(currFile);
  GeoReference currGeo;
  read_georeference(currGeo, currFile);
  Vector4 currCorners = ComputeGeoBoundary(currGeo, currImage.cols(), currImage.rows());

  for (unsigned int i = 0; i < inputFiles.size(); i++){

       int lonOverlap = 0;
       int latOverlap = 0; 

       DiskImageView<PixelMask<PixelGray<uint8> > >  image(inputFiles[i]);
       GeoReference geo;
       read_georeference(geo, inputFiles[i]);
       int width = image.cols();
       int height = image.rows();
       Vector4 corners = ComputeGeoBoundary(geo, width, height);
       
       if(  ((corners(0)>currCorners(0)) && (corners(0)<currCorners(1))) //minlon in interval 
	    ||((corners(1)>currCorners(0)) && (corners(1)<currCorners(1)))) //maxlon in interval
       {
         lonOverlap = 1;
       }
       if(  ((corners(2)>currCorners(2)) && (corners(2)<currCorners(3))) //minlat in interval 
	    ||((corners(3)>currCorners(2)) && (corners(3)<currCorners(3)))) //maxlat in interval
       {
         latOverlap = 1;
       }
     
       if ((lonOverlap == 1) && (latOverlap == 1)){
           overlapIndices.push_back(i);
       }
  }

  return overlapIndices;
}


std::vector<vector<int> > makeOverlapList(std::vector<std::string> inputFiles, std::vector<std::string> DRGFiles) 
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
  for (int i=0; i < overlapIndices.size(); i++){
    printf("%d: ", i);
    for (int j = 0; j < overlapIndices[i].size(); j++){
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
        int ret = sscanf(inVal, fmt, &(settings->assignTo)); \
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
        CHECK_VAR("MAX_NUM_ITER", "%d", maxNumIter);
        CHECK_VAR("NO_DEM_DATA_VAL", "%d", noDEMDataValue);
    }
    configFile.close();
    settings->TRConst = 1.24; //this will go into config file
    

    return(1);
  }
  else{
    printf("configFile NOT FOUND\n");
    settings->reflectanceType = LUNAR_LAMBERT;
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
    settings->useWeights = 0;//1;
    settings->maxNumIter = 10;
    settings->noDEMDataValue = -10000;
    settings->TRConst = 1.24;

    return(0);
  }
}

void PrintGlobalParams(struct GlobalParams *settings)
{
  printf("REFLECTANCE_TYPE %d\n", settings->reflectanceType);
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
  printf("MAX_NUM_ITER  %d\n", settings->maxNumIter);
  printf("NO_DEM_DATA_VAL %d\n", settings->noDEMDataValue);
  printf("TR_CONST %f\n", settings->TRConst);
}

int main( int argc, char *argv[] ) {

  std::vector<std::string> DRGFiles;
  std::vector<std::string> imageFiles;
  std::string cubDir = "../data/cub";
  std::string DEMDir = "../data/DEM";
  std::string exposureDir = "../data/exposure";
  std::string resDir = "../results";
  std::string configFilename = "photometry_settings.txt";

  po::options_description general_options("Options");
  general_options.add_options()
  
    ("dem-directory,d", po::value<std::string>(&DEMDir)->default_value("../data/DEM"), "DEM directory.")
    ("image-files,i", po::value<std::vector<std::string> >(&imageFiles), "image files.")
    ("space info-directory,s", po::value<std::string>(&cubDir)->default_value("../data/cub"), "space info directory.")
    ("exposure-directory,e", po::value<std::string>(&exposureDir)->default_value("../data/exposure"), "exposure time directory.")
    ("res-directory,r", po::value<std::string>(&resDir)->default_value("../results"), "results directory.")
    ("config-filename,c", po::value<std::string>(&configFilename)->default_value("photometry_settings.txt"), "configuration filename.")
    ("help,h", "Display this help message");
  

  po::options_description hidden_options("");

  hidden_options.add_options()
    ("DRGFiles", po::value<std::vector<std::string> >(&DRGFiles));
 
  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("DRGFiles", -1);

  std::ostringstream usage;
  usage << "Description: main code for mosaic,albedo and shape reconstruction from multiple images" << std::endl << std::endl;
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
  } catch ( po::error &e ) {
    std::cout << "An error occured while parsing command line arguments.\n";
    std::cout << "\t" << e.what() << "\n\n";
    std::cout << usage.str();
    return 1;
  }

  if( vm.count("help") ) {
    std::cerr << usage.str() << std::endl;
    return 1;
  }

  if( vm.count("DRGFiles") < 1 ) {
    std::cerr << "Error: Must specify at least one orthoprojected image file!" << std::endl << std::endl;
    std::cerr << usage.str();
    return 1;
  }

  vw_out() << "Number of Files = " << DRGFiles.size() << "\n";
  if (imageFiles.size() == 0){
    imageFiles = DRGFiles;
  }

  GlobalParams globalParams;
  int configFileFound = ReadConfigFile((char*)configFilename.c_str(), &globalParams);
  PrintGlobalParams(&globalParams);
  
  std::string sunPosFilename = cubDir + "/sunpos.txt";
  std::string spacecraftPosFilename = cubDir + "/spacecraftpos.txt";
  std::string initExpTimeFile = exposureDir + "/exposureTime.txt";
  std::string exposureInfoFilename = resDir + "/exposure/exposureInfo.txt";

  std::vector<Vector3> sunPositions;
  sunPositions = ReadSunPosition((char*)sunPosFilename.c_str(), DRGFiles.size());

  std::vector<Vector3> spacecraftPositions;
  spacecraftPositions = ReadSpacecraftPosition((char*)spacecraftPosFilename.c_str(), DRGFiles.size());

  std::vector<ModelParams> modelParamsArray(DRGFiles.size());
  std::vector<float> avgReflectanceArray(DRGFiles.size());

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
  for (unsigned int i = 0; i < DRGFiles.size(); ++i) {

    std::string temp = sufix_from_filename(DRGFiles[i]);
    modelParamsArray[i].exposureTime = 1.0;
    modelParamsArray[i].sunPosition         = 1000*sunPositions[i];
    modelParamsArray[i].spacecraftPosition  = 1000*spacecraftPositions[i];
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
    modelParamsArray[i].weightFilename      =  resDir + "/weight" + prefix_from_filename(temp) + "_weight.txt";
    modelParamsArray[i].exposureFilename    =  resDir + "/exposure" + prefix_from_filename(temp) + "_exposure.txt";
  
    ReadExposureInfoFromFile(&(modelParamsArray[i]));

    if (globalParams.useWeights == 1) {
      vw_out( VerboseDebugMessage, "photometry" ) << "Computing weights ... ";
      modelParamsArray[i].centerLineDEM = ComputeDEMCenterLine(modelParamsArray[i].DEMFilename, &(modelParamsArray[i].maxDistArrayDEM));
      modelParamsArray[i].centerLine = ComputeImageCenterLine(modelParamsArray[i].inputFilename, &(modelParamsArray[i].maxDistArray));
      vw_out( VerboseDebugMessage, "photometry" ) << "Done.\n";
    }

    vw_out( VerboseDebugMessage, "photometry" ) << modelParamsArray[i] << "\n";
  }
  
  std::vector<vector<int> > overlapIndicesArray = makeOverlapList(imageFiles, DRGFiles); 
  printOverlapList(overlapIndicesArray);

  std::vector<int> inputIndices = GetInputIndices(imageFiles, DRGFiles);
 

  //TO DO: save the overlapIndicesArray list 
 
  /*
  if (globalParams.useWeights == 1) {
      vw_out( VerboseDebugMessage, "photometry" ) << "Computing weights ... ";
      for (int i = 0; i <input_files.size(); i++)      
         modelParamsArray[i].centerLineDEM = ComputeDEMCenterLine(modelParamsArray[i].DEMFilename, &(modelParamsArray[i].maxDistArrayDEM));
         modelParamsArray[i].centerLine = ComputeImageCenterLine(modelParamsArray[i].inputFilename, &(modelParamsArray[i].maxDistArray));
         SaveWeightsParamsToFile(modelParamsArray);
      }
      vw_out( VerboseDebugMessage, "photometry" ) << "Done.\n";
  }
  */

  if (globalParams.DEMInitType == 1){
   
    TerminalProgressCallback callback("photometry","Init DEM:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {
      callback.report_progress(float(i)/float(imageFiles.size()));

      std::vector<ModelParams> overlapParamsArray(overlapIndicesArray[i].size());
      for (unsigned int j = 0; j < overlapIndicesArray[i].size(); j++){
        overlapParamsArray[j] = modelParamsArray[overlapIndicesArray[i][j]];
      }
      
     InitDEM(modelParamsArray[inputIndices[i]], overlapParamsArray, globalParams); 
    }
    callback.report_finished();
  }

  if (globalParams.shadowInitType == 1){
    TerminalProgressCallback callback("photometry","Init Shadow:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); i++){ 
      callback.report_progress(float(i)/float(imageFiles.size()));
      ComputeSaveShadowMap (modelParamsArray[inputIndices[i]], globalParams);
    }
    callback.report_finished();
  }

  if (globalParams.reflectanceType != NO_REFL){ //compute reflectance

    TerminalProgressCallback callback("photometry","Init Reflectance:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < imageFiles.size(); ++i) {

      callback.report_progress(float(i)/float(imageFiles.size()));
      vw_out(VerboseDebugMessage,"photometry") << modelParamsArray[inputIndices[i]].reliefFilename << "\n";
   
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
      
      if (globalParams.reflectanceType == NO_REFL)
        InitImageMosaicByBlocks(modelParamsArray[inputIndices[i]],
				overlapParamsArray, globalParams);
      else
        InitAlbedoMosaic(modelParamsArray[inputIndices[i]],
			 overlapParamsArray, globalParams);
      
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
        }
        else{
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
        }
        else{
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
