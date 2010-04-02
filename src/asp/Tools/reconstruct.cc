// __BEGIN_LICENSE__
// Copyright (C) 2006, 2007 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <string>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

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

std::vector<int> makeOverlapList(std::vector<int> inputIndices, int currIndex,
                                 int maxNumPrevIndices, int maxNumNextIndices) {
  int i, numPrevIndices, numNextIndices;

  //determine the number of previous overlapping files
  if (currIndex == 0) {
    numPrevIndices = 0;
  }
  else{
    int tempNumPrevIndices = currIndex - maxNumPrevIndices;
    printf("tempNumPrevFiles = %d\n", tempNumPrevIndices);
    if (tempNumPrevIndices < 0){
      numPrevIndices = currIndex;
    }
    else{
      numPrevIndices = maxNumPrevIndices;
    }
  }

  //determine the number of next overlapping files
  int temp = inputIndices.size() - (currIndex+1);
  if (temp < maxNumPrevIndices){
      numNextIndices = temp;
  }
  else{
      numNextIndices = maxNumNextIndices;
  }

  printf("curr index = %d\n", currIndex);
  printf("numPrevIndices = %d, numNextIndices = %d\n", numPrevIndices, numNextIndices);

  std::vector<int> overlapIndices(numPrevIndices+numNextIndices);
  int tmpIndex = 0;
  for (i = currIndex - numPrevIndices; i < currIndex; i++){
    overlapIndices[tmpIndex] = inputIndices[i];
    printf("%d %d\n", tmpIndex, overlapIndices[i - (currIndex - numPrevIndices)]);
    tmpIndex++;
  }
  for (i = currIndex+1; i <= currIndex + numNextIndices; i++){
    overlapIndices[tmpIndex] = inputIndices[i];
    printf("%d %d\n", tmpIndex, overlapIndices[tmpIndex]);
    tmpIndex++;
  }

  printf("-------------------------------------------\n");

  return overlapIndices;
}

int main( int argc, char *argv[] ) {

  std::vector<std::string> input_files;
  //int num_matches;
  std::string homeDir;
  std::string dataDir = "/data/orbit33";
  std::string resDir = "/results/orbit33";

  po::options_description general_options("Options");
  general_options.add_options()
    //("num-matches,m", po::value<int>(&num_matches)->default_value(1000), "Number of points to match for linear regression.")
    ("project-directory,d", po::value<std::string>(&homeDir)->default_value("../../../.."), "Base directory where data folders and result folders are.")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");

  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_files));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  std::ostringstream usage;
  usage << "Description: tonematches several images" << std::endl << std::endl;
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

  if( vm.count("input-files") < 1 ) {
    std::cerr << "Error: Must specify at least one input file!" << std::endl << std::endl;
    std::cerr << usage.str();
    return 1;
  }

  vw_out() << "Number of Files = " << input_files.size() << "\n";

  GlobalParams globalParams;
  //globalParams.reflectanceType = NO_REFL;
  globalParams.reflectanceType = LUNAR_LAMBERT;
  //globalParams.reflectanceType = LAMBERT;
  globalParams.slopeType = 1;
  globalParams.shadowThresh = 40;


  //string spacecraftPosFilename = "";
  //string sunPosFilename = "";
  globalParams.albedoInitType = 1;
  globalParams.exposureInitType = 1;//0;
  globalParams.exposureInitRefValue = 1.2; //initial estimate of the exposure time for the reference frame
  globalParams.exposureInitRefIndex = 0;   //the reference frame
  globalParams.DEMInitType = 0;//1;
  globalParams.shadowInitType = 1;//0;
  //globalParams.re_estimateExposure = 1;
  //globalParams.re_estimateAlbedo = 1;
  globalParams.computeErrors = 1;//0;
  globalParams.useWeights = 1;//0;
  globalParams.maxNumIter = 10;
  globalParams.maxNextOverlappingImages = 2;
  globalParams.maxPrevOverlappingImages = 2;

  std::string sunPosFilename = homeDir + dataDir + "/sunpos.txt";
  std::string spacecraftPosFilename = homeDir + dataDir + "/spacecraftpos.txt";
  std::string initExpTimeFile = homeDir + dataDir + "/exposureTime.txt";

  std::string exposureInfoFilename = homeDir + resDir + "/exposure/exposureInfo_0.txt";

  std::vector<Vector3> sunPositions;
  sunPositions = ReadSunPosition((char*)sunPosFilename.c_str(), input_files.size());

  std::vector<Vector3> spacecraftPositions;
  spacecraftPositions = ReadSpacecraftPosition((char*)spacecraftPosFilename.c_str(), input_files.size());

  std::vector<ModelParams> modelParamsArray(input_files.size());
  std::vector<float> avgReflectanceArray(input_files.size());


  for (unsigned int i = 0; i < input_files.size(); ++i) {
    modelParamsArray[i].exposureTime = globalParams.exposureInitRefValue;
    modelParamsArray[i].rescalingParams[0] = 1;
    modelParamsArray[i].rescalingParams[1] = 0;
    modelParamsArray[i].sunPosition = 1000*sunPositions[i];
    modelParamsArray[i].spacecraftPosition = 1000*spacecraftPositions[i];

    std::string temp = sufix_from_filename(input_files[i]);
    modelParamsArray[i].inputFilename   = input_files[i];
    modelParamsArray[i].DEMFilename     = homeDir + dataDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM.tif";
    modelParamsArray[i].infoFilename    = homeDir + resDir +"/info/" + prefix_less3_from_filename(temp)+".txt";
    modelParamsArray[i].meanDEMFilename = homeDir + resDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM_out.tif";
    modelParamsArray[i].var2DEMFilename = homeDir + resDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM_var2.tif";
    modelParamsArray[i].reliefFilename  = homeDir + resDir + "/reflectance" + prefix_from_filename(temp) + "_reflectance.tif";
    modelParamsArray[i].shadowFilename  = homeDir + resDir + "/shadow/" + prefix_from_filename(temp) + "_shadow.tif";
    modelParamsArray[i].errorFilename   = homeDir + resDir + "/error" + prefix_from_filename(temp) + "_err.tif";
    modelParamsArray[i].outputFilename  = homeDir + resDir + "/albedo" + prefix_from_filename(temp) + "_TM.tif";

    if (globalParams.useWeights == 1) {
      vw_out( VerboseDebugMessage, "photometry" ) << "Computing weights ... ";
      modelParamsArray[i].centerLineDEM = ComputeDEMCenterLine(modelParamsArray[i].DEMFilename, &(modelParamsArray[i].maxDistArrayDEM));
      modelParamsArray[i].centerLine = ComputeImageCenterLine(modelParamsArray[i].inputFilename, &(modelParamsArray[i].maxDistArray));
      vw_out( VerboseDebugMessage, "photometry" ) << "Done.\n";
    }

    vw_out( VerboseDebugMessage, "photometry" ) << modelParamsArray[i] << "\n";
  }

  std::vector<int> inputIndices(input_files.size());
  for (unsigned int i = 0; i < input_files.size(); i++){
    inputIndices[i] = i;
  }

  if (globalParams.DEMInitType == 1){
    //initialize the DEM files
    TerminalProgressCallback callback("photometry","Init DEM:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < input_files.size(); ++i) {
      callback.report_progress(float(i)/float(input_files.size()));

      std::vector<int> overlapIndices;
      overlapIndices = makeOverlapList(inputIndices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages);
      std::vector<ModelParams> overlapParams(overlapIndices.size());

      for (unsigned int j = 0; j < overlapIndices.size(); j++){
        overlapParams[j] = modelParamsArray[overlapIndices[j]];
      }

      InitDEM(modelParamsArray[i], overlapParams, globalParams);
    }
    callback.report_finished();
  }

  if (globalParams.shadowInitType == 1){
    TerminalProgressCallback callback("photometry","Init Shadow:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < input_files.size(); i++){
      callback.report_progress(float(i)/float(input_files.size()));
      ComputeSaveShadowMap (modelParamsArray[i], globalParams);
    }
    callback.report_finished();
  }

  //compute the reflectance images
  if (globalParams.exposureInitType == 1){
    modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue;

    TerminalProgressCallback callback("photometry","Init Reflectance:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < input_files.size(); ++i) {
      callback.report_progress(float(i)/float(input_files.size()));
      vw_out(VerboseDebugMessage,"photometry") << modelParamsArray[i].reliefFilename << "\n";

      avgReflectanceArray[i] = computeImageReflectance(modelParamsArray[i],
                                                       globalParams);
      if ( i == 0 )
        modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue;
      else
        modelParamsArray[i].exposureTime = (modelParamsArray[0].exposureTime*avgReflectanceArray[0])/avgReflectanceArray[i];

      vw_out(VerboseDebugMessage,"photometry") << "\tExposure Time = "
                                               << modelParamsArray[i].exposureTime
                                               << "\n";
      AppendExposureInfoToFile(exposureInfoFilename, modelParamsArray[i]);
    }
    callback.report_finished();
  }

  if (globalParams.exposureInitType == 2){
    //initialize the exposure times from overlaping areas - Not working well and not clear why?
    printf("compute reflectance ...\n");
    for (unsigned int i = 0; i < input_files.size(); ++i) {
      float reflectanceRatio;
      if (i==0){
        modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue;

        reflectanceRatio = computeImageReflectance(modelParamsArray[i], modelParamsArray[i],
                                                   globalParams);

      }
      else{

        reflectanceRatio = computeImageReflectance(modelParamsArray[i], modelParamsArray[i-1],
                                                   globalParams);


        modelParamsArray[i].exposureTime = (modelParamsArray[i-1].exposureTime)/reflectanceRatio;
      }
      printf("exposure Time = %f\n", modelParamsArray[i].exposureTime);
      AppendExposureInfoToFile(exposureInfoFilename,
                               modelParamsArray[i]);
    }
  }

  if (globalParams.exposureInitType == 3){

    std::vector<float> expTimeArray = ReadExposureInfoFile(initExpTimeFile, input_files.size());
    modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue;
    for (unsigned int i = 1; i < input_files.size(); ++i) {
      modelParamsArray[i].exposureTime = (modelParamsArray[0].exposureTime*expTimeArray[i])/expTimeArray[0];
      printf("expTimeArray[%d] = %f\n", i,  modelParamsArray[i].exposureTime);
      AppendExposureInfoToFile(exposureInfoFilename, modelParamsArray[i]);
    }
  }

  if (globalParams.albedoInitType == 1){
    TerminalProgressCallback callback("photometry","Init Albedo:");
    callback.report_progress(0);
    for (unsigned int i = 0; i < input_files.size(); ++i) {
      callback.report_progress(float(i)/float(input_files.size()));

      std::vector<int> overlapIndices;
      overlapIndices = makeOverlapList(inputIndices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages);
      std::vector<ModelParams> overlapParamsArray(overlapIndices.size());

      for (unsigned int j = 0; j < overlapIndices.size(); j++)
        overlapParamsArray[j] = modelParamsArray[overlapIndices[j]];

      if (globalParams.reflectanceType == NO_REFL)
        InitImageMosaicByBlocks(modelParamsArray[i],
                                overlapParamsArray, globalParams);
      else
        InitAlbedoMosaic(modelParamsArray[i],
                         overlapParamsArray, globalParams);
    }
    callback.report_finished();
  }


  //-----------------------------------------------------------


  //re-estimate the parameters of the image formation model
  float overallError;

  for (int iter = 1; iter < globalParams.maxNumIter; iter++){

    std::string currExposureInfoFilename;
    std::string prevExposureInfoFilename;

    char* currExposureInfoFilename_char = new char[500];
    char* prevExposureInfoFilename_char = new char[500];

    sprintf (currExposureInfoFilename_char, (char*)(homeDir + resDir + "/exposure/exposureInfo_%d.txt").c_str(), iter);
    sprintf (prevExposureInfoFilename_char, (char*)(homeDir + resDir + "/exposure/exposureInfo_%d.txt").c_str(), iter-1);
    currExposureInfoFilename = std::string(currExposureInfoFilename_char);
    prevExposureInfoFilename = std::string(prevExposureInfoFilename_char);

    delete currExposureInfoFilename_char;
    delete prevExposureInfoFilename_char;


    /*
      string prevExposureInfoFilename = string("../results/exposure/exposureInfo_0.txt");
      string currExposureInfoFilename = string("../results/exposure/exposureInfo.txt");
      vector<float> exposureTimeVector;

      exposureTimeVector = ReadExposureInfoFile(prevExposureInfoFilename,input_files.size());
      for (unsigned int i = 0; i < input_files.size(); i++){
          modelParamsArray[i].exposureTime = exposureTimeVector[i];
      }
      AppendExposureInfoToFile(currExposureInfoFilename, input_files[0], modelParamsArray[0]);
    */

    overallError = 0.0;

    //re-estimate the exposure time
    for (unsigned int i = 0; i < input_files.size(); ++i) {

      printf("iter = %d, exposure, i = %d, filename = %s\n\n", iter, i, input_files[i].c_str());

      if (globalParams.reflectanceType == NO_REFL){
        //no use of reflectance map
        ComputeExposure(&modelParamsArray[i], globalParams);
      }
      else{
        //use reflectance map
        ComputeExposureAlbedo(&modelParamsArray[i], globalParams);
      }

      //create the exposureInfoFilename
      AppendExposureInfoToFile(currExposureInfoFilename, modelParamsArray[i]);
    }

     /*
     exposureTimeVector = ReadExposureInfoFile(currExposureInfoFilename,input_files.size());
     for (unsigned int i = 0; i < input_files.size(); i++){
          modelParamsArray[i].exposureTime = exposureTimeVector[i];
     }
     */

    for (unsigned int i = 0; i < input_files.size(); ++i) {

      std::vector<int> overlapIndices;
      overlapIndices = makeOverlapList(inputIndices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages);
      std::vector<ModelParams> overlapParamsArray(overlapIndices.size());

      for (unsigned int j = 0; j < overlapIndices.size(); j++){
        overlapParamsArray[j] = modelParamsArray[overlapIndices[j]];
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


     //re-estimate the height map  - shape from shading
     for (unsigned int i = 0; i < input_files.size(); ++i) {
          if (globalParams.reflectanceType != NO_REFL){
	     printf("iter = %d, height map computation i = %d, filename = %s\n\n", iter, i, modelParamsArray[i].inputFilename.c_str());
             
             std::vector<int> overlapIndices;
             overlapIndices = makeOverlapList(inputIndices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages );
             std::vector<ModelParams> overlapParamsArray(overlapIndices.size());
             for (unsigned int j = 0; j < overlapIndices.size(); j++){
                  overlapParamsArray[j] = modelParamsArray[overlapIndices[j]];
             }
              
             UpdateHeightMap(modelParamsArray[i], overlapParamsArray, globalParams);
	  }     
     }

    if (globalParams.computeErrors == 1){

      //compute the errors
      float overallAvgError = 0;
      int overallNumSamples = 0;

      for (unsigned int i = 0; i < input_files.size(); ++i) {
  
        std::vector<int> overlapIndices;

        overlapIndices = makeOverlapList(inputIndices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages );

        std::vector<ModelParams> overlapParamsArray(overlapIndices.size());
        for (unsigned int j = 0; j < overlapIndices.size(); j++){
          overlapParamsArray[j] = modelParamsArray[overlapIndices[j]];
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


  #if 0
  for (unsigned int i = 1; i < input_files.size(); ++i) {

      printf("i = %d, filename = %s\n\n", i, input_files[i].c_str());

      std::string temp = sufix_from_filename(input_files[i]);
      std::string DEM_file = "orbit33-dem" + prefix_less3_from_filename(temp) + "DEM.tif";
      //printf("DEM=%s\n", DEM_file.c_str());

      DiskImageView<PixelGray<float> >  dem_image(DEM_file);
      GeoReference curr_dem_geo;
      read_georeference(curr_dem_geo, DEM_file);
      printf("dem_width = %d, dem_height = %d\n", dem_image.cols(), dem_image.rows());

      DiskImageView<PixelMask<PixelGray<uint8> > > curr_image(input_files[i]);
      printf("img_width = %d, img_height = %d\n", curr_image.cols(), curr_image.rows());

      /*
      ComputeSaveAlbedoMap(input_files[i], input_files[i-1], output_files[i], output_files[i-1], DEM_file, modelParamsArray[i], modelParamsArray[i-1]);

      ComputeSaveError(input_files[i], output_files[i], DEM_file, error_files[i], modelParamsArray[i]);
      */

      printf("test1\n");
      //read the original image-VERY UNEFFICIENT!!!!
      DiskImageView<PixelMask<PixelGray<uint8> > >  origImage(input_files[i]);
      ImageView<PixelGray<double> > image(origImage.cols(), origImage.rows());
      for (unsigned k=0; k < origImage.rows(); ++k) {
         for (unsigned l=0; l < origImage.cols(); ++l) {
           image(l, k) = NAN;
             if ( is_valid(origImage(l,k)) ) {
                image(l,k) = (double)(channel_cast<uint8>(origImage(l,k)));
             }
         }
      }

      printf("test2\n");
      DiskImageView<PixelMask<PixelGray<uint8> > >  origAlbedo(output_files[i]);
      ImageView<PixelGray<double> > albedo(origAlbedo.cols(), origAlbedo.rows());
      for (unsigned k=0; k < origAlbedo.rows(); ++k) {
         for (unsigned l=0; l < origAlbedo.cols(); ++l) {
           albedo(l,k) = NAN;
           if ( is_valid (origAlbedo(l,k)) ){
               albedo(l,k) = (double)(channel_cast<uint8>(origAlbedo(l,k)));
           }
         }
      }

      printf("test3\n");
      //DiskImageView<PixelMask<PixelGray<float> > >  origDEM(DEM_file);
      //TO DO: change this such that the init_dem will be the sie of the origImage+1;
      ImageView<PixelGray<double> > init_dem(origImage.cols()+1, origImage.rows()+1);
      GeoReference curr_geo;
      read_georeference(curr_geo, input_files[i]);

      for (unsigned k=0; k < origImage.rows()+1; ++k) {
         for (unsigned l=0; l < origImage.cols()+1; ++l) {

           init_dem(l, k) = 0.0;
           Vector2 curr_sample_pix(l,k);

           if ((k < origImage.rows()) && (l < origImage.cols()) ){
               init_dem(l, k) = NAN;//-10000.0;
               if ( is_valid(origImage(l,k)) ) {

                   Vector2 lon_lat = curr_geo.pixel_to_lonlat(curr_sample_pix);
                   Vector2 sample_pix_dem = curr_dem_geo.lonlat_to_pixel(curr_geo.pixel_to_lonlat(curr_sample_pix));

                   int x = (int)sample_pix_dem[0];
                   int y = (int)sample_pix_dem[1];
                   if ((x> 0) && (y>0) && (x < dem_image.cols()) && (y < dem_image.rows())){
                      init_dem(l,k) = (double)(dem_image(x,y));
                   }
               }
             }
         }
      }
      printf("test4\n");
      //ImageView<PixelGray<double> > init_dem(image.cols()+1, image.rows()+1);

      Vector<double, 3> light_direction;
      light_direction[0] = modelParamsArray[i].sunPosition[0];
      light_direction[1] = modelParamsArray[i].sunPosition[1];
      light_direction[2] = modelParamsArray[i].sunPosition[2];
      float norm = sqrt(light_direction[0]*light_direction[0]+light_direction[1]*light_direction[1]+light_direction[2]*light_direction[2]);
      light_direction[0] = light_direction[0]/norm;
      light_direction[1] = light_direction[1]/norm;
      light_direction[2] = light_direction[2]/norm;
      //TO DO:normalize the light direction !!!!!!
      printf("test5\n");
      ImageView<PixelGray<double> > image_predicted(origImage.cols(), origImage.rows());
      ImageView<PixelGray<double> > dem = copy(init_dem);

       // optimize_check_gradient(&image_predicted, &image, &dem, &init_dem, &albedo, &light_direction);
       // optimize_simple(&image_predicted, &image, &dem, &init_dem, &albedo, &light_direction);

       optimize_conjugate_gradient(&image_predicted, &image, &dem, &init_dem, &albedo, &light_direction);
       printf("test6\n");
  }
  #endif

}


















