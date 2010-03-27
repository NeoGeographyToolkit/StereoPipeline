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

#include <boost/operators.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
//#include <vw/Photometry.h>

#include <vw/Photometry/Albedo.h>
#include <vw/Photometry/Exposure.h>
#include <vw/Photometry/Reconstruct.h>
#include <vw/Photometry/Reflectance.h>
#include <vw/Photometry/Shadow.h>
#include <vw/Photometry/Shape.h>
#include <vw/Photometry/ShapeFromShading.h>
#include <vw/Photometry/Weights.h>

using namespace vw;
using namespace vw::math;
using namespace vw::cartography;
#include <math.h>

/// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

/// Erases a file suffix if one exists and returns the base string less3 characters
static std::string prefix_less3_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index-3, result.size()+3);
  return result;
}

/// Erases a file suffix if one exists and returns the base string less3 characters
static std::string sufix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind("/");
  if (index != -1) 
    result.erase(0, index);
  return result;
}

//reads the tiff DEM into a 3D coordinate
//pos is a Vector2 of pixel coordinates, GR is georeference 
template <class ViewT>
Vector3 pixel_to_cart (Vector2 pos, ImageViewBase<ViewT> const& img,  GeoReference GR) {
    Vector2 loc_longlat2=GR.point_to_lonlat(GR.pixel_to_point(pos));
    Vector3 loc_longlat3(loc_longlat2(0),loc_longlat2(1),img((int)pos[0],(int)pos[1]));
    Vector3 loc_cartesian=GR.datum().geodetic_to_cartesian(loc_longlat3);
    return loc_cartesian;
}


vector<int> makeOverlapList(vector<int> inputIndices, int currIndex, int maxNumPrevIndices, int maxNumNextIndices)
{
  int i;
  int numPrevIndices;
  int numNextIndices;
  
  //determine the number of previous overlapping files
  if (currIndex == 0){
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
  
  vector<int> overlapIndices(numPrevIndices+numNextIndices);
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
  int num_matches;

  po::options_description general_options("Options");
  general_options.add_options()
    ("help", "Display this help message")
    ("num-matches,m", po::value<int>(&num_matches)->default_value(1000), "Number of points to match for linear regression.");
    
  po::options_description hidden_options("");
  
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_files));
  
  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);
  
  po::positional_options_description p;
  p.add("input-files", -1);
  

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );
  
  std::ostringstream usage;
  usage << "Description: tonematches several images" << std::endl << std::endl;
  //usage << "Usage: reconstruct [options] <filename1> <filename2> ..." << std::endl << std::endl;
  usage << general_options << std::endl;
  
  if( vm.count("help") ) {
    std::cerr << usage.str() << std::endl;
    return 1;
  }
  
  if( vm.count("input-files") < 1 ) {
    std::cerr << "Error: Must specify at least one input file!" << std::endl << std::endl;
    std::cerr << usage.str();
    return 1;
  }


  printf("NUM FILES = %d\n", (int)input_files.size() );

  GlobalParams globalParams;
  globalParams.reflectanceType = NO_REFL;
  //globalParams.reflectanceType = LUNAR_LAMBERT;
  //globalParams.reflectanceType = LAMBERT;
  globalParams.slopeType = 1;
  globalParams.shadowThresh = 40;
 
  
  //string spacecraftPosFilename = "";
  //string sunPosFilename = "";
  globalParams.albedoInitType = 1;
  globalParams.exposureInitType = 0;
  globalParams.exposureInitRefValue = 1.2; //initial estimate of the exposure time for the reference frame
  globalParams.exposureInitRefIndex = 0;   //the reference frame
  globalParams.DEMInitType = 0;//1;
  globalParams.shadowInitType = 1;//0;
  //globalParams.re_estimateExposure = 1;
  //globalParams.re_estimateAlbedo = 1;
  globalParams.computeErrors = 0;//1;
  globalParams.useWeights = 1;//0;
  globalParams.maxNumIter = 10;
  globalParams.maxNextOverlappingImages = 2;
  globalParams.maxPrevOverlappingImages = 2;
  string homeDir = "../../../..";
  string dataDir = "/data/orbit33";
  string resDir = "/results/orbit33";

  
  string sunPosFilename = homeDir + dataDir + "/sunpos.txt";
  string spacecraftPosFilename = homeDir + dataDir + "/spacecraftpos.txt";
  string initExpTimeFile = homeDir + dataDir + "/exposureTime.txt";

  string exposureInfoFilename = homeDir + resDir + "/exposure/exposureInfo_0.txt";

  std::vector<Vector3> sunPositions;
  sunPositions = ReadSunPosition((char*)sunPosFilename.c_str(), input_files.size());
 
  std::vector<Vector3> spacecraftPositions;
  spacecraftPositions = ReadSpacecraftPosition((char*)spacecraftPosFilename.c_str(), input_files.size());
 
 
  std::vector<modelParams> modelParamsArray(input_files.size());
  std::vector<float> avgReflectanceArray(input_files.size());


 for (unsigned int i = 0; i < input_files.size(); ++i) {
      modelParamsArray[i].exposureTime = globalParams.exposureInitRefValue;
      modelParamsArray[i].rescalingParams[0] = 1;
      modelParamsArray[i].rescalingParams[1] = 0;
      modelParamsArray[i].sunPosition[0] = 1000*sunPositions[i][0];
      modelParamsArray[i].sunPosition[1] = 1000*sunPositions[i][1];
      modelParamsArray[i].sunPosition[2] = 1000*sunPositions[i][2];
      modelParamsArray[i].spacecraftPosition[0] = 1000*spacecraftPositions[i][0];
      modelParamsArray[i].spacecraftPosition[1] = 1000*spacecraftPositions[i][1];
      modelParamsArray[i].spacecraftPosition[2] = 1000*spacecraftPositions[i][2];

      std::string temp = sufix_from_filename(input_files[i]);
     
      modelParamsArray[i].DEMFilename     = homeDir + dataDir + "/orbit33_DEM" + prefix_less3_from_filename(temp) + "DEM.tif";
      modelParamsArray[i].infoFilename    = homeDir + resDir +"/info/" + prefix_less3_from_filename(temp)+".txt";
      modelParamsArray[i].meanDEMFilename = homeDir + resDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM_out.tif";
      modelParamsArray[i].var2DEMFilename = homeDir + resDir + "/DEM" + prefix_less3_from_filename(temp) + "DEM_var2.tif";
      modelParamsArray[i].reliefFilename  = homeDir + resDir + "/reflectance" + prefix_from_filename(temp) + "_reflectance.tif";

      modelParamsArray[i].shadowFilename  = homeDir + resDir + "/shadow/" + prefix_from_filename(temp) + "_shadow.tif";
      modelParamsArray[i].errorFilename   = homeDir + resDir + "/error" + prefix_from_filename(temp) + "_err.tif";
      modelParamsArray[i].outputFilename  = homeDir  + resDir + "/albedo" + prefix_from_filename(temp) + "_TM.tif";
      modelParamsArray[i].inputFilename   = input_files[i];

      if (globalParams.useWeights == 1){      
          printf("Computing the weights...");           
          modelParamsArray[i].centerLineDEM = ComputeDEMCenterLine(modelParamsArray[i].DEMFilename, &(modelParamsArray[i].maxDistArrayDEM));
          modelParamsArray[i].centerLine = ComputeImageCenterLine(modelParamsArray[i].inputFilename, &(modelParamsArray[i].maxDistArray));
          printf("Done.\n");    
    }

   }
    vector<int> input_indices(input_files.size());
   for (unsigned int i = 0; i < input_files.size(); i++){
       input_indices[i] = i;
   } 
  

   if (globalParams.DEMInitType == 1){    
      //initialize the DEM files
      for (unsigned int i = 0; i < input_files.size(); ++i) {
 
          vector<int> overlap_indices;
          overlap_indices = makeOverlapList(input_indices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages);

	  std::vector<modelParams> overlap_params(overlap_indices.size());
  
	  for (unsigned int j = 0; j < overlap_indices.size(); j++){
              overlap_params[j] = modelParamsArray[overlap_indices[j]];
	  }
          	  
          InitDEM(modelParamsArray[i], overlap_params, globalParams);

      }
   }  


   if (globalParams.shadowInitType == 1){
      printf("start computing the shadow maps ...\n");  
      for (unsigned int i = 0; i < input_files.size(); i++){
	//ComputeSaveShadowMap (modelParamsArray[i].inputFilename, modelParamsArray[i].shadowFilename, globalParams);
        ComputeSaveShadowMap (modelParamsArray[i], globalParams);
      }
      printf("done computing the shadow maps!\n");
   }
 

  //compute the reflectance images
  if (globalParams.exposureInitType == 1){

    printf("compute reflectance ...\n");
    modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue; 
  
    for (unsigned int i = 0; i < input_files.size(); ++i) {

       printf("%s\n", modelParamsArray[i].reliefFilename.c_str());
       avgReflectanceArray[i] = computeImageReflectance(modelParamsArray[i].inputFilename, 
                                                        modelParamsArray[i].meanDEMFilename, 
                                                        modelParamsArray[i].shadowFilename,  
			                                modelParamsArray[i], 
                                                        modelParamsArray[i].reliefFilename, 
                                                        globalParams);

       if (i==0){
	   modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue; 
       }
       else{
           modelParamsArray[i].exposureTime = (modelParamsArray[0].exposureTime*avgReflectanceArray[0])/avgReflectanceArray[i];
       }

       printf("exposure Time = %f\n", modelParamsArray[i].exposureTime);
    
       AppendExposureInfoToFile(exposureInfoFilename, 
                                //modelParamsArray[i].inputFilename, 
                                modelParamsArray[i]);
    }
  }
  
  if (globalParams.exposureInitType == 2){
    //initialize the exposure times from overlaping areas - Not working well and not clear why?
    printf("compute reflectance ...\n");
    for (unsigned int i = 0; i < input_files.size(); ++i) {
        float reflectanceRatio;
        if (i==0){
	    modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue;
         
            reflectanceRatio = computeImageReflectance(modelParamsArray[i].inputFilename, 
                                                       modelParamsArray[i].inputFilename, 
                                                       modelParamsArray[i].meanDEMFilename, 
                                                       modelParamsArray[i].shadowFilename, 
                                                       modelParamsArray[i].shadowFilename, 
                                                       modelParamsArray[i], 
                                                       modelParamsArray[i],
	                                               modelParamsArray[i].reliefFilename, 
                                                       globalParams);
	 
       }
       else{

           reflectanceRatio = computeImageReflectance(modelParamsArray[i].inputFilename, 
                                                      modelParamsArray[i-1].inputFilename, 
                                                      modelParamsArray[i].meanDEMFilename, 
                                                      modelParamsArray[i].shadowFilename, 
                                                      modelParamsArray[i-1].shadowFilename, 
                                                      modelParamsArray[i], 
                                                      modelParamsArray[i-1],
                                                      modelParamsArray[i].reliefFilename, 
                                                      globalParams);
         
	   modelParamsArray[i].exposureTime = (modelParamsArray[i-1].exposureTime)/reflectanceRatio;
       }
       printf("exposure Time = %f\n", modelParamsArray[i].exposureTime);
       AppendExposureInfoToFile(exposureInfoFilename, 
                                //modelParamsArray[i].inputFilename, 
                                modelParamsArray[i]);
    }
  }
 
  if (globalParams.exposureInitType == 3){
  
    std ::vector<float> expTimeArray = ReadExposureInfoFile(initExpTimeFile, input_files.size());
    modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue;
    for (unsigned int i = 1; i < input_files.size(); ++i) {
         //modelParamsArray[i].exposureTime = expTimeArray[i];
         modelParamsArray[i].exposureTime = (modelParamsArray[0].exposureTime*expTimeArray[i])/expTimeArray[0];
         printf("expTimeArray[%d] = %f\n", i,  modelParamsArray[i].exposureTime);
         AppendExposureInfoToFile(exposureInfoFilename, 
                                  //modelParamsArray[i].inputFilename, 
                                  modelParamsArray[i]);
    }
  }


  if (globalParams.albedoInitType == 1){
    
    printf("start initializing the albedo ...\n");

    for (unsigned int i = 0; i < input_files.size(); ++i) {
       
         vector<int> overlap_indices;
         overlap_indices = makeOverlapList(input_indices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages);
         std::vector<modelParams> overlap_params(overlap_indices.size());
         std::vector<string> overlap_img_files(overlap_indices.size());

         for (unsigned int j = 0; j < overlap_indices.size(); j++){
              overlap_params[j] = modelParamsArray[overlap_indices[j]];
              overlap_img_files[j] = input_files[overlap_indices[j]];
         }

         if (globalParams.reflectanceType == NO_REFL){
        
             InitImageMosaicByBlocks(modelParamsArray[i], overlap_params, globalParams);

	 }
         else{
             InitAlbedoMap(modelParamsArray[i].inputFilename, 
                           modelParamsArray[i], 
                           modelParamsArray[i].meanDEMFilename, 
		           modelParamsArray[i].shadowFilename, 
                           modelParamsArray[i].outputFilename, 
                           overlap_img_files, overlap_params, globalParams);
	 }
      
    }
  
    printf("done initializing the albedo!\n");
  }
  //---------------------------------------------------------------------------------


  //re-estimate the parameters of the image formation model
  float overallError;
  

  for (int iter = 1; iter < globalParams.maxNumIter; iter++){
      
    string currExposureInfoFilename;
    string prevExposureInfoFilename;

    char* currExposureInfoFilename_char = new char[500];
    char* prevExposureInfoFilename_char = new char[500];

    sprintf (currExposureInfoFilename_char, (char*)(homeDir + resDir + "/exposure/exposureInfo_%d.txt").c_str(), iter);
    sprintf (prevExposureInfoFilename_char, (char*)(homeDir + resDir + "/exposure/exposureInfo_%d.txt").c_str(), iter-1);
    currExposureInfoFilename = string(currExposureInfoFilename_char);
    prevExposureInfoFilename = string(prevExposureInfoFilename_char);

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
              ComputeExposure(modelParamsArray[i].inputFilename, 
                              modelParamsArray[i].outputFilename, 
                              modelParamsArray[i].meanDEMFilename, 
                              &modelParamsArray[i], globalParams);
	  }

          //create the exposureInfoFilename 
          AppendExposureInfoToFile(currExposureInfoFilename, 
                                   //modelParamsArray[i].inputFilename, 
                                   modelParamsArray[i]);
      }
   

     // std::vector<std::string> overlap_img_files; 
     /* 
     exposureTimeVector = ReadExposureInfoFile(currExposureInfoFilename,input_files.size());
     for (unsigned int i = 0; i < input_files.size(); i++){
	  modelParamsArray[i].exposureTime = exposureTimeVector[i];
     }    
     */

     vector<int> input_indices(input_files.size());
     for (unsigned int i = 0; i < input_files.size(); i++){
	  input_indices[i] = i;
     }   
     
     for (unsigned int i = 0; i < input_files.size(); ++i) {
          
          //printf("expT[%d] = %f\n", i, modelParamsArray[i].exposureTime);
	  vector<int> overlap_indices;
         
          overlap_indices = makeOverlapList(input_indices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages);
     
          std::vector<modelParams> overlapParamsArray(overlap_indices.size());
          std::vector<string> overlap_img_files(overlap_indices.size());
          std::vector<string> overlapShadowFilesArray(overlap_indices.size());
          for (unsigned int j = 0; j < overlap_indices.size(); j++){
               overlapParamsArray[j] = modelParamsArray[overlap_indices[j]];
               overlap_img_files[j] = input_files[overlap_indices[j]];
               overlapShadowFilesArray[j] = modelParamsArray[overlap_indices[j]].shadowFilename;
	  }
                     
          if (globalParams.reflectanceType == NO_REFL){
              //no use of the reflectance map
            
             UpdateImageMosaic( modelParamsArray[i], overlapParamsArray, globalParams);
            
	  }
          else{
              //use reflectance
              ComputeAlbedoMap(modelParamsArray[i].inputFilename, 
                               modelParamsArray[i].meanDEMFilename, 
                               modelParamsArray[i].shadowFilename, 
                               overlap_img_files,
			       modelParamsArray[i], 
                               overlapParamsArray,
			       overlapShadowFilesArray, 
                               modelParamsArray[i].outputFilename,
                               globalParams);
	}
	  	 
     }


     //TO DO: re-estimate the shape
     //for (unsigned int i = 1; i < input_files.size(); ++i) {
     //   printf("iter = %d, shape, i = %d, filename = %s\n\n", iter, i, input_files[i].c_str());          
     //}

     if (globalParams.computeErrors == 1){
       
        //compute the errors
       float overallAvgError = 0; 
       int overallNumSamples = 0;

       for (unsigned int i = 0; i < input_files.size(); ++i) {
          
          //printf("expT[%d] = %f\n", i, modelParamsArray[i].exposureTime);
	  vector<int> overlap_indices;
         
          overlap_indices = makeOverlapList(input_indices, i, globalParams.maxPrevOverlappingImages, globalParams.maxNextOverlappingImages );
         
          std::vector<modelParams> overlapParamsArray(overlap_indices.size());
          std::vector<string> overlap_img_files(overlap_indices.size());
          std::vector<string> overlapShadowFilesArray(overlap_indices.size());
          for (unsigned int j = 0; j < overlap_indices.size(); j++){
               overlapParamsArray[j] = modelParamsArray[overlap_indices[j]];
               overlap_img_files[j] = input_files[overlap_indices[j]];
               overlapShadowFilesArray[j] = modelParamsArray[overlap_indices[j]].shadowFilename;
	  }
           
         
          //use reflectance
          float avgError;
          int numSamples;
          ComputeAlbedoErrorMap(modelParamsArray[i].inputFilename, 
                                modelParamsArray[i].meanDEMFilename, 
                                modelParamsArray[i].shadowFilename, 
                                modelParamsArray[i].outputFilename,
                                overlap_img_files, 
			        modelParamsArray[i], 
                                overlapParamsArray,
			        overlapShadowFilesArray,
                                modelParamsArray[i].errorFilename, 
                                globalParams, &avgError, &numSamples);
           
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
       /*
       for (unsigned k=0; k < origImage.rows()+1; ++k) {
         for (unsigned l=0; l < origImage.cols()+1; ++l) {
	   printf("dem(%d, %d) = %f, %f\n", l, k, (float)init_dem(l,k), (float)dem(l,k) );
	 }
       }    
       */
  }
  #endif

}


















