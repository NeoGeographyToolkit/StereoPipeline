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
#include <vw/Photometry/Index.h>
#include <vw/Photometry/Reconstruct.h>
#include <vw/Photometry/Reflectance.h>
#include <vw/Photometry/Shadow.h>
#include <vw/Photometry/Shape.h>
#include <vw/Photometry/ShapeFromShading.h>
#include <vw/Photometry/Weights.h>

using namespace vw;
using namespace vw::math;
using namespace vw::cartography;
/*
#include "shape_from_shading.h"
#include "shape.h"
#include "albedo.h"
#include "exposure.h"
#include "reflectance.h"
#include "reconstruct.h"
#include "shadow.h"
#include "index.h"
#include "weights.h"
*/
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



//subsamples a geo referenced tiff image by two 
void subsample_image(std::string output_file, std::string input_file) {
	GeoReference geo;
	read_georeference(geo, input_file);
        DiskImageView<PixelMask<PixelGray<uint8> > >  image(input_file); 
	int cols = (image.cols()+1)/2, rows = (image.rows()+1)/2;
	ImageView<PixelMask<PixelGray<uint8> > > tm_image(cols, rows);

      
        ImageViewRef<PixelMask<PixelGray<uint8> > >  interp = interpolate(edge_extend(image.impl(), 
                                                                         ConstantEdgeExtension()), 
                                                                         BilinearInterpolation());
       
	int x, y;

	for (x=0; x<cols; ++x){
	  for (y=0; y<rows; ++y){
	      //if ( is_valid(image(2*x,2*y)) || is_valid(image(2*x+1,2*y)) || is_valid(image(2*x,2*y+1)) || is_valid(image(2*x+1,2*y+1)) ) {
	      //if (is_valid(image(2*x,2*y)) && is_valid(image(2*x+1,2*y)) && is_valid(image(2*x,2*y+1)) && is_valid(image(2*x+1,2*y+1)) ) {
	    if ( is_valid(interp(2*x+0.5, 2*y+0.5)) ){
		  tm_image(x,y) = interp(2*x+0.5, 2*y+0.5);
	      } 
              else{
		  tm_image(x,y).invalidate();
	      }
            }
	}
	
	Matrix<double> H = geo.transform();
	H(0,0) *= 2;
	H(1,1) *= 2;
	geo.set_transform(H);
	
	write_georeferenced_image(output_file, tm_image, geo, TerminalProgressCallback("{Core}","Processing:"));
}


// Given two images and two georeferences, this function picks a set
// of matching pixel samples between the two images.  It rejects
// pixels that are not valid, and it should probably also reject
// pixels that are near saturation (though it does not yet!).
template <class ViewT>
std::vector<Vector4> sample_images(ImageViewBase<ViewT> const& image1, 
                                   ImageViewBase<ViewT> const& image2,
                                   GeoReference const& geo1,
                                   GeoReference const& geo2,
                                   int num_samples,
                                   std::string const& DEM_file, 
                                   std::vector<Vector3> *normalArray,
                                   std::vector<Vector3> *xyzArray ) {
  int sample = 0;
  int numtries = 0;
  std::vector<Vector4> result;
  
  // Random numbers
  srandom((unsigned int) clock());
  float rand_max = pow(2.0,31)-1;
  
  ImageViewRef<typename ViewT::pixel_type> interp_image1 = interpolate(edge_extend(image1.impl(), 
                                                                       ConstantEdgeExtension()), 
                                                                       BilinearInterpolation());
  ImageViewRef<typename ViewT::pixel_type> interp_image2 = interpolate(edge_extend(image2.impl(), 
                                                                       ConstantEdgeExtension()), 
                                                                       BilinearInterpolation());

  // This block of code samples the images comprehensively, adding a
  // sample pair for every valid pixel in interp_image1.
  // for (unsigned j=0; j < interp_image1.rows(); ++j) {
  //   for (unsigned i=0; i < interp_image1.cols(); ++i) {
  //     Vector2 sample_pix1(i,j);
  //     Vector2 sample_pix2 = geo2.lonlat_to_pixel(geo1.pixel_to_lonlat(sample_pix1));
      
  //     // Check to see whether these pixels are valid
  //     typename ViewT::pixel_type pix1 = interp_image1(sample_pix1[0], sample_pix1[1]);
  //     typename ViewT::pixel_type pix2 = interp_image2(sample_pix2[0], sample_pix2[1]);
  //     if ( is_valid(pix1) && is_valid(pix2) &&
  //          pix1[0] > 10 && pix1[0] < 245 &&
  //          pix2[0] > 10 && pix2[0] < 245 ) {
  //       result.push_back(Vector2(pix1[0],pix2[0]));
  //       ++sample;
  //       //        std::cout << result[result.size()-1][0] << " " << result[result.size()-1][1] << "\n";
  //     }
  //     ++numtries;
  //   }
  // }
  

  //added by Ara to support DEMs - START
  DiskImageView<PixelGray<float> >  dem_image(DEM_file); 
  GeoReference GR;
  read_georeference(GR, DEM_file);
  //added by Ara to support DEMs - END

  // This block of code samples the images randomly, gathering up to
  // num_samples samples from the images. 
  while (sample < num_samples && numtries < num_samples*10) {
    
    Vector2 sample_pix1(float(random())/rand_max * image1.impl().cols(),
                        float(random())/rand_max * image1.impl().rows());
    Vector2 sample_pix2 = geo2.lonlat_to_pixel(geo1.pixel_to_lonlat(sample_pix1));

    Vector2 sample_pix_dem = GR.lonlat_to_pixel(geo1.pixel_to_lonlat(sample_pix1));

    Vector2 lonlat = geo1.pixel_to_lonlat(sample_pix1);
   
    // Check to see whether these pixels are valid
    typename ViewT::pixel_type pix1 = interp_image1(sample_pix1[0], sample_pix1[1]);
    typename ViewT::pixel_type pix2 = interp_image2(sample_pix2[0], sample_pix2[1]);
    if ( is_valid(pix1) && is_valid(pix2) ) {

       //result.push_back(Vector4(pix1[0],pix2[0],lonlat[0],lonlat[1]));
       
       int x = (int)sample_pix_dem[0];
       int y = (int)sample_pix_dem[1];

       if (x < 0){
           x = 0;
       }
       if (x > dem_image.cols()-1){
	   x = dem_image.cols()-1; 
       }
       if (y < 0){
           y = 0;
       }
       if (y > dem_image.rows()-1){
	   y = dem_image.rows()-1; 
       }
      
       Vector3 longlat3(lonlat(0),lonlat(1),(dem_image)(x, y));
       Vector3 xyz = geo1.datum().geodetic_to_cartesian(longlat3);
       
       Vector2 sample_pix_dem_left;
       sample_pix_dem_left(0) = x-1;
       if (sample_pix_dem_left(0) < 0){
	  sample_pix_dem_left(0) = 0;
	  //break;
       }
       sample_pix_dem_left(1) = y;
       lonlat = GR.pixel_to_lonlat(sample_pix_dem_left);
    
       Vector3 longlat3_left(lonlat(0),lonlat(1),(dem_image)(sample_pix_dem_left(0), sample_pix_dem_left(1)));
       Vector3 xyz_left = geo1.datum().geodetic_to_cartesian(longlat3_left);
       
       Vector2 sample_pix_dem_top;
       sample_pix_dem_top(0) = x;
       sample_pix_dem_top(1) = y-1;
       if (sample_pix_dem_top(1) < 0){
	 sample_pix_dem_top(1) = 0;
	 //break;
       }

       lonlat = GR.pixel_to_lonlat(sample_pix_dem_top);     
       Vector3 longlat3_top(lonlat(0),lonlat(1),(dem_image)(sample_pix_dem_top(0), sample_pix_dem_top(1)));
       Vector3 xyz_top = geo1.datum().geodetic_to_cartesian(longlat3_top);
       
       

       Vector3 normal = computeNormalFrom3DPoints(xyz, xyz_left, xyz_top);

       //printf("normal:%f %f %f\n", normal(0), normal(1), normal(2));
       //printf("%f %f %f\n", loc_longlat3(0), loc_longlat3(1), loc_longlat3(2));
       
       result.push_back(Vector4(pix1[0],pix2[0],lonlat[0],lonlat[1]));
        
       normalArray->push_back(normal);
       xyzArray->push_back(xyz);
       
       ++sample;
       //      std::cout << result[result.size()-1][0] << " " << result[result.size()-1][1] << "\n";
    } 
    ++numtries;
  }
  return result;
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
 
  DiskImageView<PixelMask<PixelGray<uint8> > >  input_img(string("../data/lola/ldem_45N_100m.tif")); 

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


  printf("NUM FILES = %d\n", input_files.size() );
  GlobalParams globalParams;
  globalParams.reflectanceType = NO_REFL;
  //globalParams.reflectanceType = LUNAR_LAMBERT;
  //globalParams.reflectanceType = LAMBERT;
  globalParams.slopeType = 1;
  globalParams.shadowThresh = 40;
  string exposureInfoFilename = "../results/exposure/exposureInfo_0.txt";
  //string spacecraftPosFilename = "";
  //string sunPosFilename = "";
  globalParams.albedoInitType = 1;
  globalParams.exposureInitType = 0;
  globalParams.exposureInitRefValue = 1.2; //initial estimate of the exposure time for the reference frame
  globalParams.exposureInitRefIndex = 0;   //the reference frame
  globalParams.DEMInitType = 1;//0;
  globalParams.shadowInitType = 0;//1;
  //globalParams.re_estimateExposure = 1;
  //globalParams.re_estimateAlbedo = 1;
  globalParams.computeErrors = 0;//1;
  globalParams.useWeights = 0;//1;
  globalParams.maxNumIter = 10;
  globalParams.maxNextOverlappingImages = 2;
  globalParams.maxPrevOverlappingImages = 2;

  //reflectanceConst = 60;
  //add all filename;

  //create the downsampled DRG files
  std::vector<std::string> DRG_sub4_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       
       std::string temp = sufix_from_filename(input_files[i]);
       DRG_sub4_files[i] = "../data/orbit33_DRG_sub4" + temp;
       printf("DRG_sub4_files[%d] = %s\n", i, DRG_sub4_files[i].c_str());
       
  }

  // Create the output file names
  std::vector<std::string> output_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       
       std::string temp = sufix_from_filename(input_files[i]);
       output_files[i] = "../results/albedo" + prefix_from_filename(temp) + "_TM.tif";
       //printf("output_files[%d] = %s\n", i, output_files[i].c_str());
       
  }

  // Create the error file names
  std::vector<std::string> error_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       std::string temp = sufix_from_filename(input_files[i]);
       error_files[i] = "../results/error" + prefix_from_filename(temp) + "_err.tif";
  }
 
  // Create the index file names
  std::vector<std::string> index_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       std::string temp = sufix_from_filename(input_files[i]);
       index_files[i] = "../results/index/" + prefix_from_filename(temp) + "_index.tif";
  }

  // Create the shadow map file names
  std::vector<std::string> shadow_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       std::string temp = sufix_from_filename(input_files[i]);
       shadow_files[i] = "../results/shadow/" + prefix_from_filename(temp) + "_shadow.tif";
  }

  // Create the reflectance file names
  std::vector<std::string> reflectance_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       std::string temp = sufix_from_filename(input_files[i]);
       reflectance_files[i] = "../results/reflectance" + prefix_from_filename(temp) + "_reflectance.tif";
  }

  // Create the DEM error file names
  std::vector<std::string> DEM_var2_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       std::string temp = sufix_from_filename(input_files[i]);
       DEM_var2_files[i] = "../results/DEM_errors/" + prefix_from_filename(temp) + "_DEM_err.tif";
  }

  //Create the DEM filenames
  std::vector<std::string> DEM_files(input_files.size());
  std::vector<std::string> mean_DEM_files(input_files.size());
  std::vector<std::string> var2_DEM_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       std::string temp = sufix_from_filename(input_files[i]);
       DEM_files[i] = "../data/orbit33_DEM" + prefix_less3_from_filename(temp) + "DEM.tif";
       mean_DEM_files[i] = "../results/DEM" + prefix_less3_from_filename(temp) + "DEM_out.tif";
       var2_DEM_files[i] = "../results/DEM" + prefix_less3_from_filename(temp) + "DEM_var2.tif";
       //printf("mean_DEM_files[%d] = %s, var2_DEM_files[%d] = %s\n", i, mean_DEM_files[i].c_str(), i, var2_DEM_files[i].c_str());
  }

  std::vector<Vector3> sunPositions;
  char *sunPosFilename = new char[500];
  strcpy(sunPosFilename, "sunpos.txt");
  sunPositions = ReadSunPosition(sunPosFilename, input_files.size());
  delete sunPosFilename; 
  
  std::vector<Vector3> spacecraftPositions;
  
  char *spacecraftPosFilename = new char[500];
  strcpy(spacecraftPosFilename, "spacecraftpos.txt");
  spacecraftPositions = ReadSpacecraftPosition(spacecraftPosFilename, input_files.size());
  delete spacecraftPosFilename; 
  
 
  std::vector<modelParams> modelParamsArray(input_files.size());
  std::vector<float> avgReflectanceArray(input_files.size());
 #if 0 
 //downsample the original large DRG images by a factor of four
 for (unsigned int i = 40; i < input_files.size(); ++i) {
     std::string tmp_file_sub2 = "../data/tmp_sub2.tiff";
     std::string tmp_file_sub4 = "../data/tmp_sub4.tiff";
     std::string tmp_file_sub8 = "../data/tmp_sub8.tiff";
     printf ("start downsample image %d\n", i);
     subsample_image(tmp_file_sub2, input_files[i]);
     printf("by 2\n");
     subsample_image(tmp_file_sub4, tmp_file_sub2);
     printf("by 4\n");
     subsample_image(tmp_file_sub8, tmp_file_sub4);
     printf("by 8\n");
     subsample_image(DRG_sub4_files[i], tmp_file_sub8);   
     printf("by 16\n"); 
 }
#endif


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
      modelParamsArray[i].infoFilename = "../results/info/"+prefix_less3_from_filename(temp)+".txt";

      printf("Computing the weights...");
      if (globalParams.useWeights == 1){         
         modelParamsArray[i].centerLineDEM = ComputeDEMCenterLine(DEM_files[i], &(modelParamsArray[i].maxDistArrayDEM));
         modelParamsArray[i].centerLine = ComputeImageCenterLine(input_files[i], &(modelParamsArray[i].maxDistArray));
      }
      printf("done computing the weights.\n");

      //modelParamsArray[i].horCenterLineDEM = ComputeDEMHorCenterLine(DEM_files[i], &(modelParamsArray[i].maxVerDistArrayDEM));
      //modelParamsArray[i].horCenterLine = ComputeImageHorCenterLine(input_files[i], &(modelParamsArray[i].maxVerDistArray));

      //string exposureInfoFilename = "../results/exposure/exposureInfo.txt";
      //AppendExposureInfoToFile(exposureInfoFilename, input_files[i], modelParamsArray[i]);
  }
 /*
  //add shadows back to the processed images
  //temporary functionality
  for (unsigned int i = 0; i < input_files.size(); ++i) {
     if (i==50){
        printf("input_file = %s\n", input_files[i].c_str());
        AddShadows(input_files[i],  output_files[i], shadow_files[i]);
     }
   }
 */
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
       std::vector<std::string> overlap_DEM_files(overlap_indices.size());
  
       for (unsigned int j = 0; j < overlap_indices.size(); j++){
            overlap_params[j] = modelParamsArray[overlap_indices[j]];
            overlap_DEM_files[j] = DEM_files[overlap_indices[j]];
       }
  
       InitDEM(DEM_files[i], mean_DEM_files[i], var2_DEM_files[i], modelParamsArray[i], 
	       overlap_DEM_files,  overlap_params, globalParams);
    
    }

   }
#if 0
   if (globalParams.shadowInitType == 1){
       //estimate the shadows
      printf("start computing the shadow maps ...\n");  
      for (unsigned int i = 0; i < input_files.size(); i++){
           printf("i = %d, filename = %s\n", i, input_files[i].c_str());
           ComputeSaveShadowMap (input_files[i], shadow_files[i], globalParams);
      }
      printf("done computing the shadow maps!\n");
   }
 

  //compute the reflectance images
  if (globalParams.exposureInitType == 1){

    printf("compute reflectance ...\n");
    modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue; //1.2;//1.0;
  
    for (unsigned int i = 0; i < input_files.size(); ++i) {

       printf("%s\n", reflectance_files[i].c_str());
       avgReflectanceArray[i] = computeImageReflectance(input_files[i], mean_DEM_files[i], shadow_files[i],  
			                                modelParamsArray[i], reflectance_files[i], globalParams);

       if (i==0){
	 modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue; //1.2;//1.0;
       }
       else{
          modelParamsArray[i].exposureTime = (modelParamsArray[0].exposureTime*avgReflectanceArray[0])/avgReflectanceArray[i];
       }
       printf("exposure Time = %f\n", modelParamsArray[i].exposureTime);
    
       AppendExposureInfoToFile(exposureInfoFilename, input_files[i], modelParamsArray[i]);
    }
  }
  
  if (globalParams.exposureInitType == 2){
    //initialize the exposure times from overlaping areas - Not working well and not clear why?
    printf("compute reflectance ...\n");
    for (unsigned int i = 0; i < input_files.size(); ++i) {
       float reflectanceRatio;
       if (i==0){
	 modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue;//1.2;
         
         reflectanceRatio = computeImageReflectance(input_files[i], input_files[i], 
                                                    mean_DEM_files[i], 
                                                    shadow_files[i], shadow_files[i], 
                                                    modelParamsArray[i], modelParamsArray[i],
	                                            reflectance_files[i], globalParams);
	 
       }
       else{
         reflectanceRatio = computeImageReflectance(input_files[i], input_files[i-1], 
                                                    mean_DEM_files[i], 
                                                    shadow_files[i], shadow_files[i-1], 
                                                    modelParamsArray[i], modelParamsArray[i-1],
                                                    reflectance_files[i], globalParams);
         
	 modelParamsArray[i].exposureTime = (modelParamsArray[i-1].exposureTime)/reflectanceRatio;
       }
       printf("exposure Time = %f\n", modelParamsArray[i].exposureTime);
    
       AppendExposureInfoToFile(exposureInfoFilename, input_files[i], modelParamsArray[i]);
    }
  }
 
  if (globalParams.exposureInitType == 3){
    string initExpTimeFile = "exposureTime.txt";
    std ::vector<float> expTimeArray = ReadExposureInfoFile(initExpTimeFile, input_files.size());
    modelParamsArray[0].exposureTime = globalParams.exposureInitRefValue;//1.2;//1.0;
    for (unsigned int i = 1; i < input_files.size(); ++i) {
         //modelParamsArray[i].exposureTime = expTimeArray[i];
         modelParamsArray[i].exposureTime = (modelParamsArray[0].exposureTime*expTimeArray[i])/expTimeArray[0];
         printf("expTimeArray[%d] = %f\n", i,  modelParamsArray[i].exposureTime);
         AppendExposureInfoToFile(exposureInfoFilename, input_files[i], modelParamsArray[i]);
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
             InitImageMosaicByBlocks(input_files[i], modelParamsArray[i], shadow_files[i], 
                                     output_files[i], overlap_img_files, overlap_params, globalParams);
	 }
         else{
             InitAlbedoMap(input_files[i], modelParamsArray[i], mean_DEM_files[i], 
		           shadow_files[i], output_files[i], overlap_img_files,
                           overlap_params, globalParams);
	 }
      
    }
  
    printf("done initializing the albedo!\n");
  }
  //---------------------------------------------------------------------------------


 //#if 0
  //re-estimate the parameters of the image formation model
  float overallError;
  

  for (unsigned int iter = 1; iter < globalParams.maxNumIter; iter++){
      
    string currExposureInfoFilename;
    string prevExposureInfoFilename;

    char* currExposureInfoFilename_char = new char[500];
    char* prevExposureInfoFilename_char = new char[500];

    sprintf (currExposureInfoFilename_char, "../results/exposure/exposureInfo_%d.txt", iter);
    sprintf (prevExposureInfoFilename_char, "../results/exposure/exposureInfo_%d.txt", iter-1);
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
              ComputeExposure(input_files[i], output_files[i], &modelParamsArray[i], globalParams);
	  }
          else{
              //use reflectance map
              ComputeExposure(input_files[i], output_files[i], mean_DEM_files[i], &modelParamsArray[i], globalParams);
	  }

          //create the exposureInfoFilename 
          AppendExposureInfoToFile(currExposureInfoFilename, input_files[i], modelParamsArray[i]);
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
               overlapShadowFilesArray[j] = shadow_files[overlap_indices[j]];
	  }
                     
          if (globalParams.reflectanceType == NO_REFL){
              //no use of the reflectance map
              UpdateImageMosaic(input_files[i], shadow_files[i], overlap_img_files, 
			       modelParamsArray[i], overlapParamsArray,
			       overlapShadowFilesArray, output_files[i],
                               globalParams);
	  }
          else{
              //use reflectance
              ComputeAlbedoMap(input_files[i], mean_DEM_files[i], 
                               shadow_files[i], overlap_img_files, 
			       modelParamsArray[i], overlapParamsArray,
			       overlapShadowFilesArray, output_files[i],
                               globalParams);
	  }
	  	 
       }
  

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
               overlapShadowFilesArray[j] = shadow_files[overlap_indices[j]];
	  }
           
         
          //use reflectance
          float avgError;
          int numSamples;
          ComputeAlbedoErrorMap(input_files[i], mean_DEM_files[i], 
                                shadow_files[i], output_files[i],
                                overlap_img_files, 
			        modelParamsArray[i], overlapParamsArray,
			        overlapShadowFilesArray,
                                error_files[i], globalParams,
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
       //TO DO: re-estimate the shape
       //for (unsigned int i = 1; i < input_files.size(); ++i) {
       //   printf("iter = %d, shape, i = %d, filename = %s\n\n", iter, i, input_files[i].c_str());          
       //}
       /*
       //re-estimate the reconstruction error
       float totalError = 0.0;
       for (unsigned int i = 0; i < input_files.size(); ++i) {
	   printf("compute save error i = %d\n", i);
           float imageError;
	   ComputeSaveAlbedoError(input_files[i], output_files[i], mean_DEM_files[i], error_files[i], 
                                  modelParamsArray[i], &imageError, modelParamsArray[i], globalParams);
        
           totalError = totalError + imageError;
       }

       printf("totalError = %f\n", totalError);
       */
  }
  
  ////////////////////////////////// 
  //Save the results.
  /*
  //save the first image
  DiskImageView<PixelMask<PixelGray<uint8> > > image(input_files[0]); 
  GeoReference geo;
  read_georeference(geo, input_files[0]); 
  // ImageView<PixelMask<PixelGray<float> > > tm_image(image.cols(), image.rows());

  write_georeferenced_image(output_files[0], 
                            channel_cast<uint8>(clamp(image,0.0,255.0)),
                            geo, TerminalProgressCallback());

  */
  // #endif
  

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
#endif
}


















