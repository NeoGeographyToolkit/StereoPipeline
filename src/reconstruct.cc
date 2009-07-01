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
using namespace vw;
using namespace vw::math;
using namespace vw::cartography;

typedef struct modelParams{
  float exposureTime;
  Vector2 cameraParams;
  Vector3 sunPosition; //relative to the center of the Moon
  Vector2 rescalingParams;
};

/// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
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
                                  int num_samples) {
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
  
  // This block of code samples the images randomly, gathering up to
  // num_samples samples from the images. 
  while (sample < num_samples && numtries < num_samples*10) {
    Vector2 sample_pix1(float(random())/rand_max * image1.impl().cols(),
                        float(random())/rand_max * image1.impl().rows());
    Vector2 sample_pix2 = geo2.lonlat_to_pixel(geo1.pixel_to_lonlat(sample_pix1));
    Vector2 lonlat = geo1.pixel_to_lonlat(sample_pix1);

    // Check to see whether these pixels are valid
    typename ViewT::pixel_type pix1 = interp_image1(sample_pix1[0], sample_pix1[1]);
    typename ViewT::pixel_type pix2 = interp_image2(sample_pix2[0], sample_pix2[1]);
    if ( is_valid(pix1) && is_valid(pix2) ) {
      result.push_back(Vector4(pix1[0],pix2[0],lonlat[0],lonlat[1]));
      ++sample;
      //      std::cout << result[result.size()-1][0] << " " << result[result.size()-1][1] << "\n";
    } 
    ++numtries;
  }
  return result;
}

//generates the 3D coordinates of a point from longitude and latitude on the Moon
//Vector2 lon_lat is a 2D vector. First element is the longitude and the second the latitude 
//author Ara Nefian
Vector3 get_normal(Vector2 lon_lat){

  Vector3 xyz;
  float x, y, z;
  float rho;
  float longitude, latitude; 

  longitude = lon_lat[0]*3.14/180;
  latitude = lon_lat[1]*3.14/180;
  rho = 1738; //kilometers 
  x = rho*cos(longitude)*cos(latitude);
  y = rho*sin(longitude)*cos(latitude);
  z = rho*sin(latitude);
  
  xyz[0] = x;
  xyz[1] = y;
  xyz[2] = z;

  return xyz;
}

//reads the 3D position of the Sun in Moon centric cordinates
//char *filename: the filename with the sun positions (sunposition.txt)
//int numEntres: number of entries in the sun position file  
//author: Ara Nefian
std ::vector<Vector3> ReadSunPosition(char *filename, int numEntries)
{
  std::vector<Vector3> sunPositions(numEntries);

  FILE *fp;
  fp = fopen(filename, "r");

  for (unsigned int i = 0; i < numEntries; i++){
  
      float x_l, y_l, z_l;
      float x_r, y_r, z_r;
      if (i == 0){
          fscanf(fp, "%s %f %f %f\n", filename, &x_l, &y_l, &z_l);
          //printf("%s %f %f %f\n", filename, x_l, y_l, z_l);
          fscanf(fp, "%s %f %f %f\n", filename, &x_r, &y_r, &z_r);
          //printf("%s %f %f %f\n", filename, x_r, y_r, z_r);
      }
      else{
         x_l = x_r;
         y_l = y_r;
         z_l = z_r;
         fscanf(fp, "%s %f %f %f\n", filename, &x_r, &y_r, &z_r);
    }
    sunPositions[i][0] = (x_l+x_r)/2;
    sunPositions[i][1] = (y_l+y_r)/2;
    sunPositions[i][2] = (z_l+z_r)/2;

  }
  fclose(fp);
  return sunPositions;
}

//computes the cosine of the light direction and the normal to the Moon
//Vector3  sunpos: the 3D coordinates of the Sun relative to the center of the Moon
//Vector2 lon_lat is a 2D vector. First element is the longitude and the second the latitude 
//author Ara Nefian
float computeReflectance(Vector3 sunPos, Vector2 lon_lat)
{
  float reflectance;

  Vector3 xyz = get_normal(lon_lat);

  Vector3 sunDirection; // sun coordinates relative to the xyz point on the Moon surface
  sunDirection[0] = sunPos[0]-xyz[0];
  sunDirection[1] = sunPos[1]-xyz[1];
  sunDirection[2] = sunPos[2]-xyz[2];
    
  float sunDirectionNorm;
  sunDirectionNorm = sqrt(sunDirection[0]*sunDirection[0] + sunDirection[1]*sunDirection[1] + sunDirection[2]*sunDirection[2]);

    //printf("sun_direction norm: %f, 0: %f, 1: %f, 2: %f\n", sun_direction_norm, sun_direction[0], sun_direction[1], sun_direction[2]);  
    sunDirection[0]=sunDirection[0]/sunDirectionNorm;
    sunDirection[1]=sunDirection[1]/sunDirectionNorm;
    sunDirection[2]=sunDirection[2]/sunDirectionNorm;
    //printf("sun_direction norm: %f, 0: %f, 1: %f, 2: %f\n", sun_direction_norm, sun_direction[0], sun_direction[1], sun_direction[2]);    
 
    float xyzNorm;
    xyzNorm = sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2]);

    xyz[0] = xyz[0]/xyzNorm;
    xyz[1] = xyz[1]/xyzNorm;
    xyz[2] = xyz[2]/xyzNorm;
    //printf("xyz 0: %f, 1: %f, 2: %f\n", xyz[0], xyz[1], xyz[2]); 
   
    reflectance = sunDirection[0]*xyz[0] + sunDirection[1]*xyz[1] + sunDirection[2]*xyz[2];
    
    return reflectance;
}



//estimates the exposure time from a set of images given a known albedo map 
//author: Ara Nefian
void ComputeExposureTime(std::vector<Vector4> const& samples, modelParams prevModelParams, modelParams* currModelParams, float *error) {
 
  float exposureTime;
  double sum_xx, sum_xy;
  float l_error = 0.0;

  sum_xx = 0.0;
  sum_xy = 0.0;
  
 
  for (int i = 0; i < samples.size(); ++i) {
    
    //printf("samples[%d](0) = %f, samples[%d](1) = %f\n",i, samples[i](0), i, samples[i](1));
    float longitude = samples[i](2);
    float latitude  = samples[i](3);

    Vector2 lon_lat;
    lon_lat[0] = longitude;
    lon_lat[1] = latitude;

    float curr_sun_correction = computeReflectance(currModelParams->sunPosition, lon_lat);
    float prev_sun_correction = computeReflectance(prevModelParams.sunPosition, lon_lat);
    
    curr_sun_correction = 1.0;
    prev_sun_correction = 1.0;
    //printf("prev_sun_correction = %f, prev_exp_time = %f\n", prev_sun_correction, prev_exp_time(0));

    //float albedo = samples[i](1)/(prev_sun_correction* prevModelParams.exposureTime);
   
    float albedo = samples[i](0)*prevModelParams.exposureTime*prev_sun_correction + samples[i](1)*currModelParams->exposureTime*curr_sun_correction;

    float temp_den = prev_sun_correction* prevModelParams.exposureTime * prev_sun_correction* prevModelParams.exposureTime + 
                     curr_sun_correction* currModelParams->exposureTime * curr_sun_correction* currModelParams->exposureTime;
    albedo = albedo/temp_den;

    if (albedo < 0){
       printf("albedo = %f\n", albedo);
       albedo = 0;
    }
    if ((fabs(samples[i](0) - samples[i](1)) < 200)){
   
        sum_xy = sum_xy + albedo*samples[i](1)*curr_sun_correction ;
        sum_xx = sum_xx + albedo*albedo*curr_sun_correction*curr_sun_correction;  
    }
    
    float temp1 = (samples[i](0)-albedo*prevModelParams.exposureTime);
    float temp2 = (samples[i](1)-albedo*currModelParams->exposureTime);
    l_error = l_error + (temp1*temp1 + temp2*temp2)/samples.size();

  }
  
  exposureTime = sum_xy/sum_xx;
 
  currModelParams->exposureTime = exposureTime;
  *error = l_error;
}


//input_files[i], input_files[i-1], output_files[i], output_files[i-1]
//writes the albedo of the current image in the area of overlap with the previous mage
//writes the previous image in the area of overal with the current image
void ComputeSaveAlbedoMap(std::string curr_input_file, std::string prev_input_file, 
                          std::string curr_output_file, 
                          modelParams currModelParams, modelParams prevModelParams)
{
    DiskImageView<PixelMask<PixelGray<uint8> > > curr_image(curr_input_file);
    DiskImageView<PixelMask<PixelGray<uint8> > > prev_image(prev_input_file);
   

    GeoReference prev_geo, curr_geo;
    read_georeference(prev_geo, prev_input_file);
    read_georeference(curr_geo, curr_input_file);
  
    ImageView<PixelMask<PixelGray<float> > > tm_image(curr_image.cols(), curr_image.rows());
 
    printf("exposure_time = %f, a_rescale = %f, b_rescale = %f\n", 
            currModelParams.exposureTime, currModelParams.rescalingParams[0], currModelParams.rescalingParams[1]);
    
    ImageViewRef<PixelMask<PixelGray<uint8> > >  interp_prev_image = interpolate(edge_extend(prev_image.impl(), 
                                                                                 ConstantEdgeExtension()), 
                                                                                 BilinearInterpolation());
         
    for (unsigned k=0; k < curr_image.rows(); ++k) {
      for (unsigned l=0; l < curr_image.cols(); ++l) {
	 
         Vector2 curr_sample_pix(l,k);
         
       
         if ( is_valid(curr_image(l,k)) ) {
           
           //this is a default value if there is no overalp between consecutive images.
           tm_image(l, k) = (float)curr_image(l, k)/currModelParams.exposureTime; 
	  
           //find this point in the other image
	   Vector2 lon_lat = curr_geo.pixel_to_lonlat(curr_sample_pix);
          
           Vector2 prev_sample_pix = prev_geo.lonlat_to_pixel(lon_lat);
           prev_sample_pix[0] = floor(prev_sample_pix[0]);
           prev_sample_pix[1] = floor(prev_sample_pix[1]);

           PixelMask<PixelGray<uint8> > prev_image_pixel = interp_prev_image(prev_sample_pix[0], prev_sample_pix[1]);
       
           
           if ( is_valid (prev_image_pixel) /*&& (prev_sample_pix[0] >= 0) && (prev_sample_pix[1] >= 0)*/) {
	        
	       float currSunCorrection = computeReflectance(currModelParams.sunPosition, lon_lat);
            
	       float prevSunCorrection = computeReflectance(prevModelParams.sunPosition, lon_lat);
             
               currSunCorrection = 1.0;
               prevSunCorrection = 1.0;
              
	       float albedo = (int)(channel_cast<uint8>(prev_image)((int)prev_sample_pix(0), (int)prev_sample_pix(1)))*prevModelParams.exposureTime*prevSunCorrection + 
			      (int)(channel_cast<uint8>(curr_image(l,k)))*currModelParams.exposureTime*currSunCorrection;
             
	       float temp_den = prevSunCorrection* prevModelParams.exposureTime * prevSunCorrection* prevModelParams.exposureTime + 
		                currSunCorrection* currModelParams.exposureTime * currSunCorrection* currModelParams.exposureTime;
              
              
               if (temp_den == 0){
		   temp_den = 0.001;
               }

               albedo = albedo/temp_den;      
	       int int_albedo = (int)floor(albedo);
	       
               if (albedo < 0){
		  printf("albedo = %f\n", albedo);
		  albedo = 0;
	       }

               tm_image(l, k) = (int)floor(albedo);  
	   }

	 }
       }
    }
    
    printf("curr_output_file = %s\n", curr_output_file.c_str());
    
    write_georeferenced_image(curr_output_file, 
                              channel_cast<uint8>(clamp(tm_image,0.0,255.0)),
                              curr_geo, TerminalProgressCallback());

 
}

void match_images(std::string file1, std::string file2, int num_matches, modelParams prevModelParams, modelParams *currModelParams, float*error) {
  std::cout << "Matching " << file2 << " to " << file1 << "\n";

  DiskImageView<PixelMask<PixelGray<uint8> > > image1(file1);
  DiskImageView<PixelMask<PixelGray<uint8> > > image2(file2);
  GeoReference geo1, geo2;
  read_georeference(geo1, file1);
  read_georeference(geo2, file2);

  std::vector<Vector4> samples = sample_images(image1, image2, geo1, geo2, num_matches);
  
  ComputeExposureTime(samples, prevModelParams, currModelParams, error);
  
}

int main( int argc, char *argv[] ) {
  //std::vector<std::string> temp_input_files;
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
  usage << "Usage: reconstruct [options] <filename1> <filename2> ..." << std::endl << std::endl;
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

  /*
  //reverse the input files
  std::vector<std::string> input_files(temp_input_files.size());
  for (unsigned i = 0; i < temp_input_files.size(); ++i) {
    printf("%s\n", temp_input_files[i].c_str());
    input_files[i] = temp_input_files[temp_input_files.size() - i-1];
    printf("%s\n", input_files[i].c_str());
  }
  */

  // Create the output file names
  std::vector<std::string> output_files(input_files.size());
  for (unsigned i = 0; i < input_files.size(); ++i) {
       output_files[i] = prefix_from_filename(input_files[i]) + "_TM.tif";
  }
 

  std::vector<Vector3> sunPositions;
  char *filename = new char[500];
  strcpy(filename, "sunpos.txt");
  sunPositions = ReadSunPosition(filename, input_files.size());
  delete filename; 

  for (unsigned int i = 0; i < input_files.size(); i++){
        printf("i:%d, sunpos[0]=%f, sunpos[1]=%f, sunpos[2]=%f\n", i, sunPositions[i][0], sunPositions[i][1], sunPositions[i][2]);
  }

  std::vector<modelParams> modelParamsArray(input_files.size());
   
  for (unsigned int i = 0; i < input_files.size(); ++i) {
      modelParamsArray[i].exposureTime = 1.0;
      modelParamsArray[i].rescalingParams[0] = 1;
      modelParamsArray[i].rescalingParams[1] = 0;
      modelParamsArray[i].sunPosition[0] = sunPositions[i][0];
      modelParamsArray[i].sunPosition[1] = sunPositions[i][1];
      modelParamsArray[i].sunPosition[2] = sunPositions[i][2];
  }


  float error;
  float overallError ;
  // Match the remaining images to the already matched images
  for (unsigned int iter = 0; iter < 3; iter++){
    overallError = 0.0;
    for (unsigned int i = 1; i < input_files.size(); ++i) {
      //update the model params
      //update the albedo map
      //TO DO: update the shadow map
      //TO DO: update the height map 
   
      //std::cout << "\t--> iter: " << iter <<" ExposureTime before: " << modelParamsArray[i].exposureTime <<"\n";
 
      match_images(input_files[i-1], input_files[i], num_matches, modelParamsArray[i-1], &modelParamsArray[i], &error);
      //printf("error = %f\n", error);
      overallError = overallError + error;    
      //std::cout << "\t--> ExposureTime after: " << modelParamsArray[i].exposureTime <<"\n";
    }
    printf("overallError = %f\n", overallError);
  }
  //Compute the new pixel limits and the stretching factors - START 
  // Vector2 rescalingParams = get_scaling_params(input_files, modelParamsArray);
  
  ////////////////////////////////// 
  //Save the results.
  //save the first image
  DiskImageView<PixelMask<PixelGray<uint8> > > image(input_files[0]); 
  GeoReference geo;
  read_georeference(geo, input_files[0]); 
  // ImageView<PixelMask<PixelGray<float> > > tm_image(image.cols(), image.rows());

  write_georeferenced_image(output_files[0], 
                            channel_cast<uint8>(clamp(image,0.0,255.0)),
                            geo, TerminalProgressCallback());


  for (unsigned int i = 1; i < input_files.size(); ++i) {
  //for (unsigned int i = 28; i < 32; ++i) {
      //modelParamsArray[i].rescalingParams[0] = rescalingParams[0];
      //modelParamsArray[i].rescalingParams[1] = rescalingParams[1];
      printf("i = %d, filename = %s\n\n", i, input_files[i].c_str());
      ComputeSaveAlbedoMap(input_files[i], input_files[i-1], output_files[i]/*, output_files[i-1]*/, modelParamsArray[i], modelParamsArray[i-1]);
  }
  

}


















