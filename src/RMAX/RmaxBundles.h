#ifndef __RMAX_BUNDLES_H__
#define __RMAX_BUNDLES_H__

#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/BundleAdjust.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Mosaic.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::ip;
using namespace vw::stereo;

// Global typedefs
typedef BundleAdjustment<HelicopterBundleAdjustmentModel> bundle_adjust_t;

struct MatchedPoints {
  int id1;
  int id2;
  std::vector<Vector2> ip1;
  std::vector<Vector2> ip2;
};

// Draw the two images side by side with matching interest points
// shown with lines.
static void write_match_image(std::string out_file_name, 
                              std::string const& file1,
                              std::string const& file2,
                              std::vector<Vector2> matched_ip1, 
                              std::vector<Vector2> matched_ip2) {
  // Skip image pairs with no matches.
  if (matched_ip1.size() == 0)
    return;

  DiskImageView<PixelRGB<uint8> > src1(file1);
  DiskImageView<PixelRGB<uint8> > src2(file2);

  mosaic::ImageComposite<PixelRGB<uint8> > composite;
  composite.insert(pixel_cast<PixelRGB<uint8> >(channel_cast_rescale<uint8>(src1.impl())),0,0);
  composite.insert(pixel_cast<PixelRGB<uint8> >(channel_cast_rescale<uint8>(src2.impl())),src1.impl().cols(),0);
  composite.set_draft_mode( true );
  composite.prepare();

  // Rasterize the composite so that we can draw on it.
  ImageView<PixelRGB<uint8> > comp = composite;
  
  // Draw a red line between matching interest points
  for (unsigned int i = 0; i < matched_ip1.size(); ++i) {
    Vector2 start = matched_ip1[i];
    Vector2 end = matched_ip2[i] + Vector2(src1.impl().cols(), 0);
    for (float r=0; r<1.0; r+=1/norm_2(end-start)){
      int i = (int)(0.5 + start.x() + r*(end.x()-start.x()));
      int j = (int)(0.5 + start.y() + r*(end.y()-start.y()));
      // red, green, blue
      comp(i,j) = PixelRGB<uint8>(255, 0, 0);
    }
  }
  
  write_image(out_file_name, comp);
}

void write_bundles_file(std::vector<Bundle> bundles, std::vector<std::string> image_files) {
  
  std::ofstream ofile("bundles.txt");

  ofile << image_files.size() << " " << bundles.size() << "\n";

  for (unsigned i=0; i < bundles.size(); ++i) {
    ofile << bundles[i].imaged_pixel_locations.size() << " " << bundles[i].position[0] << " " << bundles[i].position[1] << " " << bundles[i].position[2] << "\n";
    for (unsigned j = 0; j < bundles[i].imaged_pixel_locations.size(); ++j) {
      int idx = bundles[i].imaged_pixel_locations[j].first;
      ofile << idx << " " << image_files[idx] << " " << bundles[i].imaged_pixel_locations[j].second[0] << " " << bundles[i].imaged_pixel_locations[j].second[1] << "\n";
    }
  }
}

void read_bundles_file(std::string filename, std::vector<Bundle> &bundles, std::vector<std::string> &image_files) {
  std::ifstream ifile(filename.c_str());

  int total_num_images, total_num_bundles;
  ifile >> total_num_images >> total_num_bundles;
  image_files.resize(total_num_images);
  bundles.resize(total_num_bundles);

  unsigned num_pairs;
  Vector3 pos;
  for (unsigned b=0; b < bundles.size(); ++b) {
    ifile >> num_pairs >> pos[0] >> pos[1] >> pos[2];
    //    std::cout << "---> " << num_pairs << " " << pos << "\n";

    Bundle bundle;
    bundle.position = pos;
    bundle.imaged_pixel_locations.resize(num_pairs);

    int idx;
    std::string filename;
    Vector2 pix;
    for (unsigned i=0; i < num_pairs; ++i) {
      ifile >> idx >> filename >> pix[0] >> pix[1];
      bundle.imaged_pixel_locations[i].first = idx;
      bundle.imaged_pixel_locations[i].second = pix;
      image_files[idx] = filename;
      //      std::cout << image_files[idx] << " " << bundle.imaged_pixel_locations[i].first << " " << bundle.imaged_pixel_locations[i].second << "\n";
    }
    bundles[b]=bundle;
  }
}

void compute_stereo_residuals(std::vector<boost::shared_ptr<CameraModel> > const& camera_models,
                              std::vector<Bundle> &bundles) {

  // Compute pre-adjustment residuals and convert to bundles
  int n = 0;
  double error_sum = 0; 
  double min_error = ScalarTypeLimits<double>::highest();
  double max_error = ScalarTypeLimits<double>::lowest();
  for (unsigned i = 0; i < bundles.size(); ++i) {
    for (unsigned j = 0; j+1 < bundles[i].imaged_pixel_locations.size(); ++j) {
      ++n;
      int cam1 = bundles[i].imaged_pixel_locations[j].first;
      int cam2 = bundles[i].imaged_pixel_locations[j+1].first;
      Vector2 pix1 = bundles[i].imaged_pixel_locations[j].second;
      Vector2 pix2 = bundles[i].imaged_pixel_locations[j+1].second;

      StereoModel sm(*(camera_models[cam1]), *(camera_models[cam2]));
      double error;
      Vector3 pos = sm(pix1,pix2,error);
      //      std::cout << "  -- > " << error << "\n";
      error_sum += error;
      min_error = std::min(min_error, error);
      max_error = std::max(max_error, error);
    }
  }  
  std::cout << "\nStereo Intersection Residuals -- Min: " << min_error << "  Max: " << max_error << "  Average: " << (error_sum/n) << "\n";
}

int check_for_previous_entry(std::vector<Bundle> const& bundles,
                             Vector2 const& query) {
  for (unsigned i = 0; i < bundles.size(); ++i) {
    for (unsigned j = 0; j < bundles[i].imaged_pixel_locations.size(); ++j) {
      Vector2 candidate = bundles[i].imaged_pixel_locations[j].second;
      if (candidate[0] == query[0] && candidate[1] == query[1] ) 
        return i;
    }
  }
  return 0;
}

void create_bundles(std::vector<MatchedPoints> const& matched_points_list,
                    std::vector<boost::shared_ptr<CameraModel> > const& camera_models,
                    std::vector<Bundle> &bundles) {

  bundles.clear();


  // For each pair of images with matching points...
  for (unsigned i = 0; i < matched_points_list.size(); ++i) {
    
    // ... create a stereo model for this image pair...
    StereoModel sm(*(camera_models[matched_points_list[i].id1]), *(camera_models[matched_points_list[i].id2]));

    // ... and for each of the matched points...
    for (unsigned j = 0; j < matched_points_list[i].ip1.size(); ++j) {

      // Check to see if this interest point is already in the bundle list somewhere
      int bundle_id = 0;
      if ( (bundle_id = check_for_previous_entry(bundles, matched_points_list[i].ip1[j])) ) {
        //        std::cout << "Previous entry for ip1: " << matched_points_list[i].ip1[j] << " and " << matched_points_list[i].ip2[j] << " in bundle " << bundle_id << "\n";
        bundles[bundle_id].imaged_pixel_locations.push_back(std::pair<uint32, Vector2>(matched_points_list[i].id2,matched_points_list[i].ip2[j]));
      } else if ( (bundle_id = check_for_previous_entry(bundles, matched_points_list[i].ip2[j])) ) {
        //        std::cout << "Previous entry for ip2: " << matched_points_list[i].ip1[j] << " and " << matched_points_list[i].ip2[j] << " in bundle " << bundle_id << "\n";
        bundles[bundle_id].imaged_pixel_locations.push_back(std::pair<uint32, Vector2>(matched_points_list[i].id1,matched_points_list[i].ip1[j]));
      } else {
        Bundle b;
        double error;
        b.position = sm(matched_points_list[i].ip1[j],matched_points_list[i].ip2[j],error);
        b.imaged_pixel_locations.push_back(std::pair<uint32, Vector2>(matched_points_list[i].id1,matched_points_list[i].ip1[j]));
        b.imaged_pixel_locations.push_back(std::pair<uint32, Vector2>(matched_points_list[i].id2,matched_points_list[i].ip2[j]));
        bundles.push_back(b);
      }
    }
  }
}

#endif __RMAX_BUNDLES_H__
