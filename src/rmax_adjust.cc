#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/BundleAdjust.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>
#include <vw/Stereo.h>
#include <vw/Math/LevenbergMarquardt.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::ip;
using namespace vw::stereo;

#include <stdlib.h>
#include <iostream>

#include "RMAX/RMAX.h"
#include "RMAX/RmaxInterestPoint.h"
#include "RMAX/RmaxBundles.h"


struct TestSquaresModel : public math::LeastSquaresModelBase<TestSquaresModel> {
  typedef Vector<double> result_type;
  typedef Vector<double> domain_type;
  typedef Matrix<double> jacobian_type;

  std::vector<ImageInfo> m_image_infos;
  std::vector<Bundle> m_bundles;

  TestSquaresModel(std::vector<ImageInfo> const& image_infos,
                   std::vector<Bundle> bundles) :
    m_image_infos(image_infos), m_bundles(bundles) {}

  std::vector<boost::shared_ptr<camera::CameraModel> > adjusted_cameras(Vector<double> x) {
    std::vector<boost::shared_ptr<camera::CameraModel> > result(m_image_infos.size());
    for (unsigned j = 0; j < result.size(); ++j) {
      Vector3 cam_position_correction = subvector(x, 6*j, 3);
      Vector3 cam_pose_correction = subvector(x, 6*j+3,3);
      
      camera::CAHVORModel* cahvor = new camera::CAHVORModel;
      *cahvor = rmax_image_camera_model(m_image_infos[j], cam_position_correction, cam_pose_correction);

      result[j] = boost::shared_ptr<camera::CameraModel>( cahvor );
    }
    return result;
  }

  void write_adjustment(Vector<double> x, int j, std::string const& filename) {
    std::ofstream ostr(filename.c_str());
    Vector<double,6> adjustment = subvector(x, 6*j,6);
    ostr << adjustment[0] << " " << adjustment[1] << " " << adjustment[2] << "\n";
    ostr << adjustment[3] << " " << adjustment[4] << " " << adjustment[5] << "\n";
  }

  void write_adjusted_camera(Vector<double> x, std::vector<ImageInfo> image_infos, int j, std::string const& filename) {
    Vector3 cam_position_correction = subvector(x, 6*j, 3);
    Vector3 cam_pose_correction = subvector(x, 6*j+3,3);
    camera::CAHVORModel cam = rmax_image_camera_model(image_infos[j], cam_position_correction, cam_pose_correction);
    cam.write(filename);
  }


  inline result_type operator()( domain_type const& x ) const { 
    int num_pix = 0;
    for (unsigned i = 0; i < m_bundles.size(); ++i)
      num_pix += m_bundles[i].imaged_pixel_locations.size();

    Vector<double> h(2*num_pix);

    int pix_index = 0;
    for (unsigned i = 0; i < m_bundles.size(); ++i) {
      Vector3 position = m_bundles[i].position;
      for (unsigned j = 0; j < m_bundles[i].imaged_pixel_locations.size(); ++j) {
        int32 imager_num = m_bundles[i].imaged_pixel_locations[j].first;

        Vector3 cam_position_correction = subvector(x, 6*imager_num, 3);
        Vector3 cam_pose_correction = subvector(x, 6*imager_num+3,3);
        Vector3 point_position_correction = subvector(x, 6*m_image_infos.size()+3*i,3);

        camera::CAHVORModel cam = rmax_image_camera_model(m_image_infos[imager_num], cam_position_correction, cam_pose_correction);
        subvector(h,pix_index*2,2) = cam.point_to_pixel(position + point_position_correction);
        pix_index++;
      }
    }
    return h;
  }
};

int main(int argc, char* argv[]) {

  std::vector<std::string> image_files;
  std::string bundles_file;
  std::vector<Bundle> bundles;
  double lambda;

  po::options_description general_options("Options");
  general_options.add_options()
    ("bundles,b", po::value<std::string>(&bundles_file), "Load bundles from file")
    ("lambda,l", po::value<double>(&lambda), "Set the initial value of the LM parameter lambda")
    ("help", "Display this help message")
    ("verbose", "Verbose output");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&image_files));
  
  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);
  
  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <rmax image filenames>..." << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    std::cout << usage << std::endl;
    return 1;
  }
  
  if( vm.count("input-files") < 1) {
    if ( vm.count("bundles") ) {
      std::cout << "Loading bundles from file: " << bundles_file << "\n";
      read_bundles_file(bundles_file, bundles, image_files);
    } else {
      std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
      std::cout << usage.str();
      return 1;
    }
  }  

  // Read in the camera model and RMAX image info for the input
  // images.
  std::vector<std::string> camera_files(image_files.size());
  std::vector<ImageInfo> image_infos(image_files.size());
  std::vector<boost::shared_ptr<CameraModel> > camera_models(image_files.size());
  for (unsigned i = 0; i < image_files.size(); ++i) {
    read_image_info( image_files[i], image_infos[i] );
    CAHVORModel* cahvor = new CAHVORModel;
    *cahvor = rmax_image_camera_model(image_infos[i]);
    camera_models[i] = boost::shared_ptr<CameraModel>(cahvor);
    
    // Write (unadjusted) CAHVOR camera model to file.
    std::string base, extension;
    split_filename(image_files[i], base, extension);
    camera_files[i] = base + ".cahvor";
    cahvor->write(camera_files[i]);
  }

  if (!vm.count("bundles") ) {

    // First, detect all of the interest points.  These points may be
    // cached in binary *.key file, and these files are used instead
    // where possible to save time.
    std::vector<std::string> keyfiles(image_files.size());
    for (unsigned i = 0; i < image_files.size(); ++i) {
      keyfiles[i] = detect_interest_points(image_files[i]);
    }    

    // Locate all of the interest points between images that may
    // overlap based on a rough approximation of their bounding box.
    std::vector<MatchedPoints> matched_points_list;
    for (unsigned i = 0; i < image_files.size(); ++i) {
      for (unsigned j = 0; j < image_files.size(); ++j) {
        if ( i < j && may_overlap(image_files[i], image_files[j]) ) {
          std::vector<Vector2> matched_ip1, matched_ip2;
          match_interest_points(keyfiles[i], keyfiles[j], matched_ip1, matched_ip2);
          MatchedPoints m;
          m.id1 = i;
          m.id2 = j;
          m.ip1 = matched_ip1;
          m.ip2 = matched_ip2;
          matched_points_list.push_back(m);
          //          std::cout << "Adding " << (m.id1) << " <-> " << (m.id2) << "\n";
          //           for (unsigned z = 0; z < m.ip1.size(); ++z) 
          //             std::cout << "\t "  << ((m.ip1)[z]) << " " << ((m.ip2)[z]) << "\n";
          std::ostringstream ostr; 
          ostr << "debug-"<<i<<"-"<<j<<".png";
          //          write_match_image(ostr.str(), image_files[i], image_files[j], matched_ip1, matched_ip2);
        }
      }
    }

    std::cout << "\nMatching Results:\n";
    for (unsigned i = 0; i < matched_points_list.size(); ++i) 
      std::cout << "\t " << matched_points_list[i].id1 << " <-> " << matched_points_list[i].id2 << " : " << matched_points_list[i].ip1.size() << " matches.\n";
    
    // Create the bundles
    create_bundles(matched_points_list, camera_models, bundles);

    write_bundles_file(bundles, image_files);
  }

  // Print pre-alignment residuals
  compute_stereo_residuals(camera_models, bundles);

//   //---
//   int num_pix = 0;
//   for (unsigned i = 0; i < bundles.size(); ++i)
//     num_pix += bundles[i].imaged_pixel_locations.size();

//   Vector<double> target(num_pix*2);
//   int idx = 0;
//   for (unsigned i = 0; i < bundles.size(); ++i) {
//     for (unsigned j = 0; j < bundles[i].imaged_pixel_locations.size(); ++j) {
//       subvector(target, idx*2, 2) = bundles[i].imaged_pixel_locations[j].second;
//       ++idx;
//     }
//   }
//   std::cout << "TARGET: " << target << "\n";

//   Vector<double> seed(6*image_infos.size() + 3*bundles.size());
//   fill(seed, 0);

//   int status;
//   TestSquaresModel model(image_infos, bundles);
//   Vector<double> best = levenberg_marquardt( model, seed, target, status );
//   std::cout << best << "\n\n";

  
//   for (unsigned int i=0; i < image_infos.size(); ++i) {
//     std::string base, extension;
//     split_filename(image_files[i], base, extension);

//     model.write_adjustment(best, i, base+".adjust");
//     model.write_adjusted_camera(best, image_infos, i, base+".adjust.cahvor");
//   }    

//   // Write out Adjust CAHVOR models.
//   std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras2 = model.adjusted_cameras(best);

//   // Compute the pre-adjustment residuals
//   compute_stereo_residuals(adjusted_cameras2, bundles);

//   exit(0);
//   //---
    
  HelicopterBundleAdjustmentModel ba_model(image_infos);
  bundle_adjust_t bundle_adjuster(ba_model, bundles);
  if (vm.count("lambda")) {
    std::cout << "Setting initial value of lambda to " << lambda << "\n";
    bundle_adjuster.set_lambda(lambda);
  }
  std::cout << "Performing Sparse LM Bundle Adjustment\n";
  double abs_tol = 1e10, rel_tol=1e10;
  bundle_adjuster.update(abs_tol,rel_tol);
  //bundle_adjuster.update_reference_impl2(abs_tol,rel_tol);
  std::cout << "\n";
  int iterations = 0;
  while(bundle_adjuster.update(abs_tol, rel_tol)) {
  //while(bundle_adjuster.update_reference_impl2(abs_tol, rel_tol)) {
    iterations++;
    if (iterations > 30 || abs_tol < 0.001 || rel_tol < 1e-10)
      break;
  }
  std::cout << "\nFinished.  Iterations: "<< iterations << "\n";

  for (unsigned int i=0; i < ba_model.size(); ++i) {
    std::string base, extension;
    split_filename(image_files[i], base, extension);

    ba_model.write_adjustment(i, base+".adjust");
    ba_model.write_adjusted_camera(i, base+".adjust.cahvor");
  }    

  // Write out Adjust CAHVOR models.
  std::vector<boost::shared_ptr<CameraModel> > adjusted_cameras = ba_model.adjusted_cameras();

  // Compute the pre-adjustment residuals
  compute_stereo_residuals(adjusted_cameras, bundles);
}
