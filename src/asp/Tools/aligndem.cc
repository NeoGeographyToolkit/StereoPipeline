// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint.h>
#include <vw/Math.h>
#include <vw/Mosaic/ImageComposite.h>
#include <asp/ControlNetTK/Equalization.h>

using std::cout;
using std::endl;
using std::string;

using namespace vw;
using namespace vw::cartography;
using namespace vw::ip;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

template <int dim>
class HomogeneousTransformFunctor : public UnaryReturnSameType {
  Matrix<double,dim+1,dim+1> m_trans;

public:
  HomogeneousTransformFunctor(Matrix<double,dim+1,dim+1> trans) : m_trans(trans) {}

  inline Vector<double,dim> operator()(Vector<double,dim> pt) const {
    if (pt == Vector<double,dim>())
      return pt;

    Vector<double,dim+1> pt_h;
    subvector(pt_h, 0, dim) = pt;
    pt_h[dim] = 1;

    Vector<double,dim+1> result = m_trans * pt_h;
    if (result[dim] != 1)
      result /= result[dim];
    return subvector(result,0,dim);
  }
};

// Duplicate matches for any given interest point probably indicate a
// poor match, so we cull those out here.
void remove_duplicates(std::vector<InterestPoint> &ip1,
                       std::vector<InterestPoint> &ip2) {
  std::vector<InterestPoint> new_ip1, new_ip2;

  for (unsigned i = 0; i < ip1.size(); ++i) {
    bool bad_entry = false;
    for (unsigned j = 0; j < ip1.size(); ++j) {
      if (i != j &&
          ((ip1[i].x == ip1[j].x && ip1[i].y == ip1[j].y) ||
           (ip2[i].x == ip2[j].x && ip2[i].y == ip2[j].y)) ) {
        bad_entry = true;
      }
    }
    if (!bad_entry) {
      new_ip1.push_back(ip1[i]);
      new_ip2.push_back(ip2[i]);
    }
  }

  ip1 = new_ip1;
  ip2 = new_ip2;
}

void match_orthoimages( string const& left_image_name,
                        string const& right_image_name,
                        std::vector<InterestPoint> & matched_ip1,
                        std::vector<InterestPoint> & matched_ip2 )
{

  vw_out() << "\t--> Finding Interest Points for the orthoimages\n";

  fs::path left_image_path(left_image_name), right_image_path(right_image_name);
  string left_ip_file = change_extension(left_image_path, ".vwip").string();
  string right_ip_file = change_extension(right_image_path, ".vwip").string();
  string match_file = (left_image_path.branch_path() / (fs::basename(left_image_path) + "__" + fs::basename(right_image_path) + ".match")).string();

  // Building / Loading Interest point data
  if ( fs::exists(match_file) ) {

    vw_out() << "\t    * Using cached match file.\n";
    read_binary_match_file(match_file, matched_ip1, matched_ip2);
    vw_out() << "\t    * " << matched_ip1.size() << " matches\n";

  } else {
    std::vector<InterestPoint> ip1_copy, ip2_copy;

    if ( !fs::exists(left_ip_file) ||
         !fs::exists(right_ip_file) ) {

      // Worst case, no interest point operations have been performed before
      vw_out() << "\t    * Locating Interest Points\n";
      DiskImageView<PixelGray<float32> > left_disk_image(left_image_name);
      DiskImageView<PixelGray<float32> > right_disk_image(right_image_name);

      // Interest Point module detector code.
      OBALoGInterestOperator obalog_detector(0.03);
      IntegralInterestPointDetector<OBALoGInterestOperator> detector( obalog_detector, 200 );
      std::list<InterestPoint> ip1, ip2;
      vw_out() << "\t    * Processing " << left_image_name << "...\n" << std::flush;
      ip1 = detect_interest_points( left_disk_image, detector );
      vw_out() << "Located " << ip1.size() << " points.\n";
      vw_out() << "\t    * Processing " << right_image_name << "...\n" << std::flush;
      ip2 = detect_interest_points( right_disk_image, detector );
      vw_out() << "Located " << ip2.size() << " points.\n";

      vw_out() << "\t    * Generating descriptors..." << std::flush;
      SGradDescriptorGenerator descriptor;
      descriptor( left_disk_image, ip1 );
      descriptor( right_disk_image, ip2 );
      vw_out() << "done.\n";

      // Writing out the results
      vw_out() << "\t    * Caching interest points: "
               << left_ip_file << " & " << right_ip_file << std::endl;
      write_binary_ip_file( left_ip_file, ip1 );
      write_binary_ip_file( right_ip_file, ip2 );

    }

    vw_out() << "\t    * Using cached IPs.\n";
    ip1_copy = read_binary_ip_file(left_ip_file);
    ip2_copy = read_binary_ip_file(right_ip_file);

    vw_out() << "\t    * Matching interest points\n";
    ip::DefaultMatcher matcher(0.6);

    matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
            false, TerminalProgressCallback( "asp", "\t    Matching: "));

    remove_duplicates(matched_ip1, matched_ip2);
    vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " putative matches.\n";
    asp::cnettk::equalization( matched_ip1, matched_ip2, 800 );
    vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " thinned matches.\n";

    vw_out() << "\t    * Caching matches: " << match_file << "\n";
    write_binary_match_file( match_file, matched_ip1, matched_ip2);
  }

}

int main( int argc, char *argv[] ) {
  string dem1_name, dem2_name, ortho1_name, ortho2_name, output_prefix;
  double default_value;

  po::options_description desc("Options");
  desc.add_options()
    ("help,h", "Display this help message")
    ("default-value", po::value(&default_value), "The value of missing pixels in the first dem")
    ("output-prefix,o", po::value(&output_prefix), "Specify the output prefix");

  po::options_description positional("");
  positional.add_options()
    ("dem1", po::value(&dem1_name), "Explicitly specify the first dem")
    ("dem2", po::value(&dem2_name), "Explicitly specify the second dem")
    ("ortho1", po::value(&ortho1_name), "Explicitly specify the first orthoimage")
    ("ortho2", po::value(&ortho2_name), "Explicitly specify the second orthoimage");

  po::positional_options_description p;
  p.add("dem1", 1);
  p.add("ortho1", 1);
  p.add("dem2", 1);
  p.add("ortho2", 1);

  po::options_description all_options;
  all_options.add(desc).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(p).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    cout << "Error parsing: " << e.what() << "\n\t" << desc << "\n";
    return 1;
  }

  if ( dem1_name.empty() || dem2_name.empty() ||
       ortho1_name.empty() || ortho2_name.empty() || vm.count("help") ) {
    cout << "Usage: " << argv[0] << " <dem1> <ortho1> <dem2> <ortho2>" << endl;
    cout << desc << endl;
    return 1;
  }

  if ( output_prefix.empty() )
    output_prefix = change_extension(fs::path(dem1_name), "").string();

  DiskImageResourceGDAL ortho1_rsrc(ortho1_name), ortho2_rsrc(ortho2_name),
    dem1_rsrc(dem1_name), dem2_rsrc(dem2_name);

  // Pull out default value
  if ( !vm.count("default-value") && dem1_rsrc.has_nodata_read() ) {
    default_value = dem1_rsrc.nodata_read();
    vw_out() << "\tFound input nodata value: " << default_value << endl;
  }

  DiskImageView<double> dem1_dmg(dem1_name), dem2_dmg(dem2_name);
  InterpolationView<EdgeExtensionView<DiskImageView<double>, ZeroEdgeExtension>, BilinearInterpolation> dem1_interp = interpolate(dem1_dmg, BilinearInterpolation(), ZeroEdgeExtension());
  InterpolationView<EdgeExtensionView<DiskImageView<double>, ZeroEdgeExtension>, BilinearInterpolation> dem2_interp = interpolate(dem2_dmg, BilinearInterpolation(), ZeroEdgeExtension());

  GeoReference ortho1_georef, ortho2_georef, dem1_georef, dem2_georef;
  read_georeference(ortho1_georef, ortho1_rsrc);
  read_georeference(ortho2_georef, ortho2_rsrc);
  read_georeference(dem1_georef, dem1_rsrc);
  read_georeference(dem2_georef, dem2_rsrc);

  std::vector<InterestPoint> matched_ip1, matched_ip2;

  match_orthoimages(ortho1_name, ortho2_name, matched_ip1, matched_ip2);

  vw_out() << "\t--> Rejecting outliers using RANSAC.\n";

  std::vector<Vector4> ransac_ip1, ransac_ip2;

  for (unsigned i = 0; i < matched_ip1.size(); i++) {
    Vector2 point1 = ortho1_georef.pixel_to_lonlat(Vector2(matched_ip1[i].x, matched_ip1[i].y));
    Vector2 point2 = ortho2_georef.pixel_to_lonlat(Vector2(matched_ip2[i].x, matched_ip2[i].y));

    Vector2 dem_pixel1 = dem1_georef.lonlat_to_pixel(point1);
    Vector2 dem_pixel2 = dem2_georef.lonlat_to_pixel(point2);

    // I don't trust the accuracy of this code. The values for the
    // alignment matrix varies greatly with the number of matched
    // points.

    if (BBox2i(0, 0, dem1_dmg.cols(), dem1_dmg.rows()).contains(dem_pixel1) &&
        BBox2i(0, 0, dem2_dmg.cols(), dem2_dmg.rows()).contains(dem_pixel2)) {
      double alt1 = dem1_georef.datum().radius(point1.x(), point1.y()) + dem1_interp(dem_pixel1.x(), dem_pixel1.y());
      double alt2 = dem2_georef.datum().radius(point2.x(), point2.y()) + dem2_interp(dem_pixel2.x(), dem_pixel2.y());

      Vector3 xyz1 = lon_lat_radius_to_xyz(Vector3(point1.x(), point1.y(), alt1));
      Vector3 xyz2 = lon_lat_radius_to_xyz(Vector3(point2.x(), point2.y(), alt2));

      ransac_ip1.push_back(Vector4(xyz1.x(), xyz1.y(), xyz1.z(), 1));
      ransac_ip2.push_back(Vector4(xyz2.x(), xyz2.y(), xyz2.z(), 1));
    }
  }

  std::vector<int> indices;
  Matrix<double> trans;
  math::RandomSampleConsensus<math::AffineFittingFunctorN<3>,math::L2NormErrorMetric>
    ransac( math::AffineFittingFunctorN<3>(), math::L2NormErrorMetric(), 10);
  trans = ransac(ransac_ip1, ransac_ip2);
  indices = ransac.inlier_indices(trans, ransac_ip1, ransac_ip2);

  vw_out() << "\t    * Ransac Result: " << trans << "\n";
  vw_out() << "\t                     # inliers: " << indices.size() << "\n";

  { // Saving transform to human readable text
    std::string filename = (fs::path(dem1_name).branch_path() / (fs::basename(fs::path(dem1_name)) + "__" + fs::basename(fs::path(dem2_name)) + "-Matrix.txt")).string();
    std::ofstream ofile( filename.c_str() );
    ofile << std::setprecision(15) << std::flush;
    ofile << "# inliers: " << indices.size() << endl;
    ofile << trans << endl;
    ofile.close();
  }

  ImageViewRef<PixelMask<double> > dem1_masked(create_mask(dem1_dmg, default_value));

  ImageViewRef<Vector3> point_cloud = lon_lat_radius_to_xyz(project_point_image(dem_to_point_image(dem1_masked, dem1_georef), dem1_georef, false));
  ImageViewRef<Vector3> point_cloud_trans = per_pixel_filter(point_cloud, HomogeneousTransformFunctor<3>(trans));

  DiskImageResourceGDAL point_cloud_rsrc(output_prefix + "-PC.tif", point_cloud_trans.format(),
                                         Vector2i(vw_settings().default_tile_size(),
                                                  vw_settings().default_tile_size()));
  block_write_image(point_cloud_rsrc, point_cloud_trans,
                    TerminalProgressCallback("asp", "\t--> Transforming: "));

  return 0;
}
