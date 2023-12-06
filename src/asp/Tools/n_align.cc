// __BEGIN_LICENSE__
//  Copyright (c) 2018, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

// Joint n-cloud joint alignment. Reimplementing the Matlab code at
// https://searchcode.com/file/13619767/Code/matlab/GlobalProcrustesICP/globalProcrustes.m
// One tiny change is that that code had a little bug, when invoked
// with 10 iterations it would actually do only 9. So, before
// comparing this code's result to that one, un-comment the line which
// says "original incorrect logic".  Based on the paper: Global
// registration of multiple point clouds embedding the Generalized
// Procrustes Analysis into an ICP framework by Toldo, Roberto and
// Beinat, Alberto and Crosilla, Fabio.

// Here some code is used that is part of the Fast Global Registration
// software, https://github.com/IntelVCL/FastGlobalRegistration which
// is released under the MIT License (MIT).  

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/EigenUtils.h>
#include <asp/Tools/pc_align_utils.h>

#include <vw/Core/Stopwatch.h>

// Turn off warnings about things we can't control
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <flann/flann.hpp>
#pragma GCC diagnostic pop

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <iostream>
#include <fstream>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;
using namespace asp;

typedef PointMatcher<RealT> PM;
typedef PM::DataPoints DP;

// TODO: Figure out these magic numbers
#define FIFTEEN 15
#define ONE_TWO_EIGHT 128

typedef flann::Index<flann::L2<double>> KDTree_double;

/// Options container
struct Options : public vw::GdalWriteOptions {
  // Input
  string in_prefix, in_transforms, datum, csv_format_str, csv_proj4_str; 
  int    num_iter, max_num_points;
  double semi_major_axis, semi_minor_axis, rel_error_tol;
  bool   save_transformed_clouds, align_to_first_cloud, verbose;
  std::vector<std::string> cloud_files;
  // Output
  string out_prefix;

  Options(){}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("num-iterations",           po::value(&opt.num_iter)->default_value(100),
     "Maximum number of iterations.")
    ("max-num-points", po::value(&opt.max_num_points)->default_value(1000000),
     "Maximum number of (randomly picked) points from each cloud to use.")
    ("csv-format",               po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("csv-proj4",                po::value(&opt.csv_proj4_str)->default_value(""),
     "The PROJ.4 string to use to interpret the entries in input CSV files.")
    ("datum",                    po::value(&opt.datum)->default_value(""),
     "Use this datum for CSV files instead of auto-detecting it. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("semi-major-axis",          po::value(&opt.semi_major_axis)->default_value(0),
     "Explicitly set the datum semi-major axis in meters.")
    ("semi-minor-axis",          po::value(&opt.semi_minor_axis)->default_value(0),
     "Explicitly set the datum semi-minor axis in meters.")
    ("output-prefix,o",          po::value(&opt.out_prefix)->default_value(""),
     "Specify the output prefix. The computed alignment transforms and, if desired, the transformed clouds, will be saved to names starting with this prefix.")
    ("save-transformed-clouds", po::bool_switch(&opt.save_transformed_clouds)->default_value(false)->implicit_value(true),
     "Apply the obtained alignment transforms to the input clouds and save them.")
    ("initial-transforms-prefix", po::value(&opt.in_prefix)->default_value(""),
     "The prefix of the transforms to be used as initial guesses. The naming convention is the same as for the transforms written on output.")
    ("initial-transforms", po::value(&opt.in_transforms)->default_value(""),
     "Specify the initial transforms as a list of files separated by spaces and in quotes, that is, as 'trans1.txt ... trans_n.txt'.")
    ("relative-error-tolerance", po::value(&opt.rel_error_tol)->default_value(1e-10),
     "Stop when the change in the error divided by the error itself is less than this.")
    ("align-to-first-cloud", po::bool_switch(&opt.align_to_first_cloud)->default_value(false)->implicit_value(true),
     "Align the other clouds to the first one, rather than to their common centroid.")
    ("verbose", po::bool_switch(&opt.verbose)->default_value(false)->implicit_value(true),
     "Print the alignment error after each iteration.");
    
  general_options.add( vw::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("cloud-files", po::value<std::vector<std::string>>(&opt.cloud_files));
  po::positional_options_description positional_desc;
  positional_desc.add("cloud-files", -1);
  std::string usage("[options] <cloud-files> -o <output prefix>");
  
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if ( opt.out_prefix.empty() )
    vw_throw( ArgumentErr() << "Missing output prefix.\n" << usage << general_options );

  if ( opt.num_iter < 0 )
    vw_throw( ArgumentErr() << "The number of iterations must be non-negative.\n"
              << usage << general_options );

  if ( (opt.semi_major_axis != 0 && opt.semi_minor_axis == 0) ||
       (opt.semi_minor_axis != 0 && opt.semi_major_axis == 0) ){
    vw_throw( ArgumentErr() << "One of the semi-major or semi-minor axes"
              << " was specified, but not the other one.\n"
              << usage << general_options );
  }

  if (opt.semi_major_axis < 0 || opt.semi_minor_axis < 0){
    vw_throw( ArgumentErr() << "The semi-major and semi-minor axes cannot "
                            << "be negative.\n" << usage << general_options );
  }

  if (opt.datum != "" && opt.semi_major_axis != 0 && opt.semi_minor_axis != 0 ){
    vw_throw( ArgumentErr() << "Both the datum string and datum semi-axes were "
                            << "specified. At most one needs to be set.\n"
                            << usage << general_options );
  }

  if (opt.cloud_files.size() < 2)
    vw_throw( ArgumentErr() << "Must have at least two clouds to align.\n"
              << usage << general_options );

  if (opt.in_prefix != "" && opt.in_transforms != "")
    vw_throw( ArgumentErr() << "Cannot specify both the initial transforms prefix "
              << "and the list of initial transforms.\n" << usage << general_options );
}

bool triplet_less(vw::Vector3 const& p, vw::Vector3 const& q) {

  if (p.x() < q.x()) return true;
  if (p.x() > q.x()) return false;
  
  if (p.y() < q.y()) return true;
  if (p.y() > q.y()) return false;
    
  if (p.z() < q.z()) return true;
  if (p.z() > q.z()) return false;

  return false;
}

// Define equality for pairs of integers
bool operator==(std::pair<int, int> const& p, std::pair<int, int> const& q) {
  return (p.first == q.first) && (p.second == q.second);
}

// An operator used to sort pairs of integers
struct CustomCompare {
  bool operator()(std::pair<int, int> const& p, std::pair<int, int> const& q) const {
    if (p.first < q.first) return true;
    if (p.first > q.first) return false;
    
    if (p.second < q.second) return true;
    if (p.second > q.second) return false;
    
    return false;
  }
};

bool vector_less(Eigen::VectorXd const& p, Eigen::VectorXd const& q) {
  for (int i = 0; i < p.size(); i++) {
    if (p[i] < q[i]) return true;
    if (p[i] > q[i]) return false;
  }
  return false;
}

// Convert a cloud from libpointmatcher's format to a vector of points
void convert_cloud(DP const& in_cloud, std::vector<vw::Vector3> & out_cloud) {
  out_cloud.clear();
  for (int col = 0; col < in_cloud.features.cols(); col++) {
    vw::Vector3 p;
    for (int row = 0; row < 3; row++) 
      p[row] = in_cloud.features(row, col);
    out_cloud.push_back(p);
  }
}

// Read a cloud stored in a plain text file
void read_cloud(std::string const& file, std::vector<vw::Vector3> & cloud, double shift) {
  cloud.clear();
  std::ifstream ifs(file.c_str());
  double x, y, z;
  vw::Vector3 p;
  while (ifs >> x >> y >> z) {
    p = vw::Vector3(x + shift, y + shift, z + shift);
    cloud.push_back(p);
  }
  ifs.close();
}

// Apply a rotation + transform to a cloud
void apply_transform_to_cloud(std::vector<vw::Vector3> & cloud, Eigen::MatrixXd const& T) {
  for (size_t pointIter = 0; pointIter < cloud.size(); pointIter++) {
    Eigen::VectorXd v(4);
    v << cloud[pointIter][0], cloud[pointIter][1], cloud[pointIter][2], 1.0;
    v = T*v;
    for (int it = 0; it < 3; it++) 
      cloud[pointIter][it] = v[it];
  }
}

void print_cloud(std::vector<vw::Vector3> const& cloud){
  for (size_t it = 0; it < cloud.size(); it++) {
    vw_out() << cloud[it].x() << ' ' << cloud[it].y() << ' ' << cloud[it].z() << std::endl;
  }
}

void sort_and_make_unqiue(std::vector<vw::Vector3> & cloud){
  std::sort(cloud.begin(), cloud.end(), triplet_less);
  std::vector<vw::Vector3>::iterator it = std::unique(cloud.begin(), cloud.end());
  cloud.resize(std::distance(cloud.begin(), it));
}

void BuildKDTree_double(std::vector<vw::Vector3> const& cloud, KDTree_double* tree){
  int rows = cloud.size();
  int dim = vw::Vector3().size();

  // Need to have this to avoid a seg fault later
  if (rows*dim <= 0) 
    vw_throw( ArgumentErr() << "Cannot operate on empty clouds.\n" );
  
  std::vector<double> dataset(rows * dim);
  flann::Matrix<double> dataset_mat(&dataset[0], rows, dim);
  
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < dim; j++) {
      dataset[i * dim + j] = cloud[i][j];
    }
  }

  // Using the default leaf_max_size = 10 in flann::KDTreeSingleIndexParams()
  KDTree_double temp_tree(dataset_mat, flann::KDTreeSingleIndexParams(FIFTEEN));
  temp_tree.buildIndex();
  *tree = temp_tree;
}

void SearchKDTree_double(KDTree_double* tree, const vw::Vector3& input, 
                         std::vector<int>& indices,
                         std::vector<double>& dists, int nn){
  int rows_t = 1;
  int dim = input.size();
  
  std::vector<double> query;
  query.resize(rows_t*dim);
  for (int i = 0; i < dim; i++)
    query[i] = input[i];
  flann::Matrix<double> query_mat(&query[0], rows_t, dim);
  
  indices.resize(rows_t*nn);
  dists.resize(rows_t*nn);
  flann::Matrix<int> indices_mat(&indices[0], rows_t, nn);
  flann::Matrix<double> dists_mat(&dists[0], rows_t, nn);
  
  tree->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(ONE_TWO_EIGHT));
}

std::string transform_file(std::string const& out_prefix, int index){
  std::ostringstream os;
  os << out_prefix << "-transform-" << index << ".txt";
  std::string transFile = os.str();
  return transFile;
}

/// Save the transforms
void write_transforms(std::vector<Eigen::MatrixXd> const& transVec,
		     std::string const& out_prefix){

  for (size_t it = 0; it < transVec.size(); it++) {
    std::string transFile = transform_file(out_prefix, it);
    vw_out() << "Writing: " << transFile << std::endl;
    write_transform(transVec[it], transFile);
  }
}

/// Read the transforms. We assume we know their number.
void read_transforms(std::vector<Eigen::MatrixXd> & transVec,
		     std::string const& in_prefix){

  for (size_t it = 0; it < transVec.size(); it++) {
    std::string transFile = transform_file(in_prefix, it);
    read_transform(transVec[it], transFile);
  }
}

/// Read the transforms from a list. We assume we know their numbers.
void read_transforms_from_list(std::vector<Eigen::MatrixXd> & transVec,
                               std::string const& transform_list){
  std::istringstream is(transform_list);
  for (size_t it = 0; it < transVec.size(); it++) {
    std::string transFile;
    if ( !(is >> transFile) )
      vw_throw( ArgumentErr() << "Cannot parse enough transform files from: " << transform_list);
    read_transform(transVec[it], transFile);
  }
}

int main(int argc, char *argv[]){

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Parse the csv format string and csv projection string
    asp::CsvConv csv_conv;
    csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str);
    
    // Create the output directory
    vw::create_out_dir(opt.out_prefix);
    
    // Turn on logging to file
    asp::log_to_file(argc, argv, "", opt.out_prefix);
    
    // Try to read the georeference/datum info
    GeoReference geo;
    read_georef(opt.cloud_files, opt.datum, opt.csv_proj4_str,  
                opt.semi_major_axis, opt.semi_minor_axis,  
                opt.csv_format_str,  csv_conv, geo);

    int numClouds = opt.cloud_files.size();

    // The vector of transforms among the clouds
    std::vector<Eigen::MatrixXd> transVec(numClouds);
    for (int cloudIter = 0; cloudIter < numClouds; cloudIter++) {
      transVec[cloudIter] = Eigen::MatrixXd::Identity(4, 4);
    }

    std::vector<std::vector<vw::Vector3>> clouds(numClouds);

    // Load the first subsampled point cloud. Calculate the shift to apply to all clouds.
    vw::Vector3 shift;
    bool   calc_shift = true; // Shift points so the first point is (0,0,0)
    bool   is_lola_rdr_format = false;   // may get overwritten
    double mean_ref_longitude    = 0.0;  // may get overwritten
    double mean_source_longitude = 0.0;  // may get overwritten
    BBox2 empty_box;
    DP in_cloud;
    bool verbose = true;
    load_cloud(opt.cloud_files[0], opt.max_num_points, empty_box,
               calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
               mean_ref_longitude, verbose, in_cloud);
    convert_cloud(in_cloud, clouds[0]);
    
    vw_out() << "Data shifted internally by subtracting: " << shift << std::endl;
    
    calc_shift = false; // We will use the same shift from here on
      
    for (int cloudIter = 1; cloudIter < numClouds; cloudIter++) {
      load_cloud(opt.cloud_files[cloudIter], opt.max_num_points, empty_box,
                 calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
                 mean_ref_longitude, verbose, in_cloud);
      convert_cloud(in_cloud, clouds[cloudIter]);
    }
    
    // Read the clouds
    for (int cloudIter = 0; cloudIter < numClouds; cloudIter++) {
      //read_cloud(opt.cloud_files[cloudIter], clouds[cloudIter], shifts[cloudIter]);
      sort_and_make_unqiue(clouds[cloudIter]);
    }

    // Read any initial transforms, either using a prefix or an explicit list
    bool has_init_transform = false;
    if (opt.in_prefix != "" || opt.in_transforms != "") {
      has_init_transform = true;
      if (opt.in_prefix != "") 
        read_transforms(transVec, opt.in_prefix);
      else if (opt.in_transforms != "")
        read_transforms_from_list(transVec, opt.in_transforms);
      
      // The point clouds are shifted, so shift the initial transforms as well.
      for (int cloudIter = 0; cloudIter < numClouds; cloudIter++) 
        transVec[cloudIter] = apply_shift(transVec[cloudIter], shift);
    }

    // Apply the initial transforms to the clouds
    if (has_init_transform) {
      for (int cloudIter = 0; cloudIter < numClouds; cloudIter++) 
        apply_transform_to_cloud(clouds[cloudIter], transVec[cloudIter]);
    }
    
    // Build the trees
    std::vector<boost::shared_ptr<KDTree_double>> Trees;
    for (int it = 0; it < numClouds; it++) {
      boost::shared_ptr<KDTree_double>
        tree(new KDTree_double(flann::KDTreeSingleIndexParams(FIFTEEN)));
      BuildKDTree_double(clouds[it], tree.get());
      Trees.push_back(tree);
    }

    std::string errCaption = std::string("Computing the error, defined as the mean of ") +
      "pairwise distances from each cloud to the centroid cloud.\n";
    if (opt.verbose) vw_out() << errCaption;

    // Mean error from the centroid cloud points to all matching
    // points in all clouds.  We don't use the mean square error like
    // the Matlab code, which had bugs in how it was computed anyway.
    double initError = -1, errAfter = -1, prevError = -1; 

    Stopwatch sw1;
    sw1.start();

    vw::TerminalProgressCallback tpc("asp", "Performing alignment\t--> ");
    double inc_amount = 1.0 / std::max(opt.num_iter, 1);

    // Don't display the progress in verbose mode, as then it interferes with
    // printing of errors.
    if (!opt.verbose)
      tpc.report_progress(0);
    
    int step = 0; // Starting step
    // int step = 1; // original incorrect logic in the Matlab code
    while (step < opt.num_iter){

      bool firstStep = (step == 0);
      bool lastStep  = (step == opt.num_iter - 1);
      
      std::vector<int> modelSpan(numClouds+1);
      int numOfPoints = 0;
      for (int it = 0; it < numClouds; it++) {
        modelSpan[it] = numOfPoints;
        numOfPoints += clouds[it].size();
      }
      modelSpan[numClouds] = numOfPoints;

      // This will record for each point in each cloud which point in
      // every other cloud is closest to it. This matrix will store the
      // indices of these points.
      std::vector<Eigen::VectorXd> CentroidPtsBelMod(numOfPoints);
      for (size_t row = 0; row < CentroidPtsBelMod.size(); row++) {
        CentroidPtsBelMod[row] = Eigen::VectorXd(numClouds);
        for (int cloudIter = 0; cloudIter < numClouds; cloudIter++) {
          CentroidPtsBelMod[row][cloudIter] = -1;
        }
      }
    
      for (size_t i = 0; i < numClouds; i++) {

        //spanI = modelSpan(i)+1:modelSpan(i+1); 
        std::vector<int> spanI;
        int beg = modelSpan[i], end = modelSpan[i+1] - 1;
        for (int it = beg; it <= end; it++) spanI.push_back(it);
        
        //CentroidPtsBelMod(spanI,i) = 1:length(spanI);
        for (size_t it = 0; it < spanI.size(); it++) {
          CentroidPtsBelMod[spanI[it]][i] = it;
        }

        for (size_t j = i + 1; j < numClouds; j++) {
        
          //spanJ = modelSpan(j)+1:modelSpan(j+1); 
          std::vector<int> spanJ;
          int beg = modelSpan[j], end = modelSpan[j+1] - 1;
          for (int it = beg; it <= end; it++) spanJ.push_back(it);
        
          std::vector<int> match, match2;
          std::vector<double> dist;
          typedef std::set<std::pair<int, int>, CustomCompare> PairType;

          // For each point in cloud i, find a match in cloud j
          PairType Corr1;
          for (size_t index_i = 0; index_i < clouds[i].size(); index_i++){
            SearchKDTree_double(Trees[j].get(), clouds[i][index_i], match, dist, 1);
            if (match.empty()) continue; // should not happen
            int index_j = match[0];
            Corr1.insert(std::pair<int, int>(index_j, index_i));
          }

          // Now do it in reverse
          PairType Corr2;
          for (size_t index_j = 0; index_j < clouds[j].size(); index_j++){
            SearchKDTree_double(Trees[i].get(), clouds[j][index_j], match, dist, 1);
            if (match.empty()) continue; // should not happen
            int index_i = match[0];
            Corr2.insert(std::pair<int, int>(index_j, index_i));
          }

          std::vector<std::pair<int, int>> Corr;
          for (PairType::iterator it = Corr1.begin(); it != Corr1.end(); it++) {
            PairType::iterator it2 = Corr2.find(*it);
            if (it2 == Corr2.end()) continue;

            Corr.push_back(*it);
          }

          for (size_t it = 0; it < Corr.size(); it++) {
            // CentroidPtsBelMod(spanI(Corr(:,2)),j) = Corr(:,1)';
            CentroidPtsBelMod[spanI[Corr[it].second]][j] = Corr[it].first;

            // CentroidPtsBelMod(spanJ(Corr(:,1)),i) = Corr(:,2)';
            CentroidPtsBelMod[spanJ[Corr[it].first]][i] = Corr[it].second;
          }

        }
      }

      std::sort(CentroidPtsBelMod.begin(), CentroidPtsBelMod.end(), vector_less);

      // Remove non-unique elements, and remove entries which exist only in one cloud
      int pos = 0;
      for (size_t row = 0; row < CentroidPtsBelMod.size(); row++) {

        int num_good = 0;
        for (int cloudIter = 0; cloudIter < CentroidPtsBelMod[row].size(); cloudIter++) {
          if (CentroidPtsBelMod[row][cloudIter] >= 0)
            num_good++;
        }
        if (num_good < 2) continue;
        if (row > 0 && CentroidPtsBelMod[row - 1] == CentroidPtsBelMod[row]) 
          continue;
      
        CentroidPtsBelMod[pos] = CentroidPtsBelMod[row];
        pos++;
      }
      CentroidPtsBelMod.resize(pos);

      // We connected the clouds. Find the average cloud.
      std::vector<vw::Vector3> centroid(CentroidPtsBelMod.size());
      for (size_t row = 0; row < CentroidPtsBelMod.size(); row++) {
        vw::Vector3 pt(0, 0, 0);
        int num = 0;
        for (int cloudIter = 0; cloudIter < CentroidPtsBelMod[row].size(); cloudIter++) {
          int idx = CentroidPtsBelMod[row][cloudIter];
          if (idx < 0) continue;
          vw::Vector3 curr = clouds[cloudIter][idx];
          pt += curr;
          num++;
        }
        pt /= num;
        centroid[row] = pt;
      }

      double errBefore = 0;
      errAfter = 0;
      int numErrors = 0;
      
      // Find the transform from each cloud to the centroid, and apply it to each cloud
      for (size_t cloudIter = 0; cloudIter < numClouds; cloudIter++) {

        std::vector<Eigen::Vector3d> src, dst; 
        Eigen::Matrix3d rot;
        Eigen::Vector3d trans;
      
        for (int row = 0; row < CentroidPtsBelMod.size(); row++) {
          int pointIter = CentroidPtsBelMod[row][cloudIter];
          if (pointIter < 0) continue;
        
          vw::Vector3 curr = clouds[cloudIter][pointIter];
          vw::Vector3 ctr  = centroid[row];

          Eigen::Vector3d p;
          p[0] = curr[0]; p[1] = curr[1]; p[2] = curr[2];
          src.push_back(p);
        
          p[0] = ctr[0]; p[1] = ctr[1]; p[2] = ctr[2];
          dst.push_back(p);

	  errBefore += norm_2(curr - ctr);

	  numErrors++;
        }
	
        computeRigidTransform(src, dst, rot, trans);

        // Update the output transforms
        Eigen::MatrixXd currT = Eigen::MatrixXd::Identity(4, 4);;
        for (int row = 0; row < 3; row++)
          for (int col = 0; col < 3; col++) 
            currT(row, col) = rot(row, col);
        for (int row = 0; row < 3; row++)
          currT(row, 3) = trans(row);
        transVec[cloudIter] = currT*transVec[cloudIter];

        // Move the clouds to the new location for the next iteration
        apply_transform_to_cloud(clouds[cloudIter], currT);

	// Compute the error after the transform is applied
        for (int row = 0; row < CentroidPtsBelMod.size(); row++) {
          int pointIter = CentroidPtsBelMod[row][cloudIter];
          if (pointIter < 0) continue;
          vw::Vector3 curr = clouds[cloudIter][pointIter];
          vw::Vector3 ctr  = centroid[row];
	  errAfter += norm_2(curr - ctr);
        }

      }

      errBefore /= numErrors;
      errAfter /= numErrors;

      if (firstStep) {
        initError = errBefore;
        if (opt.verbose) {
          vw_out() << "Error before alignment: " << initError  << std::endl;
        }
      }
      
      if (opt.verbose) 
        vw_out() << "Error at iteration " << step << ": " << errAfter << "\n";
      else
        tpc.report_incremental_progress(inc_amount);

      if (!firstStep) {
        if (errAfter > 0 && std::abs(errAfter - prevError)/errAfter < opt.rel_error_tol) {
          break;
        }
      }
      
      prevError = errAfter;
      
      step++;
    } // End of iterations refining the transforms

    if (!verbose) {
      tpc.report_finished();
    }
    
    sw1.stop();
    
    if (!opt.verbose) {
      vw_out() << "\n" << errCaption;
      vw_out() << "Error before alignment: " << initError  << std::endl;
    }
    vw_out() << "Error after alignment:  " << errAfter << std::endl;
    
    vw_out() << "Performed: " << step + 1 << " iterations.\n";

    vw_out() << "\nAlignment took " << sw1.elapsed_seconds() << " [s]" << endl;

    if (opt.align_to_first_cloud) {
      // Make the first transform be the identity
      for (int cloudIter = 1; cloudIter < numClouds; cloudIter++) 
        transVec[cloudIter] = transVec[0].inverse() * transVec[cloudIter];
      transVec[0] = Eigen::MatrixXd::Identity(4, 4);
    }
  
    // Undo the shift when the clouds were read
    for (int cloudIter = 0; cloudIter < numClouds; cloudIter++) 
      transVec[cloudIter] = apply_shift(transVec[cloudIter], -shift);
    
    for (int cloudIter = 0; cloudIter < numClouds; cloudIter++)
      vw_out() << "Final transform for cloud: " << cloudIter << ":\n"
               << transVec[cloudIter] << std::endl;
    write_transforms(transVec, opt.out_prefix);

    if (opt.save_transformed_clouds) {
      for (int cloudIter = 0; cloudIter < numClouds; cloudIter++) {
        std::ostringstream os;
        os << opt.out_prefix << "-trans_cloud-" << cloudIter;
        std::string trans_prefix = os.str();
        save_trans_point_cloud(opt, opt.cloud_files[cloudIter], trans_prefix,
                               geo, csv_conv, transVec[cloudIter]);
      }
    }
  
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
