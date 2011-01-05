// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/BundleAdjustment/CameraRelation.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::ba;
using namespace vw::camera;

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem/path.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Core/Macros.h>

size_t find_destination_index( int const& source_idx,
                               std::map<int,std::string> const& src_map,
                               std::map<std::string,int> & dst_map,
                               size_t & dst_max_idx ) {
  std::map<std::string,int>::iterator search =
    dst_map.find( src_map.find(source_idx)->second );
  size_t dst_index;
  if ( search == dst_map.end() ) {
    dst_max_idx++;
    dst_index = dst_max_idx;
    std::string serial = src_map.find(source_idx)->second;
    vw_out() << "  Adding camera w/ serial: " << serial << "\n";
    dst_map[ serial ] = dst_index;
  } else {
    dst_index = search->second;
  }
  return dst_index;
}

void print_cnet_statistics( ControlNetwork const& cnet ) {
  int cmeasure_size = 0;
  BOOST_FOREACH( ControlPoint const& cp, cnet ) {
    cmeasure_size+=cp.size();
  }
  vw_out() << "  CP : " << cnet.size() << "   CM : " << cmeasure_size << "\n";
}

struct ContainsEqualMeasure {
  Vector2 m_position;
  ContainsEqualMeasure( Vector2 const& pos ) : m_position(pos) {}

  bool operator()( boost::shared_ptr<IPFeature> in ) {
    if ( m_position[0] == in->m_ip.x &&
         m_position[1] == in->m_ip.y )
      return true;
    return false;
  }
};

struct ContainsCloseMeasure {
  Vector2 m_position;
  double m_close;
  ContainsCloseMeasure( Vector2 const& pos, double const& close ) : m_position(pos), m_close(close) {}

  bool operator()( boost::shared_ptr<IPFeature> in ) {
    if ( norm_2( Vector2(in->m_ip.x, in->m_ip.y) - m_position ) <= m_close )
      return true;
    return false;
  }
};

struct Options {
  // Input
  std::string destination_cnet;
  std::vector<std::string> source_cnets;

  double close;

  // Output
  std::string output_prefix;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o", po::value(&opt.output_prefix)->default_value("merged"),
     "Output prefix for merge control network.")
    ("close-px", po::value(&opt.close)->default_value(-1),
     "Merge measurements are that are this pixel close. Leave -1 to only merge exact measurements." )
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("dest-cnet", po::value(&opt.destination_cnet) )
    ("source-cnets", po::value(&opt.source_cnets) );

  po::positional_options_description positional_desc;
  positional_desc.add("dest-cnet", 1 );
  positional_desc.add("source-cnets", -1 );

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <dest> <source1> ... <sourceN>\n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.destination_cnet.empty() )
    vw_throw( ArgumentErr() << "Missing destination cnets.\n"
              << usage.str() << general_options );
  if ( opt.source_cnets.empty() )
    vw_throw( ArgumentErr() << "Missing source cnets.\n"
              << usage.str() << general_options );
}

int main( int argc, char** argv ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    ControlNetwork dst_cnet("destination");
    dst_cnet.read_binary( opt.destination_cnet );

    vw_out() << "Input " << opt.destination_cnet << ":\n";
    print_cnet_statistics( dst_cnet );

    // Filling in dst_serial and dst_max_cam_idx
    std::map<std::string,int> dst_serial_to_cam_idx;
    size_t dst_max_cam_idx = 0;
    {
      float inc_amt = 1.0/float(dst_cnet.size());
      TerminalProgressCallback tpc("","Destination Idx:");
      BOOST_FOREACH( ControlPoint const& cp, dst_cnet ) {
        tpc.report_incremental_progress( inc_amt );
        BOOST_FOREACH( ControlMeasure const& cm, cp ) {
          if ( dst_serial_to_cam_idx.find( cm.serial() ) ==
               dst_serial_to_cam_idx.end() ) {
            dst_serial_to_cam_idx[cm.serial()] = cm.image_id();
            if ( cm.image_id() > dst_max_cam_idx )
              dst_max_cam_idx = cm.image_id();
          }
        }
      }
      tpc.report_finished();
    }

    CameraRelationNetwork<IPFeature> dst_crn;
    dst_crn.read_controlnetwork( dst_cnet );

    BOOST_FOREACH( std::string const& source_cnet, opt.source_cnets ) {
      ControlNetwork src_cnet("source");
      src_cnet.read_binary( source_cnet );

      // The reason we have a destination input, is that it specifies
      // the camera indexing we should use.
      vw_out() << "Destination Input " << source_cnet << ":\n";
      print_cnet_statistics( src_cnet );

      typedef std::map<int,std::string> src_map_type;
      src_map_type src_cam_idx_to_serial;
      float inc_amt = 1.0/float(src_cnet.size());
      {
        TerminalProgressCallback tpc("cnet",source_cnet+" Idx:");
        BOOST_FOREACH( ControlPoint const& cp, src_cnet ) {
          tpc.report_incremental_progress( inc_amt );
          BOOST_FOREACH( ControlMeasure const& cm, cp ) {
            if ( src_cam_idx_to_serial.find( cm.image_id() ) ==
                 src_cam_idx_to_serial.end() )
              src_cam_idx_to_serial[cm.image_id()] = cm.serial();
          }
        }
        tpc.report_finished();
      }

      {
        TerminalProgressCallback tpc("cnet",source_cnet+" Merge:");
        BOOST_FOREACH( ControlPoint const& cp, src_cnet ) {
          tpc.report_incremental_progress(inc_amt );

          typedef boost::shared_ptr<IPFeature> f_ptr;
          typedef std::list<f_ptr>::iterator f_itr;

          ControlPoint::const_iterator cm1, cm2;
          cm1 = cm2 = cp.begin();
          cm2++;
          size_t dst_index1 =
            find_destination_index( cm1->image_id(),
                                    src_cam_idx_to_serial,
                                    dst_serial_to_cam_idx,
                                    dst_max_cam_idx );

          if ( dst_index1 == dst_crn.size() )
            dst_crn.add_node( CameraNode<IPFeature>(dst_index1,"") );

          f_itr dst_feature1;
          if ( opt.close < 0 ) {
            dst_feature1 = std::find_if( dst_crn[dst_index1].begin(),
                                         dst_crn[dst_index1].end(),
                                         ContainsEqualMeasure(cm1->position()));
          } else {
            dst_feature1 = std::find_if( dst_crn[dst_index1].begin(),
                                         dst_crn[dst_index1].end(),
                                         ContainsCloseMeasure(cm1->position(),opt.close));
          }
          if ( dst_feature1 == dst_crn[dst_index1].end() ) {
            dst_crn[dst_index1].relations.push_front( f_ptr( new IPFeature(*cm1,0, dst_index1) ));
            dst_feature1 = dst_crn[dst_index1].begin();
          }
          while ( cm2 != cp.end() ) {
            size_t dst_index2 =
              find_destination_index( cm2->image_id(),
                                      src_cam_idx_to_serial,
                                      dst_serial_to_cam_idx,
                                      dst_max_cam_idx );

            if ( dst_index2 == dst_crn.size() )
              dst_crn.add_node( CameraNode<IPFeature>(dst_index2,"") );

            f_itr dst_feature2;
            if ( opt.close < 0 ) {
              dst_feature2 = std::find_if( dst_crn[dst_index2].begin(),
                                           dst_crn[dst_index2].end(),
                                           ContainsEqualMeasure(cm2->position()));
            } else {
              dst_feature2 = std::find_if( dst_crn[dst_index2].begin(),
                                           dst_crn[dst_index2].end(),
                                           ContainsCloseMeasure(cm2->position(),opt.close));
            }
            if ( dst_feature2 == dst_crn[dst_index2].end() ) {
              dst_crn[dst_index2].relations.push_front( f_ptr( new IPFeature( *cm2, 0, dst_index2 )));
              dst_feature2 = dst_crn[dst_index2].begin();
            }

            // Doubly linking
            (*dst_feature1)->connection( *dst_feature2, true );
            (*dst_feature2)->connection( *dst_feature1, true );

            dst_index1 = dst_index2;
            dst_feature1 = dst_feature2;
            cm1++; cm2++;
          }
        }
        tpc.report_finished();
      }
    }

    dst_crn.write_controlnetwork( dst_cnet );
    vw_out() << "Output Control Network:\n";
    print_cnet_statistics( dst_cnet );

    // Re apply serial
    std::map<int,std::string> reverse_dst;
    for ( std::map<std::string,int>::iterator it = dst_serial_to_cam_idx.begin();
          it != dst_serial_to_cam_idx.end(); it++ ) {
      reverse_dst[it->second] = it->first;
    }
    BOOST_FOREACH( ControlPoint & cp, dst_cnet ) {
      BOOST_FOREACH( ControlMeasure & cm, cp ) {
        cm.set_serial( reverse_dst[cm.image_id()] );
      }
    }

    dst_cnet.write_binary(opt.output_prefix);

  } ASP_STANDARD_CATCHES;
}
