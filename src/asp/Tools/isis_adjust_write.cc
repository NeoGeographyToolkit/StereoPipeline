// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <Process.h>
#include <Table.h>
#include <CubeAttribute.h>
#include <Camera.h>
#include <PvlContainer.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/Core.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/Matrix.h>
using namespace vw;

#include <asp/Core/Macros.h>
#include <asp/IsisIO/Equation.h>

struct Options {
  std::string isis_adjust_name, cube_file_name;
  bool write;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("modify,m", "Actually write the adjustment back into cube. This is a safety measure to make sure you know what you are doing.")
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("isis-adjust", po::value(&opt.isis_adjust_name))
    ("cube-file",   po::value(&opt.cube_file_name));

  po::positional_options_description positional_desc;
  positional_desc.add("isis-adjust", 1);
  positional_desc.add("cube-file", 1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e ) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <isis adjust> <cube file>\n";
  opt.write = vm.count("modify");
  if ( vm.count("help") || opt.isis_adjust_name.empty() ||
       opt.cube_file_name.empty() )
    vw_throw( ArgumentErr() << usage.str() << general_options );
}

int main(int argc, char* argv[]) {


  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Load up cube file
    Isis::Process p;
    Isis::CubeAttributeInput inAtt;
    Isis::Cube *c =
      p.SetInputCube( opt.cube_file_name, inAtt, Isis::ReadWrite );

    // Do the stand cleanup that ISIS seems to do in all their apps
    if (c->Label()->HasObject("Polygon"))
      c->Label()->DeleteObject("Polygon");
    for (int iobj=0; iobj < c->Label()->Objects(); iobj++) {
      Isis::PvlObject obj = c->Label()->Object(iobj);
      if (obj.Name() != "Table") continue;
      if (obj["Name"][0] != Isis::iString("CameraStatistics")) continue;
      c->Label()->DeleteObject(iobj);
      break;
    }

    // Load up our camera model
    std::ifstream input_adjust( opt.isis_adjust_name.c_str() );
    boost::shared_ptr<asp::BaseEquation> position_eq =
      asp::read_equation( input_adjust );
    boost::shared_ptr<asp::BaseEquation> pointing_eq =
      asp::read_equation( input_adjust );
    input_adjust.close();

    // Extract the starting cmatrix and spvector
    typedef Isis::PvlContainer::ConstPvlKeywordIterator iterator;
    Isis::Camera *cam = c->Camera();
    Isis::Table ckTable =
      cam->InstrumentRotation()->Cache("InstrumentPointing");
    Isis::Table spkTable =
      cam->InstrumentPosition()->Cache("InstrumentPosition");
    Isis::Table bodyTable =
      cam->BodyRotation()->Cache("BodyRotation");
    std::cout << "Ck Keywords: " << "\n";
    for ( iterator it = ckTable.Label().Begin();
          it != ckTable.Label().End(); it++ )
      std::cout << "\t" << *it << "\n";
    std::cout << "Spk Keywords: " << "\n";
    for ( iterator it = spkTable.Label().Begin();
          it != spkTable.Label().End(); it++ )
      std::cout << "\t" << *it << "\n";
    std::cout << "Body Keywords: " << "\n";
    for ( iterator it = bodyTable.Label().Begin();
          it != bodyTable.Label().End(); it++ )
      std::cout << "\t" << *it << "\n";

    // Apply new position
    for ( int r = 0; r < ckTable.Records(); r++ ) {
      if ( ckTable[r].Fields() != 5 &&
           ckTable[r].Fields() != 8 )
        vw_throw( NoImplErr() << "Unsupported ckTable with "
                  << ckTable[r].Fields() << " elements." );
      unsigned et_index = ckTable[r].Fields()-1;

      // Quaternion + Time
      Vector3 adj_angles =
        pointing_eq->evaluate( (double)ckTable[r][et_index] );
      Quat adj_quat = math::axis_angle_to_quaternion( adj_angles );

      // Solving for new Instrument Pointing
      cam->SetEphemerisTime( (double)ckTable[r][et_index] );
      std::vector<double> rot_inst = cam->InstrumentRotation()->Matrix();
      std::vector<double> rot_body = cam->BodyRotation()->Matrix();
      MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
      MatrixProxy<double,3,3> R_body(&(rot_body[0]));
      Quat rot_inst_mod = Quat(R_inst*transpose(R_body))*adj_quat*Quat(R_body);

      // Writing result back into table
      for ( char i = 0; i < 4; i++ )
        ckTable[r][i] = rot_inst_mod[i];
    }

    // Apply new pointing
    for ( int r = 0; r < spkTable.Records(); r++ ) {
      if ( spkTable[r].Fields() != 4 &&
           spkTable[r].Fields() != 7 )
        vw_throw( NoImplErr() << "Unsupported spkTable with " << spkTable[r].Fields() << " elements." );
      unsigned et_index = spkTable[r].Fields()-1;

      // J2000 + ET
      Vector3 adjustment =
        position_eq->evaluate( (double)spkTable[r][et_index] );
      adjustment /= 1000;

      // Solving for new Instrument Position
      cam->SetEphemerisTime( (double)spkTable[r][et_index] );
      Vector3 instrument;
      cam->InstrumentPosition(&instrument[0]);
      std::vector<double> rot_body = cam->BodyRotation()->Matrix();
      MatrixProxy<double,3,3> R_body(&(rot_body[0]));
      Vector3 instrument_prime =
        transpose(R_body)*(R_body*instrument+adjustment);

      for ( char i = 0; i < 3; i++ )
        spkTable[r][i] = instrument_prime[i];
    }

    // Write back into cube file
    ckTable.Label().AddComment("Rewritten with Isis Adjust Solution");
    spkTable.Label().AddComment("Rewritten with Isis Adjust Solution");

    if ( opt.write ) {
      c->Write(ckTable);
      c->Write(spkTable);
      p.WriteHistory(*c);
    } else {
      c->Close();
    }
  } ASP_STANDARD_CATCHES;

  return 0;
}
