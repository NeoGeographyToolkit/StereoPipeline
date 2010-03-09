#include <vw/Core.h>
using namespace vw;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <asp/MPI/BundleAdjustmentMPI.h>

int main ( int argc, char* argv[] ) {
  mpi::environment env(argc,argv);
  mpi::communicator world;
  std::vector<std::string> input_file_names;

  std::srand(time(0)+world.rank());

  // All MPI's slave work is in
  // BundleAdjustmentMPI.
  if (world.rank() > 0) {
    camera::MPISlave slave(world);
    return 0;
  }

  // Main starts here
  po::options_description general_options("Options");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-files", po::value<std::vector<std::string> >(&input_file_names));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-files", -1);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <filenames>..." << std::endl << std::endl;
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store(po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm);
    po::notify( vm );
  } catch (po::error &e) {
    std::cout << "An error occured while parsing command line arguments.\n";
    std::cout << "\t" << e.what() << "\n\n";
    std::cout << usage.str();
    int task = Finish;
    broadcast(world, task, 0);
    return 1;
  }

  if ( vm.count("help") ) {
    vw_out() << usage.str();
    int task = Finish;
    broadcast(world, task, 0);
    return 1;
  } else if ( input_file_names.size() < 1 ) {
    vw_out() << "Forgot to provide input files!\n";
    vw_out() << usage.str();
    int task = Finish;
    broadcast(world, task, 0);
    return 1;
  }

  // Performing task
  int task;
  for (uint i = 0; i < 3; i++ ) {
    task = SolveJacobian;
    Thread::sleep_ms(500);
    broadcast(world, task, 0);
    std::vector<std::vector<int> > all_data;
    std::vector<int> my_data;
    my_data.push_back(0);
    my_data.push_back(12);
    //gather(world, my_data, all_data, 0);
    for ( uint i = 0; i < all_data.size(); i++ ) {
      vw_out() << "Job " << i << " : produced " << all_data[i].size() << "\n";
    }
    vw_out() << "\n";

    Thread::sleep_ms(500);
    task = SolveUpdateError;
    broadcast(world, task, 0);
  }
  Thread::sleep_ms(500);
  task = Finish;
  broadcast(world, task, 0);

  return 0;
}
