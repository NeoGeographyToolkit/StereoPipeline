// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <vw/Core/Stopwatch.h>
#include <vw/Plate/RpcServices.h>
#include <vw/Plate/AmqpConnection.h>
#include <asp/PhotometryTK/ProjectService.h>
#include <google/protobuf/descriptor.h>
#include <signal.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
using namespace vw;
using namespace vw::platefile;
using namespace asp::pho;

// Global variable
boost::shared_ptr<ProjectServiceImpl> g_service;

// -- Signal Handler ---------------------

volatile bool process_messages = true;
volatile bool force_sync = false;

void sig_unexpected_shutdown( int sig_num ) {
  signal(sig_num, SIG_IGN);
  process_messages = false;
  signal(sig_num, sig_unexpected_shutdown);
}

void sig_sync( int sig_num ) {
  signal(sig_num, SIG_IGN);
  force_sync = true;
  signal(sig_num, sig_sync);
}

// -- Main Loop --------------------------

class ServerTask {
  boost::shared_ptr<AmqpRpcServer> m_server;

public:

  ServerTask(boost::shared_ptr<AmqpRpcServer> server) : m_server(server) {}

  void operator()() {
    std::cout << "\t--> Listening for messages.\n";
    m_server->run();
  }

  void kill() { m_server->shutdown(); }
};

int main(int argc, char** argv) {
  std::string exchange_name, root_directory;
  std::string hostname;
  float sync_interval;
  int port;

  po::options_description general_options("Runs a mosaicking daemon that listens for mosaicking requests coming in over the AMQP bus..\n\nGeneral Options:");
  general_options.add_options()
    ("exchange,e", po::value<std::string>(&exchange_name)->default_value("ptk"),
     "Specify the name of the AMQP exchange to use for the index service.")
    ("hostname", po::value<std::string>(&hostname)->default_value("localhost"),
     "Specify the hostname of the AMQP server to use for remote procedure calls (RPCs).")
    ("port,p", po::value<int>(&port)->default_value(5672),
     "Specify the port of the AMQP server to use for remote procedure calls (RPCs).")
    ("sync-interval,s", po::value<float>(&sync_interval)->default_value(60),
     "Specify the time interval (in minutes) for automatically synchronizing the index to disk.")
    ("debug", "Output debug messages.")
    ("help,h", "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("root-directory", po::value<std::string>(&root_directory));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("root-directory", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " root_directory" << std::endl << std::endl;
  usage << general_options << std::endl;

  if( vm.count("help") ) {
    std::cout << usage.str();
    return 0;
  }

  if( vm.count("root-directory") != 1 ) {
    std::cerr << "Error: must specify a root directory that contains plate files!"
              << std::endl << std::endl;
    std::cout << usage.str();
    return 1;
  }

  std::string queue_name = unique_name("ptk_server");

  boost::shared_ptr<AmqpConnection> connection( new AmqpConnection(hostname, port) );
  boost::shared_ptr<AmqpRpcServer> server( new AmqpRpcServer(connection, exchange_name, 
                                                             queue_name, vm.count("debug")) );
  g_service.reset( new ProjectServiceImpl(root_directory) );
  server->bind_service(g_service, "ptk");

  // Start the server task in another thread
  boost::shared_ptr<ServerTask> server_task( new ServerTask(server) );
  Thread server_thread( server_task );

  // Install Unix Signal Handlers.  These will help us to gracefully
  // recover and salvage the index under most unexpected error
  // conditions.
  signal(SIGINT,  sig_unexpected_shutdown);
  signal(SIGUSR1, sig_sync);

  std::cout << "Starting photometry project server\n\n";
  long long t0 = Stopwatch::microtime();

  
  long long sync_interval_seconds = uint64(sync_interval * 60);
  long long seconds_until_sync = sync_interval_seconds;

  while(process_messages) {
    // Check to see if the user generated a sync signal.
    if (force_sync) {
      std::cout << "\nReceived signal USR1.  Synchronizing project files to disk:\n";
      long long sync_t0 = Stopwatch::microtime();
      g_service->sync();
      float sync_dt = float(Stopwatch::microtime() - sync_t0) / 1e6;
      std::cout << "Sync complete (took " << sync_dt << " seconds).\n";
      force_sync = false;
    }

    // Check to see if our sync timeout has occurred.
    if (seconds_until_sync-- <= 0) {
      std::cout << "\nAutomatic sync of project files started (interval = " << sync_interval << " minutes).\n";
      long long sync_t0 = Stopwatch::microtime();
      g_service->sync();
      float sync_dt = float(Stopwatch::microtime() - sync_t0) / 1e6;
      std::cout << "Sync complete (took " << sync_dt << " seconds).\n";

      // Restart the count-down
      seconds_until_sync = sync_interval_seconds;
    }

    int queries = server->queries_processed();
    size_t bytes = server->bytes_processed();
    int n_outstanding_messages = server->incoming_message_queue_size();
    server->reset_stats();

    float dt = float(Stopwatch::microtime() - t0) / 1e6;
    t0 = Stopwatch::microtime();

    std::cout << "[ptk_server] : "
              << float(queries/dt) << " qps    "
              << float(bytes/dt)/1000.0 << " kB/sec    "
              << n_outstanding_messages << " outstanding messages                          \r"
              << std::flush;
    sleep(1.0);
  }

  std::cout << "\nShutting down the index service safely.\n";
  g_service->sync();

  return 0;
}
