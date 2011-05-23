// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <vw/Core/Stopwatch.h>
#include <vw/Core/Log.h>
#include <vw/Plate/Rpc.h>
#include <vw/Plate/HTTPUtils.h>
#include <vw/Plate/IndexService.h>
#include <asp/PhotometryTK/ProjectService.h>
#include <asp/Core/Macros.h>
#include <signal.h>

#include <google/protobuf/descriptor.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace vw;
using namespace vw::platefile;
using namespace asp::pho;

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

struct Options {
  Url url, index_url;
  std::string ptk_file;
  float sync_interval;
  bool debug, help;
};

// -- Main Loop --------------------------

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("Runs a mosaicking daemon that listens for mosaicking requests coming in over the AMQP bus..\n\nGeneral Options:");
  general_options.add_options()
    ("url", po::value(&opt.url), "Url to listen in on")
    ("sync-interval,s", po::value(&opt.sync_interval)->default_value(60),
     "Specify the time interval (in minutes) for automatically synchronizing the index to disk.")
    ("debug", po::bool_switch(&opt.debug)->default_value(false),
     "Output debug messages.")
    ("help,h", po::bool_switch(&opt.help)->default_value(false),
     "Display this help message");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("ptk-file", po::value(&opt.ptk_file));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description p;
  p.add("ptk-file", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <ptk file> --url <url>" << std::endl << std::endl;
  usage << general_options << std::endl;

  if ( opt.help )
    vw_throw( ArgumentErr() << usage.str() );
  if ( opt.ptk_file.size() < 5 )
    vw_throw( ArgumentErr() << "Error: must specify a ptk file to be servering!\n\n" << usage.str() );
}

int main(int argc, char** argv) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Install Unix Signal Handlers.  These will help us to gracefully
    // recover and salvage the index under most unexpected error
    // conditions.
    signal(SIGINT,  sig_unexpected_shutdown);
    signal(SIGUSR1, sig_sync);

    // Clean & Create new url
    if ( opt.ptk_file.substr(opt.ptk_file.size()-4,4) != ".ptk" )
      vw_throw( ArgumentErr() << "Failed to provide input ptk file." );
    std::string scheme = opt.url.scheme();
    if ( scheme == "pf" || scheme == "amqp" ) {
#if defined(VW_HAVE_PKG_RABBITMQ_C) && VW_HAVE_PKG_RABBITMQ_C==1
      opt.index_url = opt.url;
      opt.index_url.path( opt.url.path()+"_index" );
#else
      vw_throw( ArgumentErr() << "ptk_server: This build does not support AMQP.\n" );
#endif
    } else if ( scheme == "zmq" || scheme == "zmq+ipc" ||
                scheme == "zmq+tcp" || scheme == "zmq+inproc" ) {
#if defined(VW_HAVE_PKG_ZEROMQ) && VW_HAVE_PKG_ZEROMQ==1
      opt.index_url = Url( opt.url.scheme() + "://" + opt.url.hostname() + ":" +
                           boost::lexical_cast<std::string>(opt.url.port()+1) );
#else
      vw_throw( ArgumentErr() << "ptk_server: This build does not support ZeroMQ.\n" );
#endif
    } else {
      vw_throw( ArgumentErr() << "ptk_server: Unknown URL scheme \"" << scheme
                << "\".\n" );
    }

    fs::path ptk_path( opt.ptk_file );

    // Start the ptk server task in another thread
    RpcServer<ProjectServiceImpl> server_1(opt.url, new ProjectServiceImpl(ptk_path.string()));
    RpcServer<IndexServiceImpl> server_2(opt.index_url, new IndexServiceImpl(ptk_path.string()));
    vw_out(InfoMessage) << "Starting ptk server\n";
    vw_out(InfoMessage) << "\tw/ URL: " << opt.url << "\n";
    vw_out(InfoMessage) << "Starting index server\n";
    vw_out(InfoMessage) << "\tw/ URL: " << opt.index_url << "\n";
    uint64 sync_interval_us = uint64(opt.sync_interval * 60000000);
    uint64 t0 = Stopwatch::microtime(), t1;
    uint64 next_sync = t0 + sync_interval_us;

    size_t success = 0, fail = 0, calls = 0;

    while(process_messages) {
      if ( server_1.error() ) {
        vw_out(InfoMessage) << "PTK Service has terminated with message: " << server_1.error() << "\n";
        break;
      } else if ( server_2.error() ) {
        vw_out(InfoMessage) << "Index Service has terminated with message: " << server_2.error() << "\n";
        break;
      }

      bool should_sync = force_sync || (Stopwatch::microtime() >= next_sync);

      if ( should_sync ) {
        vw_out(InfoMessage) << "\nStarting sync to disk. (" << (force_sync ? "auto" : "manual") << ")\n";
        uint64 s0 = Stopwatch::microtime();
        server_1.impl()->sync();
        server_2.impl()->sync();
        uint64 s1 = Stopwatch::microtime();
        next_sync = s1 + sync_interval_us;
        vw_out(InfoMessage) << "Sync complete (took " << float(s1-s0) / 1e6  << " seconds).\n";
        force_sync = false;
      }

      t1 = Stopwatch::microtime();

      size_t success_dt, fail_dt, calls_dt;
      {
        ThreadMap::Locked stats = server_2.stats();
        success_dt = stats.get("msgs");
        fail_dt    = stats.get("server_error") + stats.get("client_error");
        calls_dt = success_dt + fail_dt;
        stats.clear();
      }
      success += success_dt;
      fail    += fail_dt;
      calls   += calls_dt;

      float dt = float(t1 - t0) / 1e6f;
      t0 = t1;

      vw_out(InfoMessage)
        << "[ptk_server] : "
        << float(calls_dt)/dt << " qps "
        << "(" << (100. * success / (calls ? calls : 1)) << "% success)                    \r"
        << std::flush;

      Thread::sleep_ms(1000);
    }

    server_2.stop();
    vw_out(InfoMessage) << "\nShutting down the index service safely.\n";
    server_2.impl()->sync();
    server_1.stop();
    vw_out(InfoMessage) << "\nShutting down the ptk service safely.\n";
    server_1.impl()->sync();

  } ASP_STANDARD_CATCHES;

  return 0;
}
