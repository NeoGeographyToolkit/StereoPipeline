// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest_ASP.h>
#include <test/Helpers.h>
#include <vw/Core/Settings.h>

#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

namespace {
  vw::uint32 SEED;
}

int main(int argc, char **argv) {
  // Disable the user's config file
  vw::vw_settings().set_rc_filename("");
  ::testing::InitGoogleTest(&argc, argv);

  // Default to the "threadsafe" style because we can't delete our singletons
  // yet; this style of test launches a new process, so the singletons are
  // fresh.
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  if (getenv("VW_DEBUG")) {
    vw::vw_log().console_log().rule_set().add_rule(vw::VerboseDebugMessage, "*");
  }

  // TODO: Make it so the seed is settable so we can reproduce failures in
  // probabilistic algorithms. This uses clock() instead of time() because
  // clock() (being measured in "processor ticks" instead of seconds) is likely
  // to exhibit more variation when tests are run many times in a short time
  // span.
  SEED = boost::numeric_cast<unsigned int>(clock());
  std::srand(SEED);

  fs::path start_dir(TEST_SRCDIR);

  fs::current_path(start_dir);
  VW_ASSERT(fs::equivalent(fs::current_path(), start_dir),
            vw::LogicErr() << "Could not change the working directory to " << start_dir);

  int ret = RUN_ALL_TESTS();
  fs::path end_dir = fs::current_path();

  VW_ASSERT( fs::equivalent(start_dir, end_dir),
             vw::LogicErr() << "Something changed the working directory");
  return ret;
}

namespace vw {
namespace test {

UnlinkName::UnlinkName(const std::string& base, const std::string& directory)
  : std::string(directory + "/" + base) {

  VW_ASSERT(!directory.empty(), ArgumentErr() << "An empty directory path is dangerous");
  fs::remove_all(this->c_str());
}

UnlinkName::UnlinkName(const char *base, const std::string& directory)
  : std::string(directory + "/" + base) {

  VW_ASSERT(!directory.empty(), ArgumentErr() << "An empty directory path is dangerous");
  fs::remove_all(this->c_str());
}

UnlinkName::~UnlinkName() {
  if (!this->empty())
    fs::remove_all(this->c_str());
}

std::string getenv2(const char *key, const std::string& Default) {
  const char *val = getenv(key);
  return val ? val : Default;
}

vw::uint32 get_random_seed() {
  return SEED;
}

}} // namespace vw::test
