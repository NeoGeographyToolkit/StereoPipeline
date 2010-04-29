#include <gtest/gtest.h>
#include <test/Helpers.h>
#include <vw/Core/Settings.h>

int main(int argc, char **argv) {
  // Disable the user's config file
  vw::vw_settings().set_rc_filename("");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
