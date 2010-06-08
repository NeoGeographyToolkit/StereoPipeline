// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <test/Helpers.h>
#include <vw/Core/Settings.h>

int main(int argc, char **argv) {
  // Disable the user's config file
  vw::vw_settings().set_rc_filename("");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
