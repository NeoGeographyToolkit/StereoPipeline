// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

#include <asp/Camera/TimeProcessing.h>

namespace asp {

// Boost does not like a time string such as "2017-12-07 15:36:40.90795Z"
// because it expects precisely 6 digits after the dot. Fix that.
std::string fix_millisecond(std::string const& in_str) {

  std::string out_str = "";
  bool found_dot = false;
  int num_digits_after_dot = 0;
  for (size_t it = 0; it < in_str.size(); it++) {

    if (it + 1 < in_str.size()) {
      // Not yet at the last character

      if (in_str[it] == '.') {
        // Found the dot
        found_dot = true;
        out_str += in_str[it];
        continue;
      }

      if (!found_dot) {
        // Not at the dot yet
        out_str += in_str[it];
        continue;
      }

      // After the dot
      if (num_digits_after_dot < 6) {
        out_str += in_str[it];
        num_digits_after_dot++;
      }
      continue;
    }

    // At the last character
    if (in_str[it] >= '0' && in_str[it] <= '9') {
      // The last character is a digit, just append it
      if (num_digits_after_dot < 6) {
        out_str += in_str[it];
        num_digits_after_dot++;
      }

      // See if to append more
      while (found_dot && num_digits_after_dot < 6) {
        out_str += "0";
        num_digits_after_dot++;
      }

    } else {

      // The last character is not a digit, it is likely a "Z"
      while (found_dot && num_digits_after_dot < 6) {
        // Append zeros
        out_str += "0";
        num_digits_after_dot++;
      }

      // Append the last character, whatever it is
      out_str += in_str[it];
    }

  } // End iterating over characters

  return out_str;
}

boost::posix_time::ptime parse_time(std::string const& s) {

  // Replace the T with a space so the default Boost function can
  // parse the time.
  std::string s2 = s;
  boost::replace_all(s2, "T", " ");

  // Ensure there are exactly 6 digits for the millisecond or else
  // Boost will complain.
  s2 = asp::fix_millisecond(s2);
  boost::posix_time::ptime time = boost::posix_time::time_from_string(s2);

  return time;
}

// Return the time in seconds since January 1, 2000, down to the microsecond.
// Avoid starting at the epoch (1970) as that may create some loss in precision.
double to_seconds(const boost::posix_time::ptime& pt) {
  boost::posix_time::ptime epoch(boost::gregorian::date(2000, 1, 1));
  boost::posix_time::time_duration diff = pt - epoch;
  return diff.total_microseconds() / 1.0e+6;
}

} // end namespace asp
