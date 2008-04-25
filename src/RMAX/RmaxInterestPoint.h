#ifndef __RMAX_INTEREST_POINTS_H__
#define __RMAX_INTEREST_POINTS_H__

#include <string>
#include <vector>
#include <vw/Math/Vector.h>

// Returns the name of the binary file containing the extracted
// interest points.
std::string detect_interest_points(std::string image_filename);

void match_interest_points(std::string ip_filename1, std::string ip_filename2, 
                           std::vector<vw::Vector2> &final_ip1, std::vector<vw::Vector2> &final_ip2);

#endif // __RMAX_INTEREST_POINTS_H__
