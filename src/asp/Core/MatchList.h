// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
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


/// \file MatchList.h
///
/// Interest point matching logic. Used in the GUI.
///
#ifndef __ASP_CORE_MATCHLIST_H__
#define __ASP_CORE_MATCHLIST_H__

// Vision Workbench
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/InterestPoint/InterestData.h>

#include <string>
#include <vector>
#include <list>
#include <set>

namespace asp { 

  /// Helper class to keep track of all the matching interest points
  /// - Each image must have the same number of interest points
  ///   but in some situations some of the points can be flagged as invalid.
  
  // Populate the match files and leftIndices vectors
  void populateMatchFiles(std::vector<std::string> const& image_files,
                          std::string const& output_prefix,
                          std::string const& first_match_file,
                          // Outputs
                          std::vector<std::string> & matchFiles,
                          std::vector<size_t> & leftIndices,
                          bool & matchfiles_found);
  
  class MatchList {
  public:
    /// Clear all exiting points and set up for a new image count.
    void resize(size_t num_images);

    /// Add a single point to the list.
    /// - Points must be added in order, low index to high index.
    /// - Returns false if the point could not be added.
    bool addPoint(size_t image, vw::ip::InterestPoint const &pt, bool valid=true);

    /// Return the number of images.
    size_t getNumImages() const;

    /// Return the number of points (usually but not always the same in each image).
    size_t getNumPoints(size_t image=0) const;

    /// Get a handle to a specific IP.
    vw::ip::InterestPoint const& getPoint(size_t image, size_t point) const;

    /// Returns the x/y coordinate of a point.
    vw::Vector2 getPointCoord(size_t image, size_t point) const;

    /// Return true if the point exists (valid or invalid)
    bool pointExists(size_t image, size_t point) const;

    /// Return true if a point is valid.
    bool isPointValid(size_t image, size_t point) const;

    /// Set the validity of a point.
    void setPointValid(size_t image, size_t point, bool newValue=true);

    /// Change the position of an interest point.
    void setPointPosition(size_t image, size_t point, float x, float y);

    /// Return the index of the nearest match point to the given pixel.
    /// - Returns -1 if no match was found.
    /// - If distLimit is set, return -1 if best match distance is over the limit.
    int findNearestMatchPoint(size_t image, vw::Vector2 P, double distLimit) const;

    /// Delete all IP for an image.
    void deletePointsForImage(size_t image);

    /// Delete the same IP in each image.
    bool deletePointAcrossImages(size_t point);

    /// Returns true if all points are valid and none are missing.
    bool allPointsValid() const;

    /// Try to load all of the points from match files on disk.
    /// - The match files correspond to files 0 < i < N and leftIndices
    ///   contains the index of the other file they match to (0 or i-1).
    /// - Any points that cannot be loaded will be flagged as invalid.
    /// - Return the number of points loaded, or -1 for failure.
    bool loadPointsFromMatchFiles(std::vector<std::string> const& matchFiles,
                                  std::vector<size_t>      const& leftIndices);

    /// Try to load the interest points from a GCP file.
    bool loadPointsFromGCPs(std::string const gcpPath,
                            std::vector<std::string> const& imageNames);

    /// Try to load the interest points from vwip files.
    bool loadPointsFromVwip(std::vector<std::string> const& vwipFiles,
                            std::vector<std::string> const& imageNames);
    
    // Populate from two vectors of interest point matches
    void populateFromIpPair(std::vector<vw::ip::InterestPoint> const& ip1,
                            std::vector<vw::ip::InterestPoint> const& ip2);
    
    /// Write all points out using a given prefix.
    bool savePointsToDisk(std::string const& prefix,
                          std::vector<std::string> const& imageNames,
                          std::string const& match_file="") const;

  private:

    /// Throw an exception if the specified point does not exist.
    void throwIfNoPoint(size_t image, size_t point) const;

    /// Set all IP for the image as valid.
    void setIpValid(size_t image);

    /// A set of interest points for each input image
    /// - There is always one set of matched interest points shared among all images.
    /// - The only way the counts can differ is if the user is in the process of manually
    ///   adding an interest point to the images.
    /// - The length of the outer vector is equal to the number of MainWidget objects
    std::vector<std::vector<vw::ip::InterestPoint>> m_matches;
    /// Stay synced with m_matches, set to false if that match is not 
    std::vector<std::vector<bool>> m_valid_matches;

  }; // End class MatchList

} // namespace asp

#endif  // __ASP_CORE_MATCHLIST_H__
