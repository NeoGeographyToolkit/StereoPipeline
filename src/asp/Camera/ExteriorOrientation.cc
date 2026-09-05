// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
//  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
//  License for the specific language governing permissions and limitations
//  under the License.
// __END_LICENSE__

/// \file ExteriorOrientation.cc

#include <asp/Camera/ExteriorOrientation.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/ReportUtils.h>
#include <asp/Core/CameraTransforms.h>

#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/Datum.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>

#include <set>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <vector>

namespace fs = boost::filesystem;

namespace asp {

// One exterior-orientation record parsed from the vendor file.
struct EoRecord {
  double E, N, Z;           // position in the projected CRS (easting, northing, height)
  double omega, phi, kappa; // orientation angles, in degrees
};

// Elementary rotation matrices (angles in radians).
namespace {

vw::Matrix3x3 Rx(double a) {
  vw::Matrix3x3 m;
  m.set_identity();
  m(1,1) =  cos(a);
  m(1,2) = -sin(a);
  m(2,1) =  sin(a);
  m(2,2) =  cos(a);
  return m;
}

vw::Matrix3x3 Ry(double a) {
  vw::Matrix3x3 m;
  m.set_identity();
  m(0,0) =  cos(a);
  m(0,2) =  sin(a);
  m(2,0) = -sin(a);
  m(2,2) =  cos(a);
  return m;
}

vw::Matrix3x3 Rz(double a) {
  vw::Matrix3x3 m;
  m.set_identity();
  m(0,0) =  cos(a);
  m(0,1) = -sin(a);
  m(1,0) =  sin(a);
  m(1,1) =  cos(a);
  return m;
}

double deg2rad(double d) {
  return d * M_PI / 180.0;
}

// Split on any of the given separator chars, collapsing runs, trimming each
// field, and dropping empty fields. Used for the exterior-orientation rows (once
// the delimiter is chosen) and for the distortion coefficient lists.
void splitDrop(std::string const& line, std::string const& seps,
               std::vector<std::string> & out) {
  out.clear();
  std::vector<std::string> raw;
  boost::split(raw, line, boost::is_any_of(seps), boost::token_compress_on);
  for (size_t i = 0; i < raw.size(); i++) {
    std::string v = boost::trim_copy(raw[i]);
    if (!v.empty())
      out.push_back(v);
  }
}

} // end anonymous namespace

// Read the ESRI exterior-orientation report. It is a tab or whitespace separated
// table whose header names the columns (Filename, X, Y, Z, Omega, Phi, Kappa, ...).
// We locate columns by name (not position) since vendors reorder them. Positions
// are keyed by the image base name (no directory, no extension).
static void parseEsriEoFile(std::string const& eo_file,
                            std::map<std::string, EoRecord> & records) {
  std::ifstream ifs(eo_file.c_str());
  if (!ifs.good())
    vw_throw(vw::ArgumentErr() << "Could not open the exterior-orientation file: "
             << eo_file << ".\n");

  std::string line;
  if (!std::getline(ifs, line))
    vw_throw(vw::ArgumentErr() << "The exterior-orientation file is empty: "
             << eo_file << ".\n");

  // Choose the delimiter from the header: if it has a tab, the file is
  // tab-delimited (column names such as "Image ID" contain spaces, so space is
  // NOT a separator here); otherwise fall back to whitespace. The same delimiter
  // is used for the data rows.
  std::string delim = (line.find('\t') != std::string::npos) ? "\t" : " \t";

  // Parse the header, mapping a lowercased column name to its index.
  std::vector<std::string> cols;
  splitDrop(line, delim, cols);
  std::map<std::string, int> col;
  for (size_t i = 0; i < cols.size(); i++) {
    std::string c = boost::trim_copy(cols[i]);
    boost::to_lower(c);
    col[c] = (int)i;
  }
  const char* need[] = {"filename", "x", "y", "z", "omega", "phi", "kappa"};
  for (int k = 0; k < 7; k++)
    if (col.find(need[k]) == col.end())
      vw_throw(vw::ArgumentErr() << "The exterior-orientation file " << eo_file
               << " is missing the '" << need[k] << "' column. Found header: "
               << line << ".\n");

  while (std::getline(ifs, line)) {
    if (boost::trim_copy(line).empty()) continue;
    // Data values never contain spaces, so split rows on any whitespace (tab or
    // space). This tolerates a file that mixes tabs and spaces between columns
    // (common after copy-paste), while the header above kept its spaced names.
    std::vector<std::string> vals;
    splitDrop(line, " \t", vals);
    // The number of values in a row must equal the number of header columns.
    if (vals.size() != cols.size())
      vw_throw(vw::ArgumentErr() << "In the exterior-orientation file " << eo_file
               << ", a data row has " << vals.size() << " values but the header has "
               << cols.size() << " columns. They must be one-to-one. Row: "
               << line << ".\n");
    std::string fname = boost::trim_copy(vals[col["filename"]]);
    std::string base = fs::path(fname).stem().string(); // no dir, no extension
    EoRecord r;
    r.E     = atof(vals[col["x"]].c_str());
    r.N     = atof(vals[col["y"]].c_str());
    r.Z     = atof(vals[col["z"]].c_str());
    r.omega = atof(vals[col["omega"]].c_str());
    r.phi   = atof(vals[col["phi"]].c_str());
    r.kappa = atof(vals[col["kappa"]].c_str());
    records[base] = r;
  }
  if (records.empty())
    vw_throw(vw::ArgumentErr() << "No records parsed from the exterior-orientation file: "
             << eo_file << ".\n");
  vw::vw_out() << "Parsed " << records.size() << " exterior-orientation records from "
               << eo_file << ".\n";
}

// Read the ESRI camera (interior orientation) CSV. It has a header line and one
// data line: FocalLength (microns), PrincipalX/Y, NRows, NCols, PixelSize (microns).
// We return the intrinsics in pixel units with a pixel pitch of 1, which is the
// convention CSM uses natively and which avoids the mm-vs-pixel ambiguity.
// Fields may be separated by any of space, tab, comma, or semicolon (see
// tokenize). Named columns give the focal length and pixel size (microns), the
// principal point, and the image size. If a DistortionType column is present, the
// numbers after it are the OpenCV radial-tangential coefficients in the vendor
// order K1, K2, K3, P1, P2 (a longer radial list is reduced to the first three
// radial and the last two tangential). We return the intrinsics in pixel units
// (pitch 1) and the distortion in the TsaiLensDistortion order k1, k2, p1, p2, k3
// (left empty if all zero, i.e. undistorted).
static void parseEsriCameraCsv(std::string const& camera_csv,
                               double & focal_px, double & cu_px, double & cv_px,
                               std::vector<double> & dist) {
  dist.clear();
  std::ifstream ifs(camera_csv.c_str());
  if (!ifs.good())
    vw_throw(vw::ArgumentErr() << "Could not open the camera file: "
             << camera_csv << ".\n");
  std::string header, data;
  if (!std::getline(ifs, header) || !std::getline(ifs, data))
    vw_throw(vw::ArgumentErr() << "The camera file must have a header and a data line: "
             << camera_csv << ".\n");

  // Comma-separated (the vendor format), keeping empty fields so the header and
  // data stay column-aligned. Columns are then located BY NAME, so the many extra
  // columns in the delivery are ignored and the layout may vary.
  std::vector<std::string> hc, dc;
  boost::split(hc, header, boost::is_any_of(","), boost::token_compress_off);
  boost::split(dc, data,   boost::is_any_of(","), boost::token_compress_off);
  // The header and the data line must have the same number of fields.
  if (hc.size() != dc.size())
    vw_throw(vw::ArgumentErr() << "In the camera file " << camera_csv
             << ", the header has " << hc.size() << " columns but the data line has "
             << dc.size() << " values. They must be one-to-one.\n");
  std::map<std::string, std::string> f;
  for (size_t i = 0; i < hc.size(); i++)
    f[boost::to_lower_copy(boost::trim_copy(hc[i]))] = boost::trim_copy(dc[i]);
  const char* need[] = {"focallength", "pixelsize", "nrows", "ncols"};
  for (int k = 0; k < 4; k++)
    if (f.find(need[k]) == f.end())
      vw_throw(vw::ArgumentErr() << "The camera file " << camera_csv
               << " is missing the '" << need[k] << "' column.\n");

  double focal_um = atof(f["focallength"].c_str());
  double pix_um   = atof(f["pixelsize"].c_str());
  int nrows = atoi(f["nrows"].c_str());
  int ncols = atoi(f["ncols"].c_str());
  if (pix_um <= 0 || focal_um <= 0 || nrows <= 0 || ncols <= 0)
    vw_throw(vw::ArgumentErr() << "Invalid focal/pixel/size values in the camera file: "
             << camera_csv << ".\n");

  focal_px = focal_um / pix_um;
  // PrincipalX/Y are an offset from the frame center, in the same units as pixel size.
  double ppx = (f.count("principalx") ? atof(f["principalx"].c_str()) : 0.0) / pix_um;
  double ppy = (f.count("principaly") ? atof(f["principaly"].c_str()) : 0.0) / pix_um;
  cu_px = ncols / 2.0 + ppx;
  cv_px = nrows / 2.0 + ppy;

  // Lens distortion, from the Radial and Tangential columns (located by name).
  // Radial holds K1, K2, K3 (a longer list is truncated to the first three);
  // Tangential holds P1, P2. Inside those fields the sub-separator may be ';',
  // ',' cannot occur (it is the field delimiter), or whitespace.
  double k1 = 0, k2 = 0, k3 = 0, p1 = 0, p2 = 0;
  if (f.count("radial")) {
    std::vector<std::string> rv;
    splitDrop(f["radial"], "; \t", rv);
    if (rv.size() >= 1) k1 = atof(rv[0].c_str());
    if (rv.size() >= 2) k2 = atof(rv[1].c_str());
    if (rv.size() >= 3) k3 = atof(rv[2].c_str());
  }
  if (f.count("tangential")) {
    std::vector<std::string> tv;
    splitDrop(f["tangential"], "; \t", tv);
    if (tv.size() >= 1) p1 = atof(tv[0].c_str());
    if (tv.size() >= 2) p2 = atof(tv[1].c_str());
  }
  if (k1 == 0 && k2 == 0 && k3 == 0 && p1 == 0 && p2 == 0)
    return; // undistorted: leave dist empty

  // TsaiLensDistortion order is k1, k2, p1, p2, k3.
  dist.push_back(k1);
  dist.push_back(k2);
  dist.push_back(p1);
  dist.push_back(p2);
  dist.push_back(k3);
}

// Grid convergence at (E, N): the true-north bearing of the projected-grid north
// direction, derived geometrically from the georeference (no PROJ factors needed,
// no sign guessing). A small step north in grid coordinates is converted to
// geographic; its azimuth is the convergence. Works for any projected CRS.
static double gridConvergence(vw::cartography::GeoReference const& geo,
                              double E, double N) {
  double step = 1.0; // meters, in grid coordinates
  vw::Vector2 ll0 = geo.point_to_lonlat(vw::Vector2(E, N));
  vw::Vector2 ll1 = geo.point_to_lonlat(vw::Vector2(E, N + step));
  double lat0 = deg2rad(ll0[1]);
  double dlon = deg2rad(ll1[0] - ll0[0]) * cos(lat0);
  double dlat = deg2rad(ll1[1] - ll0[1]);
  return atan2(dlon, dlat); // radians, grid-north bearing east of true north
}

// Assemble the camera-to-ECEF rotation from ESRI omega/phi/kappa (referenced to
// the projected grid). Compose: ENU->ECEF * Rz(grid->true north) * R_opk *
// (photo->computer-vision axis flip). The grid-to-true-north step is the
// negative of the grid convergence, verified against known-good cameras.
static vw::Matrix3x3 esriRotationToEcef(vw::cartography::GeoReference const& geo,
                                        EoRecord const& r, double lon, double lat) {
  vw::Matrix3x3 R_io = Rz(deg2rad(r.kappa)) * Ry(deg2rad(r.phi)) * Rx(deg2rad(r.omega));

  // ENU->ECEF from the datum NED basis (columns are N, E, D). ENU = [E, N, U=-D].
  vw::Matrix3x3 ned = geo.datum().lonlat_to_ned_matrix(vw::Vector3(lon, lat, 0));
  vw::Matrix3x3 enu;
  for (int i = 0; i < 3; i++) {
    enu(i,0) =  ned(i,1); // East
    enu(i,1) =  ned(i,0); // North
    enu(i,2) = -ned(i,2); // Up
  }

  // Grid north to true north is the negative of the grid convergence.
  double conv = gridConvergence(geo, r.E, r.N);
  vw::Matrix3x3 gridToTrue = Rz(-conv);

  // Photo axes (x right, y up, z up) to computer-vision axes (x right, y down,
  // z forward).
  vw::Matrix3x3 F;
  F.set_identity();
  F(1,1) = -1;
  F(2,2) = -1;

  return enu * gridToTrue * R_io * F;
}

void camerasFromExteriorOrientation(EoOptions const& opt) {

  if (opt.vendor != "esri")
    vw_throw(vw::ArgumentErr() << "Unsupported --vendor value: '" << opt.vendor
             << "'. Only 'esri' is supported at this time.\n");
  if (opt.extrinsics_file.empty())
    vw_throw(vw::ArgumentErr() << "Must set --extrinsics (the vendor "
             << "exterior-orientation report).\n");
  if (opt.image_list.empty())
    vw_throw(vw::ArgumentErr() << "Must set --image-list (the input images).\n");
  if (opt.output_dir.empty())
    vw_throw(vw::ArgumentErr() << "Must set --output-dir (where the cameras "
             << "are written).\n");
  if (opt.proj_str.empty())
    vw_throw(vw::ArgumentErr() << "Must set --t_srs (the CRS of the exterior-orientation "
             << "positions). It cannot be inferred from easting/northing alone.\n");
  if (opt.intrinsics_file.empty() && opt.sample_tsai.empty())
    vw_throw(vw::ArgumentErr() << "Must set --intrinsics (vendor interior "
             << "orientation) or --sample-file (a sample .tsai with the intrinsics).\n");

  // Georeference for the projected CRS -> lon/lat and ECEF. Let the CRS string
  // (e.g. EPSG:32617) define the datum; do not force a user datum, which would
  // build a non-single CRS.
  vw::cartography::Datum datum(opt.datum_str.empty() ? "WGS_1984" : opt.datum_str);
  vw::cartography::GeoReference geo;
  vw::cartography::set_srs_string(opt.proj_str, false, datum, geo);

  // Interior orientation (in pixels, pitch = 1). dist is the radtan lens
  // distortion (TsaiLensDistortion order), empty if the camera is undistorted.
  double focal_px = 0, cu_px = 0, cv_px = 0;
  std::vector<double> dist;
  if (!opt.intrinsics_file.empty()) {
    parseEsriCameraCsv(opt.intrinsics_file, focal_px, cu_px, cv_px, dist);
  } else {
    vw::camera::PinholeModel s(opt.sample_tsai);
    double pitch = s.pixel_pitch();
    focal_px = s.focal_length()[0] / pitch;
    cu_px = s.point_offset()[0] / pitch;
    cv_px = s.point_offset()[1] / pitch;
  }
  vw::vw_out() << "Interior orientation (pixels): focal = " << focal_px
               << ", optical center = (" << cu_px << ", " << cv_px << ").\n";

  // Exterior orientation records, keyed by image base name.
  std::map<std::string, EoRecord> records;
  parseEsriEoFile(opt.extrinsics_file, records);

  // Input images, matched to EO records by base name.
  std::vector<std::string> images;
  asp::read_list(opt.image_list, images);

  fs::create_directories(opt.output_dir);
  std::vector<std::string> out_cams;

  int matched = 0;
  for (size_t i = 0; i < images.size(); i++) {
    std::string base = fs::path(images[i]).stem().string();
    std::map<std::string, EoRecord>::const_iterator it = records.find(base);
    if (it == records.end()) {
      vw::vw_out() << "Warning: no exterior orientation for image " << images[i]
                   << ", skipping.\n";
      continue;
    }
    EoRecord const& r = it->second;

    vw::Vector2 lonlat = geo.point_to_lonlat(vw::Vector2(r.E, r.N));
    vw::Vector3 llh(lonlat[0], lonlat[1], r.Z);
    vw::Vector3 ctr = geo.datum().geodetic_to_cartesian(llh);
    vw::Matrix3x3 R = esriRotationToEcef(geo, r, lonlat[0], lonlat[1]);

    // Build the Pinhole. A metric aerial camera is typically undistorted (dist
    // empty); otherwise apply the radtan lens distortion read from the vendor file.
    double pixel_pitch = 1.0;
    vw::camera::PinholeModel pin(ctr, R, focal_px, focal_px, cu_px, cv_px,
                                 NULL, pixel_pitch);
    if (!dist.empty()) {
      vw::Vector<double> coeffs;
      coeffs.set_size(dist.size());
      for (size_t k = 0; k < dist.size(); k++)
        coeffs[k] = dist[k];
      vw::camera::TsaiLensDistortion distModel(coeffs);
      pin.set_lens_distortion(&distModel);
    }

    std::string out_cam = (fs::path(opt.output_dir) / (base + ".tsai")).string();
    pin.write(out_cam);
    out_cams.push_back(out_cam);
    matched++;
  }

  if (matched == 0)
    vw_throw(vw::ArgumentErr() << "No images matched exterior-orientation records "
             << "by file name.\n");

  std::string cam_list = (fs::path(opt.output_dir) / "camera_list.txt").string();
  asp::write_list(cam_list, out_cams);
  vw::vw_out() << "Wrote " << matched << " cameras to " << opt.output_dir
               << " and the camera list " << cam_list << ".\n";
  vw::vw_out() << "Validate by mapprojecting a frame onto a reference DEM to confirm "
               << "it lands correctly.\n";
}

void camerasFromExtrinsics(EoOptions const& opt) {

  // Read the extrinsics report (roll/pitch/yaw over lon/lat/height).
  std::set<std::string> str_col_names = {"image"};
  std::set<std::string> num_col_names = {"lon", "lat", "height_above_datum",
                                         "roll", "pitch", "yaw"};
  std::map<std::string, std::vector<std::string>> str_map;
  std::map<std::string, std::vector<double>> num_map;
  asp::readReportFile(opt.extrinsics_file, str_col_names, num_col_names,
                      str_map, num_map);

  // Read the intrinsics from the sample camera, and the datum.
  vw::camera::PinholeModel pinhole(opt.sample_tsai);
  vw::cartography::Datum datum(opt.datum_str);

  int num_cameras = str_map["image"].size();
  if (num_cameras == 0)
    vw_throw(vw::ArgumentErr() << "No extrinsics found in: "
             << opt.extrinsics_file << ".\n");

  for (int i = 0; i < num_cameras; i++) {

    double lon                = num_map["lon"][i];
    double lat                = num_map["lat"][i];
    double height_above_datum = num_map["height_above_datum"][i];
    double roll               = num_map["roll"][i];
    double pitch              = num_map["pitch"][i];
    double yaw                = num_map["yaw"][i];

    // Camera center in ECEF.
    vw::Vector3 llh(lon, lat, height_above_datum);
    vw::Vector3 P = datum.geodetic_to_cartesian(llh);

    // Camera-to-world rotation.
    vw::Matrix3x3 ned = datum.lonlat_to_ned_matrix(llh);
    vw::Matrix3x3 R = ned * asp::rollPitchYaw(roll, pitch, yaw) * asp::rotationXY();

    pinhole.set_camera_center(P);
    pinhole.set_camera_pose(R);

    // Write the camera next to the image, with a .tsai extension.
    std::string imageFile = str_map["image"][i];
    std::string camFile = fs::path(imageFile).replace_extension(".tsai").string();
    vw::create_out_dir(camFile);
    vw::vw_out() << "Writing: " << camFile << "\n";
    pinhole.write(camFile);
  }
}

} // end namespace asp
