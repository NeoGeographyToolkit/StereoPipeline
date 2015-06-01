// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
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


/// \file PointUtils.cc
///

#include <liblas/liblas.hpp>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>
#include <vw/Cartography/Chipper.h>
#include <boost/math/special_functions/fpclassify.hpp>

using namespace vw;
using namespace vw::cartography;
using namespace pdal::filters;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3> { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

namespace asp{

  // Classes to read points from CSV and LAS files one point at a
  // time. We basically implement an interface for CSV files
  // mimicking the existing interface for las files in liblas.

  class BaseReader{

  public:
    boost::uint64_t m_num_points;
    bool m_has_georef;
    GeoReference m_georef;
    virtual bool ReadNextPoint() = 0;
    virtual Vector3 GetPoint() = 0;

    virtual ~BaseReader(){}
  };

  class LasReader: public BaseReader{
    liblas::Reader& m_reader;
  public:

    LasReader(liblas::Reader & reader):m_reader(reader){
      liblas::Header const& header = m_reader.GetHeader();
      m_num_points = header.GetPointRecordsCount();

      std::string wkt = header.GetSRS().GetWKT();
      m_has_georef = false;
      if (wkt != ""){
        m_has_georef = true;
        m_georef.set_wkt(wkt);
      }
    }

    virtual bool ReadNextPoint(){
      return m_reader.ReadNextPoint();
    }

    virtual Vector3 GetPoint(){
      liblas::Point const& p = m_reader.GetPoint();
      return Vector3(p.GetX(), p.GetY(), p.GetZ());
    }

  };

  class CsvReader: public BaseReader{
    std::string m_csv_file;
    asp::CsvConv m_csv_conv;
    bool m_is_first_line;
    bool m_has_valid_point;
    Vector3 m_curr_point;
    std::ifstream * m_ifs;
  public:

    CsvReader(std::string const & csv_file,
              asp::CsvConv const& csv_conv,
              GeoReference const& georef)
      : m_csv_file(csv_file), m_csv_conv(csv_conv),
        m_is_first_line(true), m_has_valid_point(false){

      // We will convert from projected space to xyz, unless points
      // are already in this format.
      m_has_georef = (m_csv_conv.format != asp::XYZ);

      m_georef      = georef;
      m_num_points  = asp::csv_file_size(m_csv_file);

      m_ifs = new std::ifstream ( m_csv_file.c_str() );
      if ( !*m_ifs ) {
        vw_throw( vw::IOErr() << "Unable to open file \"" << m_csv_file << "\"" );
      }

      VW_ASSERT(m_csv_conv.csv_format_str != "",
                ArgumentErr() << "CsvReader: The CSV format was not specified.\n");

    }

    virtual bool ReadNextPoint(){

      std::string line;
      Vector3 vals;

      // Need to try twice, since perhaps the first time the header
      // was encountered. The routine below will throw if the second
      // pass did not succeed or if we try two passes more than once.

      for (int i = 0; i < 2; i++){

        m_has_valid_point = getline(*m_ifs, line, '\n');
        if (!m_has_valid_point) return m_has_valid_point; // reached end of file

        bool success;
        vals = asp::parse_csv_line(m_is_first_line, success, line, m_csv_conv);
        if (success) break;
      }

      // Will return projected point and height or xyz
      bool return_point_height = true;
      m_curr_point = asp::csv_to_cartesian_or_point_height(vals, m_georef,
                                                           m_csv_conv,
                                                           return_point_height);

      return m_has_valid_point;
    }

    virtual Vector3 GetPoint(){
      return m_curr_point;
    }

    virtual ~CsvReader(){
      delete m_ifs;
      m_ifs = NULL;
    }

  };

  // Create a point cloud image from a las file. The image will be
  // created block by block, when it needs to be written to disk. It is
  // important that the writer invoking this image be single-threaded,
  // as we read from the las file sequentially.

  template <class ImageT>
  class LasOrCsvToTif:
    public ImageViewBase< LasOrCsvToTif<ImageT> > {
    typedef typename ImageT::pixel_type PixelT;
    asp::BaseReader * m_reader;
    int m_rows, m_cols;
    int m_block_size;

  public:

    typedef PixelT pixel_type;
    typedef PixelT result_type;
    typedef ProceduralPixelAccessor<LasOrCsvToTif> pixel_accessor;

    LasOrCsvToTif(asp::BaseReader * reader, int num_rows, int tile_len, int block_size):
      m_reader(reader), m_block_size(block_size){

      boost::uint64_t num_points = m_reader->m_num_points;
      m_rows = tile_len*std::max(1, (int)ceil(double(num_rows)/tile_len));
      m_cols = (int)ceil(double(num_points)/m_rows);
      m_cols = tile_len*std::max(1, (int)ceil(double(m_cols)/tile_len));
    }

    inline int32 cols() const { return m_cols; }
    inline int32 rows() const { return m_rows; }
    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor(*this); }

    inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {

      vw_throw( NoImplErr() << "LasOrCsvToTif::operator(...) has not been implemented.\n");
      return result_type();
    }

    typedef CropView<ImageView<PixelT> > prerasterize_type;
    inline prerasterize_type prerasterize( BBox2i const& bbox ) const{

      // Read a chunk of the las file, and store it in the current tile.

      int num_cols = bbox.width();
      int num_rows = bbox.height();

      VW_ASSERT(num_rows % m_block_size == 0 && num_cols % m_block_size == 0,
                ArgumentErr() << "LasOrCsvToTif: Expecting the number of rows "
                << "to be a multiple of the block size.\n");

      int max_num_pts_to_read = num_cols*num_rows;
      int count = 0;
      PointBuffer in;
      std::vector<PointBuffer> outVec;
      while (m_reader->ReadNextPoint()){
        in.push_back(m_reader->GetPoint());
        count++;
        if (count >= max_num_pts_to_read) break;
      }

      ImageView<Vector3> Img;
      Chipper(in, m_block_size, m_reader->m_has_georef, m_reader->m_georef,
              num_cols, num_rows, Img);

      VW_ASSERT(num_cols == Img.cols() && num_rows == Img.rows(),
                ArgumentErr() << "LasOrCsvToTif: Size mis-match.\n");

      return crop( Img, -bbox.min().x(), -bbox.min().y(), cols(), rows() );

    }

    template <class DestT>
    inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }

  };

} // namespace asp

bool asp::is_las(std::string const& file){
  std::string lfile = boost::to_lower_copy(file);
  return (boost::iends_with(lfile, ".las")  || boost::iends_with(lfile, ".laz"));
}

bool asp::is_csv(std::string const& file){
  std::string lfile = boost::to_lower_copy(file);
  return ( boost::iends_with(lfile, ".csv")  || boost::iends_with(lfile, ".txt")  );
}

bool asp::is_las_or_csv(std::string const& file){
  return asp::is_las(file) || is_csv(file);
}

bool asp::georef_from_las(std::string const& las_file,
                          vw::cartography::GeoReference & georef){

  if (!is_las(las_file))
    vw_throw( ArgumentErr() << "Not a LAS file: " << las_file << "\n");

  // Initialize
  georef = GeoReference();

  std::ifstream ifs;
  ifs.open(las_file.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);
  liblas::Header const& header = reader.GetHeader();

  std::string wkt = header.GetSRS().GetWKT();
  if (wkt == "")
    return false;

  georef.set_wkt(wkt);
  return true;
}

bool asp::georef_from_las(std::vector<std::string> const& files,
                          vw::cartography::GeoReference & georef){

  // Get the georeference from the first las file

  // Initialize
  georef = GeoReference();

  for (int i = 0; i < (int)files.size(); i++){
    GeoReference local_georef;
    if (!is_las(files[i])) continue;
    if (asp::georef_from_las(files[i], local_georef)){
      georef = local_georef;
      return true;
    }
  }

  return false;
}

boost::uint64_t asp::las_file_size(std::string const& las_file){
  std::ifstream ifs;
  ifs.open(las_file.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);
  liblas::Header const& header = reader.GetHeader();
  return header.GetPointRecordsCount();
}

void asp::las_or_csv_to_tif(std::string const& in_file,
                            std::string const& out_file,
                            int num_rows, int block_size,
                            asp::BaseOptions * opt,
                            vw::cartography::GeoReference const& csv_georef,
                            asp::CsvConv const& csv_conv){

  // We will fetch a chunk of the las file of area tile_len x
  // tile_len, split it into bins of spatially close points, and write
  // it to disk as a tile in a vector tif image. The bigger the tile
  // size, the more likely the binning will be more efficient. But big
  // tiles use a lot of memory.

  // To do: Study performance for large files when this number changes
  int tile_len = 2048;
  Vector2 tile_size(tile_len, tile_len);

  vw_out() << "Writing temporary file: " << out_file << std::endl;

  Vector2 original_tile_size = opt->raster_tile_size;
  opt->raster_tile_size = tile_size;

  if (asp::is_csv(in_file)){

    boost::shared_ptr<asp::CsvReader> csvReaderPtr
      ( new asp::CsvReader(in_file, csv_conv, csv_georef) );
    ImageViewRef<Vector3> Img
      = asp::LasOrCsvToTif< ImageView<Vector3> > (csvReaderPtr.get(), num_rows,
                                                  tile_len, block_size);

    // Must use a thread only, as we read the las file serially.
    asp::write_gdal_image(out_file, Img, *opt, TerminalProgressCallback("asp", "\t--> ") );

  }else if (asp::is_las(in_file)){

    std::ifstream ifs;
    ifs.open(in_file.c_str(), std::ios::in | std::ios::binary);
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    boost::shared_ptr<asp::LasReader> lasReaderPtr( new asp::LasReader(reader) );
    ImageViewRef<Vector3> Img
      = asp::LasOrCsvToTif< ImageView<Vector3> > (lasReaderPtr.get(), num_rows,
                                                  tile_len, block_size);

    // Must use a thread only, as we read the las file serially.
    asp::write_gdal_image(out_file, Img, *opt, TerminalProgressCallback("asp", "\t--> ") );
  }else
    vw_throw( ArgumentErr() << "Unknown file type: " << in_file << "\n");

  // Restore the original tile size
  opt->raster_tile_size = original_tile_size;

}

bool asp::read_user_datum(double semi_major, double semi_minor,
                          std::string const& reference_spheroid,
                          cartography::Datum& datum ) {
  // Select a cartographic datum. There are several hard coded datums
  // that can be used here, or the user can specify their own.
  if ( reference_spheroid != "" ) {
    if (reference_spheroid == "mars") {
      datum.set_well_known_datum("D_MARS");
      vw_out() << "\t--> Re-referencing altitude values using a Mars spheroid\n";
    } else if (reference_spheroid == "moon") {
      datum.set_well_known_datum("D_MOON");
      vw_out() << "\t--> Re-referencing altitude values using a Lunar spheroid\n";
    } else if (reference_spheroid == "earth") {
      vw_out() << "\t--> Re-referencing altitude values using the Earth WGS84 spheroid\n";
    } else {
      vw_throw( ArgumentErr() << "\t--> Unknown reference spheriod: "
                << reference_spheroid
                << ". Current options are [Earth, Moon, Mars].\nExiting." );
    }
    vw_out() << "\t    Axes [" << datum.semi_major_axis() << " "
             << datum.semi_minor_axis() << "] meters\n";
  } else if (semi_major > 0 && semi_minor > 0) {
    vw_out() << "\t--> Re-referencing altitude values to user supplied datum.\n"
             << "\t    Semi-major: " << semi_major << "  Semi-minor: " << semi_minor << "\n";
    datum = cartography::Datum("User Specified Datum",
                               "User Specified Spheroid",
                               "Reference Meridian",
                               semi_major, semi_minor, 0.0);
  } else {
    return false;
  }
  return true;
}

void asp::handle_easting_northing(asp::CsvConv const& csv_conv,
                                  vw::cartography::GeoReference & georef){

  // If the user passed in a csv file containing easting, northing, height
  // above datum, and either a utm zone or a custom proj4 string,
  // pass that info into the georeference for the purpose of converting
  // later from easting and northing to lon and lat.

  if (csv_conv.format != asp::EASTING_HEIGHT_NORTHING)
    return; // nothing to do

  if (csv_conv.utm_zone >= 0) {
    try{
      georef.set_UTM(csv_conv.utm_zone, csv_conv.utm_north);
    } catch ( const std::exception& e ) {
      vw_throw(ArgumentErr() << "Detected error: " << e.what()
               << "\nPlease check if you are using an Earth datum.\n");
    }
  } else if (csv_conv.csv_proj4_str != "") {
    bool have_user_datum = false;
    Datum user_datum;
    asp::set_srs_string(csv_conv.csv_proj4_str,
                        have_user_datum, user_datum, georef);

  }else{
    vw_throw( ArgumentErr() << "When a CSV file has easting and northing, the PROJ.4 string must be set via --csv_proj4.\n" );
  }

}

void asp::parse_utm_str(std::string const& utm, int & zone, bool & north){

  // Parse the string 58N

  // Initialize
  zone = -1; north = false;

  std::string a, b;
  for (int s = 0; s < (int)utm.size(); s++){
    if (utm[s] >= '0' && utm[s] <= '9'){
      a += utm[s];
    }else{
      b = utm[s];
      break;
    }
  }

  if (a == "" || b == "")
    vw_throw(ArgumentErr() << "Could not parse UTM string: '" << utm << "'\n");

  zone = atoi(a.c_str());
  if (b == "n" || b == "N"){
    north = true;
  }else if (b == "s" || b == "S"){
    north = false;
  }else
    vw_throw(ArgumentErr() << "Could not parse UTM string: '" << utm << "'\n");
}

void asp::parse_csv_format(std::string const& csv_format_str,
                           std::string const& csv_proj4_str,
                           asp::CsvConv & C){

  // Parse the CSV format string and build the data structure which
  // will enable to convert from CSV to Cartesian and vice-versa.

  C = asp::CsvConv(); // reset

  C.csv_format_str = csv_format_str;
  C.csv_proj4_str = csv_proj4_str;

  std::string local = csv_format_str;
  boost::algorithm::to_lower(local);

  if (local == "") return;

  boost::replace_all(local, ":", " ");
  boost::replace_all(local, ",", " ");
  std::istringstream is(local);

  // The case of utm: "utm:23N 1:x 2:y 3:height_above_datum"
  std::string str;
  is >> str;
  if (str == "utm"){
    is >> str;
    asp::parse_utm_str(str, C.utm_zone, C.utm_north);
  }else{
    // Go back to the original string
    is.clear();
    is.str(local);
  }

  int col1, col2, col3;
  std::string name1, name2, name3;
  if (! (is >> col1 >> name1 >> col2 >> name2 >> col3 >> name3) )
    vw_throw(ArgumentErr() << "Could not parse: '" << csv_format_str
             << "'\n");

  col1--;
  col2--;
  col3--;
  if (col1 < 0 || col2 < 0 || col3 < 0)
    vw_throw(ArgumentErr() << "The column indices must be positive in: '"
             << csv_format_str << "'\n");

  if (col1 == col2 || col1 == col3 || col2 == col3 )
    vw_throw(ArgumentErr() << "The column indices must be distinct in: '"
             << csv_format_str << "'\n");

  C.name2col[name1] = col1;
  C.name2col[name2] = col2;
  C.name2col[name3] = col3;

  // For each column index to read, map to corresponding column name
  C.col2name[col1] = name1;
  C.col2name[col2] = name2;
  C.col2name[col3] = name3;

  // Find the position of the longitude field
  int k = 0;
  for (std::map<int, std::string>::iterator it = C.col2name.begin();
       it != C.col2name.end() ; it++){
    if (it->second == "lon") C.lon_index = k;
    if (it->second == "lat") C.lat_index = k;
    k++;
  }

  // Find the position of a column after the columns are sorted
  // alphabetically.
  std::vector<std::string> sorted_names;
  int count = 0;
  for (std::map<std::string, int>::iterator it = C.name2col.begin();
       it != C.name2col.end() ; it++){
    sorted_names.push_back(it->first);
    C.col2sort[it->second] = count;
    count++;
  }

  if (sorted_names[0] == "x" && sorted_names[1] == "y"
      && sorted_names[2] == "z"){
    C.format = asp::XYZ;
  }else if (sorted_names[0] == "lat" &&
            sorted_names[1] == "lon" &&
            sorted_names[2] == "radius_m"){
    C.format = asp::LAT_LON_RADIUS_M;
  }else if (sorted_names[0] == "lat" &&
            sorted_names[1] == "lon" &&
            sorted_names[2] == "radius_km"){
    C.format = asp::LAT_LON_RADIUS_KM;
  }else if (sorted_names[0] == "height_above_datum" &&
            sorted_names[1] == "lat"                &&
            sorted_names[2] == "lon"){
    C.format = asp::HEIGHT_LAT_LON;
  }else if (sorted_names[0] == "easting"            &&
            sorted_names[1] == "height_above_datum" &&
            sorted_names[2] == "northing"){
    C.format = asp::EASTING_HEIGHT_NORTHING;
  }else{
    vw_throw( ArgumentErr() << "Cannot understand the csv format string: "
              << csv_format_str << ".\n" );
  }

}

vw::Vector3 asp::parse_csv_line(bool & is_first_line, bool & success,
                                std::string const& line,
                                asp::CsvConv const& csv_conv){

  // Parse a CSV file line in given format

  success = true;

  const int bufSize = 1024;
  char temp[bufSize];

  std::string sep = asp::csv_separator();

  strncpy(temp, line.c_str(), bufSize);

  int col_index = -1;
  int num_read = 0;

  char * ptr = temp;
  Vector3 vals;
  while(1){

    col_index++;
    const char* token = strtok(ptr, sep.c_str()); ptr = NULL;
    if ( token == NULL ) break; // no more tokens
    if ( num_read >= 3 ) break; // read enough numbers

    // Look only at indices we are supposed to read
    if (csv_conv.col2name.find(col_index) == csv_conv.col2name.end()) continue;

    double val;
    int flag = sscanf(token, "%lg", &val);
    if (flag == 0){
      success = false;
      break;
    }
    vals[num_read] = val;
    num_read++;
  }

  if (num_read != (int)vals.size()) success = false;

  // Be prepared for the fact that the first line may be the header.
  if (!success){
    if (!is_first_line){
      // Not the header
      vw_throw( vw::IOErr() << "Failed to read line: " << line << "\n" );
    }
  }

  is_first_line = false;
  return vals;
}

Vector3 asp::csv_to_cartesian_or_point_height(Vector3 const& csv,
                                              GeoReference const& geo,
                                              asp::CsvConv const& C,
                                              bool return_point_height){

  // Convert values read from a csv file (in the same order as in the file)
  // to a Cartesian point. If return_point_height is true, and the csv point is not
  // in xyz format, return instead the projected point and height above datum.

  // Convert the entries from the order they were in the csv file to
  // the order in which the names of the entries are sorted
  // alphabetically (e.g., in order height_above_datum, lat, lon).
  Vector3 ordered_csv;
  int count = 0;
  for (std::map<int, int>::const_iterator it = C.col2sort.begin();
       it != C.col2sort.end(); it++){
    ordered_csv[it->second] = csv[count];
    count++;
  }

  if (C.format == asp::XYZ)
    return ordered_csv; // already as xyz


  Vector3 xyz;

  if (C.format == asp::EASTING_HEIGHT_NORTHING){

    // go from easting, height, northing to easting, northing, height
    Vector3 point_height = Vector3(ordered_csv[0], ordered_csv[2], ordered_csv[1]);
    if (return_point_height) return point_height;

    Vector2 ll = geo.point_to_lonlat(Vector2(point_height[0], point_height[1]));
    Vector3 llh = Vector3(ll[0], ll[1], point_height[2]); // now lon, lat, height
    xyz = geo.datum().geodetic_to_cartesian(llh);

  }else if (C.format == asp::HEIGHT_LAT_LON){

    std::swap(ordered_csv[0], ordered_csv[2]); // now lon, lat, height
    if (return_point_height) return ordered_csv;

    xyz = geo.datum().geodetic_to_cartesian(ordered_csv);

  }else{

    // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM

    std::swap(ordered_csv[0], ordered_csv[1]); // now lon, lat, radius_(k)m

    if (C.format == asp::LAT_LON_RADIUS_KM)
      ordered_csv[2] *= 1000.0; // now lon, lat, radius_m

    Vector3 tmp = ordered_csv; tmp[2] = 0; // now lon, lat, 0
    xyz = geo.datum().geodetic_to_cartesian( tmp );

    // Update the radius
    xyz = ordered_csv[2]*(xyz/norm_2(xyz));

    if (return_point_height) return geo.datum().cartesian_to_geodetic( xyz );

  }

  return xyz;
}

Vector3 asp::cartesian_to_csv(Vector3 const& xyz,
                              GeoReference const& geo,
                              double mean_longitude,
                              asp::CsvConv const& C){

  // Convert an xyz point to the fields we can write in a CSV file, in
  // the same order as in the input CSV file.

  Vector3 csv;
  if (C.format == asp::XYZ){
    csv = xyz; // order is x, y, z

  }else{

    // Must assert here that the datum was specified.

    Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);   // lon-lat-height
    llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjust

    if (C.format == asp::EASTING_HEIGHT_NORTHING){

      // go from lon, lat to easting, northing
      Vector2 en = geo.lonlat_to_point(Vector2(llh[0], llh[1]));
      csv = Vector3(en[0], llh[2], en[1]); // order is easting, height, northing

    }else if (C.format == asp::HEIGHT_LAT_LON){

      std::swap(llh[0], llh[2]); // order is height, lat, lon
      csv = llh;

    }else{

      // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM
      std::swap(llh[0], llh[1]); // order is lat, lon, height

      llh[2] = norm_2(xyz); // order is lat, lon, radius_m

      if (C.format == asp::LAT_LON_RADIUS_KM){
        llh[2] /= 1000.0; // order is lat, lon, radius_km
      }
      csv = llh;
    }

  }

  // Now we have the csv fields, but they are in the order
  // corresponding to the sorted column names. Need to put them
  // in the same order as they were in the file originally.

  Vector3 csv2;
  int count = 0;
  for (std::map<int, int>::const_iterator it = C.col2sort.begin();
       it != C.col2sort.end(); it++){
    csv2[count] = csv[it->second];
    count++;
  }

  return csv2;
}

bool asp::is_valid_csv_line(std::string const& line){
  // A valid line is not empty and does not start with '#'.
  return (!line.empty()) && (line[0] != '#');
}

boost::uint64_t asp::csv_file_size(std::string const& file){

  std::ifstream fh( file.c_str() );
  if( !fh )
    vw_throw( vw::IOErr() << "Unable to open file \"" << file << "\"" );

  int num_total_points = 0;
  std::string line;
  while ( getline(fh, line, '\n') ){
    if (!asp::is_valid_csv_line(line)) continue;
    num_total_points++;
  }

  return num_total_points;
}

// Erases a file suffix if one exists and returns the base string
std::string asp::prefix_from_pointcloud_filename(std::string const& filename) {
  std::string result = filename;

  // First case: filenames that match <prefix>-PC.<suffix>
  int index = result.rfind("-PC.");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // Second case: filenames that match <prefix>.<suffix>
  index = result.rfind(".");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // No match
  return result;
}

// Compute bounding box of the given cloud. If is_geodetic is false,
// that means a cloud of raw xyz cartesian values, then Vector3()
// signifies no-data. If is_geodetic is true, no-data is suggested
// by having the z component of the point be NaN.
vw::BBox3 asp::pointcloud_bbox(vw::ImageViewRef<vw::Vector3> const& point_image,
                               bool is_geodetic) {

  vw::BBox3 result;
  vw::vw_out() << "Computing the point cloud bounding box.\n";
  vw::TerminalProgressCallback progress_bar("asp", "\t--> ");

  for (int row=0; row < point_image.rows(); ++row ) {
    progress_bar.report_fractional_progress(row, point_image.rows());
    for (int col=0; col < point_image.cols(); ++col ) {
      vw::Vector3 pt = point_image(col, row);
      if ( (!is_geodetic && pt != vw::Vector3()) ||
           (is_geodetic  &&  !boost::math::isnan(pt.z())) )
        result.grow(pt);
    }
  }
  progress_bar.report_finished();

  return result;
}
