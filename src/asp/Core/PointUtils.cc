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

// TODO(oalexan1): Move all LAS, PCD, and CSV logic to its own
// PointReader.cc class, as this file is too big.

/// \file PointUtils.cc
///

// Turn off warnings about things we can't control
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <liblas/liblas.hpp>
#pragma GCC diagnostic pop

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>
#include <vw/Cartography/Chipper.h>
#include <vw/Core/Stopwatch.h>
#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/next.hpp>

using namespace vw;
using namespace vw::cartography;
using namespace pdal::filters;


namespace asp{

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
    std::string  m_csv_file;
    asp::CsvConv m_csv_conv;
    bool         m_is_first_line;
    bool         m_has_valid_point;
    Vector3      m_curr_point;
    std::ifstream * m_ifs;
  public:

    CsvReader(std::string const & csv_file,
              asp::CsvConv const& csv_conv,
              GeoReference const& georef)
      : m_csv_file(csv_file), m_csv_conv(csv_conv),
        m_is_first_line(true), m_has_valid_point(false) {

      // We will convert from projected space to xyz, unless points
      // are already in this format.
      m_has_georef = (m_csv_conv.format != asp::CsvConv::XYZ);

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
      asp::CsvConv::CsvRecord vals;

      // Keep on reading, until a valid point is hit or the end of the file
      // is reached.
      while (1){
	
        m_has_valid_point = static_cast<bool>(getline(*m_ifs, line, '\n'));
        if (!m_has_valid_point) return m_has_valid_point; // reached end of file

        vals = m_csv_conv.parse_csv_line(m_is_first_line, m_has_valid_point, line);
        if (m_has_valid_point) break;
      }

      // Will return projected point and height or xyz. We really
      // prefer projected points, as then the chipper will have an
      // easier time grouping spatially points close together, as it
      // operates the first two coordinates.
      bool return_point_height = true;
      m_curr_point
	= m_csv_conv.csv_to_cartesian_or_point_height(vals, m_georef, return_point_height);

      return m_has_valid_point;
    }

    virtual Vector3 GetPoint(){
      return m_curr_point;
    }

    virtual ~CsvReader(){
      delete m_ifs;
      m_ifs = NULL;
    }

  }; // End class CsvReader

  void PcdReader::read_header() {
    // Open the file as text
    std::ifstream handle;
    handle.open(m_pcd_file.c_str());
    if (handle.fail()) {
      vw_throw( vw::IOErr() << "Unable to open file \"" << m_pcd_file << "\"" );
    }
    // Start checking all of the header elements
    bool valid = true;
    std::string line, dummy, value;
    std::getline(handle, line);     
    while (line[0] == '#') // Skip initial comment lines
      std::getline(handle, line);
    // Check the header version - we only support one kind for now.
    boost::to_lower(line);
    if (line.find("version 0.7") == std::string::npos) {
      vw_out() << "Error: Unsupported PCD file version: " << line << std::endl;
      valid = false;
    }
    // Verify the fields
    std::getline(handle, line);
    boost::to_lower(line);
    if (line.find("fields x y z") == std::string::npos) {
      vw_out() << "Error: Unsupported PCD fields: " << line << std::endl;
      valid = false;
    }
    // Get some other information, no checking here...
    handle >> dummy >> m_size_bytes;
    std::getline(handle, line); // Go to the next line
    if ((m_size_bytes != 4) && (m_size_bytes != 8)) {
      vw_out() << "Error: Unsupported byte size: " << m_size_bytes << std::endl;
      valid = false;
    }
    handle >> dummy >> m_type;
    std::getline(handle, line); // Go to the next line
    if (m_type == 'F')
      m_type = 'f';
    if (m_type != 'f') {
      vw_out() << "Error: Currently only Float type PCD files are supported!\n";
      valid = false;
    }
    
    // Get size info
    int width, height, count;
    handle >> dummy >> count;
    std::getline(handle, line); // Go to the next line
    if (count != 1) {
      vw_out() << "Error: Unsupported PCD count: " << count << std::endl;
      valid = false;
    }
    handle >> dummy >> width >> dummy >> height;
    std::getline(handle, line); // Skip viewpoint line
    std::getline(handle, line);
    handle >> dummy >> m_num_points;
    if (m_num_points != (static_cast<size_t>(width*height))) {
      vw_out() << "Error: PCD point count error!\n";
      valid = false;
    }
    // Get the type of file, ascii or binary
    handle >> dummy >> value;
    boost::to_lower(value);
    m_binary_format = (value != "ascii");
    
    if (handle.fail()) {
      vw_out() << "Error: PCD read error!\n";
      valid = false;
    }
    
    m_header_length_bytes = handle.tellg();
    
    // Stop reading the header file
    handle.close();
    if (!valid)
      vw_throw(ArgumentErr() << "Fatal error reading PCD file: " << m_pcd_file);
  }

  
  PcdReader::PcdReader(std::string const & pcd_file)
    : m_pcd_file(pcd_file), m_has_valid_point(false){

    // For now PCD files are required to be in XYZ GCC format.
    m_has_georef = false;

    read_header();      
    
    // Open the file for data reading in the proper format then skip past the header
    if (m_binary_format)
      m_ifs = new std::ifstream ( m_pcd_file.c_str(), std::ios_base::binary);
    else
      m_ifs = new std::ifstream ( m_pcd_file.c_str());

    m_ifs->seekg(m_header_length_bytes);
  }

  bool PcdReader::ReadNextPoint(){

    // Check if there is more data
    if (!m_ifs->good()) {
      m_has_valid_point = false;
      return false;
    }

    if (m_binary_format) {

      if (m_size_bytes == 4) { // -> float
        float x, y, z;
        m_ifs->read(reinterpret_cast<char*>(&x), m_size_bytes);
        m_ifs->read(reinterpret_cast<char*>(&y), m_size_bytes);
        m_ifs->read(reinterpret_cast<char*>(&z), m_size_bytes);
        m_curr_point = Vector3(x, y, z);
      }else { // 8 bytes -> double
        double x, y, z;
        m_ifs->read(reinterpret_cast<char*>(&x), m_size_bytes);
        m_ifs->read(reinterpret_cast<char*>(&y), m_size_bytes);
        m_ifs->read(reinterpret_cast<char*>(&z), m_size_bytes);
        m_curr_point = Vector3(x, y, z);          
      }
    
    } else { // Text format

      // Read in the next point
      double x, y, z;
      (*m_ifs) >> x >> y >> z;
      m_curr_point = Vector3(x, y, z);
    }
    
    // Make sure the reads succeeded
    if (m_ifs->fail()) {
      m_has_valid_point = false;
      return false;
    }
    
    return true;
  }

  Vector3 PcdReader::GetPoint(){
    return m_curr_point;
  }

  PcdReader::~PcdReader(){
    delete m_ifs;
    m_ifs = NULL;
  }



  std::int64_t pcd_file_size(std::string const& file) {
    PcdReader reader(file);
    return reader.m_num_points;
  }


  /// Create a point cloud image from a las file. The image will be
  /// created block by block, when it needs to be written to disk. It is
  /// important that the writer invoking this image be single-threaded,
  /// as we read from the las file sequentially.
  template <class ImageT>
  class LasOrCsvToTif_Class : public ImageViewBase< LasOrCsvToTif_Class<ImageT> > {

    typedef typename ImageT::pixel_type PixelT;

    asp::BaseReader * m_reader;
    int m_rows, m_cols; // These are pixel sizes, not tile counts.
    int m_block_size;

  public:

    typedef PixelT pixel_type;
    typedef PixelT result_type;
    typedef ProceduralPixelAccessor<LasOrCsvToTif_Class> pixel_accessor;

    LasOrCsvToTif_Class(asp::BaseReader * reader, int num_rows, int tile_len, int block_size):
      m_reader(reader), m_block_size(block_size){

      std::int64_t num_points = m_reader->m_num_points;
      int num_row_tiles = std::max(1, (int)ceil(double(num_rows)/tile_len));
      m_rows = tile_len*num_row_tiles;

      int points_per_row = (int)ceil(double(num_points)/m_rows);
      int num_col_tiles  = std::max(1, (int)ceil(double(points_per_row)/tile_len));
      m_cols = tile_len*num_col_tiles;
    }

    inline int32 cols  () const { return m_cols; }
    inline int32 rows  () const { return m_rows; }
    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor(*this); }

    inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {
      vw_throw( NoImplErr() << "LasOrCsvToTif_Class::operator(...) has not been implemented.\n");
      return result_type();
    }

    typedef CropView<ImageView<PixelT> > prerasterize_type;
    inline prerasterize_type prerasterize( BBox2i const& bbox ) const{

      // Read a chunk of the las file, and store it in the current tile.

      std::int64_t num_cols = bbox.width();
      std::int64_t num_rows = bbox.height();

      VW_ASSERT((num_rows % m_block_size == 0) && (num_cols % m_block_size == 0),
                ArgumentErr() << "LasOrCsvToTif_Class: Expecting the number of rows "
                              << "to be a multiple of the block size.\n");

      // Read the specified number of points from the file
      std::int64_t max_num_pts_to_read = num_cols * num_rows;
      std::int64_t count = 0;
      PointBuffer in;
      while (m_reader->ReadNextPoint()){
        in.push_back(m_reader->GetPoint());
        count++;
        if (count >= max_num_pts_to_read)
          break;
      }

      // Take the points just read, and put them in groups by spatial
      // location, so that later point2dem does not need to read every
      // input point when writing a given tile, but only certain groups.
      ImageView<Vector3> Img;
      Chipper(in, m_block_size, m_reader->m_has_georef, m_reader->m_georef,
              num_cols, num_rows, Img);

      VW_ASSERT(num_cols == Img.cols() && num_rows == Img.rows(),
                ArgumentErr() << "LasOrCsvToTif_Class: Size mis-match.\n");

      return crop( Img, -bbox.min().x(), -bbox.min().y(), cols(), rows() );

    }

    template <class DestT>
    inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
      vw::rasterize( prerasterize(bbox), dest, bbox );
    }

  }; // End class LasOrCsvToTif_Class

} // namespace asp

//------------------------------------------------------------------------------------------
// Class CsvConv functions

std::string asp::CsvConv::write_header_string(std::string const delimiter) const{
  std::stringstream s;
  std::map<int, std::string>::const_iterator it;
  for (it = this->col2name.begin(); it != this->col2name.end(); it++)
    s << it->second << delimiter;
  return s.str();
}

// This is a complete list of all supported column names, it must be kept up to date.
int asp::CsvConv::get_sorted_index_for_name(std::string const& name){
  if (name == "file"     ) return 3; // The string goes in a different location
  if (name == "lon"      ) return 0;
  if (name == "lat"      ) return 1;
  if (name == "radius_m" ) return 2;
  if (name == "radius_km") return 2;
  if (name == "x"        ) return 0;
  if (name == "y"        ) return 1;
  if (name == "z"        ) return 2;
  if (name == "pixel_x"  ) return 0;
  if (name == "pixel_y"  ) return 1;
  if (name == "pixel_val") return 2;
  if (name == "easting"  ) return 0;
  if (name == "northing" ) return 1;
  if (name == "height_above_datum") return 2;

  vw_throw( ArgumentErr() << "Unsupported column name: " << name );
}

// Parse the CSV format string and build the data structure which
// will enable to convert from CSV to Cartesian and vice-versa.
// The user specifies 3 fields that determine the coordinate.
// If min_num_fields 2, autocomplete the third value to 0.
void asp::CsvConv::parse_csv_format(std::string const& csv_format_str,
                                    std::string const& csv_proj4_str,
                                    int min_num_fields) {

  // Make sure that these custom terms do not appear in the proj4 string.
  if ((csv_proj4_str.find("D_MOON") != std::string::npos) ||
      (csv_proj4_str.find("D_MARS") != std::string::npos)) {
    vw_throw(ArgumentErr() << "D_MOON and D_MARS are not official proj4 names."
                           << "Specify the datum elsewhere or define radii manually.\n");
  }

  *this = asp::CsvConv(); // Reset this object to the default state

  this->csv_format_str = csv_format_str; // Record inputs
  this->csv_proj4_str  = csv_proj4_str;

  std::string local = csv_format_str; // Make lowercase
  boost::algorithm::to_lower(local);

  if (local == "") // No format string passed in
    return;

  boost::replace_all(local, ":", " "); // Convert to space-delimited
  boost::replace_all(local, ",", " ");
  std::istringstream is(local);

  // The case of utm: "utm:23N 1:x 2:y 3:height_above_datum". Parse
  // the initial bit to get utm_zone and utm_north, leave the rest
  // alone.
  std::string str;
  is >> str;
  if (str == "utm"){
    is >> str;
    asp::parse_utm_str(str, this->utm_zone, this->utm_north);
  }else{
    // Go back to the original string
    is.clear();
    is.str(local);
  }

  int col;
  std::string name;
  while (is.good()) {
    // Grab the next two elements
    if (! (is >> col >> name))
      vw_throw(ArgumentErr() << "Could not parse: '" << csv_format_str << "'\n");

    // Convert to zero-based indexing and error check
    col--;
    if (col < 0 || this->col2name.count(col))
      vw_throw(ArgumentErr() << "Illegal column index in: '" << csv_format_str << "'\n");
    
    // Store in the lookup maps
    this->name2col[name] = col;
    this->col2name[col]  = name;
  }
  this->num_fields = this->name2col.size();
  const int MAX_NUM_FIELDS = 4; // Location and a file
  if ((this->num_fields < min_num_fields) || (this->num_fields > MAX_NUM_FIELDS))
    vw_throw(ArgumentErr() << "Invalid number of column indices in: '" << csv_format_str << "'\n");
  if (min_num_fields < 2) 
    vw::vw_throw(vw::ArgumentErr() << "Expecting at least two fields in the csv format.\n");
  
  // Sort the names into a pre-specified order.
  std::vector<std::string> sorted_names(this->num_fields);
  for (auto it = this->name2col.begin(); it != this->name2col.end(); it++){
    int index = get_sorted_index_for_name(it->first);
    sorted_names[index] = it->first;
    if (it->first != "file") // Only the point data goes into a vector, not the filename
      this->col2sort[it->second] = index;
  }

  // If only two fields are set, auto-complete the third. Values for that field
  // will be set to 0. This makes it convenient to use many of the functions
  // expecting 3 fields.
  if (this->num_fields == 2 && sorted_names.size() == 2) {
    if (sorted_names[0] == "x" && sorted_names[1] == "y")
      sorted_names.push_back("z");
    else if (sorted_names[0] == "lon" && sorted_names[1] == "lat")
      sorted_names.push_back("height_above_datum");
    else if (sorted_names[0] == "easting"  && sorted_names[1] == "northing")
      sorted_names.push_back("height_above_datum");
    else if (sorted_names[0] == "pixel_x" && sorted_names[1] == "pixel_y")
      sorted_names.push_back("pixel_val");
    else
      sorted_names.push_back("not_found"); // will trigger a failure below
  }
  
  // From the input strings, determine which set type applies to this file.
  if (sorted_names[0] == "x" &&
      sorted_names[1] == "y" &&
      sorted_names[2] == "z") {
    this->format = XYZ;
  }else if (sorted_names[0] == "lon" &&
            sorted_names[1] == "lat" &&
            sorted_names[2] == "radius_m"){
    this->format = LAT_LON_RADIUS_M;
  }else if (sorted_names[0] == "lon" &&
            sorted_names[1] == "lat" &&
            sorted_names[2] == "radius_km"){
    this->format = LAT_LON_RADIUS_KM;
  }else if (sorted_names[0] == "lon" &&
            sorted_names[1] == "lat" &&
            sorted_names[2] == "height_above_datum"){
    this->format = HEIGHT_LAT_LON;
  }else if (sorted_names[0] == "easting"  &&
            sorted_names[1] == "northing" &&
            sorted_names[2] == "height_above_datum"){
    this->format = EASTING_HEIGHT_NORTHING;
  } else if (sorted_names[0] == "pixel_x" &&
             sorted_names[1] == "pixel_y" &&
             sorted_names[2] == "pixel_val"){
    this->format = PIXEL_XYVAL;
  }else{
    vw_throw( ArgumentErr() << "Cannot understand the csv format string: "
                            << csv_format_str << ".\n" );
  }
}

bool asp::CsvConv::parse_georef(vw::cartography::GeoReference & georef) const {
  // If the user passed in a csv file containing easting, northing, height
  // above datum, and either a utm zone or a custom proj4 string,
  // pass that info into the georeference for the purpose of converting
  // later from easting and northing to lon and lat.

  if (this->utm_zone >= 0) { // UTM case
    try{
      georef.set_UTM(this->utm_zone, this->utm_north);
      return true;
    } catch ( const std::exception& e ) {
      vw_throw(ArgumentErr() << "Detected error: " << e.what()
                             << "\nPlease check if you are using an Earth datum.\n");
    }
  } else if (this->csv_proj4_str != "") { // Not UTM, with proj4 string
    bool have_user_datum = false, have_input_georef = false;
    Datum user_datum;
    asp::set_srs_string(this->csv_proj4_str, have_user_datum, user_datum,
                        have_input_georef, georef);
    return true;
  }else{ // No UTM, no proj4 string
    if (this->format == EASTING_HEIGHT_NORTHING)
      vw_throw( ArgumentErr() << "When a CSV file has easting and northing, the PROJ.4 string must be set via --csv-proj4.\n" );
  }
  return false;
}

asp::CsvConv::CsvRecord asp::CsvConv::parse_csv_line(bool & is_first_line, bool & success,
                                                     std::string const& line) const {
  // Parse a CSV file line in given format
  success = true;

  // Copy the input line into a temporary buffer
  const int bufSize = 2048;
  char temp[bufSize];
  strncpy(temp, line.c_str(), bufSize);

  std::string sep = asp::csv_separator();

  int col_index = -1; // The current column we are reading
  int num_floats_read = 0;
  int num_values_read = 0;

  CsvRecord values;

  // Quietly ignore empty lines, lines with spaces only, and lines starting with comments
  if (line.empty() || line[0] == '#' || hasSpacesOnly(line)) {
    success = false;
    is_first_line = false;
    return values;
  }
  
  char * ptr = temp;
  while (1) {

    col_index++; // Increment the column counter
    const char* token = strtok(ptr, sep.c_str());  // Split line on seperator char
    ptr = NULL; // After the first call, strtok expects a null pointer as input.
    if (token == NULL) break; // no more tokens
    if (num_values_read >= this->num_fields) break; // read enough values

    // Check if this is one of the columns we need to read
    if (this->col2name.find(col_index) == this->col2name.end())
      continue;

    if (this->col2name.at(col_index) == "file") // This is a string input
      values.file = token;
    else {
      // Parse the floating point value from the token
      double val;
      int flag = sscanf(token, "%lg", &val);
      if (flag == 0){ // Handle parsing failure
        success = false;
        break;
      }
      values.point_data[num_floats_read] = val;
      num_floats_read++;
    }
    num_values_read++;

  } // End loop through columns

  // Check if enough values were read and for NaN values
  if (num_values_read != this->num_fields || values.point_data != values.point_data)
    success = false;

  if (!success) {
    if (!is_first_line) {
      // Not the header
      vw_out () << "Failed to read line: " << line << "\n";
    }
  }

  is_first_line = false;
  return values;
}

size_t asp::CsvConv::read_csv_file(std::string    const & file_path,
				   std::list<CsvRecord> & output_list,
                                   // Store contiguous blocks here, if needed to assemble polygons
                                   std::vector<int> * contiguous_blocks) const {

  // Clear output object
  output_list.clear();

  if (contiguous_blocks != NULL) {
    contiguous_blocks->clear();  
    contiguous_blocks->push_back(0);
  }
  
  // Open input file
  std::ifstream file( file_path.c_str() );
  if (!file)
    vw_throw(vw::IOErr() << "Unable to open file \"" << file_path << "\"");

  // Read through all the lines of the input file, parse each line, and build the output list.
  bool success;
  bool first_line = true; // TODO(oalexan1): Wipe this variable.
  std::string line = "";
  while (std::getline(file, line, '\n')) {
    CsvRecord new_record = asp::CsvConv::parse_csv_line(first_line, success, line);
    if (success) {
      output_list.push_back(new_record);
      if (contiguous_blocks != NULL)
        contiguous_blocks->back()++; // add an element
    } else {
      if (contiguous_blocks != NULL && contiguous_blocks->back() > 0) 
        contiguous_blocks->push_back(0);         // Add a new block
    }
    
    first_line = false;
  }

  if (contiguous_blocks != NULL) {
    // Wipe all blocks of length 0. Likely there is only one at the end.
    std::vector<int> & v = *contiguous_blocks; // alias
    v.erase(std::remove(v.begin(), v.end(), 0), v.end());
  }
  
  file.close();

  return output_list.size();
}


vw::Vector3 asp::CsvConv::sort_parsed_vector3(CsvRecord const& csv) const {
  Vector3 ordered_csv;
  int count = 0;
  const int NUM_POINT_PARAMS = 3;
  for (auto it = this->col2sort.begin(); it != this->col2sort.end(); it++){
    if (it->second < NUM_POINT_PARAMS) // Don't include elements past the first three
      ordered_csv[it->second] = csv.point_data[count];
    count++;
  }
  return ordered_csv;
}


vw::Vector3 asp::CsvConv::unsort_vector3(vw::Vector3 const& csv) const {
  Vector3 csv2;
  int count = 0;
  const int NUM_POINT_PARAMS = 3;
  for (std::map<int, int>::const_iterator it = this->col2sort.begin(); it != this->col2sort.end(); it++){
    if (it->second < NUM_POINT_PARAMS){ // Don't include elements past the first three
      csv2[count] = csv[it->second];
      count++;
    }
  }
  return csv2;
}

// There is a lot of repeated code for the next three functions in order to
// improve the speed of parsing points by doing the minimum number of conversions.

// Return either xyz or a projected point. Note that the flag return_point_height
// is not necessarily respected. 
Vector3 asp::CsvConv::csv_to_cartesian_or_point_height(CsvRecord const& csv,
                                                       GeoReference const& geo,
                                                       bool return_point_height) const{

  Vector3 ordered_csv = sort_parsed_vector3(csv);

  // If the format is XYZ, there is a good chance we don't even have a reference.
  // So we cannot return a point_height. We need things this way for the chipper,
  // but this is quite confusing. 
  if (this->format == XYZ) 
    return ordered_csv; // already as xyz

  // Convert from CSV to Cartesian. Later we may convert to point_height format,
  // which, due to the projection in the georeference, may not be the same
  // as the input CSV format. E.g., input CSV may be lon, lat, height,
  // but the output of this function may be easting, northing, height.
  
  Vector3 xyz;
  if (this->format == EASTING_HEIGHT_NORTHING){

  Vector3 point_height = Vector3(ordered_csv[0], ordered_csv[1], ordered_csv[2]);
  Vector2 ll  = geo.point_to_lonlat(Vector2(point_height[0], point_height[1]));
  Vector3 llh = Vector3(ll[0], ll[1], point_height[2]); // now lon, lat, height
  xyz = geo.datum().geodetic_to_cartesian(llh);
  
  }else if (this->format == HEIGHT_LAT_LON){

    xyz = geo.datum().geodetic_to_cartesian(ordered_csv);

  }else{ // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM

    if (this->format == LAT_LON_RADIUS_KM)
      ordered_csv[2] *= 1000.0; // now lon, lat, radius_m

    Vector3 tmp = ordered_csv; tmp[2] = 0; // now lon, lat, 0
    xyz = geo.datum().geodetic_to_cartesian(tmp);

    // Update the radius
    xyz = ordered_csv[2]*(xyz/norm_2(xyz));

  }
  
  if (return_point_height)
    return geo.geodetic_to_point(geo.datum().cartesian_to_geodetic(xyz));
  
  return xyz;
}


vw::Vector3 asp::CsvConv::csv_to_cartesian(CsvRecord const& csv,
                                           vw::cartography::GeoReference const& geo) const {
  Vector3 ordered_csv = sort_parsed_vector3(csv);

  Vector3 xyz;
  if (this->format == XYZ){
    return ordered_csv; // already as xyz

  }else if (this->format == EASTING_HEIGHT_NORTHING){
    Vector3 point_height = Vector3(ordered_csv[0], ordered_csv[1], ordered_csv[2]);
    Vector2 ll           = geo.point_to_lonlat(Vector2(point_height[0], point_height[1]));
    Vector3 llh          = Vector3(ll[0], ll[1], point_height[2]); // now lon, lat, height
    xyz = geo.datum().geodetic_to_cartesian(llh);

  }else if (this->format == HEIGHT_LAT_LON){
    xyz = geo.datum().geodetic_to_cartesian(ordered_csv);

  }else{ // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM
    if (this->format == LAT_LON_RADIUS_KM)
      ordered_csv[2] *= 1000.0; // now lon, lat, radius_m

    Vector3 tmp = ordered_csv; tmp[2] = 0; // now lon, lat, 0
    xyz = geo.datum().geodetic_to_cartesian(tmp);

    // Update the radius
    xyz = ordered_csv[2]*(xyz/norm_2(xyz));
  }
  return xyz;
}

// Returns Vector3(lon, lat, height_above_datum)
vw::Vector3 asp::CsvConv::csv_to_geodetic(CsvRecord const& csv,
                                          vw::cartography::GeoReference const& geo) const {
  Vector3 ordered_csv = sort_parsed_vector3(csv);
  Vector3 llh;

  if (this->format == XYZ){
    llh = geo.datum().cartesian_to_geodetic(ordered_csv);

  }else if (this->format == EASTING_HEIGHT_NORTHING){
    Vector3 point_height = Vector3(ordered_csv[0], ordered_csv[1], ordered_csv[2]);
    Vector2 ll           = geo.point_to_lonlat(Vector2(point_height[0], point_height[1]));
    llh = Vector3(ll[0], ll[1], point_height[2]); // now lon, lat, height

  }else if (this->format == HEIGHT_LAT_LON){
    return ordered_csv;

  }else{ // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM
    if (this->format == LAT_LON_RADIUS_KM)
      ordered_csv[2] *= 1000.0; // now lon, lat, radius_m

    Vector3 tmp = ordered_csv; tmp[2] = 0; // now lon, lat, 0
    Vector3 xyz = geo.datum().geodetic_to_cartesian(tmp);

    // Update the radius
    xyz = ordered_csv[2]*(xyz/norm_2(xyz));
    llh = geo.datum().cartesian_to_geodetic(xyz);
  }
  return llh;
}

vw::Vector2 asp::CsvConv::csv_to_lonlat(CsvRecord const& csv,
                                        vw::cartography::GeoReference const& geo) const {
  Vector3 ordered_csv = sort_parsed_vector3(csv);

  if (this->format == XYZ){
    Vector3 llh = geo.datum().cartesian_to_geodetic(ordered_csv);
    return Vector2(llh[0], llh[1]);
  }else if (this->format == EASTING_HEIGHT_NORTHING){
    return geo.point_to_lonlat(Vector2(ordered_csv[0], ordered_csv[1]));
  }else if (this->format == HEIGHT_LAT_LON){
    return Vector2(ordered_csv[0], ordered_csv[1]);
  }else{ // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM
    return Vector2(ordered_csv[0], ordered_csv[1]);
  }

}

Vector3 asp::CsvConv::cartesian_to_csv(Vector3 const& xyz,
                                       GeoReference const& geo,
                                       double mean_longitude) const{
  Vector3 csv;
  if (this->format == XYZ){
    csv = xyz; // order is x, y, z

  }else{ // format != XYZ, convert to the csv format.

    // Must assert here that the datum was specified.

    Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);   // lon-lat-height
    llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjust

    if (this->format == EASTING_HEIGHT_NORTHING){

      // go from lon, lat to easting, northing
      Vector2 en = geo.lonlat_to_point(Vector2(llh[0], llh[1]));
      csv = Vector3(en[0], en[1], llh[2]); // order is easting, northing, height

    }else if (this->format == HEIGHT_LAT_LON){
      csv = llh;

    }else{
      // Handle asp::LAT_LON_RADIUS_M and asp::LAT_LON_RADIUS_KM

      llh[2] = norm_2(xyz); // order is lon, lat, radius_m

      if (this->format == LAT_LON_RADIUS_KM){
        llh[2] /= 1000.0; // order is lon, lat, radius_km
      }
      csv = llh;
    }
  }
  // Now the csv vector contains the sorted values for our format

  // Now we have the csv fields, but they are in the order
  // corresponding to the sorted column names. Need to put them
  // in the same order as they were in the file originally.
  return unsort_vector3(csv);
}

// End class CsvConv functions

void asp::las_or_csv_to_tif(std::string const& in_file,
                            std::string const& out_file,
                            int num_rows, int block_size,
                            vw::GdalWriteOptions * opt,
                            vw::cartography::GeoReference const& csv_georef,
                            asp::CsvConv const& csv_conv) {

  // We will fetch a chunk of the las file of area TILE_LEN x
  // TILE_LEN, split it into bins of spatially close points, and write
  // it to disk as a tile in a vector tif image. The bigger the tile
  // size, the more likely the binning will be more efficient. But big
  // tiles use a lot of memory.

  // To do: Study performance for large files when this number changes
  const int TILE_LEN = 2048;
  Vector2 tile_size(TILE_LEN, TILE_LEN);

  vw_out() << "Writing temporary file: " << out_file << std::endl;

  // Temporarily change the raster tile size
  Vector2 original_tile_size = opt->raster_tile_size;
  opt->raster_tile_size = tile_size;

  boost::shared_ptr<asp::BaseReader> reader_ptr;
  std::ifstream ifs;
  liblas::ReaderFactory las_reader_factory;
  boost::shared_ptr<liblas::Reader> laslib_reader_ptr;

  if (asp::is_csv(in_file)){ // CSV

    reader_ptr = boost::shared_ptr<asp::CsvReader>
      (new asp::CsvReader(in_file, csv_conv, csv_georef));

  }else if (asp::is_pcd(in_file)){ // PCD

    reader_ptr = boost::shared_ptr<asp::PcdReader>( new asp::PcdReader(in_file) );

  }else if (asp::is_las(in_file)){ // LAS

    ifs.open(in_file.c_str(), std::ios::in | std::ios::binary);
    laslib_reader_ptr.reset(new liblas::Reader(las_reader_factory.CreateWithStream(ifs)));
    reader_ptr = boost::shared_ptr<asp::LasReader>(new asp::LasReader(*laslib_reader_ptr));

  }else
    vw_throw( ArgumentErr() << "Unknown file type: " << in_file << "\n");

  ImageViewRef<Vector3> Img
    = asp::LasOrCsvToTif_Class<ImageView<Vector3>>(reader_ptr.get(), num_rows,
                                                     TILE_LEN, block_size);

  // Must use a thread only, as we read the input file serially.
  vw::cartography::write_gdal_image(out_file, Img, *opt, TerminalProgressCallback("asp", "\t--> ") );

  // Restore the original tile size
  opt->raster_tile_size = original_tile_size;
}

std::int64_t asp::las_file_size(std::string const& las_file){
  std::ifstream ifs;
  ifs.open(las_file.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);
  liblas::Header const& header = reader.GetHeader();
  return header.GetPointRecordsCount();
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

/// Builds a GeoReference from the first cloud having a georeference in the list
bool asp::georef_from_pc_files(std::vector<std::string> const& files,
			       vw::cartography::GeoReference & georef){

  // Initialize
  georef = GeoReference();

  for (int i = 0; i < (int)files.size(); i++){
    GeoReference local_georef;

    // Sometimes ASP PC files can have georef, written there by stereo
    try {
      if (!is_las(files[i]) && read_georeference(local_georef, files[i])){
	georef = local_georef;
	return true;
      }
    }catch(...){}

    // Sometimes las files can have georef
    if (is_las(files[i]) && asp::georef_from_las(files[i], local_georef)){
      georef = local_georef;
      return true;
    }
  }

  return false;
}

bool asp::is_las(std::string const& file){
  std::string lfile = boost::to_lower_copy(file);
  return (boost::iends_with(lfile, ".las")  || boost::iends_with(lfile, ".laz"));
}

bool asp::is_csv(std::string const& file){
  std::string lfile = boost::to_lower_copy(file);
  return ( boost::iends_with(lfile, ".csv")  || boost::iends_with(lfile, ".txt")  );
}

bool asp::is_pcd(std::string const& file){
  std::string lfile = boost::to_lower_copy(file);
  return boost::iends_with(lfile, ".pcd");
}

bool asp::is_las_or_csv_or_pcd(std::string const& file){
  return asp::is_las(file) || is_csv(file);
}

bool asp::read_user_datum(double semi_major, double semi_minor,
                          std::string const& reference_spheroid,
                          cartography::Datum& datum ) {
  // Select a cartographic datum. There are several hard coded datums
  // that can be used here, or the user can specify their own.
  if ( reference_spheroid != "" ) {
    datum.set_well_known_datum(reference_spheroid);
  } else if (semi_major > 0 && semi_minor > 0) {
    datum = cartography::Datum("User Specified Datum",
                               "User Specified Spheroid",
                               "Reference Meridian",
                               semi_major, semi_minor, 0.0);
  } else {
    return false;
  }
  vw_out() << "\t--> Re-referencing altitude values using datum: " << datum.name() << ".\n";
  vw_out() << "\t    Axes [" << datum.semi_major_axis() << " "
           << datum.semi_minor_axis() << "] meters.\n";
  return true;
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

bool asp::hasSpacesOnly(std::string const& str) {
  bool only_spaces = true;
  for (size_t it = 0; it < str.size(); it++) {
    if (str[it] != ' ' && str[it] != '\n' && str[it] != '\t') {
      only_spaces = false;
      break;
    }
  }
  return only_spaces;
}

bool asp::is_valid_csv_line(std::string const& line) {
  // A valid line is not empty and does not start with '#' and does not have spaces only.

  bool only_spaces = hasSpacesOnly(line);
  
  return (!only_spaces) && (!line.empty()) && (line[0] != '#');
}

std::int64_t asp::csv_file_size(std::string const& file){

  std::ifstream fh(file.c_str());
  if (!fh)
    vw_throw( vw::IOErr() << "Unable to open file \"" << file << "\"" );

  std::int64_t num_total_points = 0;
  std::string line;
  while (getline(fh, line, '\n')){
    if (!asp::is_valid_csv_line(line)) continue;
    num_total_points++;
  }

  return num_total_points;
}

// Peek at the first valid line in a file to find how many columns it has
int asp::fileNumCols(std::string const& file) {

  const int bufSize = 2048;
  char buffer[bufSize];

  std::string sep = asp::csv_separator();
  
  int num = 0;
  std::ifstream fh(file.c_str());

  std::string line;
  while (getline(fh, line, '\n')){
    if (!asp::is_valid_csv_line(line)) continue;

    // Copy the input line into a buffer that can be modified
    strncpy(buffer, line.c_str(), bufSize);
    char * ptr = buffer;

    // Inspect the tokens
    while (1) {
      
      const char* token = strtok(ptr, sep.c_str());  // Split line on seperator char
      ptr = NULL; // After the first call, strtok expects a null pointer as input.

      if (token == NULL)
        break; // no more tokens
      
      // Parse the floating point value from the token
      double val;
      int flag = sscanf(token, "%lg", &val);
      if (flag == 0) // Handle parsing failure
        break;

      num++;
    }

    break; // done finding a good line
  }
  
  return num;
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

// Find the average longitude for a given point image with lon, lat, height values
double asp::find_avg_lon(ImageViewRef<Vector3> const& point_image){

  Stopwatch sw;
  sw.start();
  int32 subsample_amt = int32(norm_2(Vector2(point_image.cols(),
                                             point_image.rows()))/32.0);
  if (subsample_amt < 1 )
    subsample_amt = 1;
  PixelAccumulator<MeanAccumulator<Vector3> > mean_accum;
  for_each_pixel( subsample(point_image, subsample_amt),
                  mean_accum,
                  TerminalProgressCallback("asp","Statistics: ") );
  Vector3 avg_location = mean_accum.value();
  double avg_lon = avg_location.x() >= 0 ? 0 : 180;
  sw.stop();
  vw_out(DebugMessage,"asp") << "Statistics time: " << sw.elapsed_seconds() << std::endl;

  return avg_lon;
}


/// Analyze a file name to determine the file type
std::string asp::get_cloud_type(std::string const& file_name){

  if (asp::is_csv(file_name))
    return "CSV";
  if (asp::is_las(file_name))
    return "LAS";

  // Note that any tif, ntf, and cub file with one channel with georeference be
  // interpreted as a DEM.
  int nc = vw::get_num_channels(file_name);

  vw::cartography::GeoReference geo;
  bool has_georef = vw::cartography::read_georeference(geo, file_name);

  if (nc == 1 && has_georef)
    return "DEM";
  if (nc >= 3)
    return "PC";
  vw_throw(vw::ArgumentErr() << "File: " << file_name
                         << " is neither a point cloud nor a DEM.\n");
}


// Find the number of channels in the point clouds.
// If the point clouds have inconsistent number of channels,
// return the minimum of 3 and the minimum number of channels.
// This will be used to flag that we cannot reliable extract the
// error channels, which start at channel 4.
int asp::num_channels(std::vector<std::string> const& pc_files){

  VW_ASSERT(pc_files.size() >= 1, ArgumentErr() << "Expecting at least one file.\n");

  int num_channels0 = get_num_channels(pc_files[0]);
  int min_num_channels = num_channels0;
  for (int i = 1; i < (int)pc_files.size(); i++){
    int num_channels = get_num_channels(pc_files[i]);
    min_num_channels = std::min(min_num_channels, num_channels);
    if (num_channels != num_channels0)
      min_num_channels = std::min(min_num_channels, 3);
  }
  return min_num_channels;
}

// Get a handle to the error image given a set of point clouds with 4 or 6 bands
vw::ImageViewRef<double> asp::point_cloud_error_image
(std::vector<std::string> const& pointcloud_files) {

  ImageViewRef<double> error_image;
  int num_channels = asp::num_channels(pointcloud_files);
  
  if      (num_channels == 4) {
    error_image = asp::error_norm<4>(pointcloud_files);
  } else if (num_channels == 6) {
    error_image = asp::error_norm<6>(pointcloud_files);
  } else {
    // Return an empty image
    ImageView<double> image;
    image.set_size(0, 0);
    error_image = image;
  }

  return error_image;
}
