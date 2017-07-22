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


/// \file image_calc.cc
///

// Apply specified arithmetic operations to given input images and save
// the output with desired pixel type.

#include <vw/Core/FundamentalTypes.h>
#include <vw/Core/Log.h>
#include <vw/Image/Algorithms.h>
#include <vw/Image/ImageIO.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/PixelMath.h>
#include <vw/Image/Statistics.h>
#include <vw/Image/Filter.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography.h>

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>

#include <vector>

#include <boost/program_options.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
namespace po = boost::program_options;

using namespace vw;

/**
  Program implementing simple calculator functionality for large images.

*/

namespace b_s = boost::spirit;

//================================================================================
// - The operations tree structure

/*
  Boost::Spirit requires the use of some specific Boost types in order to
   store the parsed information.
*/

enum OperationType {
  OP_pass,
  // UNARY operations
  OP_number,   // This is a leaf node containing a number.
  OP_variable,
  OP_negate,
  OP_abs,
  // BINARY operations
  OP_add,
  OP_subtract,
  OP_divide,
  OP_multiply,
  OP_power,
  // MULTI operations
  OP_min,
  OP_max
};

std::string getTagName(const OperationType o) {

  switch(o) {

    case OP_pass:     return "PASS";
    case OP_number:   return "NUMBER";
    case OP_variable: return "VARIABLE";
    case OP_negate:   return "NEGATE";
    case OP_abs:      return "ABS";
    case OP_add:      return "ADD";
    case OP_subtract: return "SUBTRACT";
    case OP_divide:   return "DIVIDE";
    case OP_multiply: return "MULTIPLY";
    case OP_power:    return "POWER";
    case OP_min:      return "MIN";
    case OP_max:      return "MAX";
    default:          return "ERROR";
  }
}

const int TAB_SIZE = 4;
void tab(int indent) {

  for (int i = 0; i < indent; ++i)
    std::cout << ' ';
}

// TODO: Are there pixel functions for these?
template <typename T>
T manual_min(const std::vector<T> &vec) {

  T minVal = vec[0];
  for (size_t i=1; i<vec.size(); ++i)
    if (vec[i] < minVal)
      minVal = vec[i];
  return minVal;
}
template <typename T>
T manual_max(const std::vector<T> &vec) {

  T maxVal = vec[0];
  for (size_t i=1; i<vec.size(); ++i)
    if (vec[i] > maxVal)
      maxVal = vec[i];
  return maxVal;
}


// This type represents an operation performed on one or more inputs.
struct calc_operation {

    OperationType               opType; // The operation to be performed on the children
    double value; // If this is a leaf node, the number is stored here.  Ignored unless OP_number.
    int varName;
    std::vector<calc_operation> inputs; // The inputs to the operation

    calc_operation() : opType(OP_pass) {}

    /// Recursive function to print out the contents of this object
    void print(const int indent=0) {

      if (opType == OP_number) {
        tab(indent);
        std::cout << "Node: " << getTagName(opType) << " = " << value << std::endl;
      }
      else if (opType == OP_variable) {
        tab(indent);
        std::cout << "Node: " << getTagName(opType) << " = " << varName << std::endl;
      }
      else {
        std::cout << std::endl;
        tab(indent);
        std::cout << "tag: " << getTagName(opType) << std::endl;
        tab(indent);
        std::cout << "value: " << value << std::endl;
        tab(indent);
        std::cout << "varName: " << varName << std::endl;
        tab(indent);
        std::cout << '{' << std::endl;

        for (size_t i=0; i<inputs.size(); ++i)
          inputs[i].print(indent+TAB_SIZE);

        tab(indent);
        std::cout << '}' << std::endl;
      }

    }

    /// Recursive function to eliminate extraneous nodes created by our parsing technique
    void clearEmptyNodes() {
      // Recursively call this function on all inputs
      for (size_t i=0; i<inputs.size(); ++i)
        inputs[i].clearEmptyNodes();

      if (opType != OP_pass)
        return;

      // Check for errors
      if (inputs.size() != 1) {
        std::cout << "ERROR: pass node with " << inputs.size() << " Nodes!\n";
        return;
      }

      // Replace this node with its input node
      value   = inputs[0].value;
      opType  = inputs[0].opType;
      varName = inputs[0].varName;
      std::vector<calc_operation> temp = inputs[0].inputs;
      inputs = temp;
    }


    /// Apply the operation tree to the input parameters and return a result
    template <typename T>
    T applyOperation(const std::vector<T> &params) const {
      // Get the results from each input node.
      // - This is a recursive call.
      const size_t numInputs = inputs.size();
      std::vector<T> inputResults(numInputs);
      for (size_t i=0; i<numInputs; ++i)
        inputResults[i] = inputs[i].applyOperation(params);

      // Now perform the operation for this node
      switch(opType) {
        // Unary
        case OP_number:   return T(value);
        case OP_variable: if (varName >= static_cast<int>(params.size()))
                            vw_throw(ArgumentErr() << "Unrecognized variable input!\n");
                          return params[varName];
        if (numInputs < 1)
          vw_throw(LogicErr() << "Insufficient inputs for this operation!\n");
        case OP_negate:   return T(-1 * inputResults[0]);
        case OP_abs:      return T(std::abs(inputResults[0])); // regular abs casts to integer!
        // Binary
        if (numInputs < 2)
          vw_throw(LogicErr() << "Insufficient inputs for this operation!\n");
        case OP_add:      return (inputResults[0] + inputResults[1]);
        case OP_subtract: return (inputResults[0] - inputResults[1]);
        case OP_divide:   return (inputResults[0] / inputResults[1]);
        case OP_multiply: return (inputResults[0] * inputResults[1]);
        case OP_power:    return (pow(inputResults[0], inputResults[1]));
        // Multi
        case OP_min:      return manual_min(inputResults); // TODO: Do these functions exist?
        case OP_max:      return manual_max(inputResults);

        default:
          vw_throw(LogicErr() << "Unexpected operation type!\n");
      }
    }
};


// We need to tell fusion about our calc_operation struct
// to make it a first-class fusion citizen
BOOST_FUSION_ADAPT_STRUCT(
    calc_operation,
    (OperationType, opType)
    (double, value)
    (int, varName)
    (std::vector<calc_operation>, inputs)
)

//================================================================================
// - Boost::Spirit equation parsing

// Helper constants to aid in accessing a calc_operation struct
const int OP  = 0;
const int NUM = 1;
const int VAR = 2;
const int IN  = 3;

template <typename ITER>
struct calc_grammar : b_s::qi::grammar<ITER, calc_operation(), b_s::ascii::space_type> {
  // Constructor function
  calc_grammar() : calc_grammar::base_type(expression) {
     // Definitions
     using boost::phoenix::at;
     using boost::phoenix::at_c;
     using boost::phoenix::push_back;
     using b_s::qi::double_;
     using b_s::qi::int_;
     using b_s::qi::eps;
     using b_s::qi::_val;
     using b_s::qi::_1; // This is required to avoid namespace mixups with other Boost modules!
     using b_s::qi::_2;
     using b_s::qi::_3;


     // This approach works but it processes expressions right to left!
     // - To get what you want, use parenthesis!

     // An outer expression
     expression =
        (term  [push_back(at_c<IN>(_val), _1)] )
         >> *(  ('+' >> expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_add     ]) |  // Addition
                ('-' >> expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_subtract])    // Subtraction
             );

     // Middle priority
     term =
         (factor [push_back(at_c<IN>(_val), _1)])
         >> *( ('*' >> term [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_multiply] ) |  // Multiplication
               ('/' >> term [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_divide  ] )    // Subtraction
             );

    // The highest priority
    // - TODO: An additional layer to prevent double signs?
    factor =
        (double_          [at_c<NUM>(_val)=_1,            at_c<OP>(_val)=OP_number]    ) | // Just a number
        // The min and max operations take a comma seperated list of expressions
        ("min(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_min] % ',' > ')') |
        ("max(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_max] % ',' > ')') |
        ( ("pow(" > expression > ',' > expression > ')')
                [push_back(at_c<IN>(_val), _1), push_back(at_c<IN>(_val), _2), at_c<OP>(_val)=OP_power] ) |
        ("abs(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_abs] > ')') | // Absolute value
        ('(' > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_pass] > ')') | // Something in parenthesis
        ('-' >> factor    [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_negate]    ) | // Negative sign
        //('+' >> factor    [handler]      );  // Positive sign
        ("var_" > int_ [at_c<VAR>(_val)=_1, at_c<OP>(_val)=OP_variable] ) ;
        //(b_s::ascii::string  [at_c<VAR>(_val)=_1,  at_c<OP>(_val)=OP_variable]    ) ; // A variable name

    /*

  // This code is for proper left priority parsing, but this approach does not work at all using
  //  Boost::Spirit!  An entirely new approach is needed but it is not worth the time figuring out.


    // This split-based approach will make the parser visit the parameters in the correct order,
    // but the current method of each term call returning an OP node won't work.  There will have
    // to be more of a central authority that keeps adding operations as it encounters them.

    term = (factor [push_back(at_c<IN>(_val), _1)])
            >> (termP [push_back(at_c<IN>(_val), _1)] )
    termP =

           ( ('*' >> factor >> termP ) [push_back(at_c<IN>(_val),  b_s::qi::_0), at_c<OP>(_val)=OP_multiply]
           )
          | eps;


    // The highest priority
    factor =
        (double_          [at_c<NUM>(_val)=_1, at_c<OP>(_val)=OP_number]); // Just a number


    */
  } // End constructor


  // Grammer rules
  b_s::qi::rule<ITER, calc_operation(), b_s::ascii::space_type> expression, term, factor;

}; // End struct calc_grammer


//=================================================================================

/// List of possible output data types
enum DataType {
  DT_UINT8   = 0,
  DT_UINT16  = 1,
  DT_UINT32  = 2,
  //DT_INT8    = 3, // Not supported by GDAL!
  DT_INT16   = 3,
  DT_INT32   = 4,
  DT_FLOAT32 = 5,
  DT_FLOAT64 = 6
};

/// Returns the default nodata value for each data type (just the minimum value)
double get_default_nodata(const DataType d) {
  switch(d) {
  case    DT_UINT8  : return std::numeric_limits<vw::uint8>::min();
  case    DT_INT16  : return std::numeric_limits<vw::int16>::min();
  case    DT_UINT16 : return std::numeric_limits<vw::uint16>::min();
  case    DT_INT32  : return std::numeric_limits<vw::int32>::min();
  case    DT_UINT32 : return std::numeric_limits<vw::uint32>::min();
  case    DT_FLOAT32: return -std::numeric_limits<vw::float32>::max();
  default :           return -std::numeric_limits<vw::float64>::max();
  };
}

/// Converts a double to another numeric type with min/max value clamping.
/// - TODO: Replace this with the correct VW function!
template <typename T>
T clamp_and_cast(const double val) {
  const T minVal = std::numeric_limits<T>::min();
  const T maxVal = std::numeric_limits<T>::max();
  if (val < static_cast<double>(minVal)) return (minVal);
  if (val > static_cast<double>(maxVal)) return (maxVal);
  return static_cast<T>(val);
}

// Specializations for floating point values.
template <typename T>
T clamp_and_cast_float(const double val) {
  const T minVal = -std::numeric_limits<T>::max();
  const T maxVal = std::numeric_limits<T>::max();
  if (val < static_cast<double>(minVal)) return (minVal);
  if (val > static_cast<double>(maxVal)) return (maxVal);
  return static_cast<T>(val);
}

template <>
vw::float32 clamp_and_cast<vw::float32>(const double val) {
  return clamp_and_cast_float<vw::float32>(val);
}

template <>
vw::float64 clamp_and_cast<vw::float64>(const double val) {
  return clamp_and_cast_float<vw::float64>(val);
}

/// Image view class which applies the calc_operation tree to each pixel location.
template <class ImageT, typename OutputPixelT>
class ImageCalcView : public ImageViewBase<ImageCalcView<ImageT, OutputPixelT> > {

public: // Definitions
  typedef typename ImageT::pixel_type  input_pixel_type;
  typedef OutputPixelT pixel_type;  // This is what controls the type of image that is written to disk.
  typedef OutputPixelT result_type;

private: // Variables
  std::vector<ImageT    > m_image_vec;
  std::vector<bool      > m_has_nodata_vec;
  std::vector<input_pixel_type> m_nodata_vec;
  result_type    m_output_nodata;
  calc_operation m_operation_tree;
  int m_num_rows;
  int m_num_cols;
  int m_num_channels;

public: // Functions

  // Constructor
  ImageCalcView( std::vector<ImageT>                & imageVec,
                 std::vector<bool  >           const& has_nodata_vec,
                 std::vector<input_pixel_type> const& nodata_vec,
                 result_type outputNodata,
                 calc_operation const& operation_tree)
                  : m_image_vec(imageVec),   m_has_nodata_vec(has_nodata_vec),
                    m_nodata_vec(nodata_vec), m_output_nodata(outputNodata),
                    m_operation_tree(operation_tree) {
    const size_t numImages = imageVec.size();
    VW_ASSERT( (numImages > 0), ArgumentErr() << "ImageCalcView: One or more images required!." );
    VW_ASSERT( (has_nodata_vec.size() == numImages), LogicErr() << "ImageCalcView: Incorrect hasNodata count passed in!." );
    VW_ASSERT( (nodata_vec.size()    == numImages), LogicErr() << "ImageCalcView: Incorrect nodata count passed in!." );

    // Make sure all images are the same size
    m_num_rows     = imageVec[0].rows();
    m_num_cols     = imageVec[0].cols();
    m_num_channels = imageVec[0].planes();
    for (size_t i=1; i<numImages; ++i) {
      if ( (imageVec[i].rows()   != m_num_rows    ) ||
           (imageVec[i].cols()   != m_num_cols    ) ||
           (imageVec[i].planes() != m_num_channels)   )
        vw_throw(ArgumentErr() << "Error: Input images must all have the same size and number of channels!");
    }
  }

  inline int32 cols  () const { return m_num_cols; }
  inline int32 rows  () const { return m_num_rows; }
  inline int32 planes() const { return m_num_channels; }

  inline result_type operator()( int32 i, int32 j, int32 p=0 ) const
  {
    return 0; // NOT IMPLEMENTED!
  }

  typedef ProceduralPixelAccessor<ImageCalcView<ImageT, OutputPixelT> > pixel_accessor;
  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  typedef CropView<ImageView<result_type> > prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    //typedef typename PixelChannelType<typename input_pixel_type>::type output_channel_type; // TODO: Why does this not compile?
    typedef typename ImageChannelType<ImageView<result_type> >::type output_channel_type;

    // Set up the output image tile
    ImageView<result_type> tile(bbox.width(), bbox.height());

    // TODO: Left to right processing!

    // Set up for pixel calculations
    const size_t num_images = m_image_vec.size();
    std::vector<input_pixel_type> input_pixels(num_images);
    std::vector<double>           input_doubles(num_images);

    // Rasterize all the input images at this particular tile
    std::vector<ImageView<input_pixel_type> > input_tiles(num_images);
    for (size_t i=0; i<num_images; ++i)
      input_tiles[i] = crop(m_image_vec[i], bbox);

    // Loop through each output pixel and compute each output value
    for (int c = 0; c < bbox.width(); c++) {
      for (int r = 0; r < bbox.height(); r++) {
        // Fetch all the input pixels for this location
        bool isNodata = false;
        for (size_t i=0; i<num_images; ++i) {
          input_pixels[i] = (input_tiles[i])(c,r);

          // If any of the input pixels are nodata, the output is nodata.
          if (m_has_nodata_vec[i] && (m_nodata_vec[i] == input_pixels[i])) {
            isNodata = true;
            break;
          }
        } // End image loop

        if (isNodata) { // Output is nodata, move on to the next pixel
          tile(c, r) = m_output_nodata;
          continue;
        }


        for (int chan=0; chan<m_num_channels; ++chan) {
          for (size_t i=0; i<num_images; ++i) {
            input_doubles[i] = input_pixels[i][chan];
          } // End image loop

          // Apply the operation tree to this pixel and store in the output pixel
          double newVal = m_operation_tree.applyOperation<double>(input_doubles);
          //if (newVal > 255)
          //  std::cout << "Value: " << newVal << " --> " << static_cast<double>(clamp_and_cast<output_channel_type>(newVal)) << "\n";
          tile(c, r, chan) = clamp_and_cast<output_channel_type>(newVal);

        } // End channel loop

      } // End row loop
    } // End column loop

  // Return the tile we created with fake borders to make it look the size of the entire output image
  return prerasterize_type(tile,
                           -bbox.min().x(), -bbox.min().y(),
                           cols(), rows() );

  } // End prerasterize function

 template <class DestT>
 inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
   vw::rasterize( prerasterize(bbox), dest, bbox );
 }

}; // End class ImageCalcView


//======================================================================================================

struct Options : vw::cartography::GdalWriteOptions {
  Options() : out_nodata_value(-1) {}
  // Input
  std::vector<std::string> input_files;
  bool        has_in_nodata;
  double      in_nodata_value;

  // Settings
  std::string output_data_string;
  DataType    output_data_type;
  bool        has_out_nodata;
  double      out_nodata_value;
  std::string calc_string;
  std::string output_file;
};


// Handling input
void handle_arguments( int argc, char *argv[], Options& opt ) {

  const std::string calc_string_help =
    "The operation to be performed on the input images.  Input images must all be the same size and type.\n"
    "Currently only single channel images are supported.\n"
    "Recognized operators: +, -, /, *, (), pow(), abs(), min(), max(), var_0, var_1, ...\n"
    "Use var_n to refer to the pixel of the n'th input image."
    "Order of operations is parsed with RIGHT priority, use parenthesis to assure the order you want.\n"
    "Surround the entire string with double quotes.\n";

  const std::string data_type_string =
    "The data type of the output file:\n"
    "  uint8   \n"
    "  uint16  \n"
    "  uint32  \n"
    "  int16   \n"
    "  int32   \n"
    "  float32 \n"
    "  float64 (default)\n";

  po::options_description general_options("");
  general_options.add_options()
    ("output-file,o", po::value(&opt.output_file), "Output file name.")
    ("calc,c",            po::value(&opt.calc_string), calc_string_help.c_str())
    ("output-data-type,d",  po::value(&opt.output_data_string)->default_value("float64"), data_type_string.c_str())
    ("input-nodata-value",  po::value(&opt.in_nodata_value), "Value that is no-data in the input images.")
    ("output-nodata-value", po::value(&opt.out_nodata_value), "Value to use for no-data in the output image.")
    ("help,h",            "Display this help message");
    
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );


  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value<std::vector<std::string> >(&opt.input_files));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <image-files>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
			     positional, positional_desc, usage,
			     allow_unregistered, unregistered );

  if ( opt.input_files.empty() )
    vw_throw( ArgumentErr() << "Missing input files!\n" << usage << general_options );
  if ( opt.calc_string.size() == 0)
    vw_throw( ArgumentErr() << "Missing operation string!\n" << usage << general_options );

  if      (opt.output_data_string == "uint8"  ) opt.output_data_type = DT_UINT8;
  else if (opt.output_data_string == "uint16" ) opt.output_data_type = DT_UINT16;
  else if (opt.output_data_string == "uint32" ) opt.output_data_type = DT_UINT32;
  else if (opt.output_data_string == "int16"  ) opt.output_data_type = DT_INT16;
  else if (opt.output_data_string == "int32"  ) opt.output_data_type = DT_INT32;
  else if (opt.output_data_string == "float32") opt.output_data_type = DT_FLOAT32;
  else if (opt.output_data_string == "float64") opt.output_data_type = DT_FLOAT64;
  else
    vw_throw(ArgumentErr() << "Unsupported output data type: '" << opt.output_data_string << "'.\n" );

 // Fill out opt.has_in_nodata and opt.has_out_nodata depending if the user specified these options
 if (!vm.count("input-nodata-value")){
    opt.has_in_nodata   = false;
  }else
    opt.has_in_nodata = true;

 if (!vm.count("output-nodata-value")){
    opt.has_out_nodata   = false;
    opt.out_nodata_value = get_default_nodata(opt.output_data_type);
  }else
    opt.has_out_nodata = true;

 vw::create_out_dir(opt.output_file);
}

/// This function call is just to clean up the case statement in load_inputs_and_process
template <typename PixelT, typename OutputT>
void generate_output(const std::string                         & output_file,
                     const Options                             & opt,
                     const calc_operation                      & calc_tree,
                     const bool                                  have_georef,
                     const vw::cartography::GeoReference       & georef,
                           std::vector< ImageViewRef<PixelT> > & input_images,
                     const std::vector<bool  >                 & has_nodata_vec,
                     const std::vector<PixelT>                 & nodata_vec ) {
  vw_out() << "Writing: " << output_file << std::endl;
  vw::cartography::block_write_gdal_image( output_file,
                                ImageCalcView< ImageViewRef<PixelT>, OutputT >(input_images,
                                                                     has_nodata_vec,
                                                                     nodata_vec,
                                                                     opt.out_nodata_value,
                                                                     calc_tree),
                               have_georef, georef,
                               opt.has_out_nodata, opt.out_nodata_value,
                               opt,
                               TerminalProgressCallback("image_calc","Writing:"));
}



/// This function loads the input images and calls the main processing function
template <typename PixelT>
void load_inputs_and_process(Options &opt, const std::string &output_file, const calc_operation &calc_tree) {

  // Read the georef from the first file, they should all have the same value.
  const size_t numInputFiles = opt.input_files.size();
  std::vector< ImageViewRef<PixelT> > input_images(numInputFiles);
  std::vector<bool  >                 has_nodata_vec(numInputFiles);
  std::vector<PixelT>                 nodata_vec(numInputFiles);

  bool have_georef = false;
  vw::cartography::GeoReference georef;

  // Loop through each input file
  for (size_t i=0; i<numInputFiles; ++i) {
    const std::string input = opt.input_files[i];

    if (!have_georef) {
      have_georef = vw::cartography::read_georeference(georef, input);
      if (have_georef)
        vw_out() << "\t--> Copying georef from input image " << input << std::endl;
    }

    // Determining the format of the input
    boost::shared_ptr<vw::DiskImageResource> rsrc(vw::DiskImageResourcePtr(input));
    //ChannelTypeEnum channel_type = rsrc->channel_type();
    //PixelFormatEnum pixel_format = rsrc->pixel_format();

    // Check for nodata value in the file
    if ( rsrc->has_nodata_read() ) {
      nodata_vec[i] = rsrc->nodata_read();
      std::cout << "\t--> Extracted nodata value from file: " << nodata_vec[i] << ".\n";
      has_nodata_vec[i] = true;
      opt.has_out_nodata = true; // If any inputs have nodata, the output must have nodata!
    }
    else { // File does not specify nodata
      if (opt.has_in_nodata) { // User has specified a default nodata value
        has_nodata_vec[i]  = true;
        nodata_vec   [i]  = PixelT(opt.in_nodata_value);
      }
      else // Don't use nodata for this input
        has_nodata_vec[i] = false;
    }

    DiskImageView<PixelT> this_disk_image(input);
    input_images[i] = this_disk_image;

  } // loop through input images

  if (opt.has_out_nodata)
    vw_out() << "\t--> Writing output nodata value " << opt.out_nodata_value << std::endl;

  // Write out the selected data type
  //vw_out() << "Writing: " << output_file << " with data type " << opt.output_data_type << std::endl;
  switch(opt.output_data_type) {
    case    DT_UINT8  : generate_output<PixelT, PixelGray<vw::uint8  > >(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case    DT_INT16  : generate_output<PixelT, PixelGray<vw::int16  > >(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case    DT_UINT16 : generate_output<PixelT, PixelGray<vw::uint16 > >(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case    DT_INT32  : generate_output<PixelT, PixelGray<vw::int32  > >(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case    DT_UINT32 : generate_output<PixelT, PixelGray<vw::uint32 > >(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case    DT_FLOAT32: generate_output<PixelT, PixelGray<vw::float32> >(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    default :           generate_output<PixelT, PixelGray<vw::float64> >(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
  };

}

/// The main function calls the cmd line parsers and figures out the input image type
int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    std::string exp(opt.calc_string);

    calc_grammar<std::string::const_iterator> grammerParser;
    calc_operation calc_tree;

    std::string::const_iterator iter = exp.begin();
    std::string::const_iterator end = exp.end();
    bool r = phrase_parse(iter, end, grammerParser, boost::spirit::ascii::space, calc_tree);

    if (r && iter == end) {// Successfully parsed the calculation expression
      //std::cout << "-------------------------\n";
      //std::cout << "Parsing succeeded\n";
      //std::cout << "-------------------------\n";
      ////calc_tree.print();
      ////std::cout << "----------- pruned --------------\n";
      calc_tree.clearEmptyNodes();
      //calc_tree.print();
    }
    else { // Failed to parse the calculation expression
      std::string::const_iterator some = iter+30;
      std::string context(iter, (some>end)?end:some);
      std::cout << "-------------------------\n";
      std::cout << "Parsing calculation expression failed\n";
      std::cout << "stopped at: \": " << context << "...\"\n";
      std::cout << "-------------------------\n";
      return -1;
    }

    // Use a default output file if none provided
    const std::string firstFile = opt.input_files[0];
    //vw_out() << "Loading: " << firstFile << "\n";
    size_t pt_idx = firstFile.rfind(".");
    std::string output_file;
    if (opt.output_file.size() != 0)
      output_file = opt.output_file;
    else {
      output_file = firstFile.substr(0,pt_idx)+"_calc";
      output_file += firstFile.substr(pt_idx,firstFile.size()-pt_idx);
    }

    // Determining the format of the input images (all are assumed to be the same type!)
    boost::shared_ptr<vw::DiskImageResource> rsrc(vw::DiskImageResourcePtr(firstFile));
    ChannelTypeEnum input_data_type = rsrc->channel_type();
    //PixelFormatEnum pixel_format = rsrc->pixel_format();

    // Redirect to another function with the correct template type
    switch(input_data_type) {
    case VW_CHANNEL_INT8   : load_inputs_and_process<PixelGray<vw::int8   > >(opt, output_file, calc_tree);  break;
    case VW_CHANNEL_UINT8  : load_inputs_and_process<PixelGray<vw::uint8  > >(opt, output_file, calc_tree);  break;
    case VW_CHANNEL_INT16  : load_inputs_and_process<PixelGray<vw::int16  > >(opt, output_file, calc_tree);  break;
    case VW_CHANNEL_UINT16 : load_inputs_and_process<PixelGray<vw::uint16 > >(opt, output_file, calc_tree);  break;
    case VW_CHANNEL_INT32  : load_inputs_and_process<PixelGray<vw::int32  > >(opt, output_file, calc_tree);  break;
    case VW_CHANNEL_UINT32 : load_inputs_and_process<PixelGray<vw::uint32 > >(opt, output_file, calc_tree);  break;
    case VW_CHANNEL_FLOAT32: load_inputs_and_process<PixelGray<vw::float32> >(opt, output_file, calc_tree);  break;
    case VW_CHANNEL_FLOAT64: load_inputs_and_process<PixelGray<vw::float64> >(opt, output_file, calc_tree);  break;
    default : vw_throw(ArgumentErr() << "Input image format " << input_data_type << " is not supported!\n");
    };
  } ASP_STANDARD_CATCHES;

}
