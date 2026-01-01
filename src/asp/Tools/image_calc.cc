// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

// Apply specified arithmetic operations to given input images and save the
// output with desired pixel type.

#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspStringUtils.h>

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
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/FileIO/FileUtils.h>

#include <boost/program_options.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/math/special_functions/sign.hpp>

#include <vector>
#include <random>

namespace po = boost::program_options;
namespace b_s = boost::spirit;

// The operations tree structure. Boost::Spirit requires the use of some
// specific Boost types in order to store the parsed information.
enum OperationType {
  OP_pass,
  // UNARY operations
  OP_number,   // This is a leaf node containing a number.
  OP_variable,
  OP_negate,
  OP_abs,
  OP_sign,
  OP_rand,
  // BINARY operations
  OP_add,
  OP_subtract,
  OP_divide,
  OP_multiply,
  OP_power,
  // MULTI operations
  OP_min,
  OP_max,
  OP_lt,
  OP_gt,
  OP_lte,
  OP_gte,
  OP_eq
};

std::string getTagName(const OperationType op) {

  switch(op) {

    case OP_pass:     return "PASS";
    case OP_number:   return "NUMBER";
    case OP_variable: return "VARIABLE";
    case OP_negate:   return "NEGATE";
    case OP_abs:      return "ABS";
    case OP_sign:     return "SIGN";
    case OP_rand:     return "RAND";
    case OP_add:      return "ADD";
    case OP_subtract: return "SUBTRACT";
    case OP_divide:   return "DIVIDE";
    case OP_multiply: return "MULTIPLY";
    case OP_power:    return "POWER";
    case OP_min:      return "MIN";
    case OP_max:      return "MAX";
    case OP_lt:       return "LESS_THAN";
    case OP_gt:       return "GREATER_THAN";
    case OP_lte:      return "LESS_THAN_EQ";
    case OP_gte:      return "LESS_THAN_EQ";
    case OP_eq:       return "EQUALS";
    default:          return "ERROR";
  }
}

const int TAB_SIZE = 4;
void tab(int indent) {

  for (int i = 0; i < indent; i++)
    vw::vw_out() << ' ';
}

// TODO: Are there pixel functions for these?
template <typename T>
T manual_min(std::vector<T> const& vec) {

  T minVal = vec[0];
  for (size_t i = 1; i < vec.size(); i++)
    if (vec[i] < minVal)
      minVal = vec[i];
  return minVal;
}
template <typename T>
T manual_max(std::vector<T> const& vec) {

  T maxVal = vec[0];
  for (size_t i = 1; i < vec.size(); i++)
    if (vec[i] > maxVal)
      maxVal = vec[i];
  return maxVal;
}

// Initialize the random number generator
std::mt19937 mt(0);

// Return a random number in the range [0, 1]
template <typename T>
T custom_rand_0_1(T val) {
  return double(mt() - mt.min())/double(mt.max() - mt.min());
}

// This type represents an operation performed on one or more inputs.
struct calc_operation {

  OperationType opType; // The operation to be performed on the children
  double        value; // If this is a leaf node, the number is stored here.  Ignored unless OP_number.
  int           varName;
  std::vector<calc_operation> inputs; // The inputs to the operation

  calc_operation(): opType(OP_pass) {}

  /// Recursive function to print out the contents of this object
  void print(const int indent=0) {

    if (opType == OP_number) {
      tab(indent);
      vw::vw_out() << "Node: " << getTagName(opType) << " = " << value << "\n";
    } else if (opType == OP_variable) {
      tab(indent);
      vw::vw_out() << "Node: " << getTagName(opType) << " = " << varName << "\n";
    } else {
      vw::vw_out() << "\n";
      tab(indent);
      vw::vw_out() << "tag: " << getTagName(opType) << "\n";
      tab(indent);
      vw::vw_out() << "value: " << value << "\n";
      tab(indent);
      vw::vw_out() << "varName: " << varName << "\n";
      tab(indent);
      vw::vw_out() << '{' << "\n";

      for (size_t i = 0; i < inputs.size(); i++)
        inputs[i].print(indent+TAB_SIZE);

      tab(indent);
      vw::vw_out() << '}' << "\n";
    }

  }

  /// Recursive function to eliminate extraneous nodes created by our parsing technique
  void clearEmptyNodes() {
    // Recursively call this function on all inputs
    for (size_t i = 0; i < inputs.size(); i++)
      inputs[i].clearEmptyNodes();

    if (opType != OP_pass)
      return;

    // Check for errors
    if (inputs.size() != 1) {
      vw::vw_out() << "ERROR: pass node with " << inputs.size() << " Nodes!\n";
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
  T applyOperation(std::vector<T> const& params) const {
    // Get the results from each input node.
    // - This is a recursive call.
    size_t numInputs = inputs.size();
    std::vector<T> inputResults(numInputs);
    for (size_t i = 0; i < numInputs; i++)
      inputResults[i] = inputs[i].applyOperation(params);

    // Now perform the operation for this node
    switch(opType) {
      // Unary
      case OP_number:   return T(value);
      case OP_variable: if (varName >= static_cast<int>(params.size()))
        vw::vw_throw(vw::ArgumentErr()
                  << "Unrecognized variable input. Note that the first variable is var_0.\n");
        return params[varName];
      if (numInputs < 1)
        vw::vw_throw(vw::LogicErr() << "Insufficient inputs for this operation.\n");
      case OP_negate:   return T(-1 * inputResults[0]);
      case OP_abs:      return T(std::abs(inputResults[0])); // regular abs casts to integer.
      case OP_sign:     return T(boost::math::sign(inputResults[0]));
      case OP_rand:     return T(custom_rand_0_1(inputResults[0]));

      // Binary
      if (numInputs < 2)
        vw::vw_throw(vw::LogicErr() << "Insufficient inputs for this operation.\n");
      case OP_add:      return (inputResults[0] + inputResults[1]);
      case OP_subtract: return (inputResults[0] - inputResults[1]);
      case OP_divide:   return (inputResults[0] / inputResults[1]);
      case OP_multiply: return (inputResults[0] * inputResults[1]);
      case OP_power:    return (pow(inputResults[0], inputResults[1]));

      // Multi
      case OP_min:      return manual_min(inputResults); // TODO: Do these functions exist?
      case OP_max:      return manual_max(inputResults);

      case OP_lt:   return (inputResults[0] <  inputResults[1]) ? inputResults[2]: inputResults[3];
      case OP_gt:   return (inputResults[0] >  inputResults[1]) ? inputResults[2]: inputResults[3];
      case OP_lte:  return (inputResults[0] <= inputResults[1]) ? inputResults[2]: inputResults[3];
      case OP_gte:  return (inputResults[0] >= inputResults[1]) ? inputResults[2]: inputResults[3];
      case OP_eq:   return (inputResults[0] == inputResults[1]) ? inputResults[2]: inputResults[3];

      default:
        vw::vw_throw(vw::LogicErr() << "Unexpected operation type.\n");
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

// - Boost::Spirit equation parsing

// Helper constants to aid in accessing a calc_operation struct
const int OP  = 0;
const int NUM = 1;
const int VAR = 2;
const int IN  = 3;

template <typename ITER>
struct calc_grammar: b_s::qi::grammar<ITER, calc_operation(), b_s::ascii::space_type> {
  // Constructor function
  calc_grammar(): calc_grammar::base_type(expression) {
    // Definitions
    using boost::phoenix::at;
    using boost::phoenix::at_c;
    using boost::phoenix::push_back;
    using b_s::qi::double_;
    using b_s::qi::int_;
    using b_s::qi::eps;
    using b_s::qi::_val;
    using b_s::qi::_1; // This is required to avoid namespace mixups with other Boost modules.
    using b_s::qi::_2;
    using b_s::qi::_3;

    // This approach works but it processes expressions right to left. To get a
    // desired order, use parenthesis.

    // An outer expression
    expression =
      (term  [push_back(at_c<IN>(_val), _1)])
        >> *((
    // Addition
    '+' >> expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_add]) |  
    // Subtraction         
    ('-' >> expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_subtract]));    

    // Middle priority
    term =
        (factor [push_back(at_c<IN>(_val), _1)])
        >> *((
    // Multiplication
    '*' >> term [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_multiply]) |  
    // Division
    ('/' >> term [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_divide  ]));    

    // The highest priority
    // TODO: An additional layer to prevent double signs?
    factor =
      // Just a number
      (double_ [at_c<NUM>(_val)=_1,            at_c<OP>(_val)=OP_number]) | 
      // These operations take a comma separated list of expressions
      ("min(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_min] % ',' > ')') |
      ("max(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_max] % ',' > ')') |
      ("lt("  > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_lt ] % ',' > ')') |
      ("gt("  > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_gt ] % ',' > ')') |
      ("lte(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_lte] % ',' > ')') |
      ("gte(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_gte] % ',' > ')') |
      ("eq("  > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_eq ] % ',' > ')') |
      (("pow(" > expression > ',' > expression > ')')
        [push_back(at_c<IN>(_val), _1), 
         push_back(at_c<IN>(_val), _2), at_c<OP>(_val)= OP_power]) |
      // Absolute value
      ("abs(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_abs] > ')') | 
      // Sign function
      ("sign(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_sign] > ')') | 
      // rand function
      ("rand(" > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_rand] > ')') | 
      // Something in parenthesis
      ('(' > expression [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_pass] > ')') | 
      // Negative sign
      ('-' >> factor    [push_back(at_c<IN>(_val), _1), at_c<OP>(_val)=OP_negate]) | 
      ("var_" > int_ [at_c<VAR>(_val)=_1, at_c<OP>(_val)=OP_variable]) ;

  } // End constructor

  // Grammar rules
  b_s::qi::rule<ITER, calc_operation(), b_s::ascii::space_type> expression, term, factor;

}; // End struct calc_grammar

/// Image view class which applies the calc_operation tree to each pixel location.
template <class ImageT, typename OutputPixelT>
class ImageCalcView: public vw::ImageViewBase<ImageCalcView<ImageT, OutputPixelT>> {

public: // Definitions
  typedef typename ImageT::pixel_type  input_pixel_type;
  typedef OutputPixelT pixel_type;  // This is what controls the type of image that is written to disk.
  typedef OutputPixelT result_type;

private: // Variables
  std::vector<ImageT> const& m_image_vec;
  std::vector<bool> m_has_nodata_vec;
  std::vector<double> m_nodata_vec; // nodata is always double
  double              m_output_nodata;
  calc_operation m_operation_tree;
  int m_num_rows;
  int m_num_cols;
  int m_num_channels;

public: // Functions

  // Constructor
  ImageCalcView(std::vector<ImageT> const& imageVec,
                std::vector<bool>   const& has_nodata_vec,
                std::vector<double> const& nodata_vec,
                double outputNodata,
                calc_operation const& operation_tree):
    m_image_vec(imageVec),   m_has_nodata_vec(has_nodata_vec),
    m_nodata_vec(nodata_vec), m_output_nodata(outputNodata),
    m_operation_tree(operation_tree) {
    size_t numImages = imageVec.size();
    VW_ASSERT((numImages > 0), vw::ArgumentErr()
              << "ImageCalcView: One or more images required.");
    VW_ASSERT((has_nodata_vec.size() == numImages),
              vw::LogicErr() << "ImageCalcView: Incorrect hasNodata count passed in.");
    VW_ASSERT((nodata_vec.size() == numImages),
              vw::LogicErr() << "ImageCalcView: Incorrect nodata count passed in.");

    // Make sure all images are the same size
    m_num_rows     = imageVec[0].rows();
    m_num_cols     = imageVec[0].cols();
    m_num_channels = imageVec[0].planes();
    for (size_t i = 1; i < numImages; i++) {
      if ((imageVec[i].rows()   != m_num_rows) ||
           (imageVec[i].cols()   != m_num_cols) ||
           (imageVec[i].planes() != m_num_channels))
        vw::vw_throw(vw::ArgumentErr()
                 << "Error: Input images must all have the same size and number of channels.");
    }
  }

  inline vw::int32 cols  () const { return m_num_cols; }
  inline vw::int32 rows  () const { return m_num_rows; }
  inline vw::int32 planes() const { return m_num_channels; }

  inline result_type operator()(vw::int32 i, vw::int32 j, vw::int32 p=0) const {
    return 0; // NOT IMPLEMENTED.
  }

  typedef vw::ProceduralPixelAccessor<ImageCalcView<ImageT, OutputPixelT>> pixel_accessor;
  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  typedef vw::CropView<vw::ImageView<result_type>> prerasterize_type;
  inline prerasterize_type prerasterize(vw::BBox2i const& bbox) const {
    typedef typename vw::ImageChannelType<vw::ImageView<result_type>>::type output_channel_type;

    // Set up the output image tile
    vw::ImageView<result_type> tile(bbox.width(), bbox.height());

    // TODO: Left to right processing.

    // Set up for pixel calculations
    size_t num_images = m_image_vec.size();
    std::vector<input_pixel_type> input_pixels(num_images);
    std::vector<double>           input_doubles(num_images);

    // Rasterize all the input images at this particular tile
    std::vector<vw::ImageView<input_pixel_type>> input_tiles(num_images);
    for (size_t i = 0; i < num_images; i++)
      input_tiles[i] = crop(m_image_vec[i], bbox);

    // Loop through each output pixel and compute each output value
    for (int c = 0; c < bbox.width(); c++) {
      for (int r = 0; r < bbox.height(); r++) {
        // Fetch all the input pixels for this location
        bool isNodata = false;
        for (size_t i = 0; i < num_images; i++) {
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

        for (int chan = 0; chan < m_num_channels; chan++) {
          for (size_t i = 0; i < num_images; i++) {
            input_doubles[i] = input_pixels[i][chan];
          } // End image loop

          // Apply the operation tree to this pixel and store in the output pixel
          // TODO(oalexan1): Should we round too, if output is int?
          double newVal = m_operation_tree.applyOperation<double>(input_doubles);
          tile(c, r, chan) = vw::clamp_and_cast<output_channel_type>(newVal);

        } // End channel loop

      } // End row loop
    } // End column loop

  // Return the tile we created with fake borders to make it look the
  // size of the entire output image
  return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(), cols(), rows());

  } // End prerasterize function

 template <class DestT>
 inline void rasterize(DestT const& dest, vw::BBox2i const& bbox) const {
   vw::rasterize(prerasterize(bbox), dest, bbox);
 }

}; // End class ImageCalcView

struct Options: vw::GdalWriteOptions {
  Options(): out_nodata_value(-1), percentile_stretch(false), 
    percentile_range(vw::Vector2(2, 98)) {}
  // Input
  std::vector<std::string> input_files;
  bool        has_in_nodata;
  double      in_nodata_value, lon_offset;

  // Settings
  std::string output_data_string;
  int         output_data_type;
  bool        has_out_nodata;
  bool        no_georef;
  bool        percentile_stretch;
  vw::Vector2 percentile_range;
  double      out_nodata_value;
  std::string calc_string;
  std::string output_file, metadata;
};

// Handling input
void handle_arguments(int argc, char * argv[], Options & opt) {

  const std::string calc_string_help =
    "The operation to be performed on the input images. Input images must all be the "
    "same size and type. Currently only single channel images are supported. "
    "Recognized operators: +, -, /, *, (), pow(), abs(), sign(), rand(), min(), max(), "
    "var_0, var_1, etc. Use var_n to refer to the pixel of the n-th input image "
    "(n starts from 0). Note that the order of operations is parsed with right-to-left "
    "associativity, so, 'a * b * c' becomes 'a * (b * c)'. Use parentheses to "
    "enforce the desired order. Surround the entire string with quotes. For a "
    "single input image, if the operation is not set, it defaults to 'var_0' so the "
    "identity operation.";

  const std::string data_type_string =
    "The data type of the output file:\n"
    "  uint8   \n"
    "  uint16  \n"
    "  uint32  \n"
    "  int16   \n"
    "  int32   \n"
    "  float32 (default)\n"
    "  float64 \n";

  double nan = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("");
  general_options.add_options()
    ("output-file,o", po::value(&opt.output_file), "Output file name.")
    ("calc,c", po::value(&opt.calc_string), calc_string_help.c_str())
    ("output-data-type,d",
     po::value(&opt.output_data_string)->default_value("float32"), data_type_string.c_str())
    ("input-nodata-value",  po::value(&opt.in_nodata_value), "Set the nodata value for the input images, overriding the value in the images, if present.")
    ("output-nodata-value", po::value(&opt.out_nodata_value),
     "Manually specify a nodata value for the output image. By default it is read from the "
     "first input which has it, or, if missing, it is set to data type min.")
    ("mo",  po::value(&opt.metadata)->default_value(""),
     "Write metadata to the output file. Provide as a string in quotes if more than one "
     "item, separated by a space, such as 'VAR1=VALUE1 VAR2=VALUE2'. Neither the variable "
     "names nor the values should contain spaces.")
    ("no-georef", po::bool_switch(&opt.no_georef)->default_value(false),
     "Remove any georeference information (useful with subsequent GDAL-based processing).")
    ("stretch", po::bool_switch(&opt.percentile_stretch)->default_value(false),
     "Linearly stretch, round, and clamp the input values to the 0 - 255 range (uint8) "
     "based on the specified percentiles. See --percentile-range.")
    ("percentile-range", 
     po::value(&opt.percentile_range)->default_value(vw::Vector2(2, 98), "2 98"),
     "The percentiles to use for stretching the image to 8-bit. These are double values.")
    ("longitude-offset",  po::value(&opt.lon_offset)->default_value(nan),
     "Add this value to the longitudes in the geoheader (can be used to offset the "
     "longitudes by 360 degrees).")
    ("help,h", "Display this help message.");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value<std::vector<std::string>>(&opt.input_files));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("[options] <image-files>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.input_files.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing input files.\n" << usage << general_options);

  if (opt.percentile_stretch && opt.input_files.size() != 1)
    vw::vw_throw(vw::ArgumentErr() 
             << "The --stretch option works only with a single input image.\n");

  if (opt.percentile_range[0] < 0.0 || opt.percentile_range[0] >= opt.percentile_range[1] || 
      opt.percentile_range[1] > 100.0)
    vw::vw_throw(vw::ArgumentErr() << "The --percentile-range values must be between 0 and 100, "
             << "and the first value must be less than the second.\n");

  if (opt.calc_string.empty()) {
    if (opt.input_files.size() == 1) {
      opt.calc_string = "var_0";
    } else {
      vw::vw_throw(vw::ArgumentErr() << "Missing operation string.\n" << usage << general_options);
    }
  }

  if (opt.output_data_string == "uint8") opt.output_data_type = vw::VW_CHANNEL_UINT8;
  else if (opt.output_data_string == "uint16") opt.output_data_type = vw::VW_CHANNEL_UINT16;
  else if (opt.output_data_string == "uint32") opt.output_data_type = vw::VW_CHANNEL_UINT32;
  // GDAL does not support int8
  //else if (opt.output_data_string == "int8") opt.output_data_type  = VW_CHANNEL_INT8;
  else if (opt.output_data_string == "int16") opt.output_data_type = vw::VW_CHANNEL_INT16;
  else if (opt.output_data_string == "int32") opt.output_data_type = vw::VW_CHANNEL_INT32;
  else if (opt.output_data_string == "float32") opt.output_data_type = vw::VW_CHANNEL_FLOAT32;
  else if (opt.output_data_string == "float64") opt.output_data_type = vw::VW_CHANNEL_FLOAT64;
  else
    vw::vw_throw(vw::ArgumentErr()
             << "Unsupported output data type: " << opt.output_data_string << ".\n");

  // Fill out opt.has_in_nodata and opt.has_out_nodata depending if the user
  // specified these options
  if (!vm.count("input-nodata-value")) {
    opt.has_in_nodata = false;
  } else
    opt.has_in_nodata = true;

  if (!vm.count("output-nodata-value")) {
    opt.has_out_nodata   = false;
    opt.out_nodata_value = vw::get_default_nodata(opt.output_data_type);
  } else
    opt.has_out_nodata = true;

  if (opt.output_file.empty())
    vw::vw_throw(vw::ArgumentErr() << "The output file was not specified.\n");

  vw::create_out_dir(opt.output_file);
}

// Implement option --stretch
void image_calc_stretch(Options const& opt, bool have_georef,
                        vw::cartography::GeoReference const& georef,
                        std::map<std::string, std::string> const& keywords,
                        bool has_nodata, double nodata_val) {
  
  // Check that the input image has only one channel
  const std::string firstFile = opt.input_files[0];
  auto rsrc = vw::DiskImageResourcePtr(firstFile);
  if (rsrc->channels() != 1)
    vw::vw_throw(vw::ArgumentErr() 
             << "The --stretch option works only with single-channel images.\n");

  // Read the image
  vw::DiskImageView<double> image(firstFile);

  // Handle nodata. Mask it before doing stats.
  vw::ImageViewRef<vw::PixelMask<double>> masked_image;
  if (has_nodata)
    masked_image = vw::create_mask(image, nodata_val);
  else
    masked_image = vw::pixel_cast<vw::PixelMask<double>>(image);

  // Compute statistics at a reduced resolution. Use double and int64 to avoid overflow.
  const double numPixelSamples = 1000000;
  double num_pixels = double(image.cols()) * double(image.rows());
  vw::int64 stat_scale = vw::int64(ceil(sqrt(num_pixels / numPixelSamples)));
  vw::math::CDFAccumulator<double> accumulator;
  vw::int64 num_valid_pixels = 0;
  vw::ImageView<vw::PixelMask<double>> sub_image = vw::subsample(masked_image, stat_scale);
  vw::vw_out() << "Computing image percentiles with subsampled image size: " 
    << sub_image.cols() << " x " << sub_image.rows() << "\n";
  vw::TerminalProgressCallback tp("asp", ": ");
  for (vw::int64 col = 0; col < sub_image.cols(); col++) {
    for (vw::int64 row = 0; row < sub_image.rows(); row++) {
      if (vw::is_valid(sub_image(col, row))) {
        accumulator(sub_image(col, row).child());
        num_valid_pixels++;
      }
    }
    tp.report_fractional_progress(col, sub_image.cols());
  }
  tp.report_finished();

  double min_val = 0.0, max_val = 1.0;
  if (num_valid_pixels == 0) {
    vw::vw_out(vw::WarningMessage) << "No valid pixels found in the input image.\n";
  } else {
    min_val = accumulator.quantile(opt.percentile_range[0] / 100.0);
    max_val = accumulator.quantile(opt.percentile_range[1] / 100.0);
  }
  vw::vw_out() << "  " << opt.percentile_range[0] << "%: " << min_val << "\n"
               << "  " << opt.percentile_range[1] << "%: " << max_val << "\n";

  // Create the stretched view. Pixels below min_val and nodata are clamped to
  // 0. Pixels above max_val are clamped to 255. Round, then cast to uint8.
  vw::ImageViewRef<double> normalized_image
    = vw::normalize(vw::apply_mask(masked_image, min_val), min_val, max_val, 0.0, 255.0);
  vw::ImageViewRef<vw::uint8> cast_image 
    = vw::channel_cast_round_and_clamp<vw::uint8>(normalized_image);

  bool has_out_nodata = false;
  double out_nodata = 0.0;
  auto tpc = vw::TerminalProgressCallback("asp", ": ");
  vw::vw_out() << "Writing: " << opt.output_file << "\n";
  vw::cartography::block_write_gdal_image(opt.output_file, cast_image, have_georef, georef,
                                          has_out_nodata, out_nodata, opt, tpc, 
                                          keywords);
}

/// This function call is just to clean up the case statement in proc_img
template <typename PixelT, typename OutputT>
void write_out(std::string const& output_file,
               Options const& opt,
               calc_operation const& calc_tree,
               bool have_georef,
               vw::cartography::GeoReference const& georef,
               std::vector<vw::ImageViewRef<PixelT>> const& input_images,
               std::vector<bool> const& has_nodata_vec,
               std::vector<double> const& nodata_vec) {

  // Read previous keywords and append any new keywords from --mo. Overwrite
  // any previous value of a keyword.
  std::map<std::string, std::string> keywords;
  if (opt.metadata != "") {
    if (!opt.input_files.empty()) {
      auto rsrc = vw::DiskImageResourcePtr(opt.input_files[0]);
      vw::cartography::read_header_strings(*rsrc.get(), keywords);
    }
    asp::parse_append_metadata(opt.metadata, keywords);
  }

  if (opt.percentile_stretch) {
    image_calc_stretch(opt, have_georef, georef, keywords, 
                       has_nodata_vec[0], nodata_vec[0]);
  } else {
    if (opt.has_out_nodata)
      vw::vw_out() << "\t--> Writing output nodata value: " << opt.out_nodata_value << "\n";
    vw::vw_out() << "Writing: " << output_file << "\n";
    auto tpc = vw::TerminalProgressCallback("image_calc", "Writing:");
    vw::cartography::block_write_gdal_image
      (output_file,
      ImageCalcView<vw::ImageViewRef<PixelT>, OutputT>
        (input_images, has_nodata_vec, nodata_vec, opt.out_nodata_value, calc_tree),
      have_georef, georef, opt.has_out_nodata, opt.out_nodata_value, opt, tpc, keywords);
  }
}

void handleGeoref(Options const& opt, bool & have_georef, 
                  vw::cartography::GeoReference & georef) {
  have_georef = false;
  size_t numInputFiles = opt.input_files.size();
  for (size_t i = 0; i < numInputFiles; i++) {
    std::string input = opt.input_files[i];

    // If desired to not use a georef, skip reading it. Else read it.
    if (!opt.no_georef) {
      if (!have_georef) {
        have_georef = vw::cartography::read_georeference(georef, input);
        if (have_georef) {
          vw::vw_out() << "\t--> Copying georef from input image " << input << "\n";

          if (!std::isnan(opt.lon_offset)) {
            if (!georef.is_projected()) {
              vw::vw_out() << "\t--> Adding " << opt.lon_offset
                           << " to the longitude bounds in the geoheader.\n";
              vw::Matrix<double,3,3> T = georef.transform();
              T(0, 2) += opt.lon_offset;
              georef.set_transform(T);
            } else {
              vw::vw_throw(vw::ArgumentErr() 
                << "Can apply a longitude offset only to georeferenced "
                << "images in the longitude-latitude projection.\n");
            }
          }
        }
      }
    }
  }
}

template <typename PixelT>
void loadImagesNodata(Options & opt, 
                      std::vector<vw::ImageViewRef<PixelT>> & input_images,
                      std::vector<bool> & has_nodata_vec,
                      std::vector<double> & nodata_vec) {

  size_t numInputFiles = input_images.size();
  for (size_t i = 0; i < numInputFiles; i++) {
    std::string input = opt.input_files[i];

    // Determining the format of the input
    auto rsrc = vw::DiskImageResourcePtr(input);

    // Check for nodata value in the file
    if (opt.has_in_nodata) {
      // User has specified a default nodata value, override its value in the file
      has_nodata_vec[i]  = true;
      nodata_vec[i]      = opt.in_nodata_value;
      opt.has_out_nodata = true; // If any inputs have nodata, the output must have nodata.
      vw::vw_out() << "\t--> Using as nodata value: " << nodata_vec[i] << "\n";
    } else if (rsrc->has_nodata_read()) {
      nodata_vec[i] = rsrc->nodata_read();
      has_nodata_vec[i] = true;

      if (!opt.has_out_nodata) {
        opt.has_out_nodata   = true; // If any inputs have nodata, the output must have nodata
        opt.out_nodata_value = nodata_vec[i]; // If not set by the user, use this on output
      }
      vw::vw_out() << "\t--> Extracted nodata value from " << input << ": "
                << nodata_vec[i] << "\n";
    } else {
      vw::vw_out() << "\t--> No nodata value present in: " << input << "\n";
      has_nodata_vec[i] = false;
    }

    vw::DiskImageView<PixelT> this_disk_image(input);
    input_images[i] = this_disk_image;
  }
}

/// This function loads the input images and calls the main processing function
template <typename PixelT>
void proc_img(Options & opt, std::string const& output_file,
              calc_operation const& calc_tree) {

  // Read the georef from the first file, they should all have the same value.
  size_t numInputFiles = opt.input_files.size();
  std::vector<vw::ImageViewRef<PixelT>> input_images(numInputFiles);
  std::vector<bool> has_nodata_vec(numInputFiles);
  std::vector<double> nodata_vec(numInputFiles);

  bool have_georef = false;
  vw::cartography::GeoReference georef;
  handleGeoref(opt, have_georef, georef);

  // Load images and nodata values
  loadImagesNodata(opt, input_images, has_nodata_vec, nodata_vec);

  // Write out the selected data type
  switch(opt.output_data_type) {
    case vw::VW_CHANNEL_UINT8: 
      write_out<PixelT, vw::PixelGray<vw::uint8>>(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case vw::VW_CHANNEL_INT16: 
      write_out<PixelT, vw::PixelGray<vw::int16>>(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case vw::VW_CHANNEL_UINT16: 
      write_out<PixelT, vw::PixelGray<vw::uint16>>(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case vw::VW_CHANNEL_INT32: 
      write_out<PixelT, vw::PixelGray<vw::int32>>(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case vw::VW_CHANNEL_UINT32: 
      write_out<PixelT, vw::PixelGray<vw::uint32>>(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
    case vw::VW_CHANNEL_FLOAT32: 
      write_out<PixelT, vw::PixelGray<vw::float32>>(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
  default: 
    write_out<PixelT, vw::PixelGray<vw::float64>>(output_file, opt, calc_tree, have_georef, georef, input_images, has_nodata_vec, nodata_vec); break;
  };

}

void image_calc(Options & opt) {
  std::string exp(opt.calc_string);

  calc_grammar<std::string::const_iterator> grammarParser;
  calc_operation calc_tree;

  std::string::const_iterator iter = exp.begin();
  std::string::const_iterator end = exp.end();
  bool r = phrase_parse(iter, end, grammarParser, boost::spirit::ascii::space, calc_tree);

  if (r && iter == end) {// Successfully parsed the calculation expression
    calc_tree.clearEmptyNodes();
  } else { // Failed to parse the calculation expression
    std::string::const_iterator some = iter+30;
    std::string context(iter, (some>end)?end:some);
    vw::vw_out() << "Parsing calculation expression failed\n";
    vw::vw_out() << "stopped at: \": " << context << "...\"\n";
    vw::vw_throw(vw::ArgumentErr() << "Parsing calculation expression failed\n");
  }

  // Use a default output file if none provided
  const std::string firstFile = opt.input_files[0];
  std::string output_file = opt.output_file;

  // Determining the format of the input images
  auto rsrc = vw::DiskImageResourcePtr(firstFile);
  vw::ChannelTypeEnum input_data_type = rsrc->channel_type();

  // Assume that all inputs are of the same type. ASP does strange things if 
  // loading a uint8 file as a float, for example.
  // TODO(oalexan1): Load each file according to its format, then cast to double.
  // TODO(oalexan1): Do not rescale the pixels on input.
  for (size_t it = 1; it < opt.input_files.size(); it++) {
    auto curr_rsrc = vw::DiskImageResourcePtr(opt.input_files[it]);
    vw::ChannelTypeEnum curr_data_type = curr_rsrc->channel_type();
    if (input_data_type != curr_data_type)
      vw::vw_throw(vw::ArgumentErr() << "All input images are supposed to be of the same data type.\n");
  }

  // Redirect to another function with the correct template type
  switch(input_data_type) {
    // GDAL does not support int8
    //case vw::VW_CHANNEL_INT8: proc_img<vw::PixelGray<vw::int8 >>(opt, output_file, calc_tree);  break;
  case vw::VW_CHANNEL_UINT8: 
    proc_img<vw::PixelGray<vw::uint8>>(opt, output_file, calc_tree);  break;
  case vw::VW_CHANNEL_INT16: 
    proc_img<vw::PixelGray<vw::int16>>(opt, output_file, calc_tree);  break;
  case vw::VW_CHANNEL_UINT16: 
    proc_img<vw::PixelGray<vw::uint16>>(opt, output_file, calc_tree);  break;
  case vw::VW_CHANNEL_INT32: 
    proc_img<vw::PixelGray<vw::int32>>(opt, output_file, calc_tree);  break;
  case vw::VW_CHANNEL_UINT32: 
    proc_img<vw::PixelGray<vw::uint32>>(opt, output_file, calc_tree);  break;
  case vw::VW_CHANNEL_FLOAT32: 
    proc_img<vw::PixelGray<vw::float32>>(opt, output_file, calc_tree);  break;
  case vw::VW_CHANNEL_FLOAT64: 
    proc_img<vw::PixelGray<vw::float64>>(opt, output_file, calc_tree);  break;
  default: 
    vw::vw_throw(vw::ArgumentErr() 
      << "Input image format " << input_data_type << " is not supported.\n");
  };
}

/// The main function calls the cmd line parsers and figures out the input image type
int main(int argc, char * argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);
    image_calc(opt);

  } ASP_STANDARD_CATCHES;

  return 0;
}
