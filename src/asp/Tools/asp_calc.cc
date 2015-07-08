// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

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

enum OperationType
{
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

std::string getTagName(const OperationType o)
{
  switch(o)
  {
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
void tab(int indent)
{
  for (int i = 0; i < indent; ++i)
    std::cout << ' ';
}


// This type represents an operation performed on one or more inputs.
struct calc_operation
{
    OperationType               opType; // The operation to be performed on the children
    double value; // If this is a leaf node, the number is stored here.  Ignored unless OP_number.
    int varName;
    std::vector<calc_operation> inputs; // The inputs to the operation
    
    calc_operation() : opType(OP_pass) {}
    
    /// Recursive function to print out the contents of this object
    void print(const int indent=0)
    {
    
      if (opType == OP_number)
      {
        tab(indent);
        std::cout << "Node: " << getTagName(opType) << " = " << value << std::endl;      
      }
      else if (opType == OP_variable)
      {
        tab(indent);
        std::cout << "Node: " << getTagName(opType) << " = " << varName << std::endl;      
      }
      else
      { 
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
        {
          inputs[i].print(indent+TAB_SIZE);
        }

        tab(indent);
        std::cout << '}' << std::endl;
      }

    }
    
    /// Recursive function to eliminate extraneous nodes created by our parsing technique
    void clearEmptyNodes()
    {
    
      // Not a pass, recursively call this function on all inputs
      for (size_t i=0; i<inputs.size(); ++i)
        inputs[i].clearEmptyNodes();
        
    
      if (opType != OP_pass)
        return;
        
    
      // Check for errors
      if (inputs.size() != 1)
      {
        std::cout << "ERROR: pass node with " << inputs.size() << " Nodes!\n";
        return;
      }
      //if (inputs[0].opType != OP_number)
      //{
      //  std::cout << "ERROR: pass node with non-number input! " << getTagName(inputs[0].opType) << "\n";
      //  return;
      //}
      
      // Replace this node with its input node
      value   = inputs[0].value;
      opType  = inputs[0].opType;
      varName = inputs[0].varName;
      std::vector<calc_operation> temp = inputs[0].inputs;
      inputs = temp;
        
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


void print(double const& i) { std::cout << i << std::endl; }
void printYo() { std::cout << "Yo" << std::endl; }

// Helper constants to aid in accessing a calc_operation struct
const int OP  = 0;
const int NUM = 1;
const int VAR = 2;
const int IN  = 3;

template <typename ITER>
struct calc_grammar : b_s::qi::grammar<ITER, calc_operation(), b_s::ascii::space_type>
{
  // Constructor function
  calc_grammar() : calc_grammar::base_type(expression)
  {
     // Definitions
     using boost::phoenix::at;
     using boost::phoenix::at_c;
     using boost::phoenix::push_back;
     using b_s::qi::double_;
     using b_s::qi::int_;
     using b_s::qi::_val;
     using b_s::qi::_1; // This is required to avoid namespace mixups with other Boost modules!
     using b_s::qi::_2;
     using b_s::qi::_3;
   
   
// EBNF examples:   
   /*
expr   -> term [ ('+' | '-') term ]*
term   -> factor [ ('*' | '/') factor ]*
factor -> '(' expr ')' | identifier | number


expr     -> term [ ('+' | '-') term ]*
term     -> factor [ ('*' | '/') factor ]*
factor   -> base [ '^' exponent ]*
base     -> '(' expr ')' | identifier | number
exponent -> '(' expr ')' | identifier | number   

// TODO: Add support for more operations: min, max

*/
   
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
        
    
    
  // This code is for proper left priority parsing, but this approach does not work at all using
  //  Boost::Spirit!  An entirely new approach is needed but it is not worth the time figuring out.
/*
     // Middle priority
     term =         
     
           -( (term >> '*') [push_back(at_c<IN>(_val),  b_s::qi::_0), at_c<OP>(_val)=OP_multiply]// |  // Multiplication
              //(term >> '/') [push_back(at_c<IN>(_val),  b_s::qi::_0), at_c<OP>(_val)=OP_divide  ] // Division
            )
            
          >> (factor [push_back(at_c<IN>(_val), _1)]); 

    // The highest priority
    // - TODO: An additional layer to prevent double signs?
    factor = 
        //(double_          [_val = _1]      );   | // Just a number
        (double_          [at_c<NUM>(_val)=_1, at_c<OP>(_val)=OP_number]); // Just a number
        
        //('(' > expression [handler] > ')') | // Something in parenthesis
        //('-' >> factor    [handler]      ) | // Negative sign
        //('+' >> factor    [handler]      );  // Positive sign 
        
        */
    
  } // End constructor
  
  
  // Grammer rules
  b_s::qi::rule<ITER, calc_operation(), b_s::ascii::space_type> expression, term, factor;
  
  //qi::rule<Iterator, calc_node(), ascii::space_type> argList;
  
}; // End struct calc_grammer


//=================================================================================
// - Transfer functions that should be in a VW file somewhere

// Linear Transfer Function
class LinearTransFunc : public vw::UnaryReturnSameType {
public:
  LinearTransFunc() {}

  template <class ArgT>
  ArgT operator()( ArgT const& value ) const { return value; }
};


//=================================================================================




// TODO: This could be handled with a generic class + a functor.
//      --> May it already exists somewhere?
/// Image view class which creates a binary mask image equalling
///  255 in locations where all of the input images are non-zero.
/// - The output mask is equal in size to the smallest input image.
template <class ImageT>
class ImageCalcView : public ImageViewBase<ImageCalcView<ImageT> >
{
private:
  std::vector<ImageT> m_image_vec;
  int m_num_rows;
  int m_num_cols;

public:
  typedef typename ImageT::pixel_type pixel_type;
  typedef typename ImageT::pixel_type result_type;
  

  // Constructor
  ImageCalcView( std::vector<ImageT> & imageVec/*, CALC_TREE!*/) : m_image_vec(imageVec)
  {
    const size_t numImages = imageVec.size();
    VW_ASSERT( (numImages > 0), ArgumentErr() << "ImageAndView: One or more images required!." );

    // Determine the minimum image size
    m_num_rows = imageVec[0].rows();
    m_num_cols = imageVec[0].cols();
    for (size_t i=1; i<numImages; ++i)
    {
      if (imageVec[i].rows() < m_num_rows)  m_num_rows = imageVec[i].rows();
      if (imageVec[i].cols() < m_num_cols)  m_num_cols = imageVec[i].cols();
    }
  }

  inline int32 cols  () const { return m_num_cols; }
  inline int32 rows  () const { return m_num_rows; }
  inline int32 planes() const { return 1; }

  inline result_type operator()( int32 i, int32 j, int32 p=0 ) const 
  { 
    return 0; // NOT IMPLEMENTED!
  }

  typedef ProceduralPixelAccessor<ImageCalcView<ImageT> > pixel_accessor;
  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  typedef CropView<ImageView<result_type> > prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const 
  { 
    // Set up the output image tile
    ImageView<result_type> tile(bbox.width(), bbox.height());

    // Set up for pixel calculations
    size_t num_images = m_image_vec.size();

    // Loop through each output pixel and compute each output value
    for (int c = 0; c < bbox.width(); c++)
    {
      for (int r = 0; r < bbox.height(); r++)
      {
        // Check all of the pixels at this location and perform the AND operation
        result_type thisPixel;
        result_type outputPixel;
        outputPixel[0] = 255;
        for (size_t i=0; i<num_images; ++i) 
        {

          //TODO: Do stuff with the calculator tree!

        } // End image loop
        tile(c, r) = outputPixel;
      } // End row loop
    } // End column loop

  // Return the tile we created with fake borders to make it look the size of the entire output image
  return prerasterize_type(tile,
                           -bbox.min().x(), -bbox.min().y(),
                           cols(), rows() );

  } // End prerasterize function

 template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const 
 { 
   vw::rasterize( prerasterize(bbox), dest, bbox ); 
 }
  
}; // End class ImageCalcView


//======================================================================================================

struct Options {
  Options() : nodata(-1), feather_min(0), feather_max(255), filter("linear") {}
  // Input
  std::vector<std::string> input_files;

  // Settings
  double nodata;
  int feather_min, feather_max; // Currently these are always left at the defaults
  std::string filter;
  std::string output_filename;
};








// Handling input
void handle_arguments( int argc, char *argv[], Options& opt ) {
  size_t cache_size;

  po::options_description general_options("");
  general_options.add_options()
    ("nodata-value",      po::value(&opt.nodata), "Value that is nodata in the input image. Not used if input has alpha.")
    ("output-filename,o", po::value(&opt.output_filename), "Output file name.")
    ("cache",             po::value(&cache_size)->default_value(1024), "Source data cache size, in megabytes.")
    ("help,h",            "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value<std::vector<std::string> >(&opt.input_files));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (const po::error& e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <image-files>\n";
  boost::to_lower( opt.filter );

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.input_files.empty() )
    vw_throw( ArgumentErr() << "Missing input files!\n" << usage.str() << general_options );

  // Set the system cache size
  vw_settings().set_system_cache_size( cache_size*1024*1024 );

}



int main( int argc, char *argv[] ) {


  //std::string exp("120");
  std::string exp(argv[1]);

  calc_grammar<std::string::const_iterator> grammerParser;
  //calc_node calcTree;
  calc_operation calcTree;

  std::string::const_iterator iter = exp.begin();
  std::string::const_iterator end = exp.end();
  bool r = phrase_parse(iter, end, grammerParser, boost::spirit::ascii::space, calcTree);

  if (r && iter == end)
  {
    std::cout << "-------------------------\n";
    std::cout << "Parsing succeeded\n";
    std::cout << "-------------------------\n";
    calcTree.print();
    std::cout << "----------- pruned --------------\n";
    calcTree.clearEmptyNodes();
    calcTree.print();
    
    return 0;
  }
  else
  {
    std::string::const_iterator some = iter+30;
    std::string context(iter, (some>end)?end:some);
    std::cout << "-------------------------\n";
    std::cout << "Parsing failed\n";
    std::cout << "stopped at: \": " << context << "...\"\n";
    std::cout << "-------------------------\n";
    return 1;
  }

  return 0; // DEBUG ==============================



  Options opt;
  handle_arguments( argc, argv, opt );

  // Use a default output path if none provided
  const std::string firstPath = opt.input_files[0];
  vw_out() << "Loading: " << firstPath << "\n";
  size_t pt_idx = firstPath.rfind(".");
  std::string output;
  if (opt.output_filename.size() != 0)
    output = opt.output_filename;
  else {
    output = firstPath.substr(0,pt_idx)+"_union";
    output += firstPath.substr(pt_idx,firstPath.size()-pt_idx);
  }

  // Read the georef from the first file, they should all have the same value.
  vw::cartography::GeoReference georef;
  vw::cartography::read_georeference(georef, firstPath);

  // For now this is always the same
  typedef PixelGray<uint8> PixelT;
  
  const size_t numInputFiles = opt.input_files.size();
  std::vector< ImageViewRef<PixelT> > input_images(numInputFiles);

  // Loop through each input file
  for (size_t i=0; i<numInputFiles; ++i)
  {
    const std::string input = opt.input_files[i];

    // Determining the format of the input
    SrcImageResource *rsrc = DiskImageResource::open(input);
    ChannelTypeEnum channel_type = rsrc->channel_type();
    PixelFormatEnum pixel_format = rsrc->pixel_format();

    // Check for nodata value in the file
    if ( rsrc->has_nodata_read() ) {
      opt.nodata = rsrc->nodata_read();
      std::cout << "\t--> Extracted nodata value from file: " << opt.nodata << ".\n";
    }
    delete rsrc;

    DiskImageView<PixelT> this_disk_image(input);
    input_images[i] = this_disk_image;

  } // loop through input images

/*

  vw_out() << "Writing: " << output_path << std::endl;
  cartography::write_georeferenced_image(output_path, 
                                         ImageCalcView< ImageViewRef<PixelT> >(input_images, CALC),
                                         georef,
                                         TerminalProgressCallback("bigMaskMaker","Writing:"));


*/

}
