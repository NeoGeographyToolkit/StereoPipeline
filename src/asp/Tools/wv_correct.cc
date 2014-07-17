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


/// \file wv_correct.cc
///

// Correct CCD artifacts in WorldView 1 and 2 images with given TDI.

// The problem: A WV image is obtained by mosaicking from left to
// right image blocks which are as tall is the entire image (each
// block comes from an individual CCD image sensor). Blocks are
// slightly misplaced in respect to each other by some unknown
// subpixel offset. The goal of this tool is to locate these CCD
// artifact boundary locations, and undo the offsets.
// 

// Observations:
// 1. The CCD artifact locations are periodic, but the starting offset
//    is not positioned at exactly one period. It is less by one fixed
//    value which we name 'shift'.  
// 2. There are CCD offsets in both x and y at each location.
// 3. The magnitude of all CCD offsets in x is the same, but their sign
//    alternates. The same is true in y.
// 4. The period of CCD offsets is inversely proportional to the detector
//    pitch.
// 5. The CCD offsets are pretty consistent among all images of given
//    scan direction (forward or reverse).
// We used all these and a lot of images to heuristically find the
// period and offset of the artifacts, and the 'shift' value. We
// correct these below. We allow the user to override the value of CCD
// offsets if desired.

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/DG/XML.h>
#include <asp/Sessions/RPC/RPCModel.h>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>

namespace po = boost::program_options;
using namespace vw;
using namespace asp;
using namespace vw::cartography;
using namespace xercesc;
using namespace std;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  typedef Vector<float64,6> Vector6;
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector3f>  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector6>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

struct Options : asp::BaseOptions {
  std::string camera_image_file, camera_model_file, output_image; 
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  
  po::options_description general_options("");
  general_options.add( asp::BaseOptionsDescription(opt) );
  
  po::options_description positional("");
  positional.add_options()
    ("camera-image", po::value(&opt.camera_image_file))
    ("camera-model", po::value(&opt.camera_model_file))
    ("output-image", po::value(&opt.output_image));
  
  po::positional_options_description positional_desc;
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);
  positional_desc.add("output-image",1);
  
  std::string usage("[options] <camera-image> <camera-model> <output-image>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( !vm.count("camera-image") || !vm.count("camera-model") || !vm.count("output-image") )
    vw_throw( ArgumentErr() << "Requires <camera-image>, <camera-model> and <output-image> "
              << "in order to proceed.\n\n"
              << usage << general_options );

  asp::create_out_dir(opt.output_image);
  
}

void arr_to_vec(double arr[], int len, vector<double> & vec){
  vec.clear();
  for (int i = 0; i < len; i++) vec.push_back(arr[i]);
}

void get_offsets(int tdi, bool is_wv01, bool is_forward,
                 std::vector<double> & posx, std::vector<double> & ccdx,
                 std::vector<double> & posy, std::vector<double> & ccdy
                 ){

  // Here we tabulate all ccds offsets and their column pixel positions.
  if (!is_wv01){
  }else{
    if (is_forward){
      if (tdi == 16){
        double posx_arr[] = {6.2900000000000000e+02,1.3570000000000000e+03,2.0190000000000000e+03,2.7690000000000000e+03,3.4720000000000000e+03,4.1790000000000000e+03,4.8670000000000000e+03,5.5680000000000000e+03,6.2730000000000000e+03,6.9730000000000000e+03,7.6250000000000000e+03,8.3790000000000000e+03,9.0910000000000000e+03,9.7920000000000000e+03,1.0499000000000000e+04,1.1204000000000000e+04,1.1911000000000000e+04,1.2620000000000000e+04,1.3332000000000000e+04,1.4040000000000000e+04,1.4750000000000000e+04,1.5459000000000000e+04,1.6170000000000000e+04,1.6879000000000000e+04,1.7591000000000000e+04,1.8300000000000000e+04,1.9008000000000000e+04,1.9720000000000000e+04,2.0426000000000000e+04,2.1141000000000000e+04,2.1848000000000000e+04,2.2559000000000000e+04,2.3271000000000000e+04,2.3977000000000000e+04,2.4682000000000000e+04,2.5419000000000000e+04,2.6095000000000000e+04,2.6803000000000000e+04,2.7446000000000000e+04,2.8302000000000000e+04,2.8908000000000000e+04,2.9608000000000000e+04,3.0313000000000000e+04,3.1009000000000000e+04,3.1705000000000000e+04,3.2406000000000000e+04,3.3100000000000000e+04,3.3792000000000000e+04,3.4483000000000000e+04};
        arr_to_vec(posx_arr, sizeof(posx_arr)/sizeof(double), posx);
        
        double ccdx_arr[] = {6.7322551801454480e-02,5.5941279131803634e-02,-6.3757124794674186e-02,-6.8350222655965132e-02,1.7691659140427704e-01,-6.5546000611674385e-02,6.3907733085781446e-02,-9.4966625926268172e-02,9.4161850926099605e-02,-1.0542259530055144e-01,-8.1558061694499709e-02,-6.3956300595019416e-02,4.1564490770397769e-02,-6.0005436499402910e-02,2.7687811096602116e-01,-6.7650615566507560e-02,8.9531829438824359e-02,-7.5787800987529685e-02,6.0918450645685199e-02,-4.9435252430723518e-02,7.8217062959282352e-02,-9.4972312086564092e-02,6.8771948144368461e-02,-5.5916563953937647e-02,2.5576397858317801e-01,-6.2953016311800947e-02,4.5986274236245760e-02,-5.2151312380337433e-02,3.5461672386768096e-02,-7.8840968976700190e-02,5.8279888571948685e-02,-5.6318493405969880e-02,-2.3130788594668883e-02,-2.5727892445735229e-02,6.7618551931374907e-02,-3.2635798235672329e-02,-6.7421171442540700e-02,8.6289010551310996e-02,-4.2940282592809562e-02,-6.7487297331515511e-02,-1.6954569447987194e-01,1.5768945188280239e-01,-1.3197137444641416e-01,2.2044168881490209e-01,-1.7686345973293444e-01,2.8571073037673689e-01,-2.5204743971722710e-01,2.9477986650443111e-01,-3.1357641656875035e-01};
        arr_to_vec(ccdx_arr, sizeof(ccdx_arr)/sizeof(double), ccdx);
        
        double posy_arr[] ={5.5700000000000000e+02,1.3840000000000000e+03,2.0790000000000000e+03,2.7720000000000000e+03,4.9010000000000000e+03,5.6020000000000000e+03,6.8740000000000000e+03,7.6810000000000000e+03,8.3770000000000000e+03,9.0860000000000000e+03,9.7910000000000000e+03,1.0500000000000000e+04,1.1206000000000000e+04,1.1914000000000000e+04,1.2623000000000000e+04,1.3331000000000000e+04,1.4041000000000000e+04,1.4749000000000000e+04,1.5459000000000000e+04,1.6170000000000000e+04,1.6881000000000000e+04,1.7590000000000000e+04,1.8301000000000000e+04,1.9011000000000000e+04,1.9722000000000000e+04,2.0431000000000000e+04,2.1140000000000000e+04,2.1851000000000000e+04,2.2558000000000000e+04,2.3267000000000000e+04,2.3976000000000000e+04,2.4682000000000000e+04,2.5388000000000000e+04,2.6094000000000000e+04,2.6800000000000000e+04,2.7503000000000000e+04,2.8206000000000000e+04,2.8909000000000000e+04,2.9611000000000000e+04,3.0309000000000000e+04,3.1009000000000000e+04,3.1707000000000000e+04,3.2401000000000000e+04,3.3098000000000000e+04,3.3791000000000000e+04,3.4482000000000000e+04};
        arr_to_vec(posy_arr, sizeof(posy_arr)/sizeof(double), posy);
        
        double ccdy_arr[] = {5.2874163366164850e-02,1.4813114779561809e-01,-1.3228522060801995e-01,5.1229840789382905e-02,4.1734055216619194e-02,-3.4922349728371403e-02,2.3489411511399848e-02,8.0756339158257071e-02,-4.1430846518602682e-02,1.3825891634375403e-01,-1.0281921942927558e-01,1.8102049531293241e-01,-1.4505660180914340e-01,1.8186344173432878e-01,-1.8620316464758269e-01,1.9090650984864999e-01,-2.8086787970093174e-01,2.7066109799278149e-01,-2.2564204861733023e-01,3.2153128507658019e-01,-2.5708859257733496e-01,2.6364737894970347e-01,-2.8693208660602365e-01,2.7970516529790845e-01,-3.5311023659542107e-01,3.4701280163120907e-01,-3.1032664634310730e-01,3.5125263768893017e-01,-3.3322268782288833e-01,3.6955614181826035e-01,-3.3683473663931718e-01,3.2182908581461295e-01,-2.8261547868377229e-01,3.4742873160768417e-01,-2.9435215444692314e-01,3.0697747027458971e-01,-2.9547216462701609e-01,3.2267414829365171e-01,-3.4291155301037418e-01,2.8631768612226638e-01,-2.6625338683543259e-01,3.4282658200878574e-01,-2.5735046181265991e-01,2.6016063755950836e-01,-2.7861330621270186e-01,2.4248759221995142e-01};
        arr_to_vec(ccdy_arr, sizeof(ccdy_arr)/sizeof(double), ccdy);
      }else if (tdi == 64){
        double posx_arr[] = {6.9000000000000000e+02,1.3830000000000000e+03,2.0780000000000000e+03,2.7720000000000000e+03,3.4690000000000000e+03,4.1670000000000000e+03,4.8670000000000000e+03,5.5680000000000000e+03,6.2690000000000000e+03,6.9720000000000000e+03,7.6760000000000000e+03,8.3800000000000000e+03,9.0830000000000000e+03,9.7910000000000000e+03,1.0500000000000000e+04,1.1205000000000000e+04,1.1914000000000000e+04,1.2621000000000000e+04,1.3330000000000000e+04,1.4040000000000000e+04,1.4749000000000000e+04,1.5460000000000000e+04,1.6169000000000000e+04,1.6880000000000000e+04,1.7590000000000000e+04,1.8301000000000000e+04,1.9011000000000000e+04,1.9721000000000000e+04,2.0431000000000000e+04,2.1140000000000000e+04,2.1849000000000000e+04,2.2558000000000000e+04,2.3267000000000000e+04,2.3974000000000000e+04,2.4682000000000000e+04,2.5388000000000000e+04,2.6094000000000000e+04,2.6798000000000000e+04,2.7502000000000000e+04,2.8207000000000000e+04,2.8910000000000000e+04,2.9611000000000000e+04,3.0310000000000000e+04,3.1009000000000000e+04,3.1707000000000000e+04,3.2402000000000000e+04,3.3097000000000000e+04,3.3792000000000000e+04,3.4483000000000000e+04};
        arr_to_vec(posx_arr, sizeof(posx_arr)/sizeof(double), posx);
        
        double ccdx_arr[] = {-3.7076621412738331e-01,3.5550141824393666e-01,-3.7452164290863260e-01,3.9262901152010804e-01,-1.8769753478951337e-01,3.2533478794703546e-01,-3.9198625112916013e-01,3.8704689431976808e-01,-3.6590725753218167e-01,2.9694949527702069e-01,-2.8649015958327789e-01,2.5095203437462293e-01,-2.5594103973106508e-01,2.9784859886309106e-01,-5.0437005448967877e-02,3.0611842084503493e-01,-3.0153284752317600e-01,2.0276849696751786e-01,-2.0712754463124641e-01,2.6109764756808007e-01,-2.2666349274312650e-01,2.3376431780780377e-01,-2.6069675734949660e-01,2.6614804197141900e-01,-5.5124060735859098e-02,2.2050257712868293e-01,-2.0378727419899301e-01,2.2297050538677374e-01,-2.1933919134564028e-01,2.6974389428912521e-01,-2.8334172256605766e-01,2.8226204448624104e-01,-3.2839327224057546e-01,3.3317765659611515e-01,-2.5260613914060148e-01,3.0139375552168429e-01,-2.5322738471653850e-01,2.9237543019936735e-01,-3.8084707684599189e-01,3.6959147307478779e-01,-2.9855475098684858e-01,4.4687261135014311e-01,-4.1245626765449583e-01,4.3868167286793502e-01,-3.4043488815006340e-01,5.1527680733525516e-01,-5.1464001731134690e-01,5.7874986010458207e-01,-5.0969991788199742e-01};
        arr_to_vec(ccdx_arr, sizeof(ccdx_arr)/sizeof(double), ccdx);
        
        double posy_arr[] ={6.9100000000000000e+02,1.3840000000000000e+03,2.0760000000000000e+03,2.7740000000000000e+03,3.3350000000000000e+03,4.1650000000000000e+03,4.8680000000000000e+03,5.5660000000000000e+03,6.2730000000000000e+03,6.8570000000000000e+03,7.5250000000000000e+03,8.3060000000000000e+03,9.0490000000000000e+03,9.9610000000000000e+03,1.0499000000000000e+04,1.1204000000000000e+04,1.1913000000000000e+04,1.2624000000000000e+04,1.3332000000000000e+04,1.4042000000000000e+04,1.4750000000000000e+04,1.5460000000000000e+04,1.6171000000000000e+04,1.6881000000000000e+04,1.7591000000000000e+04,1.8301000000000000e+04,1.9011000000000000e+04,1.9724000000000000e+04,2.0427000000000000e+04,2.1140000000000000e+04,2.1952000000000000e+04,2.2429000000000000e+04,2.3267000000000000e+04,2.3978000000000000e+04,2.4676000000000000e+04,2.5386000000000000e+04,2.6095000000000000e+04,2.6797000000000000e+04,2.7501000000000000e+04,2.8207000000000000e+04,2.8907000000000000e+04,2.9611000000000000e+04,3.0310000000000000e+04,3.1009000000000000e+04,3.1707000000000000e+04,3.2403000000000000e+04,3.3100000000000000e+04,3.3792000000000000e+04,3.4484000000000000e+04};
        arr_to_vec(posy_arr, sizeof(posy_arr)/sizeof(double), posy);
        
        double ccdy_arr[] = {-1.3311624221121157e-01,1.3543950906131863e-01,-1.3970984442456114e-01,1.2955375522840876e-01,3.1915480374165583e-02,9.1611276956722235e-02,-7.1078947881086069e-02,6.7174865370183856e-02,-4.2913477901772940e-02,-3.0902817703133391e-02,2.5469430059308677e-02,1.6196413083673863e-02,-2.5484293281108773e-02,-1.6531951186222246e-02,3.0202966754843706e-02,-2.5605278108099649e-02,2.0591317539084622e-02,-3.8491567273431844e-02,4.3197913812702673e-02,-5.3586887711306076e-02,4.9493979609867761e-02,-5.4176191083512099e-02,3.3121056670827966e-02,-4.5217992532030522e-02,5.6452244806208293e-02,-5.8790870634925628e-02,2.3189000445542639e-02,-2.8950496812307429e-02,2.3915192746001082e-02,-2.3313898134445021e-02,-1.3590226932693400e-02,1.4438860079747286e-02,-3.2175506390344777e-02,-1.7493062354573888e-02,-2.1034719178906199e-02,4.1588545859633057e-02,-5.2414577729730724e-02,5.2531937588743027e-02,-6.5982937142221310e-02,8.5553436028909052e-02,-1.3045620955293741e-01,1.2375979227007976e-01,-1.6502028917676223e-01,2.0278679445134914e-01,-9.4396081442271351e-02,2.3343697852327072e-01,-2.2766189718537050e-01,2.5591620501863988e-01,-2.4578279306085260e-01};
        arr_to_vec(ccdy_arr, sizeof(ccdy_arr)/sizeof(double), ccdy);
      }else{
        vw_throw(NoImplErr() << "wv_correct corrections not implemented for TDI " << tdi
                 << "\n");
      }
    }else{
      if (tdi == 16){
        double posx_arr[] = {6.9000000000000000e+02,1.3820000000000000e+03,2.0760000000000000e+03,2.7720000000000000e+03,3.4650000000000000e+03,4.1670000000000000e+03,4.8670000000000000e+03,5.5680000000000000e+03,6.2680000000000000e+03,6.9710000000000000e+03,7.6770000000000000e+03,8.3780000000000000e+03,9.0850000000000000e+03,9.7910000000000000e+03,1.0503000000000000e+04,1.1206000000000000e+04,1.1913000000000000e+04,1.2622000000000000e+04,1.3330000000000000e+04,1.4040000000000000e+04,1.4749000000000000e+04,1.5459000000000000e+04,1.6170000000000000e+04,1.6881000000000000e+04,1.7591000000000000e+04,1.8299000000000000e+04,1.9008000000000000e+04,1.9721000000000000e+04,2.0429000000000000e+04,2.1140000000000000e+04,2.1849000000000000e+04,2.2557000000000000e+04,2.3265000000000000e+04,2.3974000000000000e+04,2.4682000000000000e+04,2.5388000000000000e+04,2.6094000000000000e+04,2.6798000000000000e+04,2.7503000000000000e+04,2.8206000000000000e+04,2.8909000000000000e+04,2.9611000000000000e+04,3.0310000000000000e+04,3.1010000000000000e+04,3.1707000000000000e+04,3.2404000000000000e+04,3.3098000000000000e+04,3.3792000000000000e+04,3.4484000000000000e+04};
        arr_to_vec(posx_arr, sizeof(posx_arr)/sizeof(double), posx);
        
        double ccdx_arr[] = {-3.6563699517936904e-01,3.2374257439446741e-01,-3.0665689416227304e-01,2.8779817609507674e-01,-1.4391974546538006e-01,2.5213647269676465e-01,-2.7431622419889917e-01,2.2048856906422215e-01,-2.6590706716919182e-01,1.9957802948970854e-01,-2.7095596052316773e-01,2.3435367631109566e-01,-2.5315591932358317e-01,1.9856962666552852e-01,-3.5920061119356986e-02,1.5980621898278752e-01,-1.6159921442794922e-01,1.9744720254044340e-01,-1.9378974298741097e-01,1.3105906502192419e-01,-1.5230954549604461e-01,1.7447700273627750e-01,-1.4295214614100249e-01,1.6062348332303306e-01,4.8050851400649242e-02,1.3750807699320705e-01,-7.3113271749859196e-02,7.1346049905560857e-02,-9.9108950389247746e-02,1.9229689329792210e-01,-1.7243313894497606e-01,1.7966640657714811e-01,-1.5356877017491991e-01,1.7351553213620843e-01,-1.0112703267223394e-01,1.6278473565663124e-01,-1.2930576980595232e-01,1.4836628451840536e-01,-2.0982163366775036e-01,2.6205776957416932e-01,-2.0192748468363880e-01,2.0991625990584037e-01,-2.3878054471338728e-01,3.2054730743521237e-01,-2.2902300508023066e-01,3.5625055719764054e-01,-3.2197370046832202e-01,3.8652575820766899e-01,-3.6059189451754942e-01};
        arr_to_vec(ccdx_arr, sizeof(ccdx_arr)/sizeof(double), ccdx);
        
        double posy_arr[] ={6.9000000000000000e+02,1.3820000000000000e+03,2.0770000000000000e+03,2.7720000000000000e+03,3.4670000000000000e+03,4.1680000000000000e+03,4.8670000000000000e+03,5.5670000000000000e+03,6.2680000000000000e+03,6.9720000000000000e+03,7.6750000000000000e+03,8.3790000000000000e+03,9.0860000000000000e+03,9.7910000000000000e+03,1.0497000000000000e+04,1.1204000000000000e+04,1.1913000000000000e+04,1.2622000000000000e+04,1.3331000000000000e+04,1.4041000000000000e+04,1.4750000000000000e+04,1.5459000000000000e+04,1.6169000000000000e+04,1.6878000000000000e+04,1.7590000000000000e+04,1.8301000000000000e+04,1.9009000000000000e+04,1.9725000000000000e+04,2.0435000000000000e+04,2.1141000000000000e+04,2.1875000000000000e+04,2.2579000000000000e+04,2.3267000000000000e+04,2.3975000000000000e+04,2.4683000000000000e+04,2.5388000000000000e+04,2.6094000000000000e+04,2.6799000000000000e+04,2.7503000000000000e+04,2.8207000000000000e+04,2.8908000000000000e+04,2.9610000000000000e+04,3.0310000000000000e+04,3.1010000000000000e+04,3.1707000000000000e+04,3.2404000000000000e+04,3.3098000000000000e+04,3.3792000000000000e+04,3.4484000000000000e+04};
        arr_to_vec(posy_arr, sizeof(posy_arr)/sizeof(double), posy);
        
        double ccdy_arr[] = {1.6682712498426239e-01,-1.8658497293945167e-01,1.8716800259901073e-01,-2.0080511706846771e-01,2.5776226096916355e-01,-1.8014397580283278e-01,2.3187602202653296e-01,-1.8888815612376539e-01,2.3825312684105868e-01,-1.9764347168914259e-01,2.1303867405775190e-01,-2.1864997247176085e-01,1.9060281719060299e-01,-2.1095297916034275e-01,2.1385786807211804e-01,-2.0243697148649251e-01,1.6808369230471218e-01,-1.8247654347570377e-01,1.6321496740677194e-01,-1.6534240035866782e-01,1.3358190259939087e-01,-1.3762340408262302e-01,1.2517965805100878e-01,-1.2516492091413561e-01,9.9679467259256083e-02,-8.5525125627462728e-02,7.0833977291224953e-02,-4.1071933536127661e-02,2.4648656550307783e-02,-6.2804188860494159e-02,-1.4575088847986997e-02,2.2637929246120417e-02,-5.1578725614456332e-02,3.1892413968958873e-02,-4.9620043941112163e-02,1.0853263900788856e-01,-1.3861588547237652e-01,1.5996598004665616e-01,-1.5596844034381802e-01,2.1230696422520279e-01,-2.2320393916806852e-01,2.4987976967881814e-01,-2.7165466145699768e-01,3.4162810522906795e-01,-2.9219664119329974e-01,3.7353398140209576e-01,-4.0425355608407010e-01,4.8807824720156195e-01,-4.9484352711504931e-01};
        arr_to_vec(ccdy_arr, sizeof(ccdy_arr)/sizeof(double), ccdy);
      }else if (tdi == 64){
        double posx_arr[] = {6.9000000000000000e+02,1.3830000000000000e+03,2.0780000000000000e+03,2.7720000000000000e+03,3.4690000000000000e+03,4.1670000000000000e+03,4.8670000000000000e+03,5.5680000000000000e+03,6.2690000000000000e+03,6.9720000000000000e+03,7.6760000000000000e+03,8.3800000000000000e+03,9.0830000000000000e+03,9.7910000000000000e+03,1.0500000000000000e+04,1.1205000000000000e+04,1.1914000000000000e+04,1.2621000000000000e+04,1.3330000000000000e+04,1.4040000000000000e+04,1.4749000000000000e+04,1.5460000000000000e+04,1.6169000000000000e+04,1.6880000000000000e+04,1.7590000000000000e+04,1.8301000000000000e+04,1.9011000000000000e+04,1.9721000000000000e+04,2.0431000000000000e+04,2.1140000000000000e+04,2.1849000000000000e+04,2.2558000000000000e+04,2.3267000000000000e+04,2.3974000000000000e+04,2.4682000000000000e+04,2.5388000000000000e+04,2.6094000000000000e+04,2.6798000000000000e+04,2.7502000000000000e+04,2.8207000000000000e+04,2.8910000000000000e+04,2.9611000000000000e+04,3.0310000000000000e+04,3.1009000000000000e+04,3.1707000000000000e+04,3.2402000000000000e+04,3.3097000000000000e+04,3.3792000000000000e+04,3.4483000000000000e+04};
        arr_to_vec(posx_arr, sizeof(posx_arr)/sizeof(double), posx);
        
        double ccdx_arr[] = {-3.7076621412738331e-01,3.5550141824393666e-01,-3.7452164290863260e-01,3.9262901152010804e-01,-1.8769753478951337e-01,3.2533478794703546e-01,-3.9198625112916013e-01,3.8704689431976808e-01,-3.6590725753218167e-01,2.9694949527702069e-01,-2.8649015958327789e-01,2.5095203437462293e-01,-2.5594103973106508e-01,2.9784859886309106e-01,-5.0437005448967877e-02,3.0611842084503493e-01,-3.0153284752317600e-01,2.0276849696751786e-01,-2.0712754463124641e-01,2.6109764756808007e-01,-2.2666349274312650e-01,2.3376431780780377e-01,-2.6069675734949660e-01,2.6614804197141900e-01,-5.5124060735859098e-02,2.2050257712868293e-01,-2.0378727419899301e-01,2.2297050538677374e-01,-2.1933919134564028e-01,2.6974389428912521e-01,-2.8334172256605766e-01,2.8226204448624104e-01,-3.2839327224057546e-01,3.3317765659611515e-01,-2.5260613914060148e-01,3.0139375552168429e-01,-2.5322738471653850e-01,2.9237543019936735e-01,-3.8084707684599189e-01,3.6959147307478779e-01,-2.9855475098684858e-01,4.4687261135014311e-01,-4.1245626765449583e-01,4.3868167286793502e-01,-3.4043488815006340e-01,5.1527680733525516e-01,-5.1464001731134690e-01,5.7874986010458207e-01,-5.0969991788199742e-01};
        arr_to_vec(ccdx_arr, sizeof(ccdx_arr)/sizeof(double), ccdx);
        
        double posy_arr[] ={6.9100000000000000e+02,1.3840000000000000e+03,2.0760000000000000e+03,2.7740000000000000e+03,3.3350000000000000e+03,4.1650000000000000e+03,4.8680000000000000e+03,5.5660000000000000e+03,6.2730000000000000e+03,6.8570000000000000e+03,7.5250000000000000e+03,8.3060000000000000e+03,9.0490000000000000e+03,9.9610000000000000e+03,1.0499000000000000e+04,1.1204000000000000e+04,1.1913000000000000e+04,1.2624000000000000e+04,1.3332000000000000e+04,1.4042000000000000e+04,1.4750000000000000e+04,1.5460000000000000e+04,1.6171000000000000e+04,1.6881000000000000e+04,1.7591000000000000e+04,1.8301000000000000e+04,1.9011000000000000e+04,1.9724000000000000e+04,2.0427000000000000e+04,2.1140000000000000e+04,2.1952000000000000e+04,2.2429000000000000e+04,2.3267000000000000e+04,2.3978000000000000e+04,2.4676000000000000e+04,2.5386000000000000e+04,2.6095000000000000e+04,2.6797000000000000e+04,2.7501000000000000e+04,2.8207000000000000e+04,2.8907000000000000e+04,2.9611000000000000e+04,3.0310000000000000e+04,3.1009000000000000e+04,3.1707000000000000e+04,3.2403000000000000e+04,3.3100000000000000e+04,3.3792000000000000e+04,3.4484000000000000e+04};
        arr_to_vec(posy_arr, sizeof(posy_arr)/sizeof(double), posy);
        
        // There was not enough good data to reliably estimate ccdy
        // offsets. Some faint CCD artifacts are still visible after
        // correction. Need to revisit this case when more data is
        // available.
        ccdy = vector<double>(posy.size(), 0);
      }else{
        vw_throw(NoImplErr() << "wv_correct corrections not implemented for TDI " << tdi
                 << "\n");
      }
    }
  }
}

template <class ImageT>
class WVCorrectView: public ImageViewBase< WVCorrectView<ImageT> >{
  ImageT m_img;
  int m_tdi;
  bool m_is_wv01, m_is_forward;
  double m_shift, m_period, m_xoffset, m_yoffset;
  std::vector<double> m_posx, m_ccdx, m_posy, m_ccdy;

  typedef typename ImageT::pixel_type PixelT;

public:
  WVCorrectView( ImageT const& img, int tdi, bool is_wv01, bool is_forward,
                 double shift, double period, double xoffset, double yoffset):
    m_img(img), m_tdi(tdi), m_is_wv01(is_wv01), m_is_forward(is_forward),
    m_shift(shift), m_period(period),
    m_xoffset(xoffset), m_yoffset(yoffset){

    get_offsets(m_tdi, m_is_wv01, m_is_forward,  
                m_posx, m_ccdx, m_posy, m_ccdy);

    VW_ASSERT(m_posx.size() == m_ccdx.size() &&
              m_posy.size() == m_ccdy.size(),
              ArgumentErr() << "wv_correct: Expecting the arrays of positions and offsets "
              << "to have the same sizes.");
  }
  
  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef ProceduralPixelAccessor<WVCorrectView> pixel_accessor;

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "WVCorrectView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // Need to see a bit more of the input image for the purpose
    // of interpolation.
    int bias = (int)ceil(std::max(std::abs(m_xoffset), std::abs(m_yoffset)))
      + BilinearInterpolation::pixel_buffer + 1;
    BBox2i biased_box = bbox;
    biased_box.expand(bias);
    biased_box.crop(bounding_box(m_img));
    
    ImageView<result_type> cropped_img = crop(m_img, biased_box);
    InterpolationView<EdgeExtensionView< ImageView<result_type>, ConstantEdgeExtension >, BilinearInterpolation> interp_img
      = interpolate(cropped_img, BilinearInterpolation(),
                    ConstantEdgeExtension());

    
    ImageView<result_type> tile(bbox.width(), bbox.height());
    for (int col = bbox.min().x(); col < bbox.max().x(); col++){
      
      // The sign of CCD offsets alternates as one moves along the image
      // columns. As such, at "even" blocks, the offsets accumulated so
      // far cancel each other, so we need to correct the "odd" blocks
      // only.
      int block_index = (int)floor((col - m_shift)/m_period);
      double valx = 0, valy = 0;
      if (block_index % 2 == 1){
        valx = -m_xoffset;
        valy = -m_yoffset;
      }

      // Special treatment for WV01
      if (m_is_wv01){
        
        valx = 0.0;
        for (size_t t = 0; t < m_ccdx.size(); t++){
          if (m_posx[t] < col){
            valx -= m_ccdx[t];
          }
        }
        
        valy = 0.0;
        for (size_t t = 0; t < m_ccdy.size(); t++){
          if (m_posy[t] < col){
            valy -= m_ccdy[t];
          }
        }
        
      }
      
      for (int row = bbox.min().y(); row < bbox.max().y(); row++){
        tile(col - bbox.min().x(), row - bbox.min().y() )
          = interp_img(col - biased_box.min().x() + valx,
                       row - biased_box.min().y() + valy);
      }
    }
    
    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};
template <class ImageT>
WVCorrectView<ImageT> wv_correct(ImageT const& img,
                                 int tdi, bool is_wv01, bool is_forward, double shift,
                                 double period, double xoffset, double yoffset){
  return WVCorrectView<ImageT>(img, tdi, is_wv01, is_forward, shift, period,
                               xoffset, yoffset);
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    GeometricXML geo;
    AttitudeXML att;
    EphemerisXML eph;
    ImageXML img;
    RPCXML rpc;
    std::string scan_dir, sat_id;
    double det_pitch;
    int tdi;
    try{
      XMLPlatformUtils::Initialize();
      read_xml( opt.camera_model_file, geo, att, eph, img, rpc );
      
      scan_dir = boost::to_lower_copy( img.scan_direction );
      if (scan_dir != "forward" && scan_dir != "reverse")
        vw_throw( ArgumentErr() << "XML file \"" << opt.camera_model_file
                  << "\" is lacking a valid image scan direction.\n" );

      sat_id = img.sat_id;
      if (sat_id != "WV01" && sat_id != "WV02")
        vw_throw( ArgumentErr() << "Can apply CCD artifacts corrections only "
                  << "for WV01 and WV02 camera images.\n" );
      
      det_pitch = geo.detector_pixel_pitch;
      if (det_pitch <= 0.0)
        vw_throw( ArgumentErr() << "XML file \"" << opt.camera_model_file
                  << "\" has a non-positive pixel pitch.\n" );

      tdi = img.tdi;
      if (tdi != 16 && sat_id != "WV01")
        vw_throw( ArgumentErr() << "For camera of type " << sat_id
                  << ", can apply CCD artifacts corrections only for TDI 16.\n" );
      
    } catch ( const std::exception& e ) {                
      vw_throw( ArgumentErr() << e.what() );
    }

    bool is_forward = (scan_dir == "forward");
    
    // Defaults, depending on satellite and scan direction
    double xoffset, yoffset, period, shift;
    bool is_wv01 = (sat_id == "WV01");
    if (is_wv01){
      if (is_forward){
        period = 0;
        shift  = 0;
        xoffset = 0;
        yoffset = 0;
      }else{
        period  = 0;
        shift   = 0;
        xoffset = 0;
        yoffset = 0;
      }
    }else{
      period = 705;
      shift  = -35.0;
      if (is_forward){
        xoffset = 0.2842;
        yoffset = 0.2369;
      }else{
        xoffset = 0.3396;
        yoffset = 0.3725;
      }
    }

    // Adjust for detector pitch
    period = period*(8.0e-3/det_pitch);
    
    // Internal sign adjustments
    if (is_forward){
      yoffset = -yoffset;
    }else{
      xoffset = -xoffset;
    }

    DiskImageView<float> input_img(opt.camera_image_file);
    bool has_nodata = false;
    double nodata = numeric_limits<double>::quiet_NaN();
    boost::shared_ptr<DiskImageResource> img_rsrc
      ( new DiskImageResourceGDAL(opt.camera_image_file) );
    if (img_rsrc->has_nodata_read()){
      has_nodata = true;
      nodata = img_rsrc->nodata_read();
    }

    vw_out() << "Writing: " << opt.output_image << std::endl;
    if (has_nodata){
      asp::block_write_gdal_image(opt.output_image,
                                  apply_mask
                                  (wv_correct(create_mask(input_img,
                                                          nodata),
                                              tdi, is_wv01, is_forward,
                                              shift, period, xoffset, yoffset),
                                   nodata),
                                  nodata, opt,
                                  TerminalProgressCallback("asp", "\t-->: "));
    }else{
      asp::block_write_gdal_image(opt.output_image,
                                  wv_correct(input_img,
                                             tdi, is_wv01, is_forward, shift, period,
                                             xoffset, yoffset),
                                  opt,
                                  TerminalProgressCallback("asp", "\t-->: "));
    }
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
