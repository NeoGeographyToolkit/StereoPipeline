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

/// \file nonlin_trans_corr.cc
///

// Use VTK's thin plate transform to warp left and right images based
// on disparity to make them more similar, compute the disparity among
// the warped images, and then unwarp the disparity.

#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkThinPlateSplineTransform.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Image/PixelMask.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Image/Interpolation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/ImageViewRef.h>

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

#include <iostream>

// Find the max error between two arrays of points. Note that
// cannot use const here as GetPoint() is not const.
double max_error(vtkNew<vtkPoints> & a, vtkNew<vtkPoints> & b) {

  int npoints = a->GetNumberOfPoints();
  double max_err = 0.0;

  for (int i = 0; i < npoints; i++) {
    double testDstPoint[3] = {0.0};
    a->GetPoint(i, testDstPoint);
    double transSrcPoint[3] = {0.0};
    b->GetPoint(i, transSrcPoint);
    double d = vtkMath::Distance2BetweenPoints(testDstPoint, transSrcPoint);
    max_err = std::max(max_err, d);
  }
  return max_err;
}

void TestThinPlateSplineTransform() {

  int npoints = 20;

  const double P[20][3] = {
    { -0.8316301300814422, -0.06992580859519772, -1.6034524068257419 },
    { -2.151893827785692, 0.38244721645095636, -0.9275967632551845 },
    { 0.8147291118075928, -0.7016483698682392, 0.15003863332602096 },
    { 0.918239421266975, 0.5515514723709805, -1.0230600499321258 },
    { -0.4977939747967184, 1.5000786176083494, 0.892455159403953 },
    { 2.137759080794324, -0.7876029858279091, 0.23676951564894347 },
    { 0.07659657475437548, 0.37528421293358666, 1.061745743663681 },
    { -0.7908820649026604, 1.4270955106455065, 2.2665387247459576 },
    { -0.5663930529602919, 1.9402635876094498, 1.1531767242062774 },
    { 0.22529528853908187, -1.5938090446587108, -0.7004997748768814 },
    { 0.6165064084492409, -0.2761336076050157, -0.7930056820043028 },
    { -1.6122391974605947, -1.4200010952872733, 1.0567292903013055 },
    { 0.17993263043615856, -0.9038514957133562, -2.1611068227229695 },
    { -1.4186794357559613, 0.85026116269838, -1.7600646313947719 },
    { 0.9690209792801024, 0.7018737798529897, 0.3923799957082836 },
    { -0.6586203767750309, -2.1468680342265904, 0.762954972139701 },
    { 1.2872860659137344, 0.8557080868402649, 0.3905931440107816 },
    { -0.18996464681200217, 0.8315184491297033, -1.0227889589485941 },
    { 1.0636210067525393, -0.24736478911115908, -0.7581101375259237 },
    { -0.09448165336394657, -1.1381967760924927, -0.7171168342666931 },
  };

  const double noise[20][3] = {
    { 1.5137019295427425, 0.6858246680960894, 0.07331883771349512 },
    { -0.34081703057234036, 0.47987804772801446, 0.982197518178181 },
    { -0.1106079068591361, 1.0523148414328571, 0.17910578196163454 },
    { 0.05724784633562011, -0.08459760939107724, -0.7665637643588622 },
    { -0.4333381262791796, 0.018412807528038692, 0.6889623031683394 },
    { -1.1692454358073843, -0.6875830563599973, 0.9077463799204326 },
    { -1.9329042505569662, 1.0529789607437061, -0.29738186972471486 },
    { -0.12079407626315326, 0.9261998453458427, 1.0938543547601083 },
    { -0.6384715430732077, -0.2606527602354865, 1.417882645305744 },
    { -0.10127708027623447, -0.7470111486643078, 0.726100633329295 },
    { 0.36659507636859245, 1.4194144006017144, 0.41878644928947467 },
    { 1.0325034539790547, -0.2291631905797599, -1.3490582933020208 },
    { -0.7186165872334461, 0.4613954758072554, -1.1318559861004829 },
    { 2.455035378196603, -0.01476716688473253, -0.0890030227805104 },
    { 1.6498918075463915, 2.7557006973876508, -0.6466098561563114 },
    { 1.16764314555201, -1.5226214641344893, 0.13000979083980121 },
    { -0.9640219699623079, 1.3071375444488553, 0.5668689159057715 },
    { 0.40366181757487013, 2.308315254377135, 0.8202651493656881 },
    { -1.0267515231555335, -0.2853656137629097, -1.1599391275129292 },
    { -0.09199656043877075, 0.35274602605225164, 2.5626579880899327 },
  };

  // Good API
  //sourceTransform->TransformPoint(src, src);
  
  // create the two point sets
  vtkNew<vtkPoints> srcPoints, dstPoints;
  vtkNew<vtkPoints> testSrcPoints, testDstPoints;
  double s = 0.1;
  double a = 0.1, b = 0.1;

  // Create a set of points to find the transform for, and a set of new test
  // points to verify how well it generalized
  for (int i = 0; i < npoints; i++) {
    double src[3] = {P[i][0], P[i][1], 0};
    double testSrc[3] = {src[0] + s * noise[i][0], src[1] + s * noise[i][1], 0};

    double dst[3] = {src[0] + a * src[1] *src[0] * src[0],
                     src[1] + b * src[0] * src[1],
                     0};
    double testDst[3] = {testSrc[0] + a * testSrc[1] * testSrc[0] * testSrc[0],
                         testSrc[1] + b * testSrc[0] * testSrc[1],
                         0};
    
    srcPoints->InsertNextPoint(src);
    testSrcPoints->InsertNextPoint(testSrc);
    dstPoints->InsertNextPoint(dst);
    testDstPoints->InsertNextPoint(testDst);
  }

  // Set up the transform
  vtkNew<vtkThinPlateSplineTransform> trans;
  //trans->SetBasisToR(); // 3D
  trans->SetBasisToR2LogR(); // 2D
  trans->SetRegularizeBulkTransform(true);
  trans->SetSourceLandmarks(srcPoints);
  trans->SetTargetLandmarks(dstPoints);
  trans->Update();

  // Test the forward transform on the points used to create it
  vtkNew<vtkPoints> transSrcPoints;
  trans->TransformPoints(srcPoints, transSrcPoints);
  std::cout << "Error using forward transform on points which defined it: "
            << max_error(dstPoints, transSrcPoints) << "\n";
  
  // Test the forward transform on a new set of points for which we
  // know the answer
  vtkNew<vtkPoints> transTestSrcPoints;
  trans->TransformPoints(testSrcPoints, transTestSrcPoints);
  std::cout << "Error using forward transform on additional test points: "
            << max_error(testDstPoints, transTestSrcPoints) << "\n";
  
  // Compute the inverse transform. This changes the direction
  // in which TransformPoints() is applied.
  trans->Inverse();

  // Test how reliably the inverse brings one back

  vtkNew<vtkPoints> srcPoints2;
  trans->TransformPoints(transSrcPoints, srcPoints2);
  std::cout << "Error after forward then inverse transform on orig points: "
            << max_error(srcPoints, srcPoints2) << "\n";

  vtkNew<vtkPoints> testSrcPoints2;
  trans->TransformPoints(transTestSrcPoints, testSrcPoints2);
  std::cout << "Error after forward then inverse transform on additional points: "
            << max_error(testSrcPoints, testSrcPoints2) << "\n";

  vtkNew<vtkPoints> transTestDstPoints;
  trans->TransformPoints(testDstPoints, transTestDstPoints);
  std::cout << "Error using inverse transform on additional test points: "
            << max_error(testSrcPoints, transTestDstPoints) << "\n";

  return;
}

struct Options : vw::cartography::GdalWriteOptions {
};

using namespace vw;

// Testcase:
// cd projects/grand_mesa
// nonlin_trans_corr stereo_rpc_local_epi_2_1_rpc_v2/run 1000
// Then run:
// parallel_stereo --correlator-mode run-L_sub.tif aligned-R_sub.tif

int main(int argc, char ** argv) {

  Options opt;
  try {

    // TestThinPlateSplineTransform();

    std::string prefix = argv[1];
    
    int num_samples = atof(argv[2]);
    std::cout << "Num samples is " << num_samples << std::endl;
  
    std::string left_file = prefix + "-L_sub.tif";
    std::string right_file = prefix + "-R_sub.tif";
    std::string disp_file = prefix + "-D_sub.tif";
    //std::string disp_file = prefix + "-D_sub_asp_bm.tif";
  
    DiskImageView<float> left(left_file);
    DiskImageView<float> right(right_file);
    DiskImageView<PixelMask<Vector2f>> disp(disp_file);

    float right_nodata = -32768.0;
    if (read_nodata_val(right_file, right_nodata)) {
      std::cout << "Read right image nodata value: " << right_nodata << std::endl;
    }
    ImageViewRef<PixelMask<float>> masked_right  = create_mask(right, right_nodata);
  
    std::cout << "L cols and rows " << left.cols() << ' ' << left.rows() << std::endl;
    std::cout << "R cols and rows " << right.cols() << ' ' << right.rows() << std::endl;
    std::cout << "D cols and rows " << disp.cols() << ' ' << disp.rows() << std::endl;

    if (left.cols() != disp.cols() || left.rows() != disp.rows())
      vw_throw( ArgumentErr() << "Left image and disparity must have same dims.");

    double area = double(left.cols()) * double(left.rows()); // careful to not overflow
    int sample_rate = round(sqrt(double(area) / double(num_samples)));
    if (sample_rate < 1) 
      sample_rate = 1;
    std::cout << "Sample rate " << sample_rate << std::endl;
  
    Stopwatch sw;
    sw.start();

    // Create a set of points to find the transform for, and a set of new test
    // points to verify how well it generalized
    int max_points = 1000, count = 0;
    vtkNew<vtkPoints> srcPoints, dstPoints;
    for (int col = 0; col < disp.cols(); col += sample_rate) {
      for (int row = 0; row < disp.rows(); row += sample_rate) {
        PixelMask<Vector2f> pix = disp(col, row);
        if (!is_valid(pix))
          continue;

        double src[3] = {double(col), double(row), 0.0};
        double dst[3] = {col + pix.child().x(), row + pix.child().y(), 0.0};
        srcPoints->InsertNextPoint(src);
        dstPoints->InsertNextPoint(dst);
        count++;

        if (count >= max_points) 
          break;
      }
      if (count >= max_points) 
        break;
    }
    sw.stop();
    vw_out() << "Elapsed time to add points: " << sw.elapsed_seconds() << " seconds.\n";

    int npoints = srcPoints->GetNumberOfPoints();
    std::cout << "Num points " << npoints << std::endl;
    sw.start();

    // Set up the transform
    vtkNew<vtkThinPlateSplineTransform> trans;
    //trans->SetBasisToR(); // 3D
    trans->SetBasisToR2LogR(); // 2D
    trans->SetRegularizeBulkTransform(true);
    trans->SetSourceLandmarks(srcPoints);
    trans->SetTargetLandmarks(dstPoints);
    trans->Update();
  
    sw.stop();
    vw_out() << "Elapsed time to calc transform: " << sw.elapsed_seconds() << " seconds.\n";

    // An invalid pixel value used for edge extension
    PixelMask<float> nodata_pix(0); nodata_pix.invalidate();
    ValueEdgeExtension<PixelMask<float>> nodata_ext(nodata_pix); 
  
    // Interpolate into the right image. Avoid using an ImageViewRef to
    // avoid a per-pixel virtual function overhead. The 'auto' keyword
    // will use the exact type.
    auto interp_right = interpolate(create_mask(right, right_nodata),
                                    BilinearInterpolation(), nodata_ext);

    vw_out() << "Warping the image.\n";
    TerminalProgressCallback tpc("asp", "\t--> ");
    
    // Warp the right image to make it more similar to the left one
    ImageView<PixelMask<float>> trans_right(left.cols(), left.rows());
    //#pragma omp parallel for
    for (int col = 0; col < left.cols(); col++) {
      tpc.report_fractional_progress(col, left.cols());

      for (int row = 0; row < left.rows(); row++) {
      
        double src[3] = {double(col), double(row), 0.0};
        double dst[3] = {0.0, 0.0, 0.0};
        trans->TransformPoint(src, dst);

        trans_right(col, row) = interp_right(dst[0], dst[1]);
      }
    }

    tpc.report_finished();
    
    vw::cartography::GeoReference left_georef;
    bool has_left_georef = read_georeference(left_georef, left_file);
    bool has_nodata      = true;
    std::string output_image = prefix + "-trans_right.tif";
    vw_out() << "Writing: " << output_image << "\n";
    vw::cartography::block_write_gdal_image
      (output_image,
       apply_mask(trans_right, right_nodata),
       has_left_georef, left_georef,
       has_nodata, right_nodata, opt,
       TerminalProgressCallback("asp", "\t--> Nonlinear warping:"));
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}
