#include <asp/Core/SurfaceFitView.h>

#include <ceres/ceres.h>

using namespace vw;

void asp::SurfaceFitViewBase::fit_2d_polynomial_surface( ImageView<PixelMask<Vector2i> > const& input,
                                                         Matrix3x3* output_h, Matrix3x3* output_v,
                                                         Vector2* xscaling, Vector2* yscaling) const {
  // Figure out what our scaling parameters should be
  // output coordinates = scaling.y * (input coordinates + scaling.x)
  xscaling->x() = -double(input.cols()) / 2.0;
  yscaling->x() = -double(input.rows()) / 2.0;
  xscaling->y() = 2.0/double(input.cols());
  yscaling->y() = 2.0/double(input.rows());

  {
    // Build a ceres problem to fit a polynomial robustly
    ceres::Problem problem;
    for (int j = 0; j < input.rows(); j+=2) {
      for (int i = 0; i < input.cols(); i+=2 ) {
        if (is_valid(input(i,j)))
          problem.AddResidualBlock
            (new ceres::AutoDiffCostFunction<PolynomialSurfaceFit, 1, 9>
             (new PolynomialSurfaceFit
              (input(i,j)[0],
               (double(i) + xscaling->x()) * xscaling->y(),
               (double(j) + yscaling->x()) * yscaling->y())),
             new ceres::CauchyLoss(4),
             &(*output_h)(0,0));
      }
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 300;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.BriefReport() << std::endl;
  }

  {
    // Build a ceres problem to fit a polynomial robustly
    ceres::Problem problem;
    for (int j = 0; j < input.rows(); j+=2) {
      for (int i = 0; i < input.cols(); i++ ) {
        if (is_valid(input(i,j)))
          problem.AddResidualBlock
            (new ceres::AutoDiffCostFunction<PolynomialSurfaceFit, 1, 9>
             (new PolynomialSurfaceFit
              (input(i,j)[1],
               (double(i) + xscaling->x()) * xscaling->y(),
               (double(j) + yscaling->x()) * yscaling->y())),
             new ceres::CauchyLoss(4),
             &(*output_v)(0,0));
      }
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 300;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.BriefReport() << std::endl;
  }
}


void asp::SurfaceFitViewBase::render_polynomial_surface(Matrix3x3 const& polynomial_coeff,
                                                        ImageView<float>* output ) const {
  const double* polyarray = &polynomial_coeff(0,0);
  for (int j = 0; j < output->rows(); j++ ) {
    double jn = (double(j) * 2.0 - double(output->rows())) / double(output->rows());
    for (int i = 0; i < output->cols(); i++ ) {
      double in = (double(i) * 2.0 - double(output->cols())) / double(output->cols());
      (*output)(i, j) =
        polyarray[0] +
        polyarray[1] * in +
        polyarray[2] * in * in +
        polyarray[3] * jn +
        polyarray[4] * jn * in +
        polyarray[5] * jn * in * in +
        polyarray[6] * jn * jn +
        polyarray[7] * jn * jn * in +
        polyarray[8] * jn * jn * in * in;
    }
  }
}
