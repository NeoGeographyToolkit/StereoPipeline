//===========================================================================
// SINTEF Multilevel B-spline Approximation library - version 1.1
//
// Copyright (C) 2000-2005 SINTEF ICT, Applied Mathematics, Norway.
//
// This program is free software; you can redistribute it and/or          
// modify it under the terms of the GNU General Public License            
// as published by the Free Software Foundation version 2 of the License. 
//
// This program is distributed in the hope that it will be useful,        
// but WITHOUT ANY WARRANTY; without even the implied warranty of         
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          
// GNU General Public License for more details.                           
//
// You should have received a copy of the GNU General Public License      
// along with this program; if not, write to the Free Software            
// Foundation, Inc.,                                                      
// 59 Temple Place - Suite 330,                                           
// Boston, MA  02111-1307, USA.                                           
//
// Contact information: e-mail: tor.dokken@sintef.no                      
// SINTEF ICT, Department of Applied Mathematics,                         
// P.O. Box 124 Blindern,                                                 
// 0314 Oslo, Norway.                                                     
//
// Other licenses are also available for this software, notably licenses
// for:
// - Building commercial software.                                        
// - Building software whose source code you wish to keep private.        
//===========================================================================

#include "checkWIN32andSGI.h"

#ifndef SGI
#include <cmath>
#else
#include <math.h>
#endif

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <algorithm>
using namespace std;

#include <MBA.h>
#include <UCBsplines.h>

#include <stdio.h> 

using namespace std;

namespace {
    vector<UCBspl_real> generate_smoothing_filter();
    UCBspl_real extrapolate_point(int i, int j, const GenMatrixType& matrix);
}; 

const std::vector<UCBspl_real> MBA::smoothing_filter_ = generate_smoothing_filter();

void MBA::init(UCBspl::SplineSurface& surf) {
    PHI_ = surf.getCoefficients();

    data_.umin_ = surf.umin();
    data_.vmin_ = surf.vmin();
    data_.umax_ = surf.umax();
    data_.vmax_ = surf.vmax();

    data_.urange_inv_ = double(1) / (data_.umax_ - data_.umin_);
    data_.vrange_inv_ = double(1) / (data_.vmax_ - data_.vmin_);

    data_.offset_ = 0.0;


    m_ = PHI_->noX() - 3;
    n_ = PHI_->noY() - 3;
}

bool MBA::adjustForBaseSurface() {
    if (data_.baseType_ != MBA_ZERO) {
	if (data_.baseType_ != MBA_CONSTLS && data_.baseType_ != MBA_CONSTVAL) {
	    throw runtime_error("ERROR, not support for this type of base surface");
	    return false;
	}
    
	(*PHI_) += data_.offset_; 
	data_.offset_ = 0.0;
	data_.baseType_ = MBA_ZERO;
    }
    return true;
}

void MBA::cleanup(int type)
{
    if (type == 0 || type == 2) {
	delta_.clear();
	omega_.clear();
    }

    if (type == 2)
	data_.clear(); 
}


void MBA::flagZeros(GenMatrix<bool>& zeromat) const 
{
    zeromat.fill(true);

    int noPoints = data_.size();

    double u_range_inv = data_.rangeUInv();
    double v_range_inv = data_.rangeVInv();

    for (int ip = 0; ip < noPoints; ip++) {

	double uc = (data_.U(ip) - data_.umin()) * u_range_inv * (double)m_;
	double vc = (data_.V(ip) - data_.vmin()) * v_range_inv * (double)n_;
    
	int i, j;
	double s, t;
	UCBspl::ijst(m_, n_, uc, vc, i, j, s, t);

	i++;
	j++;
	int offset = 1; 

	for (int ii = i - offset; ii <= i + offset + 1; ii++)
	    for (int jj = j-offset; jj <= j + offset + 1; jj++)
		zeromat(ii,jj) = false;
    }
}

void MBA::smoothMatrix(GenMatrixType& matrix, int no_iter)
{
    cout << "Now smoothing with " << no_iter << " iterations.\n";
    if (no_iter%2 != 0) {
	throw runtime_error("Number of smoothing iterations must be pair.");
    }

    int resX = matrix.noX();
    int resY = matrix.noY();
    GenMatrixType mat_temp(resX, resY);

    UCBspl_real temp = 0;
    for (int iter = 0; iter < no_iter; ++iter) {
	for (int j = -1; j < resY - 1; ++j) {
	    for (int i = -1; i < resX - 1; ++i) {
		mat_temp(i, j) = 0;
		for (int m2 = -2; m2 <= 2; ++m2) {
		    for (int m1 = -2; m1 <= 2; ++m1) {
			temp = smoothing_filter_[(m2+2) * 5 + (m1+2)];
			if (i+m1 < -1 || 
			    i+m1 >= resX - 1 ||
			    j+m2 < -1 ||
			    j+m2 >= resY - 1) {
			    temp *= extrapolate_point(i+m1, j+m2, matrix);
			} else {
			    temp *= matrix(i + m1, j + m2);
			}
			mat_temp(i, j) += temp;
		    }
		}
	    }
	}
	matrix.swap(mat_temp);
    }
}


void MBA::smoothZeros(int no_iter) 
{
    cout << "Smoothing zeros with " << no_iter << " iterations.\n";
    if (no_iter == 0)
	return;

    cout << "Flagging zeros:\n";
    GenMatrix<bool> zeromat(PHI_->noX(), PHI_->noY());
    flagZeros(zeromat);
    
    int noU = PHI_->noX()-2; 
    int noV = PHI_->noY()-2;

    GenMatrixType mat_temp(noU+2, noV+2);

    UCBspl_real temp = 0;
    for (int iter = 0; iter < no_iter; ++iter) {
	for (int j = 0; j < noV; ++j) {
	    for (int i = 0; i < noU; ++i) {
		if (zeromat(i, j)) {
		    mat_temp(i, j) = 0;
		    for (int m2 = -2; m2 <= 2; ++m2) {
			for (int m1 = -2; m1 <= 2; ++m1) {
			    temp = smoothing_filter_[(m2+2) * 5 + (m1+2)];
			    if (i+m1 < -1 || 
				i+m1 >= noU + 1 ||
				j+m2 < -1 ||
				j+m2 >= noV + 1) {
				temp *= extrapolate_point(i+m1, j+m2, *PHI_);
			    } else {
				temp *= (*PHI_)(i + m1, j + m2);
			    }
			    mat_temp(i, j) += temp;
			}
		    }
		} else {
		    mat_temp(i, j) = -1 * (*PHI_)(i, j);
		}

	    }
	}
	PHI_->swap(mat_temp);
    }
}

double MBA::f_pure(double u, double v) const { 
  
  

    
    double uc = (u - data_.umin()) * data_.rangeUInv() * (double)m_;
    double vc = (v - data_.vmin()) * data_.rangeVInv() * (double)n_;
  
    int i, j;
    double s, t;
    UCBspl::ijst(m_, n_, uc, vc, i, j, s, t);

    double Bks[4];
    double Blt[4];

    Bks[0] = UCBspl::B_0(s); Blt[0] = UCBspl::B_0(t);
    Bks[1] = UCBspl::B_1(s); Blt[1] = UCBspl::B_1(t);
    Bks[2] = UCBspl::B_2(s); Blt[2] = UCBspl::B_2(t);
    Bks[3] = UCBspl::B_3(s); Blt[3] = UCBspl::B_3(t);


    double val = 0.0;
    for (int k = 0; k <= 3; k++)
	for (int l = 0; l <= 3; l++)
	    val += (*PHI_)(i+k,j+l)*Bks[k]*Blt[l];
        
    return val;
}

void MBA::checkSparsity() const {
    int no_zeros = 0;
    for (int i = -1; i <= m_+1; i++) {
	for (int j = -1; j <= n_+1; j++) {
	    if ((*PHI_)(i,j) == 0.0)
		no_zeros++;
	}
    }
  
    cout << "Entries in PHI_(" << m_ << "," << n_ << "): " << (m_+3)*(n_+3) << endl;
    cout << "No. of zeros: " << no_zeros << endl;
    cout << "Sparsity: " << (double)no_zeros/(double)((m_+3)*(n_+3))*100.0 << endl;
}


void MBA::checkError() const {
  
    cout << "Checking max error..." << endl;

    UCBspl::SplineSurface surf = getSplineSurface();

      
    int noPoints = data_.size();
    if (noPoints == 0) {
	throw runtime_error("ERROR, no points. Has cleanup() been run?");
    }
  
    double maxErr = -99999.0;
    double up, vp, zc, zval;
    double zmin = 1.0e+30;
    double zmax = -99999999999999999.0;
  
    double sum_err = 0;
    double sum_err2 = 0;
    double up_max, vp_max;
    int p_maxno;

    std::vector<double> Zorig = *data_.Zorig_;
    for (int ip = 0; ip < noPoints; ip++) {
    
	up = data_.U(ip);
	vp = data_.V(ip);
	zc = Zorig[ip];
#ifdef WIN32ORSGI
	double err = fabs(surf.f(up,vp) - zc);
#else
	double err = std::fabs(surf.f(up,vp) - zc);
#endif
	sum_err += err;
	sum_err2 += (err*err);
    
	if (err > maxErr) {
	    maxErr = err;
	    zval = surf.f(up,vp);
	    p_maxno = ip+1;
	    up_max = up;
	    vp_max = vp;
	}
	if (zc > zmax) zmax = zc;
	if (zc < zmin) zmin = zc;
    }
  
    double zdiff = zmax - zmin;
    double perc = maxErr/zdiff*100.0;
  
#ifdef WIN32ORSGI
    double rms_err = sqrt(sum_err2/(double)noPoints);
#else
    double rms_err = std::sqrt(sum_err2/(double)noPoints);
#endif
  
    perc = (sum_err/noPoints)/zdiff*100.0;
    cout << "Mean err = " << sum_err/noPoints << " (" << perc << "%)" << endl;
    perc = rms_err/zdiff*100.0;
    cout << "RMS err = "  << rms_err << " (" << perc << "%)" << endl;
}


void MBA::BAalg() {

#ifdef  UNIFORM_CUBIC_C1_SPLINES
    delta_.resize(2*m_+2, 2*n_+2);
    omega_.resize(2*m_+2, 2*n_+2);
#else
    delta_.resize(m_+3, n_+3);
    omega_.resize(m_+3, n_+3);
#endif

    delta_.fill(0);
    omega_.fill(0);
  
    int qwe = 0; 

    int noPoints = data_.size();

    double interval_normalization_factor_u = double(m_) * data_.rangeUInv();
    double interval_normalization_factor_v = double(n_) * data_.rangeVInv();
  
    for (int ip = 0; ip < noPoints; ip++) {

	double uc = (data_.U(ip) - data_.umin()) * interval_normalization_factor_u; 
	double vc = (data_.V(ip) - data_.vmin()) * interval_normalization_factor_v;
      
	int i, j;
	double s, t;
	UCBspl::ijst(m_, n_, uc, vc, i, j, s, t);
      
	double w_kl[4][4];
	int k,l;
	double sum_w_ab2_inv = 0.0; 
	UCBspl::WKLandSum2(s, t, w_kl, sum_w_ab2_inv);
    
	sum_w_ab2_inv = double(1) / sum_w_ab2_inv;

	double zc = data_.Z()[ip];

	for (k = 0; k <= 3; k++) {
	    for (l = 0; l <=3; l++) {
        
		double tmp = w_kl[k][l];

		double phi_kl = tmp * zc * sum_w_ab2_inv;
        

        

		tmp *= tmp;

		delta_(i+k,j+l) += tmp*phi_kl;
		omega_(i+k,j+l) += tmp;
               
	    }
	} 
    }
  
    int noCU, noCV; 
#ifdef  UNIFORM_CUBIC_C1_SPLINES
    noCU = 2*m_ + 2;
    noCV = 2*n_ + 2;
#else
    noCU = m_ + 3;
    noCV = n_ + 3;
#endif
  
    for (int i = -1; i <= noCU-2; i++) {
	for (int j = -1; j <= noCV-2; j++) {
	    double tmp = omega_(i,j);
	    if (tmp != 0.0) { 
		(*PHI_)(i,j) = delta_(i,j)/tmp;
	    }
	    else {
		(*PHI_)(i,j) = 0.0;
		qwe++;
	    }
	}
    }
}

void MBA::MBAalg(int m0, int n0, int h, int num_smoothing) {
  
    if (data_.umin() == MBA_UNDEFREAL)
	data_.initDefaultDomain();
  
    data_.buildBaseSurface();
  
    m_ = m0;
    n_ = n0;
  
    if (PHI_.get() == NULL)
	PHI_.reset(new GenMatrix<UCBspl_real>);

    if (h == 0) {
#ifdef  UNIFORM_CUBIC_C1_SPLINES
	PHI_->resize(2*m0+2, 2*n0+2);
#else
	PHI_->resize(m0+3, n0+3);
#endif
	BAalg();
	smoothZeros(num_smoothing);
	adjustForBaseSurface();
	return;
    }
  
  
    int m_last = m0 * (2<<(h-1));
    int n_last = n0 * (2<<(h-1));

#ifdef  UNIFORM_CUBIC_C1_SPLINES
    int rzU = 2*m_last + 2;
    int rzV = 2*n_last + 2;
#else
    int rzU = m_last + 3;
    int rzV = n_last + 3;
#endif

    GenMatrix<UCBspl_real> PSI;
    GenMatrix<UCBspl_real> PSIprime;

    delta_.reserve(rzU, rzV);
    omega_.reserve(rzU, rzV);
    PHI_->reserve(rzU, rzV);
    PSI.reserve(rzU, rzV);
    PSIprime.reserve(rzU, rzV); 

#ifdef  UNIFORM_CUBIC_C1_SPLINES
    PHI_->resize(2*m0+2,2*n0+2);
    PSI.resize(2*m0+2, 2*n0+2);
    PSIprime.resize(2*m0+2, 2*n0+2);
#else
    PHI_->resize(m0+3,n0+3);
    PSI.resize(m0+3, n0+3);
    PSIprime.resize(m0+3, n0+3);
#endif
  
    int noPoints = data_.size();
    int k = 0;

    while(k <= h) {
      
	BAalg();
	smoothZeros(num_smoothing);
      
	if (k < h) {
	    for (int ip = 0; ip < noPoints; ip++) {
		double u = data_.U(ip);
		double v = data_.V(ip);
		double z = data_.Z()[ip];
		data_.Z()[ip] = z - f_pure(u,v); 
	    }
	}
  

	PSI.swap(*PHI_);  
	if (k != 0) {
	    PSI += PSIprime;   
	}
      
	if (k < h) {
#ifdef  UNIFORM_CUBIC_C1_SPLINES
	    UCBspl::refineCoeffsC1(PSI, PSIprime);
	    PHI_->resize(4*m_+2, 4*n_+2);
	    PSI.resize(4*m_+2, 4*n_+2);
#else
	    UCBspl::refineCoeffsC2(PSI, PSIprime); 
	    PHI_->resize(2*m_+3, 2*n_+3);
	    PSI.resize(2*m_+3, 2*n_+3);
#endif
	  
	    m_ *= 2;
	    n_ *= 2;
	}
      
	k += 1;
    } 
  
    PHI_->swap(PSIprime); 

  
    delta_.clear();
    omega_.clear();
  
    data_.Z().clear();

    adjustForBaseSurface();
}

namespace {
    vector<UCBspl_real> generate_smoothing_filter()
    {


	UCBspl_real data[] = {-1,   24,   14,   24, -1,
			      24,  -56, -176,  -56, 24,
			      14, -176,    0, -176, 14,
			      24,  -56, -176,  -56, 24,
			      -1,   24,   14,   24, -1};

	for (int i = 0; i < 25; ++i) {
	    data[i] /= UCBspl_real(684);
	}

	return vector<UCBspl_real>(data, data+25);
    }

    UCBspl_real extrapolate_point(int i, int j, const GenMatrixType& matrix)
    {
	int resX = matrix.noX();
	int resY = matrix.noY();
	if (i < -1) {
	    if (j < -1) {
		int ix = -i;
		int jx = -j;
		return 
		    (ix+1) * (jx+1) * matrix(-1, -1)
		    - (ix+1) * jx * matrix(0, -1)
		    - ix * (jx+1) * matrix(-1, 0)
		    + ix * jx * matrix(0, 0);

	    } else if (j >= resY - 1) {
		int ix = -i;
		int jx = j - (resY-2);
		return
		    (ix+1) * (jx+1) * matrix(-1, resY-2) 
		    - (ix+1) * jx * matrix(-1, resY-3)
		    - ix * (jx+1) * matrix(0, resY-2)
		    + ix * jx * matrix(0, resY-3);

	    } else {
		int ix = -i;
		return (ix+1) * matrix(0, j) - ix * matrix(1, j);
	    }
	} 

	else if (i >= resX - 1) {
	    if (j < -1) {
		int ix = i - (resX - 2);
		int jx = -j;
		return 
		    (ix+1)*(jx+1) * matrix(resX-2, -1) 
		    - (ix+1) * jx * matrix(resX-2, 0)
		    - ix * (jx+1) * matrix(resX-3, -1)
		    + ix * jx * matrix(resX-3, 0);
	    
	    } else if (j >= resY - 1) {
		int ix = i - (resX - 2);
		int jx = j - (resY - 2);
		return
		    (ix+1)*(jx+1) * matrix(resX-2, resY-2)
		    - (ix+1) * jx * matrix(resX-2, resY-3)
		    - ix * (jx+1) * matrix(resX-3, resY-2)
		    + ix * jx * matrix(resX-3, resY-3);
	    } else {
		int ix = i - (resX - 2);
		return (ix+1) * matrix(resX-2, j) - ix * matrix(resX-3, j);

	    }
	} else { 
	    if (j < -1) {
		int jx = -j;
		return (jx+1) * matrix(i, -1) - jx * matrix(i, 0);
	    } else if (j >= resY - 1) {
		int jx = j - (resY - 2);
		return (jx + 1) * matrix(i, resY-2) - jx * matrix(i, resY-3);

	    } else {
		return matrix(i, j);
	    }
	}
    
    }

}; 


 
