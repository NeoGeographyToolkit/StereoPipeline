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
#ifndef _MBA_H_
#define _MBA_H_

#include <MBAtypedef.h>
#include <UCBtypedef.h>
#include <GenMatrix.h>
#include <MBAdata.h>

#include <vector>
#include <boost/shared_ptr.hpp>

#include <UCBsplineSurface.h>
//===========================================================================
/** \brief \b Main \b interaface \b for \b multilevel \b B-spline \b approximation
 *
 * MBA - Multilevel B-spline approximation (and interpolation).
 * This is the main interface to multilevel B-spline approximation of scattered
 * data for functional surfaces. The method is based on B-spline refinement
 * and produces one single B-spline surface, as opposed to MBAadaptive that
 * produces a hierarchy of spline surfaces.
 * The domain of the surface will be the rectangular domain of the given scattered
 * data in the (x,y)-axes directions as default.
 * (We use \em u and \em v for \em x and \em y in function headers)
 * The function MBA::MBAalg produces the spline surface from the scattered data.
 * The spline surface can be retrieved as a SplineSurface object with
 * MBA::getSplineSurface.
 * Note that this is a (mathematical) lower level interface; thus there is no check
 * for the validity of given scattered data or of arguments passed to
 * member functions.
 *
 * \anchor mba_example1
 * Examle of use (minimal):
 * \code
   MBA mba(x_arr, y_arr, z_arr);      // initialize with scattered data
   mba.MBAalg(1,1,7);                 // create spline surface

   UCBspl::SplineSurface surf = mba.getSplineSurface();
   double z = surf.f(x,y);            // evaluate (height of) surface in (x,y)
   surf.normalVector(x,y, nx,ny,nz);  // evaluate normal vector in (x,y)
   \endcode
 */
//===========================================================================
class MBA {
  
  MBAdata data_;
  int m_,n_; // the lattice is from -1,0,...,m_+1  -1,0,...,n_+1
  boost::shared_ptr<GenMatrixType> PHI_;

  static const std::vector<UCBspl_real> smoothing_filter_;

  GenMatrix<UCBspl_real> delta_; // temporary array for BA/MBA algorithm
  GenMatrix<UCBspl_real> omega_; // temporary array for BA/MBA algorithm
  
  void BAalg();
  double f_pure(double u, double v) const; // without base surface, used in MBAalg

  // Smoothing
  void flagZeros(GenMatrix<bool>& qwe_) const;

  /* Update spline coefficients by adding the base surface over which the surface is defined.
   * This ensures that PHI() returns the correct coefficients of the spline surface.
   * This is always run after MBAalg.
   */
  bool adjustForBaseSurface();

public:

  MBA(){};

  /** Constructor with (standard) shared pointers to scattered data
    */
  MBA(boost::shared_ptr<dVec> U, boost::shared_ptr<dVec> V, boost::shared_ptr<dVec> Z)
     {data_.init(U, V, Z);}
    
 ~MBA(){}

  /** Initialization that takes reference to arrays of scattered data.
    * Note: see documenation of corresponding constructor above.
    */
  void init(boost::shared_ptr<dVec> U, boost::shared_ptr<dVec> V, boost::shared_ptr<dVec> Z)
     {data_.init(U, V, Z);}

  /** Initialization of surface using data retrieved by getSplineSurface()
    */
  void init(UCBspl::SplineSurface& surf);


  /* Expand the rectangular domain of the surface beyond the domain of the given 
   *  scattered data.
   *  The given arguments must be grater than or equal to zero;
   *  e.g. \a Dumin is the increment in the negative u-direction.
   *  (If used, this must be done before creating the surface.)
   */
  //void expandDomain(double Dumin, double Dvmin, double Dumax, double Dvmax)
  //                  {data_.expandDomain(Dumin, Dvmin, Dumax, Dvmax);}

  /** Set the domain over which the surface is defined.
    * The default is the xy-range of the scattered data.
    *
    * \note This function can only be used to expand the domain boyond the xy-range
    *       of the scattered data. It is the users responsibility to check that
    *       no scattered data falls outside the domain.
    *       (use std::min_element and std::max_element to find range of data for std::vector)
    */
  void setDomain(double umin, double vmin, double umax, double vmax)
                 {data_.setDomain(umin,vmin,umax,vmax);}

  /** Set surface base.
   *  Since the multilevel spline method is not affine-invariant, it is
   *  recommended to set a base surface over which the approximation is
   *  created.
   *  \param MBA_ZERO:     A planar surface at level zero.
   *  \param MBA_CONSTLS:  A konstant (least square) surface at the level of the mean z-value (default and recommended).
   *  \param MBA_CONSTVAL: A konstant surface at the level as given by setBaseValue.
   */
  void setBaseType(MBAbaseType baseType) {data_.baseType_ = baseType;}
  
  /// Level of the base surface if MBA_CONSTVAL is used as base type.
  void setBaseValue(double base) {data_.offset_ = base; data_.baseType_ = MBA_CONSTVAL;}
  
  /** Create a B-spline approximation to the scattered data.
   *
   *  \param m0,n0 (>=1): The initial size of the spline space in the hierarchical construction.
   *               If the rectangular domain is a square, m0=n0=1 is recommended.
   *               If the rectangular domain in the y-direction is twice of that
   *               the x-direction, m0=1, n0=2 is recommended.
   *               In general, if the rectangular domain in the y-direction is 
   *               k times the length in the x-direction, m0=1, n0=k is recommended.
   *               
   *
   *  \param h:    Number of levels in the hierarchical construction.
   *               If, e.g., m0=n0=1 and h=8, The resulting spline surface has a
   *               coefficient grid of size 2^h + 3 = 259 in each direction if
   *               a cubic C^2 spline surface is created.
   *               (Currently a compile option \c UNIFORM_CUBIC_C1_SPLINES can be set in \c MBAunifBsplines.h
   *               to create a cubic C^1 surface. In this case the coefficient grid
   *               will be approx. twice the size of that in the C^2 case when using
   *               the same h value.)
   */
  void MBAalg(int m0, int n0, int h = 0, int smoothing_steps = 0);    
  
  /// Get the data object (See class MBAdata)
	const MBAdata& getData() const {return data_;}

  /// Read scattered data from file in the format u v z on each line.
  void readScatteredData(char filename[]) {data_.readScatteredData(filename);}

  /// Smooth the spline surface without affecting the approximation accuracy
  void smoothZeros(int no_iter);

  /** Retrieve the spline surface. 
    */
  UCBspl::SplineSurface getSplineSurface() const {return UCBspl::SplineSurface(PHI_, data_.umin(), data_.vmin(),
                                                  data_.umax(), data_.vmax());}

  /** Index domain of spline coefficient matrix.
    * The surface can also be evaluated by f(i,j) and other functions with
    * 0 <= i <= m and  0 <= j <= n.
    * Note that the size of the tensor product grid (in the C2 case) is
    * (m+3) x (n+3)

    */
  void getIndexDomain(int& m, int& n) const {m = m_; n = n_;}

  /** Clean-up array structures and reduce memory usage.
    * Depending on the given \a type described below, temporary array structures
    * will be deleted.
    * All options preserve information necessary to evaluate the surface.
    * \param 0: Temporary (mathematical) stuff not used further after MBA::MBAalg.
    * \param 1: The scattered data arrays and data derived from them.
    * \param 2: Everything, including that above, that is not needed by evaluators.
    */
  void cleanup(int type = 0);

  /** Get the coefficient grid of the tensor product spline surface.
    */
  boost::shared_ptr<GenMatrixType> PHI() const {return PHI_;}

  // (Temporary) utilities
  void checkSparsity() const;
  //void printSplineSurface(char filename[]) const;
  void checkError() const;

  static void smoothMatrix(GenMatrixType& matrix, int no_iter);

};

#endif
