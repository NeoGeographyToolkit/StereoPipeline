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
#ifndef UCB_SPLINE_SURFACE
#define UCB_SPLINE_SURFACE

#include <boost/shared_ptr.hpp>

#include <UCBtypedef.h>
#include <GenMatrix.h>


namespace UCBspl {
  
  /** \brief Represents a uniform cubic B-spline surface
   *
   *  SplineSurface - A uniform cubic B-spline surface compatible with that
   *  produced by the SINTEF MBA library.
   * \author Øyvind Hjelle <Oyvind.Hjelle@math.sintef.no>
   */
  class SplineSurface {
    typedef GenMatrix<UCBspl_real> GenMatrixType;
    boost::shared_ptr<GenMatrixType> PHI_;

    double umin_;
    double vmin_;
    double umax_;
    double vmax_;
    // bool C2_;
  public:
    

   /** Default constructor makes the domain over the unit square, but
    *  coefficient matrix is not allocated.
    */
    SplineSurface() {umin_=vmin_=0.0; umax_=vmax_=1.0;} // unit square

   /** Constructor with (standard) shared pointers to the uniform tensor product grid,
    * and the domain.
    */
    SplineSurface(boost::shared_ptr<GenMatrixType> PHI,
                  double umin, double vmin, 
                  double umax, double vmax);
    
    /** Copy constructor */
    SplineSurface(const SplineSurface& surf);

    ~SplineSurface(){}


    /** Initialization similar to constructor */
    void init(boost::shared_ptr<GenMatrixType> PHI,
              double umin, double vmin, 
              double umax, double vmax);

   /** The domain over which the spline surface is defined. */
    void getDomain(double& umin, double& vmin, double& umax, double& vmax) const
                  {umin = umin_; vmin = vmin_; umax = umax_; vmax = vmax_;}

    // Convenience
    double umin() const {return umin_;}
    double vmin() const {return vmin_;}
    double umax() const {return umax_;}
    double vmax() const {return vmax_;}

    /** Evaluates the functional value of the surface in position (u,v)
    * (u,v) must be inside the domain
    */
    double f(double u, double v) const;
    
    /** Evaluates the functional value of the surface in position (i,j),
    * where (i,j) is an index in the spline coefficient matrix.
    * (i,j) must be inside the domain of the index values:
    * 0 <= i <= m and  0 <= j <= n, where m_ and n_ are retrieved by
    * the function getIndexDomain
    * This is much faster than f(u,v) since pre-evaluate bases are used.
    */
    double f(int i, int j) const;
        
    /** Evaluates the normal vector (normalized with length=1) in position in position (u,v).
    * (u,v) must be inside the domain, see getDomain
    */
    void normalVector(double u, double v, double& gx, double& gy, double& gz) const;
    
    /** Same as gradf(double u, double v,...), in a grid point. 
    * See also documentation of f(int i, int j).
    */
    void normalVector(int i, int j, double& gx, double& gy, double& gz) const;
    
    
    // The following three functions are not properly tested and are thus
    // not tagged for documentation with doxygen
    // ------------------------------------------------------------------
    
    /* Evaluates derivatives in x- and y-direction in position (u,v).
    * (u,v) must be inside the domain, see getDomain.
    */
    void derivatives(double u, double v, double& dx, double& dy) const;
    
    /* Evaluates second derivatives in x- and y-direction in position (u,v).
    * (u,v) must be inside the domain, see getDomain.
    */
    void secondDerivatives(double u, double v, double& ddx, double& ddy, double& dxdy) const;
    
    /* Evaluates both profile- and planform curvature in position (u,v).
    * (u,v) must be inside the domain, see getDomain.
    */
    void curvatures(double u, double v, double& profC, double& planC) const;
    
    /** Evaluates both function value and normalized normal vector.
    * (u,v) must be inside the domain, see getDomain.
    * This is faster than calling the functions above separately,
    * e.g., when making triangle strips for visualization.
    */
    void eval(double u, double v, double& z, double& gx, double& gy, double& gz) const;
    
    /** Same as eval(double u, double v,...), in a grid point. 
    * See also documentation of UCBsplineSurface::f (int i, int j).
    */
    void eval(int i, int j, double& z, double& gx, double& gy, double& gz) const;
    
    /** Get the coefficient grid of the tensor product spline surface. */
    const boost::shared_ptr<GenMatrixType> getCoefficients() const {return PHI_;}
    
    /** Index domain of spline coefficient matrix.
    * The surface can also be evaluated by f(i,j) and other functions with
    * 0 <= i <= m and  0 <= j <= n.
    * Note that the size of the tensor product grid (in the C2 case) is
    * [(m+3) x (n+3)]
    */
    void getIndexDomain(int& m, int& n) const {m = PHI_->noX()-3; n = PHI_->noY()-3;}

	/** Refine spline surface to the next finer dyadic level.
	*   That is, make the tensor product grid twice as dense inside the active domain.
	*   Only the mathematical representation of the spline surface is changed, 
	*   and not the geometry.
	*   (Simlar to the Oslo Algorithm.)
	*/
	void refineCoeffs();
	
	/** Restrict the spline surface to the next coarser dyadic level.
	*   That is, make the tensor product grid twice as coarse inside the active domain.
	*   Both the mathematical representation and the geometry of the spline surface is changed.
	*   The effect is that the spline surfaces looks smoother.
	*   This operator has the so-called variational property with respect to the refinement operator
	*   above.
	*
	*   \note
	*   Restriction can only be done if the modulo of (noX-3)/2 and (noY-3)/2 are both zero.
	*   Here noX and noY is the size of the coefficient matrix.
	*
	*   \retval bool \c false if the coefficient grid does not meet the constraints above
	*/
    bool restrictCoeffs();
  };
  
}; // end namespace

#endif
