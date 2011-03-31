// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#ifndef __ASP_PHO_PARTITIONED_SUM_H__
#define __ASP_PHO_PARTITIONED_SUM_H__

#include <vw/Plate/TileManipulation.h>
using namespace vw;
using namespace vw::platefile;

namespace asp {
namespace pho {

  /**
   *       [a b c d e f g h i j]
   *   [a b c d e]   +   [f g h i j]
   * [a b c] + [d e] + [f g h] + [i j]
   * 
   */
  template<typename ScalarT, typename ElementT>
  class AccumulatorFunc : public vw::ReturnFixedType<void> {
  public:
    AccumulatorFunc() {}

    virtual ~AccumulatorFunc() {}

    // Subclasses should call parent method
    virtual void operator()(ElementT const& e)=0;

    virtual void operator()(AccumulatorFunc const& e)=0;
    
    virtual ScalarT value() const=0;
  };

  template<typename AccumFuncT>
  class RecursiveBBoxAccumulator {
  protected:
    int m_accumListMaxSize;
    AccumFuncT m_funcProto;

    // Debugging only - remove once tested
    int m_recurseLevel;

  public:
  RecursiveBBoxAccumulator(int accumListMaxSize, AccumFuncT const& func) : m_accumListMaxSize(accumListMaxSize), m_funcProto(func), m_recurseLevel(-1) {}

    AccumFuncT operator()(BBox2i const& box) {
      
      m_recurseLevel++;

      int tileCount = box.width() * box.height();

      if (tileCount <= m_accumListMaxSize) {
	AccumFuncT f(m_funcProto);

	for(int32 ix = box.min().x(); ix < box.max().x(); ix++) {
	  for(int32 iy = box.min().y(); iy < box.max().y(); iy++) {
	    Vector2i tile(ix, iy);
	    f(tile);
	    //std::cout << "adding value to func\n";
	  }
	}

	std::cout << "sum at level [" << m_recurseLevel << "] = " << f.value() << "\n";

	return f;
      }
      else {
	std::list<BBox2i> boxes = bbox_tiles(box, box.width()/2, box.height()/2);

	//std::cout << "boxes=" << boxes.size() << "\n";

	std::list<BBox2i>::iterator iter;
	AccumFuncT f(m_funcProto);
	for(iter = boxes.begin(); iter != boxes.end(); iter++) {
	  f( (*this)(*iter) );
	  //std::cout << "adding value to func\n";
	}

	std::cout << "sum at level [" << m_recurseLevel << "] = " << f.value() << "\n";
	
	if (m_recurseLevel > 0)
	  m_recurseLevel--;
	
	return f;
      }
    }
  };

}} // end namespace asp::pho

#endif//__ASP_PHO_PARTITIONED_SUM_H__
