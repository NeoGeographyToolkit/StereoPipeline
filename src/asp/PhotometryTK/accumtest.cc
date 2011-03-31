// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <asp/PhotometryTK/RecursiveBBoxAccumulator.h>

using namespace vw;
using namespace vw::platefile;

using namespace asp::pho;

using namespace std;

template<typename ScalarT, typename ElementT>
class TestAccumFunc : public AccumulatorFunc<ScalarT,ElementT> {
protected:
  ScalarT m_sum;
public:
  TestAccumFunc() : m_sum(0) {}
  ~TestAccumFunc() {}

  TestAccumFunc(TestAccumFunc<ScalarT,ElementT> const& af) : AccumulatorFunc<ScalarT,ElementT>(), m_sum(af.m_sum) { }

  void operator()(ElementT const& e) {
    m_sum += 1;
  }

  void operator()(AccumulatorFunc<ScalarT,ElementT> const& e) {
    m_sum += e.value();
  }

  ScalarT value() const {
    return m_sum;
  }
};

int main( int argc, char *argv[] ) 
{
  TestAccumFunc<int,Vector2i> funcProto;
  
  RecursiveBBoxAccumulator<TestAccumFunc<int,Vector2i> > accum(64, funcProto);

  BBox2i box(0,0,1024,1024);
  TestAccumFunc<int,Vector2i> ret = accum(box);
}
