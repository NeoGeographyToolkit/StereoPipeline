// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <gtest/gtest.h>
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

  void reset() {
    m_sum = 0;
  }
};

class RecursiveBBoxAccumulatorTest : public ::testing::Test {
protected:
  RecursiveBBoxAccumulatorTest() {}

  virtual void SetUp() {
    int tiles = 1024*1024;
    for(int i = 0; i < 9; i++) {
      int size = 1<<(2*i);
      levelSizeMap[i] = size;
      levelValMap[i] = tiles/size;
    }
  }

  std::map<int,int> levelSizeMap;
  std::map<int,int> levelValMap;
};

TEST_F( RecursiveBBoxAccumulatorTest, SumOnes ) {
  TestAccumFunc<int,Vector2i> funcProto;
  
  RecursiveBBoxAccumulator<TestAccumFunc<int,Vector2i> > accum(16, funcProto);

  BBox2i box(0,0,1024,1024);
  TestAccumFunc<int,Vector2i> ret = accum(box);

  std::map<int,std::vector<double> > rt = accum.getRecurseTracking();
  std::map<int,std::vector<double> >::iterator iter;
  for(iter = rt.begin(); iter != rt.end(); iter++) {
    int level = (*iter).first;
    std::vector<double> vals = (*iter).second;

    EXPECT_EQ( levelSizeMap[level], vals.size() );
    EXPECT_EQ( levelValMap[level], vals[0] );

    std::cout << "[" << level << "] = (" << vals.size() << " * [" << vals[0] << "] )\n";
  }  
}
