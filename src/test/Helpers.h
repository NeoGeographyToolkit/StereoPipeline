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


#ifndef __VW_TESTS_CONFIG_TEST_H__
#define __VW_TESTS_CONFIG_TEST_H__

#include <vw/config.h>
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/PixelTypeInfo.h>
#include <vw/Image/PixelMath.h>

#include <gtest/gtest_ASP.h>
#include <cmath>
#include <complex>
#include <string>
#include <queue>
#include <cstdlib>

#include <boost/function.hpp>
#include <boost/range/iterator_range_core.hpp>

#if defined(VW_ENABLE_EXCEPTIONS) && (VW_ENABLE_EXCEPTIONS==1)
#define HAS_EXCEPTIONS(x) x
#else
#define HAS_EXCEPTIONS(x) DISABLED_ ## x
#endif

#if defined(VW_ENABLE_CONFIG_FILE) && (VW_ENABLE_CONFIG_FILE==1)
#define HAS_CONFIG_FILE(x) x
#else
#define HAS_CONFIG_FILE(x) DISABLED_ ## x
#endif

namespace gi = ::testing::internal;

namespace vw { namespace test { }}
namespace t  = vw::test;

namespace vw {
  namespace test {

using namespace ::testing;

#ifndef TEST_OBJDIR
#error TEST_OBJDIR is not defined! Define it before including this header.
#endif

// Create a temporary filename that is unlinked when constructed and destructed
class UnlinkName : public std::string {
  public:
    UnlinkName() {}
    UnlinkName(const std::string& base, const std::string& directory=TEST_OBJDIR);
    UnlinkName(const char *base,        const std::string& directory=TEST_OBJDIR);
    ~UnlinkName();
};

// A getenv with a default value
std::string getenv2(const char *key, const std::string& Default);

// Fetch the seed we used. You should only need to do this if you're
// initializing a different random number generator (i.e. boost::rand48). Do
// NOT use this to reseed a global random number generator.
uint32 get_random_seed();

// reduce the damage from using gtest internal bits, and make sure uint8 is
// seen as numeric.
template <typename T>
::std::string format(const T& x) {
  return PrintToString(_numeric(x));
}

// Value diffs
template <typename ElemT, typename Elem2T>
double value_diff(const vw::PixelMathBase<ElemT>& a, const vw::PixelMathBase<Elem2T>& b) {
  BOOST_STATIC_ASSERT((boost::is_same<ElemT, Elem2T>::value));
  typedef typename CompoundChannelType<ElemT>::type channel_type;
  double acc = 0.0;
  for( size_t c=0; c < PixelNumChannels<ElemT>::value; ++c ) {
    channel_type const& a_x = compound_select_channel<channel_type const&>(a.impl(),c);
    channel_type const& b_x = compound_select_channel<channel_type const&>(b.impl(),c);
    double diff = double(a_x) - double(b_x);
    acc += diff*diff;
  }
  return ::sqrt(acc);
}

template <typename T1, typename T2>
double value_diff(const std::complex<T1>& a, const std::complex<T2>& b) {
  return std::abs(std::complex<double>(a) - std::complex<double>(b));
}

template <typename T1, typename T2>
struct both_are_arithmetic : boost::mpl::and_<boost::is_arithmetic<T1>, boost::is_arithmetic<T2> > {};

template <typename T1, typename T2>
typename boost::enable_if<both_are_arithmetic<T1,T2>, double>::type
value_diff(const T1& a, const T2& b) {
  BOOST_STATIC_ASSERT(boost::is_arithmetic<T1>::value);
  BOOST_STATIC_ASSERT(boost::is_arithmetic<T2>::value);
  return ::fabs(double(a) - double(b));
}

// A version of std::mismatch that returns the set of differences rather than
// just the first
template <class InputIterator1, class InputIterator2, class Pred>
void mismatch_queue(InputIterator1 first1, InputIterator1 last1, InputIterator2 first2,
                    std::queue<std::pair<InputIterator1, InputIterator2> >& answer,
                    const Pred& p)
{
  while ( first1!=last1 ) {
    if (!p(*first1, *first2))
      answer.push(std::make_pair(first1, first2));
    ++first1; ++first2;
  }
}

template <typename ImplT>
class CmpWorker {
  public:
    inline ImplT& impl() { return static_cast<ImplT&>(*this); }
    inline ImplT const& impl() const { return static_cast<ImplT const&>(*this); }

    template <typename T1, typename T2>
    bool operator()(const T1& e, const T2& a) const {return impl()(e, a);}

    template <typename T1, typename T2>
    Message what(const std::string& ename, const std::string& aname, const T1& e, const T2& a) const {
      return impl().what(ename, aname, e, a);
    }
};

class CmpEqual : public CmpWorker<CmpEqual> {
  public:
    template <typename T1, typename T2>
    bool operator()(const T1& a, const T2& b) const { return a == b; }

    template <typename T1, typename T2>
    Message what(const std::string& ename, const std::string& aname, const T1& e, const T2& a) const {
      return Message() << gi::EqFailure(ename.c_str(), aname.c_str(), t::format(e), t::format(a), false).message();
    }
};

class CmpTypeEqual : public CmpWorker<CmpTypeEqual> {
  public:
    template <typename T1, typename T2>
    bool operator()(const T1& a, const T2& b) const {
      return boost::is_same<T1,T2>::value && a == b;
    }

    template <typename T1, typename T2>
    Message what(const std::string& ename, const std::string& aname, const T1& e, const T2& a) const {
      if (!boost::is_same<T1,T2>::value)
        return Message() << ename << " and " << aname << " are not the same type";
      return Message() << gi::EqFailure(ename.c_str(), aname.c_str(), t::format(e), t::format(a), false).message();
    }
};

class CmpNear : public CmpWorker<CmpNear> {
    const char *dexpr;
    const double& delta;
  public:
    CmpNear(const char* dexpr, const double& delta) : dexpr(dexpr), delta(delta) {}

    template <typename T1, typename T2>
    bool operator()(const T1& a, const T2& b) const { return value_diff(a, b) <= delta; }

    template <typename T1, typename T2>
    Message what(const std::string& ename, const std::string& aname, const T1& e, const T2& a) const {
      Message msg;
      msg << "The difference between "
          << ename   << " and " << aname
          << " is "  << t::format(value_diff(e, a))
          << ", which exceeds " << dexpr << ", where\n"
          << ename << " evaluates to " << t::format(e) << ",\n"
          << aname << " evaluates to " << t::format(a)
          << ", and\n" << dexpr << " evaluates to " << t::format(delta) << ".";
      return msg;
    }
};

#if 0
template <typename ExpectT>
struct CmpULP : public CmpWorker<CmpULP<ExpectT> > {
  BOOST_STATIC_ASSERT(boost::is_floating_point<ExpectT>::value);
  public:
    template <typename ActualT>
    bool operator()(const ExpectT& a, const ActualT& b) const {
      const gi::FloatingPoint<ExpectT> lhs(a), rhs(b);
      return lhs.AlmostEquals(rhs);
    }

    template <typename ActualT>
    Message what(const std::string& ename, const std::string& aname, const ExpectT& e, const ActualT& a) const {
      std::ostringstream es, as;
      es << std::setprecision(std::numeric_limits<ExpectT>::digits10 + 2) << e;
      as << std::setprecision(std::numeric_limits<ExpectT>::digits10 + 2) << a;
      return Message() << gi::EqFailure(ename.c_str(), aname.c_str(), es.str(), as.str(), false);
    }
};
#endif

template <typename CmpT>
class _CheckOne {
    const CmpT& cmp;
  public:
    _CheckOne() : cmp(CmpT()) {}
    _CheckOne(const CmpT& cmp) : cmp(cmp) {}

    template <typename ExpectT, typename ActualT>
    AssertionResult operator()(const char* ename, const char* aname, const ExpectT& e, const ActualT& a) const
    {
      if (cmp(e, a))
        return AssertionSuccess();
      return AssertionFailure() << cmp.what(ename, aname, e, a);
    }
};
template <typename CmpT>
_CheckOne<CmpT> check_one(const CmpT& cmp) {
  return _CheckOne<CmpT>(cmp);
}

template <typename CmpT>
class _CheckRange {
    const CmpT& cmp;
  public:
    _CheckRange() : cmp(CmpT()) {}
    _CheckRange(const CmpT& cmp) : cmp(cmp) {}

    template <typename Range1T, typename Range2T>
    AssertionResult operator()(const char* ename, const char* aname,
                               const Range1T& e, const Range2T& a) const
    {
      if (a.size() != e.size())
        return AssertionFailure()
                << "Iterator ranges (" << ename << ") and (" << aname
                << ") represent different-sized ranges.";

      typedef std::pair<typename Range1T::const_iterator, typename Range2T::const_iterator> pair_t;

      std::queue<pair_t> ret;
      mismatch_queue(e.begin(), e.end(), a.begin(), ret, cmp);

      if (ret.empty())
        return AssertionSuccess();

      Message msg;
      bool need_newline = false;
      while (!ret.empty()) {
        const pair_t& r = ret.front();
        const std::string idx = "[" + stringify(std::distance(e.begin(), r.first)) + "]";
        if (need_newline)
          msg << std::endl;
        msg << cmp.what(ename + idx, aname + idx, *(r.first), *(r.second));
        need_newline = true;
        ret.pop();
      }
      return AssertionFailure(msg);
    }

    template <typename Iter1T, typename Iter2T>
    AssertionResult operator()(const char* e0name, const char* /*e1name*/,
                               const char* a0name, const char* /*a1name*/,
                               const Iter1T& e0, const Iter1T& e1,
                               const Iter2T& a0, const Iter2T& a1) const
    {
      return this->operator()(e0name, a0name, boost::make_iterator_range(e0, e1), boost::make_iterator_range(a0, a1));
    }
};

template <typename CmpT>
_CheckRange<CmpT> check_range(const CmpT& cmp) {
  return _CheckRange<CmpT>(cmp);
}

template <typename CmpT>
class _CheckNDRange {
    const CmpT& cmp;

    typedef Vector<size_t, 0> SVec;

    template <typename T>
    SVec size_helper(const MatrixBase<T>& m) {return Vector<size_t, 2>(m.impl().rows(), m.impl().cols());}

    template <typename T>
    SVec size_helper(const ImageViewBase<T>& m) {return Vector<size_t, 3>(m.impl().planes(), m.impl().rows(), m.impl().cols());}

    template <typename T>
    SVec size_helper(const VectorBase<T>& m) {return m.size();}

    SVec stride_from_size(const SVec& size) {
      SVec stride(size.size());
      stride(size.size()-1) = 1;
      for (ssize_t i = stride.size()-2; i >= 0; --i)
        stride[i] = size[i+1] * stride[i+1];
      return stride;
    }

    template <typename IterT1>
    std::string to_idx(const SVec& stride, const IterT1& begin, const IterT1& wrong) {
      std::ostringstream ss;
      ss << "[";
      bool comma = false;
      ssize_t d = std::distance(begin, wrong);
      for (size_t i = 0; i < stride.size(); ++i) {
        if (comma)
          ss << ',';
        ss << size_t(d / stride[i]);
        d = d % stride[i];
        comma = true;
      }
      ss << "]";
      return ss.str();
    }

  public:
    _CheckNDRange() : cmp(CmpT()) {}
    _CheckNDRange(const CmpT& cmp) : cmp(cmp) {}

    template <typename T1, typename T2>
    AssertionResult operator()(const char* ename, const char* aname, const T1& e, const T2& a)
    {
      SVec esize = size_helper(e), asize = size_helper(a);

      if ( esize != asize )
        return AssertionFailure() << "Cannot compare " << ename << " and " << aname << ": Different size: "
                                  << esize << " != " << asize;

      typedef std::pair<typename T1::const_iterator, typename T2::const_iterator> pair_t;
      std::queue<pair_t> ret;
      mismatch_queue(e.begin(), e.end(), a.begin(), ret, cmp);

      if (ret.empty())
        return AssertionSuccess();

      SVec stride = stride_from_size(esize);

      Message msg;
      bool need_newline = false;
      while (!ret.empty()) {
        const pair_t& r = ret.front();
        const std::string idx = to_idx(stride, e.begin(), r.first);
        if (need_newline)
          msg << std::endl;
        msg << cmp.what(ename + idx, aname + idx, *(r.first), *(r.second));
        need_newline = true;
        ret.pop();
      }
      return AssertionFailure(msg);
    }
};

template <typename CmpT>
_CheckNDRange<CmpT> check_nd_range(const CmpT& cmp) {
  return _CheckNDRange<CmpT>(cmp);
}

#define EXPECT_RANGE_EQ(expect0, expect1, actual0, actual1) \
  EXPECT_PRED_FORMAT4(t::check_range(t::CmpEqual()), expect0, expect1, actual0, actual1)
#define ASSERT_RANGE_EQ(expect0, expect1, actual0, actual1) \
  ASSERT_PRED_FORMAT4(t::check_range(t::CmpEqual()), expect0, expect1, actual0, actual1)
#define EXPECT_RANGE_NEAR(expect0, expect1, actual0, actual1, delta) \
  EXPECT_PRED_FORMAT4(t::check_range(t::CmpNear(#delta, delta)), expect0, expect1, actual0, actual1)
#define ASSERT_RANGE_NEAR(expect0, expect1, actual0, actual1, delta) \
  ASSERT_PRED_FORMAT4(t::check_range(t::CmpNear(#delta, delta)), expect0, expect1, actual0, actual1)

#define EXPECT_SEQ_EQ(expect, actual)\
  EXPECT_PRED_FORMAT2(t::check_nd_range(t::CmpEqual()), expect, actual)
#define ASSERT_SEQ_EQ(expect, actual)\
  ASSERT_PRED_FORMAT2(t::check_nd_range(t::CmpEqual()), expect, actual)
#define EXPECT_SEQ_NEAR(expect, actual, delta)\
  EXPECT_PRED_FORMAT2(t::check_nd_range(t::CmpNear(#delta, delta)), expect, actual)
#define ASSERT_SEQ_NEAR(expect, actual, delta)\
  ASSERT_PRED_FORMAT2(t::check_nd_range(t::CmpNear(#delta, delta)), expect, actual)

#define EXPECT_VECTOR_EQ(expect, actual)\
  EXPECT_PRED_FORMAT2(t::check_range(t::CmpEqual()), expect, actual)
#define ASSERT_VECTOR_EQ(expect, actual)\
  ASSERT_PRED_FORMAT2(t::check_range(t::CmpEqual()), expect, actual)
#define EXPECT_VECTOR_NEAR(expect, actual, delta)\
  EXPECT_PRED_FORMAT2(t::check_range(t::CmpNear(#delta, delta)), expect, actual)
#define ASSERT_VECTOR_NEAR(expect, actual, delta)\
  ASSERT_PRED_FORMAT2(t::check_range(t::CmpNear(#delta, delta)), expect, actual)

#define EXPECT_TYPE_EQ( expect, actual )\
  EXPECT_PRED_FORMAT2(t::check_one(t::CmpTypeEqual()), expect, actual )
#define ASSERT_TYPE_EQ( expect, actual )\
  ASSERT_PRED_FORMAT2(t::check_one(t::CmpTypeEqual()), expect, actual )

#define EXPECT_PIXEL_NEAR(expect, actual, delta)\
  EXPECT_PRED_FORMAT2(t::check_one(t::CmpNear(#delta, delta)), expect, actual)
#define ASSERT_PIXEL_NEAR(expect, actual, delta)\
  ASSERT_PRED_FORMAT2(t::check_one(t::CmpNear(#delta, delta)), expect, actual)

#define EXPECT_VW_EQ(expect, actual)\
  EXPECT_PRED_FORMAT2(t::check_one(t::CmpTypeEqual()), expect, actual)
#define ASSERT_VW_EQ(expect, actual)\
  ASSERT_PRED_FORMAT2(t::check_one(t::CmpTypeEqual()), expect, actual)

#define VW_TEST_THROW_(statement, expected_exception, expected_substr, fail) \
  GTEST_AMBIGUOUS_ELSE_BLOCKER_ \
  if (::testing::internal::ConstCharPtr gtest_msg = "") { \
    bool gtest_caught_expected = false; \
    try { \
      GTEST_SUPPRESS_UNREACHABLE_CODE_WARNING_BELOW_(statement); \
    } \
    catch (expected_exception const& e) { \
      gtest_caught_expected = true; \
      GTEST_PRED_FORMAT2_(t::IsSubstring, expected_substr, e.what(), fail);\
    } \
    catch (...) { \
      gtest_msg.value = \
          "Expected: " #statement " throws an exception of type " \
          #expected_exception ".\n  Actual: it throws a different type."; \
      goto GTEST_CONCAT_TOKEN_(gtest_label_testthrow_, __LINE__); \
    } \
    if (!gtest_caught_expected) { \
      gtest_msg.value = \
          "Expected: " #statement " throws an exception of type " \
          #expected_exception ".\n  Actual: it throws nothing."; \
      goto GTEST_CONCAT_TOKEN_(gtest_label_testthrow_, __LINE__); \
    } \
  } else \
    GTEST_CONCAT_TOKEN_(gtest_label_testthrow_, __LINE__): \
      fail(gtest_msg.value)

// these are unlikely to work right in death tests or other weird circumstances.
#define EXPECT_THROW_MSG(statement, expected_exception, expected_substr) \
  VW_TEST_THROW_(statement, expected_exception, expected_substr, GTEST_NONFATAL_FAILURE_)
#define ASSERT_THROW_MSG(statement, expected_exception, expected_substr) \
  VW_TEST_THROW_(statement, expected_exception, expected_substr, GTEST_FATAL_FAILURE_)

// DEPRECATED
#define EXPECT_MATRIX_FLOAT_EQ(e, a)          EXPECT_MATRIX_NEAR(e, a, 1e20)
#define EXPECT_MATRIX_DOUBLE_EQ(e, a)         EXPECT_MATRIX_NEAR(e, a, 1e45)
#define EXPECT_COMPLEX_MATRIX_NEAR(e, a, d)   EXPECT_MATRIX_NEAR(e, a, d)
#define EXPECT_VECTOR_FLOAT_EQ(e, a)          EXPECT_VECTOR_NEAR(e, a, 1e20)
#define EXPECT_VECTOR_DOUBLE_EQ(e, a)         EXPECT_VECTOR_NEAR(e, a, 1e45)
#define EXPECT_PIXEL_EQ(e, a)                 EXPECT_TYPE_EQ(e,a)
#define ASSERT_PIXEL_EQ(e, a)                 ASSERT_TYPE_EQ(e,a)
#define EXPECT_MATRIX_EQ(e, a)                EXPECT_SEQ_EQ(e,a)
#define ASSERT_MATRIX_EQ(e, a)                ASSERT_SEQ_EQ(e,a)
#define EXPECT_MATRIX_NEAR(e, a, delta)       EXPECT_SEQ_NEAR(e,a,delta)
#define ASSERT_MATRIX_NEAR(e, a, delta)       ASSERT_SEQ_NEAR(e,a,delta)

}} // namespace t

#endif
