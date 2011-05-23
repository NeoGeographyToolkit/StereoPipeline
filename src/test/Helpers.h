// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __VW_TESTS_CONFIG_TEST_H__
#define __VW_TESTS_CONFIG_TEST_H__

#include <cmath>
#include <complex>
#include <string>
#include <boost/function.hpp>

#include <vw/config.h>
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/PixelTypeInfo.h>
#include <vw/Image/PixelMath.h>

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

namespace vw {
  namespace test {

using namespace ::testing;

#ifndef TEST_SRCDIR
#error TEST_SRCDIR is not defined! Define it before including this header.
#endif

// Create a temporary filename that is unlinked when constructed and destructed
class UnlinkName : public std::string {
  public:
    UnlinkName() {}
    UnlinkName(std::string base, std::string directory=TEST_SRCDIR);
    ~UnlinkName();
};

#define EXPECT_MATRIX_NEAR(val1, val2, delta)\
  EXPECT_PRED_FORMAT2(vw::test::MatrixHelper<double>(vw::test::NearImpl(#delta, delta)), val1, val2)
#define ASSERT_MATRIX_NEAR(val1, val2, delta)\
  ASSERT_PRED_FORMAT2(vw::test::MatrixHelper<double>(vw::test::NearImpl(#delta, delta)), val1, val2)

#define EXPECT_MATRIX_DOUBLE_EQ(val1, val2)\
  EXPECT_PRED_FORMAT2(vw::test::MatrixHelper<double>(vw::test::ULPEq<double>()), val1, val2)
#define ASSERT_MATRIX_DOUBLE_EQ(val1, val2)\
  ASSERT_PRED_FORMAT2(vw::test::MatrixHelper<double>(vw::test::ULPEq<double>()), val1, val2)

#define EXPECT_MATRIX_FLOAT_EQ(val1, val2)\
  EXPECT_PRED_FORMAT2(vw::test::MatrixHelper<float>(vw::test::ULPEq<float>()), val1, val2)
#define ASSERT_MATRIX_FLOAT_EQ(val1, val2)\
  ASSERT_PRED_FORMAT2(vw::test::MatrixHelper<float>(vw::test::ULPEq<float>()), val1, val2)

#define EXPECT_VECTOR_NEAR(val1, val2, delta)\
  EXPECT_PRED_FORMAT2(vw::test::VectorHelper<double>(vw::test::NearImpl(#delta, delta)), val1, val2)
#define ASSERT_VECTOR_NEAR(val1, val2, delta)\
  ASSERT_PRED_FORMAT2(vw::test::VectorHelper<double>(vw::test::NearImpl(#delta, delta)), val1, val2)

#define EXPECT_VECTOR_DOUBLE_EQ(val1, val2)\
  EXPECT_PRED_FORMAT2(vw::test::VectorHelper<double>(vw::test::ULPEq<double>()), val1, val2)
#define ASSERT_VECTOR_DOUBLE_EQ(val1, val2)\
  ASSERT_PRED_FORMAT2(vw::test::VectorHelper<double>(vw::test::ULPEq<double>()), val1, val2)

#define EXPECT_VECTOR_FLOAT_EQ(val1, val2)\
  EXPECT_PRED_FORMAT2(vw::test::VectorHelper<float>(vw::test::ULPEq<float>()), val1, val2)
#define ASSERT_VECTOR_FLOAT_EQ(val1, val2)\
  ASSERT_PRED_FORMAT2(vw::test::VectorHelper<float>(vw::test::ULPEq<float>()), val1, val2)

#define EXPECT_COMPLEX_MATRIX_NEAR(val1, val2, delta)\
  EXPECT_PRED_FORMAT2(vw::test::MatrixHelper<std::complex<double> >(vw::test::NearImpl(#delta, delta)), val1, val2)
#define ASSERT_COMPLEX_MATRIX_NEAR(val1, val2, delta)\
  ASSERT_PRED_FORMAT2(vw::test::MatrixHelper<std::complex<double> >(vw::test::NearImpl(#delta, delta)), val1, val2)


#define EXPECT_PIXEL_NEAR(val1, val2, delta)\
  EXPECT_PRED_FORMAT2(vw::test::PixelNearHelper(#delta, delta), val1, val2)
#define ASSERT_PIXEL_NEAR(val1, val2, delta)\
  ASSERT_PRED_FORMAT2(vw::test::PixelNearHelper(#delta, delta), val1, val2)
#define EXPECT_PIXEL_EQ(val1, val2)\
  EXPECT_PRED_FORMAT2(vw::test::PixelEqHelper(), val1, val2)
#define ASSERT_PIXEL_EQ(val1, val2)\
  ASSERT_PRED_FORMAT2(vw::test::PixelEqHelper(), val1, val2)

template <typename ElemT>
inline double value_diff(const ElemT& a, const ElemT& b) {
  return std::fabs(a - b);
}

template <>
inline double value_diff(const std::complex<double>& a, const std::complex<double>& b) {
  return abs(a - b);
}

template <typename ElemT>
class MatrixHelper {
  typedef boost::function<AssertionResult (const std::string& ename, const std::string& aname, const ElemT& expected, const ElemT& actual)> Pred2;
  const Pred2& p;
  public:
  MatrixHelper(const Pred2& p) : p(p) {}
  AssertionResult operator()(const char* ename, const char* aname, Matrix<ElemT> expected, Matrix<ElemT> actual)
  {
    Message msg;
    bool failed = false;

    if ( expected.rows() != actual.rows() ) {
      failed = true;
      msg << "Cannot compare " << ename << " and " << aname << ": Different number of rows (" << expected.rows() << " vs " << actual.rows() << ")";
    }
    if ( expected.cols() != actual.cols() ) {
      failed = true;
      msg << "Cannot compare " << ename << " and " << aname << ": Different number of cols (" << expected.cols() << " vs " << actual.cols() << ")";
    }

    if (failed)
      return AssertionFailure(msg);

    for (size_t i = 0; i < expected.rows(); ++i) {
      for (size_t j = 0; j < expected.cols(); ++j) {
        const std::string idx = "(" + stringify(i) +  ","  + stringify(j) + ")";
        AssertionResult ret = p(ename + idx, aname + idx, expected(i,j), actual(i,j));

        if (!ret) {
          if (failed)
            msg << "\n";
          failed = true;
          msg << ret.failure_message();
        }
      }
    }

    if (failed)
      return AssertionFailure(msg);
    return AssertionSuccess();
  }
};

template <typename ElemT>
class VectorHelper {
  typedef boost::function<AssertionResult (const std::string& ename, const std::string& aname, const ElemT& expected, const ElemT& actual)> Pred2;
  const Pred2& p;
  public:
  VectorHelper(const Pred2& p) : p(p) {}
  AssertionResult operator()(const char* ename, const char* aname, Vector<ElemT> expected, Vector<ElemT> actual)
  {
    Message msg;
    bool failed = false;

    if ( expected.size() != actual.size() )
      return AssertionFailure(Message() << "Cannot compare " << ename << " and " << aname << ": Different size (" << expected.size() << " vs " << actual.size() << ")");

    for (size_t i = 0; i < expected.size(); ++i) {
      const std::string idx = "(" + stringify(i) + ")";
      AssertionResult ret = p(ename + idx, aname + idx, expected(i), actual(i));

      if (!ret) {
        if (failed)
          msg << "\n";
        failed = true;
        msg << ret.failure_message();
      }
    }

    if (failed)
      return AssertionFailure(msg);
    return AssertionSuccess();
  }
};

class NearImpl {
    const char *dexpr;
    double delta;
  public:
    NearImpl(const char *dexpr = "0", double delta = 0) : dexpr(dexpr), delta(delta) {}
    template <typename ElemT>
    AssertionResult operator()(const std::string& ename, const std::string& aname, const ElemT& expected, const ElemT& actual) {
      const double diff = value_diff(expected, actual);
      if (diff <= delta)
        return AssertionSuccess();

      Message msg;
      msg << "The difference between "
          << ename   << " and " << aname
          << " is "  << diff << ", which exceeds " << dexpr << ", where\n"
          << ename << " evaluates to " << expected << ",\n"
          << aname << " evaluates to " << actual
          << ", and\n" << dexpr << " evaluates to " << delta << ".";
      return AssertionFailure(msg);
    }
};

#if 0
EXPECT_FLOAT_EQ

  EXPECT_PRED_FORMAT2(::testing::internal::CmpHelperFloatingPointEQ<float>, \

AssertionResult CmpHelperFloatingPointEQ(const char* expected_expression,
                                         const char* actual_expression,
                                         RawType expected,
                                         RawType actual) {
#endif

template <typename ExpectedT>
class ULPEq {
  public:
    ULPEq() {}
    template <typename ElemT>
    AssertionResult operator()(const std::string& ename, const std::string& aname, const ExpectedT& expected, const ElemT& actual) {
      return ::testing::internal::CmpHelperFloatingPointEQ<ExpectedT>(ename.c_str(), aname.c_str(), expected, actual);
    }
};

class PixelNearHelper {
  NearImpl p;

 public:
  PixelNearHelper(const char *dexpr, double delta ) : p(dexpr,delta) {}

  template <class T1, class T2>
  AssertionResult operator()( const char* ename, const char* aname,
                              PixelMathBase<T1> const& expected,
                              PixelMathBase<T2> const& actual ) {
    BOOST_STATIC_ASSERT( (boost::is_same<T1,T2>::value) );
    Message msg;
    bool failed = false;

    for ( size_t i = 0; i < CompoundNumChannels<T1>::value; i++ ) {
      const std::string idx = "["+stringify(i)+"]";
      AssertionResult ret = p( ename+idx, aname+idx,
                               expected.impl()(i), actual.impl()(i));

      if (!ret) {
        if (failed)
          msg << "\n";
        failed = true;
        msg << ret.failure_message();
      }
    }

    if (failed)
      return AssertionFailure(msg);
    return AssertionSuccess();
  }
};

template <typename ElemT>
inline AssertionResult comp_helper( const std::string& ename, const std::string& aname,
                                    const ElemT& expected, const ElemT& actual ) {
  return ::testing::internal::CmpHelperEQ(ename.c_str(), aname.c_str(), expected, actual);
}

template <>
inline AssertionResult comp_helper<float>( const std::string& ename, const std::string& aname,
                                           const float& expected, const float& actual ) {
  return ::testing::internal::CmpHelperFloatingPointEQ<float>(ename.c_str(), aname.c_str(), expected, actual);
}

template <>
inline AssertionResult comp_helper<double>( const std::string& ename, const std::string& aname,
                                            const double& expected, const double& actual ) {
  return ::testing::internal::CmpHelperFloatingPointEQ<double>(ename.c_str(), aname.c_str(), expected, actual);
}

template <>
inline AssertionResult comp_helper<uint8>( const std::string& ename, const std::string& aname,
                                           const uint8& expected, const uint8& actual ) {

  if ( expected == actual )
    return AssertionSuccess();

  Message msg;
  msg << "Value of: " << aname << "\n"
      << "  Actual: " << int(actual) << "\n"
      << "Expected: " << ename << "\n"
      << "Which is: " << int(expected) << "\n";
  return AssertionFailure(msg);
}

class PixelEqHelper {

 public:
  PixelEqHelper() {}

  template <class T1, class T2>
  AssertionResult operator()( const char* ename, const char* aname,
                              PixelMathBase<T1> const& expected,
                              PixelMathBase<T2> const& actual ) {
    BOOST_STATIC_ASSERT( (boost::is_same<T1,T2>::value) );
    Message msg;
    bool failed = false;

    for ( size_t i = 0; i < CompoundNumChannels<T1>::value; i++ ) {
      const std::string idx = "["+stringify(i)+"]";
      AssertionResult ret = comp_helper( ename+idx, aname+idx,
                                         expected.impl()(i), actual.impl()(i) );

      if (!ret) {
        if (failed)
          msg << "\n";
        failed = true;
        msg << ret.failure_message();
      }
    }

    if (failed)
      return AssertionFailure(msg);
    return AssertionSuccess();
  }
};

}} // namespace vw::test

#endif
