
######################################################################
#			Tests.mak				     #
######################################################################


SUFFIXES = .cxx

CXXTEST_DIR :=   $(top_srcdir)/thirdparty/cxxtest
CXXTEST_GEN :=   $(CXXTEST_DIR)/cxxtestgen.pl
CXXTEST_ARGS :=  --error-printer

TESTS_SRCS := $(TESTS_H:.h=.cxx)
TESTS_PROGS := $(TESTS_SRCS:.cxx=.test)

%.cxx: %.h 
	$(CXXTEST_GEN) $(CXXTEST_ARGS) -o $@ $<

%.test: %.cxx 
	$(CXX) $(CC_OPTIMIZE_DEBUG) $(CFLAGS) $(CPPFLAGS) -I$(CXXTEST_DIR) -I.. -I. $(INCLUDES) -o $@ $< -L$(libdir) -lvw $(TESTS_FLAGS) $(LDFLAGS)


tests: $(TESTS_PROGS) 

check-local: tests
