# GNU Makefile for LaTeX stuff using the latex-mk system.

# Rule to convert .dot files into .pdf files.
%.pdf : %.dot
	@dot -Tpdf $< > $@

NAME = asp_book
TEXSRCS =	acknowledgments.tex \
			bundle_adjustment.tex \
			correlation.tex \
			examples.tex \
			installation.tex \
			introduction.tex \
			outputfiles.tex \
			scripting.tex \
			stereo_testing_plan.tex \
			stereodefault.tex \
			tools.tex \
			tutorial.tex \
			vwlogconfig.tex\
			pinholemodels.tex

# To regenerate this diagram from the dot file one needs graphviz with the triangulation library
# which is not available on CentOS 7 without rebuilding graphviz from scratch.
OTHER_PDF=	graph/asp_deps.pdf

#OTHER+=	$(OTHER_PDF) asp_book.tex
OTHER+=		asp_book.tex

#CLEAN_FILES+= $(OTHER_PDF) asp_book.tex
CLEAN_FILES+= asp_book.tex
USE_PDFLATEX = YES

ASP_VERSION:=$(shell cat ../../src/CMakeLists.txt | grep -E --color=no 'set.PACKAGE_VERSION' | awk -F\" '{print $$2}' | sed 's/_/\\\\_/g')
$(info ASP version is $(ASP_VERSION))

# Use 'open' on Mac systems to view the PDF
OS := $(shell uname -s)
ifeq ($(OS),Darwin)
VIEWPDF = open
endif

# Find GMKFILE.
# First try being clever.
GMKFILE := $(shell which latex-mk)
GMKFILE := $(shell dirname $(GMKFILE))/../share/latex-mk/latex.gmk
$(info Trying: $(GMKFILE))

ifeq ("$(wildcard $(GMKFILE))","")
  # Then try hard-coded 
  GMKFILE := /usr/local/share/latex-mk/latex.gmk
  $(info Trying: $(GMKFILE))
endif
ifeq ("$(wildcard $(GMKFILE))","")
  # Then try custom
  GMKFILE := /home/smcmich1/programs/latexmk/share/latex-mk/latex.gmk
  $(info Trying: $(GMKFILE))
endif

ENV_PROG=$(shell which env)
$(info ENV_PROG=$(ENV_PROG))
include $(GMKFILE)

# Paste in the ASP version into the text
asp_book.tex: asp_book.tex.in
	sed 's/@ASP_VERSION@/$(ASP_VERSION)/g' < $< > $@

