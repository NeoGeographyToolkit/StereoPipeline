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
OTHER_PDF=	graph/asp_deps.pdf
OTHER+=		$(OTHER_PDF) asp_book.tex

CLEAN_FILES+= $(OTHER_PDF) asp_book.tex
USE_PDFLATEX = YES

ASP_VERSION:=$(shell cat ../../src/asp/asp_config.h | grep ASP_VERSION --color=no | awk -F\" '{print $$2}' | sed 's/_/\\\\_/g')

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

include $(GMKFILE)

# Paste in the ASP version into the text
asp_book.tex: asp_book.tex.in
	sed 's/@ASP_VERSION@/$(ASP_VERSION)/g' < $< > $@

