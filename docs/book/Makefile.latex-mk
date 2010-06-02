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
			vwlogconfig.tex
OTHER_PDF=	graph/asp_deps.pdf
OTHER+=		$(OTHER_PDF)

CLEAN_FILES+= $(OTHER_PDF)
USE_PDFLATEX = YES

# Use 'open' on Mac systems to view the PDF
OS := $(shell uname -s)
ifeq ($(OS),Darwin)
VIEWPDF = open
endif

# This should find the location of the latex.gmk file on your system
GMKFILE := $(shell which latex-mk | awk -F 'latex-mk' '{print $$1}')
GMKFILE := $(GMKFILE)../share/latex-mk/latex.gmk

include $(GMKFILE)
