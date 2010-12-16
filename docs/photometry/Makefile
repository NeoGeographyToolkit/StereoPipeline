# GNU Make for LaTeX stuff using the latex-mk system.

# Rule to convert .dot files into .pdf files.
%.pdf : %.dot
	@dot -Tpdf $< > $@

NAME = photometry
TEXSRCS = developmentplan.tex \
	introduction.tex
OTHER_PDF = graph/phoalgorithm.pdf graph/exec_init.pdf graph/exec_solve.pdf
OTHER += $(OTHER_PDF)

CLEAN_FILES += $(OTHER_PDF)
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


# all: graph setup final

# .SUFFIXES:.dot .pdf

# .dot.pdf:
# 	@dot -Teps $< | epstopdf -f -o=$@

# graph: graph/phoalgorithm.pdf graph/exec_init.pdf graph/exec_solve.pdf
# 	@echo "Graphing ..."

# setup:
# 	@echo "Running Latex pass 1/3 ..."
# 	@pdflatex -interaction nonstopmode photometry.tex | egrep 'Error' | perl -nle 'print "  *** $$_"'
# 	@echo "Running Latex pass 2/3 ..."
# 	@pdflatex -interaction nonstopmode photometry.tex > /dev/null

# final:
# 	@echo "Running Latex pass 3/3 ..."
# 	@pdflatex -interaction nonstopmode photometry.tex | egrep 'Error|Warning|Underfull|Overfull|Output written on' | grep -v 'pdf inclusion' | perl -nle 'print "  *** $$_"'

# clean:
# 	rm -rf *.o *~ \#* *.aux *.log

# distclean: clean
# 	rm -rf generate photometry.toc photometry.pdf .libs photometry.out graph/*pdf

# .PHONY: graph setup final clean distclean
