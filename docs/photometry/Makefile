all: graph setup final

.SUFFIXES:.dot .pdf

.dot.pdf:
	@dot -Teps $< | epstopdf -f -o=$@

graph: graph/phoalgorithm.pdf graph/executables.pdf
	@echo "Graphing ..."

setup:
	@echo "Running Latex pass 1/3 ..."
	@pdflatex -interaction nonstopmode photometry.tex | egrep 'Error' | perl -nle 'print "  *** $$_"'
	@echo "Running Latex pass 2/3 ..."
	@pdflatex -interaction nonstopmode photometry.tex > /dev/null

final:
	@echo "Running Latex pass 3/3 ..."
	@pdflatex -interaction nonstopmode photometry.tex | egrep 'Error|Warning|Underfull|Overfull|Output written on' | grep -v 'pdf inclusion' | perl -nle 'print "  *** $$_"'

clean:
	rm -rf *.o *~ \#* *.aux *.log

distclean: clean
	rm -rf generate photometry.toc photometry.pdf .libs photometry.out graph/*pdf

.PHONY: graph setup final clean distclean
