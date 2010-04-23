all: graph setup final

.SUFFIXES:.dot .pdf

.dot.pdf:
	@dot -Teps $< | epstopdf -f -o=$@

graph: graph/asp_deps.pdf
	@echo "Graphing ..."

setup:
	@echo "Running Latex pass 1/3 ..."
	@pdflatex -interaction nonstopmode asp_book.tex | egrep 'Error' | perl -nle 'print "  *** $$_"'
	@echo "Running Latex pass 2/3 ..."
	@pdflatex -interaction nonstopmode asp_book.tex > /dev/null

final:
	@echo "Running Latex pass 3/3 ..."
	@pdflatex -interaction nonstopmode asp_book.tex | egrep 'Error|Warning|Underfull|Overfull|Output written on' | grep -v 'pdf inclusion' | perl -nle 'print "  *** $$_"'

clean:
	rm -rf *.o *~ \#* *.aux *.log

distclean: clean
	rm -rf generate asp_book.toc asp_book.pdf .libs asp_book.out

.PHONY: images clean distclean all
