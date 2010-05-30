all: graph final

.SUFFIXES:.dot .pdf

.dot.pdf:
	@dot -Tpdf $< > $@

graph: graph/asp_deps.pdf
	@echo "Graphing ..."

final:
	@echo "Calling Latex-MK ..."
	@latex-mk --pdflatex asp_book.tex

clean:
	rm -rf *.o *~ \#* *.aux *.log

distclean: clean
	rm -rf generate asp_book.toc asp_book.pdf .libs asp_book.out

.PHONY: images clean distclean all
