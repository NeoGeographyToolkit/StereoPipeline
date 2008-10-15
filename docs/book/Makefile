all: asp_book.pdf

asp_book.pdf:
	pdflatex asp_book.tex

clean:
	rm -rf *.o *~ \#* *.aux *.log 

distclean: clean
	rm -rf generate asp_book.toc asp_book.pdf .libs

.PHONY: images clean distclean all
