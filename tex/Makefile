# Compile the document
all: main.tex main.bib
	rm -f main.aux
	rm -f main.bbl
	rm -f main.blg
	pdflatex main
	bibtex main
	pdflatex main
	pdflatex main

# Build and open in OSX
mac: main.tex main.bib
	rm -f main.aux
	rm -f main.bbl
	rm -f main.blg
	pdflatex main
	bibtex main
	pdflatex main
	pdflatex main
	open main.pdf

# TODO: Build and open in Linux


clean: main.tex main.bib
	rm -f *.aux
	rm -f *.bbl
	rm -f *.blg
	rm -f *.log
	rm -f *.blg
	rm -f *.dvi
	rm -f *.pdf
