PDFLATEX = pdflatex
DATEINAME = main

all: AI_Drones.pdf

AI_Drones.pdf: $(DATEINAME).pdf
	cp main.pdf AI_Drones.pdf

$(DATEINAME).pdf: $(wildcard *.tex) $(DATEINAME).tex
	$(PDFLATEX) -interaction=nonstopmode $(DATEINAME).tex
	$(PDFLATEX) -interaction=nonstopmode $(DATEINAME).tex

clean:
	rm -f *.idx *.ilg *.ind *.log *toc *.dvi *.aux *.out

distclean: clean
	rm -f $(DATEINAME).pdf AI_Drones.pdf
