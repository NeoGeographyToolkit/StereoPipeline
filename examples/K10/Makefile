# This Makefile performs an example of processing user provided
# images. These images are from IRG's K10 Black robot from a testing
# session behind the 80x120.

RESULTSDIR = results
OUT = k10black
PREFIX = $(RESULTSDIR)/$(OUT)

all: $(PREFIX)-F-H.tif $(PREFIX).ive

$(PREFIX)-PC.tif:
	mkdir -p $(RESULTSDIR)
	stereo left4.png right4.png black_left.tsai black_right.tsai $(PREFIX)

$(PREFIX)-F-H.tif: $(PREFIX)-PC.tif
	cd $(RESULTSDIR); disparitydebug $(OUT)-F.tif

$(PREFIX).ive: $(PREFIX)-PC.tif
	cd $(RESULTSDIR); point2mesh -s 1 $(OUT)-PC.tif $(OUT)-L.tif

clean:
	rm -rf $(RESULTSDIR)

distclean: clean
