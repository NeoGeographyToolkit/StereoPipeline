# This Makefile executes the steps detailed in Chapter 3 of the
# Stereo Pipeline Users' Guide

RESULTSDIR = results
OUT = E0201461-M0100115
STEREO = stereo
CUBES = E0201461.cub M0100115.cub
MAPPED = E0201461.map.cub M0100115.map.cub

# Pattern rule to create .cub files from .imq files
%.cub: %.imq
	mocproc from= $< to= $@ Mapping= NO

all: stereo

cubes: $(CUBES)
	cam2map from=M0100115.cub to=M0100115.map.cub
	cam2map from=E0201461.cub to=E0201461.map.cub map=M0100115.map.cub matchmap=true

stereo: cubes
	mkdir -p $(RESULTSDIR)
	$(STEREO) $(MAPPED) $(RESULTSDIR)/$(OUT)

disparity: stereo
	cd $(RESULTSDIR); disparitydebug $(OUT)-F.exr

mesh: stereo
	cd $(RESULTSDIR); point2mesh $(OUT)-PC.tif $(OUT)-L.tif

dem: stereo
	cd $(RESULTSDIR); \
	point2dem --xyz-to-lonlat -r mars -n $(OUT)-PC.tif; \
	point2dem --xyz-to-lonlat -r mars --orthoimage $(OUT)-L.tif $(OUT)-PC.tif

products: disparity mesh dem

clean:
	rm -rf $(RESULTSDIR)

distclean: clean
	rm -rf print.prt *.cub
	rm -rf *.vwip *.match
