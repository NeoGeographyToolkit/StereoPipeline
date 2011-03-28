# This Makefile executes the steps detailed in Chapter 3 of the
# Stereo Pipeline Users' Guide

RESULTSDIR = results
OUT = E0201461-M0100115
STEREO = stereo
MAPPED = E0201461.map.cub M0100115.map.cub
PREFIX = $(RESULTSDIR)/$(OUT)
POINTCLOUD = $(PREFIX)-PC.tif

# Pattern rule to create .cub files from .imq files
%.cub: %.imq
	mocproc from= $< to= $@ Mapping= NO

all: products

M0100115.map.cub: M0100115.cub
	cam2map from=M0100115.cub to=M0100115.map.cub

E0201461.map.cub: E0201461.cub M0100115.map.cub
	cam2map from=E0201461.cub to=E0201461.map.cub map=M0100115.map.cub matchmap=true

M0100115.map/M0100115.kml: M0100115.map.cub
	image2qtree -m kml --normalize $<

E0201461.map/E0201461.kml: E0201461.map.cub
	image2qtree -m kml --normalize $<

$(POINTCLOUD): $(MAPPED)
	mkdir -p $(RESULTSDIR)
	$(STEREO) $(MAPPED) $(RESULTSDIR)/$(OUT)

$(PREFIX)-F-H.tif: $(POINTCLOUD)
	cd $(RESULTSDIR); disparitydebug $(OUT)-F.tif

$(PREFIX).ive: $(POINTCLOUD)
	cd $(RESULTSDIR); point2mesh $(OUT)-PC.tif $(OUT)-L.tif

$(PREFIX)-DRG.tif: $(POINTCLOUD)
	cd $(RESULTSDIR); \
	point2dem -r mars -n $(OUT)-PC.tif; \
	point2dem -r mars --orthoimage $(OUT)-L.tif $(OUT)-PC.tif;

$(PREFIX)-DEM.tif: $(POINTCLOUD)
	cd $(RESULTSDIR); \
	point2dem -r mars --nodata -32767 $(OUT)-PC.tif;

$(PREFIX)-OrthoR.tif: $(PREFIX)-DEM.tif
	cd $(RESULTSDIR); \
	orthoproject $(OUT)-DEM.tif ../M0100115.cub $(OUT)-OrthoR.tif;


$(PREFIX)-OrthoL.tif: $(PREFIX)-DEM.tif
	cd $(RESULTSDIR); \
	orthoproject $(OUT)-DEM.tif ../E0201461.cub $(OUT)-OrthoL.tif;

products: $(PREFIX)-F-H.tif $(PREFIX).ive $(PREFIX)-DRG.tif $(PREFIX)-OrthoL.tif $(PREFIX)-OrthoR.tif E0201461.map/E0201461.kml M0100115.map/M0100115.kml 

clean:
	rm -rf $(RESULTSDIR)
	rm -rf M0100115.map E0201461.map

distclean: clean
	rm -rf print.prt *.cub
	rm -rf *.vwip *.match
