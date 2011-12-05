# This Makefile executes the steps detailed in Chapter 3 of the
# Stereo Pipeline Users' Guide

# Users with custom paths must override this variable
# EXECDIR  == executable directory
# EEXECDIR == executable directory if in a result dir. (For handling
#             relative paths)
EXECDIR ?= ../../bin/
ifeq ($(EXECDIR),)
	EEXECDIR=$(EXECDIR)
else
	EEXECDIR=../$(EXECDIR)
endif

# Executables
STEREO = $(EXECDIR)stereo
IMAGE2QTREE = $(EXECDIR)image2qtree
POINT2DEM = $(EEXECDIR)point2dem
ORTHOPROJECT = $(EEXECDIR)orthoproject
HILLSHADE = $(EEXECDIR)hillshade
COLORMAP = $(EEXECDIR)colormap
DISPARITYDEBUG = $(EEXECDIR)disparitydebug
POINT2MESH = $(EEXECDIR)point2mesh

# Output variables
RESULTSDIR = map
OUT = map
MAPPED = E0201461.map.cub M0100115.map.cub
PREFIX = $(RESULTSDIR)/$(OUT)
POINTCLOUD = $(PREFIX)-PC.tif

RESULTSDIRU = nonmap
NONMAPPED = E0201461.cub M0100115.cub
PREFIXU = $(RESULTSDIRU)/$(OUT)
POINTCLOUDU = $(PREFIXU)-PC.tif

# Pattern rule to create .cub files from .imq files
%.cub: %.imq
	mocproc from= $< to= $@ Mapping= NO

all: products_map products_nonmap

M0100115.map.cub: M0100115.cub
	cam2map from=M0100115.cub to=M0100115.map.cub

E0201461.map.cub: E0201461.cub M0100115.map.cub
	cam2map from=E0201461.cub to=E0201461.map.cub map=M0100115.map.cub matchmap=true

M0100115.map/M0100115.kml: M0100115.map.cub
	$(IMAGE2QTREE) -m kml --normalize $<

E0201461.map/E0201461.kml: E0201461.map.cub
	$(IMAGE2QTREE) -m kml --normalize $<

$(POINTCLOUD): $(MAPPED)
	mkdir -p $(RESULTSDIR)
	$(STEREO) $(MAPPED) $(RESULTSDIR)/$(OUT) -s stereo.map

$(PREFIX)-F-H.tif: $(POINTCLOUD)
	cd $(RESULTSDIR); $(DISPARITYDEBUG) $(OUT)-F.tif

$(PREFIX).ive: $(POINTCLOUD)
	cd $(RESULTSDIR); $(POINT2MESH) $(OUT)-PC.tif $(OUT)-L.tif

$(PREFIX)-DRG.tif: $(POINTCLOUD)
	cd $(RESULTSDIR); $(POINT2DEM) -r mars --orthoimage $(OUT)-L.tif $(OUT)-PC.tif;

$(PREFIX)-DEM.tif: $(POINTCLOUD)
	cd $(RESULTSDIR); $(POINT2DEM) -r mars -n --nodata -32767 $(OUT)-PC.tif;

$(PREFIX)-OrthoR.tif: $(PREFIX)-DEM.tif
	cd $(RESULTSDIR); $(ORTHOPROJECT) $(OUT)-DEM.tif ../M0100115.cub $(OUT)-OrthoR.tif;

$(PREFIX)-OrthoL.tif: $(PREFIX)-DEM.tif
	cd $(RESULTSDIR); $(ORTHOPROJECT) $(OUT)-DEM.tif ../E0201461.cub $(OUT)-OrthoL.tif;

$(PREFIX)-DEM_CMAP.tif: $(PREFIX)-DEM.tif
	cd $(RESULTSDIR); \
	$(HILLSHADE) $(OUT)-DEM.tif; \
	$(COLORMAP) -s $(OUT)-DEM_HILLSHADE.tif $(OUT)-DEM.tif

products_map: $(PREFIX)-F-H.tif $(PREFIX).ive $(PREFIX)-DRG.tif $(PREFIX)-OrthoL.tif $(PREFIX)-OrthoR.tif E0201461.map/E0201461.kml M0100115.map/M0100115.kml $(PREFIX)-DEM_CMAP.tif

$(POINTCLOUDU): $(NONMAPPED)
	mkdir -p $(RESULTSDIRU)
	$(STEREO) $(NONMAPPED) $(RESULTSDIRU)/$(OUT) -s stereo.nonmap

$(PREFIXU).ive: $(POINTCLOUDU)
	cd $(RESULTSDIRU); $(POINT2MESH) $(OUT)-PC.tif $(OUT)-L.tif

$(PREFIXU)-DRG.tif: $(POINTCLOUDU)
	cd $(RESULTSDIRU); \
	$(POINT2DEM) -r mars --orthoimage $(OUT)-L.tif $(OUT)-PC.tif;

$(PREFIXU)-DEM.tif: $(POINTCLOUDU)
	cd $(RESULTSDIRU); \
	$(POINT2DEM) -r mars -n --nodata -32767 $(OUT)-PC.tif;

$(PREFIXU)-DEM_CMAP.tif: $(PREFIXU)-DEM.tif
	cd $(RESULTSDIRU); \
	$(HILLSHADE) $(OUT)-DEM.tif; \
	$(COLORMAP) -s $(OUT)-DEM_HILLSHADE.tif $(OUT)-DEM.tif

products_nonmap: $(PREFIXU).ive $(PREFIXU)-DRG.tif $(PREFIXU)-DEM_CMAP.tif

clean:
	rm -rf $(RESULTSDIR) $(RESULTSDIRU)
	rm -rf M0100115.map E0201461.map

distclean: clean
	rm -rf print.prt *.cub
	rm -rf *.vwip *.match
