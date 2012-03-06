# This is makefile to run the demo for MRO CTX of North Terra Meridiani

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
POINT2DEM = $(EEXECDIR)point2dem
HILLSHADE = $(EEXECDIR)hillshade
COLORMAP = $(EEXECDIR)colormap
CAM2MAP4STEREO = $(EXECDIR)cam2map4stereo.py

all: nonmap/nonmap-DEM_CMAP.tif map/map-DEM_CMAP.tif

P02_001981_1823_XI_02N356W.IMG:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+data/mro/mars_reconnaissance_orbiter/ctx/mrox_0031/data/P02_001981_1823_XI_02N356W.IMG+AND+RT+%3D+RAW'

P03_002258_1817_XI_01N356W.IMG:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+data/mro/mars_reconnaissance_orbiter/ctx/mrox_0042/data/P03_002258_1817_XI_01N356W.IMG+AND+RT+%3D+RAW'

%.cal.cub: %.IMG
	mroctx2isis from= $< to= $<.temp.cub
	spiceinit from= $<.temp.cub
	ctxcal from= $<.temp.cub to= $@
	rm $<.temp.cub

nonmapprojected: P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub

mapprojected: nonmapprojected
	if [ ! -e P02_001981_1823_XI_02N356W.map.cub ] && [ ! -e P03_002258_1817_XI_01N356W.map.cub ]; then $(CAM2MAP4STEREO) P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub; fi

map/map-PC.tif: mapprojected
	$(STEREO) P02_001981_1823_XI_02N356W.map.cub P03_002258_1817_XI_01N356W.map.cub map/map -s stereo.map

map/map-DEM.tif: map/map-PC.tif
	cd map; \
	$(POINT2DEM) -r mars --nodata -32767 map-PC.tif --error --orthoimage map-L.tif

map/map-DEM_CMAP.tif: map/map-DEM.tif
	cd map; \
	$(HILLSHADE) map-DEM.tif; \
	$(COLORMAP) -s map-DEM_HILLSHADE.tif map-DEM.tif; \
	$(COLORMAP) map-DEMError.tif --lut ../../TriangulationError.lut

nonmap/nonmap-PC.tif: nonmapprojected
	$(STEREO) P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub nonmap/nonmap -s stereo.nonmap

nonmap/nonmap-DEM.tif: nonmap/nonmap-PC.tif
	cd nonmap; \
	$(POINT2DEM) -r mars --nodata -32767 nonmap-PC.tif

nonmap/nonmap-DEM_CMAP.tif: nonmap/nonmap-DEM.tif
	cd nonmap; \
	$(HILLSHADE) nonmap-DEM.tif; \
	$(COLORMAP) -s nonmap-DEM_HILLSHADE.tif nonmap-DEM.tif; \
	$(COLORMAP) map-DEMError.tif --lut ../../TriangulationError.lut

clean:
	rm -rf nonmap map *.vwip

distclean: clean
	rm -rf *.cub *~ *.match