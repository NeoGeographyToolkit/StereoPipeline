# This is makefile to run the demo for MRO CTX of North Terra Meridiani

STEREO = stereo

all: nonmap/nonmap-DEM_CMAP.tif map/map-DEM_CMAP.tif

P02_001981_1823_XI_02N356W.IMG:
	wget -O P02_001981_1823_XI_02N356W.IMG 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+data/mro/mars_reconnaissance_orbiter/ctx/mrox_0031/data/P02_001981_1823_XI_02N356W.IMG+AND+RT+%3D+RAW'

P03_002258_1817_XI_01N356W.IMG:
	wget -O P03_002258_1817_XI_01N356W.IMG 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+data/mro/mars_reconnaissance_orbiter/ctx/mrox_0042/data/P03_002258_1817_XI_01N356W.IMG+AND+RT+%3D+RAW'

%.cal.cub: %.IMG
	mroctx2isis from= $< to= $<.temp.cub
	spiceinit from= $<.temp.cub
	ctxcal from= $<.temp.cub to= $@
	rm $<.temp.cub

nonmapprojected: P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub

mapprojected: nonmapprojected
	cam2map4stereo.py P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub

map/map-PC.tif: mapprojected
	$(STEREO) P02_001981_1823_XI_02N356W.map.cub P03_002258_1817_XI_01N356W.map.cub map/map -s stereo.map

map/map-DEM.tif: map/map-PC.tif
	cd map; \
	point2dem -r mars --nodata -32767 map-PC.tif

map/map-DEM_CMAP.tif: map/map-DEM.tif
	cd map; \
	hillshade map-DEM.tif; \
	colormap -s map-DEM_HILLSHADE.tif map-DEM.tif

nonmap/nonmap-PC.tif: nonmapprojected
	$(STEREO) P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub nonmap/nonmap -s stereo.default

nonmap/nonmap-DEM.tif: nonmap/nonmap-PC.tif
	cd nonmap; \
	point2dem -r mars --nodata -32767 nonmap-PC.tif

nonmap/nonmap-DEM_CMAP.tif: nonmap/nonmap-DEM.tif
	cd nonmap; \
	hillshade nonmap-DEM.tif; \
	colormap -s nonmap-DEM_HILLSHADE.tif nonmap-DEM.tif

clean:
	rm -rf nonmap map *.vwip

distclean:
	rm -rf *.cub