# This is makefile to run the demo for MRO CTX of North Terra Meridiani


all: nonmap/nonmap-DEM_CMAP.tif map/map-DEM_CMAP.tif

P02_001981_1823_XI_02N356W.IMG:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+data/mro/mars_reconnaissance_orbiter/ctx/mrox_0031/data/P02_001981_1823_XI_02N356W.IMG+AND+RT+%3D+RAW'

P03_002258_1817_XI_01N356W.IMG:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+data/mro/mars_reconnaissance_orbiter/ctx/mrox_0042/data/P03_002258_1817_XI_01N356W.IMG+AND+RT+%3D+RAW'

%.cal.cub: %.IMG
	mroctx2isis from= $< to= $<.temp.cub
	spiceinit from= $<.temp.cub
	ctxcal from= $<.temp.cub to= $<.temp2.cub
	crop from=$<.temp2.cub to=$@ line=10000 nlines=3000
	rm $<.temp.cub
	rm $<.temp2.cub


nonmapprojected: P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub

mapprojected: nonmapprojected
	if [ ! -e P02_001981_1823_XI_02N356W.map.cub ] && [ ! -e P03_002258_1817_XI_01N356W.map.cub ]; then cam2map4stereo.py P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub; fi

map/map-PC.tif: mapprojected
	if [ ! -e map/map-PC.tif ]; then stereo P02_001981_1823_XI_02N356W.map.cub P03_002258_1817_XI_01N356W.map.cub map/map -s stereo.map; fi

map/map-DEM.tif: map/map-PC.tif
	cd map; \
	point2dem -r mars --nodata -32767 map-PC.tif --error --orthoimage map-L.tif

map/map-DEM_CMAP.tif: map/map-DEM.tif
	cd map; \
	hillshade map-DEM.tif; \
	colormap -s map-DEM_HILLSHADE.tif map-DEM.tif; \
	colormap map-IntersectionErr.tif --colormap-style ../../TriangulationError.lut

nonmap/nonmap-PC.tif: nonmapprojected
	if [ ! -e nonmap/nonmap-PC.tif ]; then stereo P02_001981_1823_XI_02N356W.cal.cub P03_002258_1817_XI_01N356W.cal.cub nonmap/nonmap -s stereo.nonmap; fi

nonmap/nonmap-DEM.tif: nonmap/nonmap-PC.tif
	cd nonmap; \
	point2dem -r mars --nodata -32767 nonmap-PC.tif --errorimage

nonmap/nonmap-DEM_CMAP.tif: nonmap/nonmap-DEM.tif
	cd nonmap; \
	hillshade nonmap-DEM.tif; \
	colormap -s nonmap-DEM_HILLSHADE.tif nonmap-DEM.tif; \
	colormap nonmap-IntersectionErr.tif

clean:
	rm -rf nonmap map *.vwip

distclean: clean
	rm -rf *.cub *~ *.match
