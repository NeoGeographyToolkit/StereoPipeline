# This Makefile executes the steps detailed in Chapter 3 of the
# Stereo Pipeline Users' Guide

all: products_map products_nonmap

# Pattern rule to create .cub files from .imq files
%.cub: %.imq
	mocproc from= $< to= $@ Mapping= NO

jigsaw: M0100115.cub E0201461.cub
	echo M0100115.cub > cube.lis; \
	echo E0201461.cub >> cube.lis; \
	jigsaw fromlist=cube.lis radius=yes update=yes twist=no cnet=control.net onet=control_jigsaw.net spsolve=position

M0100115.map.cub: jigsaw
	cam2map from=M0100115.cub to=M0100115.map.cub

E0201461.map.cub: jigsaw M0100115.map.cub
	cam2map from=E0201461.cub to=E0201461.map.cub map=M0100115.map.cub matchmap=true

M0100115.map/M0100115.kml: M0100115.map.cub
	image2qtree -m kml --normalize $<

E0201461.map/E0201461.kml: E0201461.map.cub
	image2qtree -m kml --normalize $<

map/map-PC.tif: M0100115.map.cub E0201461.map.cub
	mkdir -p map; \
	if [ ! -e map/map-PC.tif ]; then stereo $^ map/map -s stereo.map; fi

map/map-F-H.tif: map/map-PC.tif
	cd map; disparitydebug map-F.tif

map/map.ive: map/map-PC.tif
	cd map; point2mesh map-PC.tif map-L.tif

map/map-DEM.tif: map/map-PC.tif
	cd map; point2dem -r mars --orthoimage map-L.tif -n --error --nodata -32767 map-PC.tif;

map/map-OrthoR.tif: map/map-DEM.tif
	cd map; mapproject map-DEM.tif ../M0100115.cub map-OrthoR.tif;

map/map-OrthoL.tif: map/map-DEM.tif
	cd map; mapproject map-DEM.tif ../E0201461.cub map-OrthoL.tif;

map/map-DEM_CMAP.tif: map/map-DEM.tif
	cd map; \
	hillshade map-DEM.tif; \
	colormap -s map-DEM_HILLSHADE.tif map-DEM.tif; \
	colormap map-IntersectionErr.tif --colormap-style ../../TriangulationError.lut

products_map: map/map-F-H.tif map/map.ive map/map-OrthoL.tif map/map-OrthoR.tif E0201461.map/E0201461.kml M0100115.map/M0100115.kml map/map-DEM_CMAP.tif

nonmap/nonmap-PC.tif: jigsaw
	mkdir -p nonmap
	if [ ! -e nomap/nomap-PC.tif ]; then stereo M0100115.cub E0201461.cub nonmap/nonmap -s stereo.nonmap; fi

nonmap/nonmap.ive: nonmap/nonmap-PC.tif
	cd nonmap; point2mesh nonmap-PC.tif nonmap-L.tif

nonmap_point2dem: nonmap/nonmap-PC.tif
	cd nonmap; \
	point2dem -r mars --orthoimage nonmap-L.tif nonmap-PC.tif --errorimage --nodata -32767 -n; \
	hillshade nonmap-DEM.tif; \
	colormap -s nonmap-DEM_HILLSHADE.tif nonmap-DEM.tif; \
	colormap nonmap-IntersectionErr.tif

products_nonmap: nonmap/nonmap.ive nonmap_point2dem

clean:
	rm -rf map nonmap
	rm -rf M0100115.map E0201461.map cube.lis control_ba.net *.match bundleout* residuals.csv control_jigsaw.net

distclean: clean
	rm -rf print.prt *.cub *.vwip *.match *~
