# This Makefile is just a demo to verify hirise processing works.

all: nonmap/nonmap-DEM_CMAP.tif


%.IMG:
	chmod a+x ./downloader.sh && ./downloader.sh $@ 

# Commands to process the full image, these take a long time!
#PSP_005201_1640_RED.mos_hijitreged.norm.cub: PSP_005201_1640_RED0_0.IMG PSP_005201_1640_RED0_1.IMG PSP_005201_1640_RED1_0.IMG PSP_005201_1640_RED1_1.IMG PSP_005201_1640_RED2_0.IMG PSP_005201_1640_RED2_1.IMG PSP_005201_1640_RED3_0.IMG PSP_005201_1640_RED3_1.IMG PSP_005201_1640_RED4_0.IMG PSP_005201_1640_RED4_1.IMG PSP_005201_1640_RED5_0.IMG PSP_005201_1640_RED5_1.IMG PSP_005201_1640_RED6_0.IMG PSP_005201_1640_RED6_1.IMG PSP_005201_1640_RED7_0.IMG PSP_005201_1640_RED7_1.IMG PSP_005201_1640_RED8_0.IMG PSP_005201_1640_RED8_1.IMG PSP_005201_1640_RED9_0.IMG PSP_005201_1640_RED9_1.IMG
#	hiedr2mosaic.py $^

#PSP_005913_1640_RED.mos_hijitreged.norm.cub: PSP_005913_1640_RED0_0.IMG PSP_005913_1640_RED0_1.IMG PSP_005913_1640_RED1_0.IMG PSP_005913_1640_RED1_1.IMG PSP_005913_1640_RED2_0.IMG PSP_005913_1640_RED2_1.IMG PSP_005913_1640_RED3_0.IMG PSP_005913_1640_RED3_1.IMG PSP_005913_1640_RED4_0.IMG PSP_005913_1640_RED4_1.IMG PSP_005913_1640_RED5_0.IMG PSP_005913_1640_RED5_1.IMG PSP_005913_1640_RED6_0.IMG PSP_005913_1640_RED6_1.IMG PSP_005913_1640_RED7_0.IMG PSP_005913_1640_RED7_1.IMG PSP_005913_1640_RED8_0.IMG PSP_005913_1640_RED8_1.IMG PSP_005913_1640_RED9_0.IMG PSP_005913_1640_RED9_1.IMG
#	hiedr2mosaic.py $^


# Abbreviated processing commands
PSP_005201_1640_RED.mos_hijitreged.norm.cub: PSP_005201_1640_RED0_0.IMG PSP_005201_1640_RED0_1.IMG PSP_005201_1640_RED1_0.IMG PSP_005201_1640_RED1_1.IMG
	hiedr2mosaic.py $^

PSP_005913_1640_RED.mos_hijitreged.norm.cub: PSP_005913_1640_RED0_0.IMG PSP_005913_1640_RED0_1.IMG PSP_005913_1640_RED1_0.IMG PSP_005913_1640_RED1_1.IMG
	hiedr2mosaic.py $^



PSP_005201_1640_RED.mos_hijitreged.norm.crop.cub: PSP_005201_1640_RED.mos_hijitreged.norm.cub
	crop from=$^ to=$@ line=27000 nlines=3000

PSP_005913_1640_RED.mos_hijitreged.norm.crop.cub: PSP_005913_1640_RED.mos_hijitreged.norm.cub
	crop from=$^ to=$@ line=33000 nlines=3000

nonmap/nonmap-PC.tif: PSP_005913_1640_RED.mos_hijitreged.norm.crop.cub PSP_005201_1640_RED.mos_hijitreged.norm.crop.cub
	stereo $^ nonmap/nonmap -s stereo.nonmap

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

distclean:
	rm -rf *cub *~ *.match
