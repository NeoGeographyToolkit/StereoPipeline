# This Makefile is just a demo of what is possible with MER. I hope to
# document at some point and to get the hazard cams working.

# Stereo from the Microscopic imager doesn't work unforunately because
# the CAHVOR is not modelling the variable focal length they're using.

# User interfaces
###############################################

all: imagery stereo mesh

imagery: 1p270664103esf90csp2566l2m1.tif 1p270664103esf90csp2566r2m1.tif 1n270487304eff90cip1952l0m1.tif 1n270487304eff90cip1952r0m1.tif 2m147677487eff8800p2976m2f1.tif 2m147677547eff8800p2976m2f1.tif 1p270664103esf90csp2566l2m1.cahvor 1p270664103esf90csp2566r2m1.cahvor 1n270487304eff90cip1952l0m1.cahvor 1n270487304eff90cip1952r0m1.cahvor 2m147677487eff8800p2976m2f1.cahvor 2m147677547eff8800p2976m2f1.cahvor 2f194370083fflap00p1214r0m1.tif 2f194370083fflap00p1214r0m1.cahv 2f194370083fflap00p1214l0m1.tif 2f194370083fflap00p1214l0m1.cahv 2f194370083effap00p1214l0m1.tif 2f194370083effap00p1214r0m1.tif 2r195521583effaq00p1301l0m1.tif 2r195521583effaq00p1301r0m1.tif 2f191703508effanhep1110l0m1.tif 2f191703508effanhep1110r0m1.tif 2f194370083effap00p1214l0m1.cahvore 2f194370083effap00p1214r0m1.cahvore 2r195521583effaq00p1301l0m1.cahvore 2r195521583effaq00p1301r0m1.cahvore 2f191703508effanhep1110l0m1.cahvore 2f191703508effanhep1110r0m1.cahvore

stereo: pan01/pan01-PC.tif nav01/nav01-PC.tif mi01/mi01-PC.tif fh01lin/fh01lin-PC.tif fh01/fh01-PC.tif fh02/fh02-PC.tif rh01/rh01-PC.tif

mesh: pan01/pan01.ive nav01/nav01.ive mi01/mi01.ive fh01lin/fh01lin.ive fh01/fh01.ive fh02/fh02.ive rh01/rh01.ive

# Actual processing
###############################################

%.tif: %.img
	gdal_translate -scale -ot byte $< $@

%.cahvor: %.img
	mer2camera $<

%.cahv: %.img
	mer2camera $<

%.cahvore: %.img
	mer2camera $<

pan01/pan01-PC.tif: imagery
	stereo 1p270664103esf90csp2566l2m1.tif 1p270664103esf90csp2566r2m1.tif 1p270664103esf90csp2566l2m1.cahvor 1p270664103esf90csp2566r2m1.cahvor pan01/pan01

nav01/nav01-PC.tif: imagery
	stereo 1n270487304eff90cip1952l0m1.tif 1n270487304eff90cip1952r0m1.tif 1n270487304eff90cip1952l0m1.cahvor 1n270487304eff90cip1952r0m1.cahvor nav01/nav01

mi01/mi01-PC.tif: imagery
	stereo 2m147677487eff8800p2976m2f1.tif 2m147677547eff8800p2976m2f1.tif 2m147677487eff8800p2976m2f1.cahvor 2m147677547eff8800p2976m2f1.cahvor mi01/mi01

fh01lin/fh01lin-PC.tif: imagery
	stereo 2f194370083fflap00p1214r0m1.tif 2f194370083fflap00p1214l0m1.tif 2f194370083fflap00p1214r0m1.cahv 2f194370083fflap00p1214l0m1.cahv fh01lin/fh01lin

fh01/fh01-PC.tif: imagery
	stereo 2f194370083effap00p1214l0m1.tif 2f194370083effap00p1214r0m1.tif 2f194370083effap00p1214l0m1.cahvore 2f194370083effap00p1214r0m1.cahvore fh01/fh01

fh02/fh02-PC.tif: imagery
	stereo 2f191703508effanhep1110l0m1.tif 2f191703508effanhep1110r0m1.tif 2f191703508effanhep1110l0m1.cahvore 2f191703508effanhep1110r0m1.cahvore fh02/fh02

rh01/rh01-PC.tif: imagery
	stereo 2r195521583effaq00p1301l0m1.tif 2r195521583effaq00p1301r0m1.tif 2r195521583effaq00p1301l0m1.cahvore 2r195521583effaq00p1301r0m1.cahvore rh01/rh01

pan01/pan01.ive: pan01/pan01-PC.tif
	point2mesh -s 1 -l pan01/pan01-PC.tif pan01/pan01-L.tif

nav01/nav01.ive: nav01/nav01-PC.tif
	point2mesh -s 1 -l nav01/nav01-PC.tif nav01/nav01-L.tif

mi01/mi01.ive: mi01/mi01-PC.tif
	point2mesh -s 1 -l mi01/mi01-PC.tif mi01/mi01-L.tif

fh01lin/fh01lin.ive: fh01lin/fh01lin-PC.tif
	point2mesh -s 1 -l fh01lin/fh01lin-PC.tif fh01lin/fh01lin-L.tif

fh01/fh01.ive: fh01/fh01-PC.tif
	point2mesh -s 1 -l fh01/fh01-PC.tif fh01/fh01-L.tif

fh02/fh02.ive: fh02/fh02-PC.tif
	point2mesh -s 1 -l fh02/fh02-PC.tif fh02/fh02-L.tif

rh01/rh01.ive: rh01/rh01-PC.tif
	point2mesh -s 1 -l rh01/rh01-PC.tif rh01/rh01-L.tif

# Downloading data
##############################################

# PAN01
1p270664103esf90csp2566l2m1.img:
	wget -O 1p270664103esf90csp2566l2m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1po_0xxx/data/sol1605/edr//1p270664103esf90csp2566l2m1.img+AND+RT+%3D+RAW'
1p270664103esf90csp2566r2m1.img:
	wget -O 1p270664103esf90csp2566r2m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1po_0xxx/data/sol1605/edr//1p270664103esf90csp2566r2m1.img+AND+RT+%3D+RAW'
# NAV01
1n270487304eff90cip1952l0m1.img:
	wget -O 1n270487304eff90cip1952l0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1no_0xxx/data/sol1603/edr//1n270487304eff90cip1952l0m1.img+AND+RT+%3D+RAW'
1n270487304eff90cip1952r0m1.img:
	wget -O 1n270487304eff90cip1952r0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1no_0xxx/data/sol1603/edr//1n270487304eff90cip1952r0m1.img+AND+RT+%3D+RAW'
# FH01
2f194370083effap00p1214l0m1.img:
	wget -O 2f194370083effap00p1214l0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0766/edr//2f194370083effap00p1214l0m1.img+AND+RT+%3D+RAW'
2f194370083effap00p1214r0m1.img:
	wget -O 2f194370083effap00p1214r0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0766/edr//2f194370083effap00p1214r0m1.img+AND+RT+%3D+RAW'
# FH01 (linearized)
2f194370083fflap00p1214r0m1.img:
	wget -O 2f194370083fflap00p1214r0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0766/rdr//2f194370083fflap00p1214r0m1.img+AND+RT+%3D+RAW'
2f194370083fflap00p1214l0m1.img:
	wget -O 2f194370083fflap00p1214l0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0766/rdr//2f194370083fflap00p1214l0m1.img+AND+RT+%3D+RAW'
# RH01
2r195521583effaq00p1301l0m1.img:
	wget -O 2r195521583effaq00p1301l0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0779/edr//2r195521583effaq00p1301l0m1.img+AND+RT+%3D+RAW'
2r195521583effaq00p1301r0m1.img:
	wget -O 2r195521583effaq00p1301r0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0779/edr//2r195521583effaq00p1301r0m1.img+AND+RT+%3D+RAW'
# FH02
2f191703508effanhep1110l0m1.img:
	wget -O 2f191703508effanhep1110l0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0736/edr//2f191703508effanhep1110l0m1.img+AND+RT+%3D+RAW'
2f191703508effanhep1110r0m1.img:
	wget -O 2f191703508effanhep1110r0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0736/edr//2f191703508effanhep1110r0m1.img+AND+RT+%3D+RAW'
# MI01
2m147677487eff8800p2976m2f1.img:
	wget -O 2m147677487eff8800p2976m2f1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ms_0xxx/data/sol0240/edr//2m147677487eff8800p2976m2f1.img+AND+RT+%3D+RAW'
2m147677547eff8800p2976m2f1.img:
	wget -O 2m147677547eff8800p2976m2f1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ms_0xxx/data/sol0240/edr//2m147677547eff8800p2976m2f1.img+AND+RT+%3D+RAW'


# Clean up functions
##############################################

clean:
	rm -rf *.cahv *.cahvor *.cahvore *.tif nav01 pan01 mi01 fh01lin fh01 rh01 fh02

distclean: clean
	rm *.img *.IMG *~