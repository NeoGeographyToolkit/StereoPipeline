# This Makefile is just a demo of what is possible with MER.

# Stereo from the Microscopic imager doesn't work unforunately because
# the CAHVOR is not modelling the variable focal length they're using.

# User interfaces
###############################################

all: stereo mesh

# These are the full commands that test more sensors
#stereo: pan01/pan01-PC.tif nav01/nav01-PC.tif mi01/mi01-PC.tif fh01lin/fh01lin-PC.tif fh01/fh01-PC.tif fh02/fh02-PC.tif rh01/rh01-PC.tif pan02/pan02-PC.tif

#mesh: pan01/pan01.ive nav01/nav01.ive mi01/mi01.ive fh01lin/fh01lin.ive fh01/fh01.ive fh02/fh02.ive rh01/rh01.ive pan02/pan02.ive

# These commands skip a few instruments to speed up the test
stereo: pan01/pan01-PC.tif nav01/nav01-PC.tif mi01/mi01-PC.tif fh01/fh01-PC.tif rh01/rh01-PC.tif

mesh: pan01/pan01.ive nav01/nav01.ive mi01/mi01.ive fh01/fh01.ive rh01/rh01.ive


# Actual processing
###############################################

%.cahvor: %.img
	mer2camera $<

%.cahv: %.img
	mer2camera $<

%.cahvore: %.img
	mer2camera $<

pan01/pan01-PC.tif: 1p270664103esf90csp2566l2m1.img 1p270664103esf90csp2566r2m1.img 1p270664103esf90csp2566l2m1.cahvor 1p270664103esf90csp2566r2m1.cahvor
	stereo $^ pan01/pan01

nav01/nav01-PC.tif: 1n270487304eff90cip1952l0m1.img 1n270487304eff90cip1952r0m1.img 1n270487304eff90cip1952l0m1.cahvor 1n270487304eff90cip1952r0m1.cahvor
	stereo $^ nav01/nav01

mi01/mi01-PC.tif: 2m147677487eff8800p2976m2m1.img 2m147677547eff8800p2976m2m1.img 2m147677487eff8800p2976m2m1.cahvor 2m147677547eff8800p2976m2m1.cahvor
	stereo $^ mi01/mi01

fh01lin/fh01lin-PC.tif: 2f194370083fflap00p1214r0m1.img 2f194370083fflap00p1214l0m1.img 2f194370083fflap00p1214r0m1.cahv 2f194370083fflap00p1214l0m1.cahv
	stereo $^ fh01lin/fh01lin

fh01/fh01-PC.tif: 2f194370083effap00p1214l0m1.img 2f194370083effap00p1214r0m1.img 2f194370083effap00p1214l0m1.cahvore 2f194370083effap00p1214r0m1.cahvore
	stereo $^ fh01/fh01

fh02/fh02-PC.tif: 2f191703508effanhep1110l0m1.img 2f191703508effanhep1110r0m1.img 2f191703508effanhep1110l0m1.cahvore 2f191703508effanhep1110r0m1.cahvore
	stereo $^ fh02/fh02

rh01/rh01-PC.tif: 2r195521583effaq00p1301l0m1.img 2r195521583effaq00p1301r0m1.img 2r195521583effaq00p1301l0m1.cahvore 2r195521583effaq00p1301r0m1.cahvore
	stereo $^ rh01/rh01

pan02/pan02-PC.tif: 1p227709787eff80k7p2389l2c1.img 1p227709787eff80k7p2389r2c1.img 1p227709787eff80k7p2389l2c1.cahvor 1p227709787eff80k7p2389r2c1.cahvor
	stereo $^ pan02/pan02

pan01/pan01.ive: pan01/pan01-PC.tif
	point2mesh -s 1 -l $^ pan01/pan01-L.tif

nav01/nav01.ive: nav01/nav01-PC.tif
	point2mesh -s 1 -l $^ nav01/nav01-L.tif

mi01/mi01.ive: mi01/mi01-PC.tif
	point2mesh -s 1 -l $^ mi01/mi01-L.tif

fh01lin/fh01lin.ive: fh01lin/fh01lin-PC.tif
	point2mesh -s 1 -l $^ fh01lin/fh01lin-L.tif

fh01/fh01.ive: fh01/fh01-PC.tif
	point2mesh -s 1 -l $^ fh01/fh01-L.tif

fh02/fh02.ive: fh02/fh02-PC.tif
	point2mesh -s 1 -l $^ fh02/fh02-L.tif

rh01/rh01.ive: rh01/rh01-PC.tif
	point2mesh -s 1 -l $^ rh01/rh01-L.tif

pan02/pan02.ive: pan02/pan02-PC.tif
	point2mesh -s 1 -l $^ pan02/pan02-L.tif

# Downloading data
##############################################

# PAN01
1p270664103esf90csp2566l2m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1po_0xxx/data/sol1605/edr//1p270664103esf90csp2566l2m1.img+AND+RT+%3D+RAW'
1p270664103esf90csp2566r2m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1po_0xxx/data/sol1605/edr//1p270664103esf90csp2566r2m1.img+AND+RT+%3D+RAW'
# NAV01
1n270487304eff90cip1952l0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1no_0xxx/data/sol1603/edr//1n270487304eff90cip1952l0m1.img+AND+RT+%3D+RAW'
1n270487304eff90cip1952r0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1no_0xxx/data/sol1603/edr//1n270487304eff90cip1952r0m1.img+AND+RT+%3D+RAW'
# FH01
2f194370083effap00p1214l0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0766/edr//2f194370083effap00p1214l0m1.img+AND+RT+%3D+RAW'
2f194370083effap00p1214r0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0766/edr//2f194370083effap00p1214r0m1.img+AND+RT+%3D+RAW'
# FH01 (linearized)
2f194370083fflap00p1214r0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0766/rdr//2f194370083fflap00p1214r0m1.img+AND+RT+%3D+RAW'
2f194370083fflap00p1214l0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0766/rdr//2f194370083fflap00p1214l0m1.img+AND+RT+%3D+RAW'
# RH01
2r195521583effaq00p1301l0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0779/edr//2r195521583effaq00p1301l0m1.img+AND+RT+%3D+RAW'
2r195521583effaq00p1301r0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0779/edr//2r195521583effaq00p1301r0m1.img+AND+RT+%3D+RAW'
# FH02
2f191703508effanhep1110l0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0736/edr//2f191703508effanhep1110l0m1.img+AND+RT+%3D+RAW'
2f191703508effanhep1110r0m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit//mer2ho_0xxx/data/sol0736/edr//2f191703508effanhep1110r0m1.img+AND+RT+%3D+RAW'
# MI01
2m147677487eff8800p2976m2m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit///mer2mo_0xxx/data/sol0240/edr//2m147677487eff8800p2976m2m1.img+AND+RT+%3D+RAW'
2m147677547eff8800p2976m2m1.img:
	wget -O $@ 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/spirit///mer2mo_0xxx/data/sol0240/edr//2m147677547eff8800p2976m2m1.img+AND+RT+%3D+RAW'
# PAN02
1p227709787eff80k7p2389l2c1.img:
	wget -O $@ 'http://pds-geosciences.wustl.edu/mer/mer1-m-pancam-2-edr-sci-v1/mer1pc_0xxx/data/sol1121/1p227709787eff80k7p2389l2c1.img'
1p227709787eff80k7p2389r2c1.img:
	wget -O $@ 'http://pds-geosciences.wustl.edu/mer/mer1-m-pancam-2-edr-sci-v1/mer1pc_0xxx/data/sol1121/1p227709787eff80k7p2389r2c1.img'

# Clean up functions
##############################################

clean:
	rm -rf *.cahv *.cahvor *.cahvore nav01 pan01 mi01 fh01lin fh01 rh01 fh02 pan02

distclean: clean
	rm *.img *.IMG *~
