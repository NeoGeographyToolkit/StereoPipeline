all: input_data navcam/navcam-PC.tif navcam/navcam.ive

input_data: 1n228235084eff8100p1962l0m1.tif 1n228235084eff8100p1962r0m1.tif 1n228235084eff8100p1962l0m1.cahvor 1n228235084eff8100p1962r0m1.cahvor

1n228235084eff8100p1962l0m1.img:
	wget -O 1n228235084eff8100p1962l0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1no_0xxx/data/sol1127/edr//1n228235084eff8100p1962l0m1.img+AND+RT+%3D+RAW'

1n228235084eff8100p1962r0m1.img:
	wget -O 1n228235084eff8100p1962r0m1.img 'http://pds-imaging.jpl.nasa.gov/pds/prod?q=OFSN+%3D+/data/mer/opportunity//mer1no_0xxx/data/sol1127/edr//1n228235084eff8100p1962r0m1.img+AND+RT+%3D+RAW'

%.tif: %.img
	gdal_translate -scale 330 2500 -ot byte $< $@

%.cahvor: %.img
	mer2camera $<

navcam/navcam-PC.tif: input_data
	stereo 1n228235084eff8100p1962l0m1.tif 1n228235084eff8100p1962r0m1.tif 1n228235084eff8100p1962l0m1.cahvor 1n228235084eff8100p1962r0m1.cahvor navcam/navcam -s stereo.default.navcam

navcam/navcam.ive: navcam/navcam-PC.tif
	point2mesh -s 1 -l navcam/navcam-PC.tif navcam/navcam-L.tif

clean:
	rm -rf *.cahv *.cahvor *.tif navcam

distclean: clean
	rm *.img *.IMG *~