#!/usr/bin/env python

"""Convert all TIFF files output by reconstruct into JPEG files viewable
in a web browser for debugging."""

import os
import stat
import re

BASE_DIR = '%s/projects/StereoPipeline/reconstructTest' % os.environ['HOME']
RESULTS_DIR = '%s/results' % BASE_DIR

COLORMAP = '%s/projects/VisionWorkbench/build/x86_64_linux_gcc4.1/bin/colormap' % os.environ['HOME']

def getTime(f):
    try:
        s = os.stat(f)
    except OSError:
        return 0
    return s[stat.ST_MTIME]

def dosys(cmd):
    print cmd
    os.system(cmd)

def convert(tif, opts):
    stem, suffix = os.path.splitext(tif)
    out = stem + '.jpg'
    if getTime(tif) > getTime(out) or opts.force:
        if re.search('dem', tif, re.IGNORECASE):
            # file is a 16-bit TIFF DEM -- colormap to tif then convert to jpg
            tmp = stem + '_colormap.tif'
            dosys('%s %s -o %s' % (COLORMAP, tif, tmp))
            dosys('convert %s %s' % (tmp, out))
            dosys('rm -f %s' % tmp)
            return 1
        else:
            # file is an 8-bit TIFF -- contrast stretch and convert to jpeg
            dosys('convert -contrast-stretch 0x0 %s %s' % (tif, out))
            return 1
    return 0

def doit(opts):
    tiffsName = '/tmp/convertAllTiffs.txt'
    dosys('find %s -name *.tif > %s' % (RESULTS_DIR, tiffsName))
    tiffsFile = file(tiffsName, 'r')
    tiffs = [line[:-1] for line in tiffsFile]
    tiffsFile.close()

    numConverted = 0
    for t in tiffs:
        numConverted += convert(t, opts)

    print '%d of %d images were up to date' % (len(tiffs) - numConverted, len(tiffs))

def main():
    import optparse
    parser = optparse.OptionParser('usage: convertAll.py')
    parser.add_option('-f', '--force',
                      action='store_true', default=False,
                      help='Convert all files, even if they are up to date')
    opts, args = parser.parse_args()
    doit(opts)

if __name__ == '__main__':
    main()
