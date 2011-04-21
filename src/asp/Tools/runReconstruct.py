#!/usr/bin/env python

import os
import sys
import re
import stat
import time
import datetime
import logging
import tempfile
from glob import glob
from string import Template

######################################################################
# EDITABLE CONFIG
######################################################################

DEFAULT_MISSION = 'a15'
DEFAULT_DATA_DIR = '$HOME/$mission'
DEFAULT_RESULTS_DIR = '$HOME/results/$mission/${subsampleLevel}_${date}'
DEFAULT_RECONSTRUCT_BINARY = './reconstruct'
DEFAULT_SUB = 'sub16'
DEFAULT_NUM_PROCESSORS = 14
# 0 means process all
DEFAULT_NUM_FILES = 0

DEFAULT_STEPS = ['weights', '0', '1', '2', 'jpg', 'kml']

######################################################################
# GENERIC HELPERS
######################################################################

MAX_EXPAND_STACK_DEPTH = 4

def expand(tmpl, env):
    escaped = re.sub(r'\$\$', '00DOLLAR00', tmpl)
    for i in xrange(0, MAX_EXPAND_STACK_DEPTH):
        oldTmpl = tmpl
        tmpl = Template(tmpl).substitute(env)
        if tmpl == oldTmpl:
            break
    if tmpl != oldTmpl:
        raise ValueError('max expand stack depth exceeded -- variable reference loop?')
    unescaped = re.sub(r'00DOLLAR00', '$', tmpl)
    return unescaped

def dosys(cmd, stopOnErr=False):
    logging.info(cmd)
    ret = os.system(cmd)
    if stopOnErr and ret != 0:
        raise Exception('command exited with non-zero return value %d' % ret)
    return ret

def getTime(f):
    try:
        s = os.stat(f)
    except OSError:
        return 0
    return s[stat.ST_MTIME]

class JobQueue(object):
    def __init__(self, numProcessors=1, deleteTmpFiles=True):
        self.numProcessors = numProcessors
        self.deleteTmpFiles = deleteTmpFiles
        self.jobs = []
        
    def addJob(self, job, step=0):
        logging.debug('addJob %s [step %s]' % (job, step))
        self.jobs.append((step, job))
        
    def clear(self):
        self.jobs = []

    def run(self):
        maxStep = max([step for step, job in self.jobs])
        for currentStep in xrange(0, maxStep+1):
            #print '\n*** JobQueue: running step %d of %d ***\n' % (currentStep+1, maxStep+1)
            jobsForStep = [job for jobStep, job in self.jobs
                           if jobStep == currentStep]
            fd, jobsFileName = tempfile.mkstemp('jobQueueStep%d.txt' % currentStep)
            os.close(fd)
            jobsFile = file(jobsFileName, 'w')
            for job in jobsForStep:
                jobsFile.write('%s\n' % job)
            jobsFile.close()
            logging.info('Running JobQueue files from: %s' % jobsFileName)
            dosys('cat %s | xargs -t -P%d -I__cmd__ bash -c __cmd__' % (jobsFileName, self.numProcessors),
                  stopOnErr=True)
        if self.deleteTmpFiles:
            dosys('rm -f %s' % jobsFileName)
        self.clear()

jobQueueG = None

######################################################################
# RECONSTRUCT SCRIPTS
######################################################################

def run_weights(opts, drgs):
    pass # implement me

def runReconstruct(opts, drgs, step):
    opts.settings = 'photometry_init_%s_settings.txt' % step
    opts.allFiles = ' '.join(drgs)
    for f in drgs:
        opts.inputFile = f
        cmd = '$reconstructBinary -d $dataDir/DEM$undersub -s $dataDir/meta -e $dataDir/meta -r $resultsDir -c $settings -i $inputFile $allFiles'
        jobQueueG.addJob(expand(cmd, vars(opts)))
    jobQueueG.run()

def run_0(opts, drgs):
    runReconstruct(opts, drgs, 0)

def run_1(opts, drgs):
    runReconstruct(opts, drgs, 1)

def run_2(opts, drgs):
    runReconstruct(opts, drgs, 2)

def run_kml(opts, drgs):
    pass # implement me

def run_jpg(opts, drgs):
    resultsDir = expand('$resultsDir', vars(opts))
    jpgDir = '%s/albedoJpg' % resultsDir
    dosys('mkdir -p %s' % jpgDir)
    tifs = glob('%s/albedo/*.tif' % resultsDir)
    for tif in tifs:
        jpg = '%s/%s.jpg' % (jpgDir, os.path.splitext(os.path.basename(tif))[0])
        if getTime(tif) > getTime(jpg):
            jobQueueG.addJob('convert -contrast-stretch 0x0 %s %s' % (tif, jpg))
    jobQueueG.run()

def runStep(opts, drgs, stepName):
    logging.info("""
######################################################################
# RUNNING STEP %s
######################################################################
""" % stepName)
    funcName = 'run_%s' % stepName
    if funcName in globals():
        func = globals()[funcName]
    else:
        raise ValueError('invalid step name %s' % stepName)
    func(opts, drgs)

def runMaster(opts):
    global jobQueueG
    jobQueueG = JobQueue(opts.numProcessors, deleteTmpFiles=False)

    if opts.subsampleLevel:
        opts.undersub = '_' + opts.subsampleLevel
    else:
        opts.undersub = ''
    drgPattern = expand('$dataDir/DRG$undersub/*.tif', vars(opts))
    logging.info('Processing DRGs matching pattern: %s' % drgPattern)
    drgs = glob(drgPattern)
    drgs.sort()
    logging.info('Number of matches: %d' % len(drgs))
    if opts.numFiles > 0:
        logging.info('Limiting processing to first %d DRGs' % opts.numFiles)
        drgs = drgs[:opts.numFiles]
    logging.info('First DRG: %s' % drgs[0])
    logging.info('Last DRG:  %s' % drgs[-1])

    if opts.clean:
        dosys(expand('rm -rf $resultsDir', vars(opts)))

    for step in opts.steps:
        runStep(opts, drgs, step)

def main():
    import optparse
    parser = optparse.OptionParser('usage: %prog')

    parser.add_option('-c', '--clean',
                      action='store_true', default=False,
                      help='Delete output directory before beginning')

    parser.add_option('-m', '--mission',
                      default=DEFAULT_MISSION,
                      help='Mission [%default]')

    parser.add_option('-d', '--dataDir',
                      default=DEFAULT_DATA_DIR,
                      help='Data dir [%default]')

    parser.add_option('-r', '--resultsDir',
                      default=DEFAULT_RESULTS_DIR,
                      help='Results dir [%default]')

    parser.add_option('--reconstructBinary',
                      default=DEFAULT_RECONSTRUCT_BINARY,
                      help='Path to reconstruct binary [%default]')

    parser.add_option('-s', '--subsampleLevel',
                      default=DEFAULT_SUB,
                      help='Subsample level (can be empty) [%default]')

    parser.add_option('-P', '--numProcessors',
                      default=DEFAULT_NUM_PROCESSORS, type='int',
                      help='Number of processors to use [%default]')

    parser.add_option('-n', '--numFiles',
                      default=DEFAULT_NUM_FILES, type='int',
                      help='Number of files to process (0 to process all) [%default]')
    
    parser.add_option('--showConfig',
                      action='store_true', default=False,
                      help='Show config and exit')

    opts, args = parser.parse_args()
    if args:
        opts.steps = args
    else:
        opts.steps = DEFAULT_STEPS
    opts.HOME = os.environ['HOME']
    opts.date = time.strftime('%Y%m%d')
    if opts.showConfig:
        import json
        print json.dumps(vars(opts), indent=4, sort_keys=True)
        sys.exit(0)

    # set up logging
    resultsDir = expand('$resultsDir', vars(opts))
    timeStr = datetime.datetime.now().isoformat()
    timeStr = re.sub(r':', '', timeStr)
    timeStr = re.sub(r'\.\d+$', '', timeStr)
    fnameTmpl = resultsDir + '/reconstructLog-%s.txt'
    fname = fnameTmpl % timeStr
    fnameLatest = fnameTmpl % 'latest'
    if os.path.lexists(fnameLatest):
        os.unlink(fnameLatest)
    os.symlink(fname, fnameLatest)
    print 'Logging to %s' % fname
    logging.basicConfig(filename=fname,
                        level=logging.DEBUG,
                        format="%(levelname)-6s %(asctime)-15s %(message)s")

    runMaster(opts)

if __name__ == '__main__':
    main()
