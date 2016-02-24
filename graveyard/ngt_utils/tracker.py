#!/usr/bin/env python
# encoding: utf-8

## __BEGIN_LICENSE__
## Copyright (C) 2006-2010 United States Government as represented by
## the Administrator of the National Aeronautics and Space Administration
## All Rights Reserved.
## __END_LICENSE__

"""
tracker.py

Utility for tracking long iterative processes.
It can be used to wrap an iterator.
"""

import sys
import os
from datetime import datetime

class Tracker(object):
    """
    Tracks the progress of a given iterator.
    """
    def __init__(self, 
        iter=None, 
        report_every=1,
        target=None, 
        progress=False,
        name="TRACKER", 
        output_to=sys.stderr
    ):

        self.count = 0
        self.output_stream = output_to
        self.name = name
        self.iter = iter.__iter__() if iter != None else None
        if not target and iter and hasattr(iter, '__len__'):
            self.target = len(iter) or None
        elif not target and iter and hasattr(iter, 'count') and callable(iter.count):
            self.target = iter.count() or None # django querysets use count() instead of __len__
        else:
            self.target = target
        self.progress = progress
        if self.progress:
            assert self.target != None
        if self.target == 0:
            raise ValueError("Tracker can't deal with a target of 0.")
        self.report_every = report_every
        self.starttime = datetime.now()
    
    def _report_spew(self):
        if self.target:
            remaining = (datetime.now() - self.starttime) / self.count * (self.target - self.count)
            self.output_stream.write( "\r%s: %d of %d done. (%s remaining)\n" % (self.name, self.count, self.target, str(remaining)) )
        else:
            self.output_stream.write( "\r%s: %d done. (%s)\n" % (self.name, self.count, str(datetime.now() - self.starttime) ) )
            
    def _report_bar(self):
        scale = 80
        barlength = int(float(self.count) / float(self.target) * scale)
        #self.output_stream.write("\r"+''.join(( '=' for i in range(1,barlength)))+'>'+''.join((' ' for i in range(1,scale-barlength-1))) + " %d"%count)
        self.output_stream.write("\r[%s>%s]%d" % (''.join(['=' for i in range(1,barlength)]), ''.join([' ' for i in range(1,scale - barlength)]), self.count))
        self.output_stream.flush()
        if self.count == self.target: # last one...
            self.output_stream.write("\n")
            self.output_stream.flush()
    
    def _report(self):
        if self.progress:
            self._report_bar()
        else:
            self._report_spew()
    
    def next(self):
        self.count += 1
        if self.count % self.report_every == 0:
            if not self.target or self.count <= self.target:
                self._report()
        if self.iter:
            return self.iter.next()
        else:
            return self.count
            
    def __iter__(self):
        return self
        
class Progress(Tracker):
    ''' 
        A progress bar that's basically just an alias for Tracker
        Initialize with Progress(iterator), then just iterate over it.
        If the iterator does not have a __len__ or count() member, you must provide a "target" kwarg with the expected size.
    '''

    def __init__(self, *args, **kwargs):
        kwargs['progress'] = True
        Tracker.__init__(self, *args, **kwargs)       

def test():
    "Just a quick functional test."
    import time
    t=Tracker(target=1000, iter=range(1000), progress=True)
    for i in t:
        time.sleep(0.005)
    
    p = Progress(range(1000))
    for i in p:
        time.sleep(0.005)


if __name__ == '__main__':
    test()

