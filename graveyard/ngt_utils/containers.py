## __BEGIN_LICENSE__
## Copyright (C) 2006-2010 United States Government as represented by
## the Administrator of the National Aeronautics and Space Administration
## All Rights Reserved.
## __END_LICENSE__

import collections
import threading
import Queue

KEY, PREV, NEXT = range(3)

class OrderedSet(collections.MutableSet):

    def __init__(self, iterable=None):
        self.end = end = [] 
        end += [None, end, end]         # sentinel node for doubly linked list
        self.map = {}                   # key --> [key, prev, next]
        if iterable is not None:
            self |= iterable

    def __len__(self):
        return len(self.map)

    def __contains__(self, key):
        return key in self.map

    def add(self, key):
        if key not in self.map:
            end = self.end
            curr = end[PREV]
            curr[NEXT] = end[PREV] = self.map[key] = [key, curr, end]

    def discard(self, key):
        if key in self.map:        
            key, prev, next = self.map.pop(key)
            prev[NEXT] = next
            next[PREV] = prev

    def __iter__(self):
        end = self.end
        curr = end[NEXT]
        while curr is not end:
            yield curr[KEY]
            curr = curr[NEXT]

    def __reversed__(self):
        end = self.end
        curr = end[PREV]
        while curr is not end:
            yield curr[KEY]
            curr = curr[PREV]

    def pop(self, last=False):
        if not self:
            raise KeyError('set is empty')
        key = next(reversed(self)) if last else next(iter(self))
        self.discard(key)
        return key

    def __repr__(self):
        if not self:
            return '%s()' % (self.__class__.__name__,)
        return '%s(%r)' % (self.__class__.__name__, list(self))

    def __eq__(self, other):
        if isinstance(other, OrderedSet):
            return len(self) == len(other) and list(self) == list(other)
        return not self.isdisjoint(other)

    def __del__(self):
        self.clear()                    # remove circular references

class LockingOrderedSet(OrderedSet):
    ''' Like an ordered set, but methods that modify the set have a lock around them to make it threadsafe. '''

    def __init__(self, *args, **kwargs):
        super(LockingOrderedSet, self).__init__(*args, **kwargs)
        self.lock = threading.RLock()

    def add(self, key):
        self.lock.acquire()
        try:
            super(LockingOrderedSet, self).add(key)
        finally:
            self.lock.release()
        
    def discard(self, key):
        self.lock.acquire()
        try:
            super(LockingOrderedSet, self).discard(key)
        finally:
            self.lock.release()
        
    def pop(self, last=False):
        self.lock.acquire()
        try:
            retval = super(LockingOrderedSet, self).pop(last=last)
        finally:
            self.lock.release()
        return retval
        
        
class UniquePriorityQueue(Queue.PriorityQueue):
    def __init__(self, maxsize):
        self.items = set()
        Queue.PriorityQueue.__init__(self, maxsize)
        
    def put(self, *args):
        item = args[0]
        if item not in self.items:
            self.items.add(item)
            Queue.PriorityQueue.put(self, *args)
    
    def get(self, *args):
        item = Queue.PriorityQueue.get(self, *args)
        self.items.remove(item)
        return item

class dotdict(dict):
    """ Dictionary subclass that gives you JavaScript-style dot-operator access to members. """
    def __getattr__(self, attr):
        return self.get(attr, None)
    __setattr__= dict.__setitem__
    __delattr__= dict.__delitem__

