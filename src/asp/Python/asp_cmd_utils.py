#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __BEGIN_LICENSE__
#  Copyright (c) 2009-2013, United States Government as represented by the
#  Administrator of the National Aeronautics and Space Administration. All
#  rights reserved.
#
#  The NGT platform is licensed under the Apache License, Version 2.0 (the
#  "License"); you may not use this file except in compliance with the
#  License. You may obtain a copy of the License at
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
# __END_LICENSE__

# TODO: Move this to asp_system_utils

"""General systems related utilities"""

import sys, os, re, shutil, subprocess, string, time, errno, multiprocessing, shlex


class CmdRunException(Exception):
    '''Exception type indicating an error with a cmd call'''
    pass

def isCmdOption(arg):
    """Returns True if the string is a command line option,
    False otherwise (if it is an argument)"""

    # An option must start with '-' and not consist of all numbers
    if ( arg.startswith('-') and not re.match('^-[0-9.]+$', arg) ):
        return True
    else:
        return False
 
#==================================================
# This class implements a variant of OptionParser which ignores unknown options.

from optparse import (OptionParser,BadOptionError,AmbiguousOptionError)

class PassThroughOptionParser(OptionParser):

    # Overwrite the default implementation which deletes newlines
    def format_epilog(self, formatter):
        return self.epilog

    def _process_args(self, largs, rargs, values):
        while rargs:
            try:
                self._process_args2(largs,rargs,values)
            except (BadOptionError,AmbiguousOptionError) as e:
                # On failure, pass option to output list
                if sys.version_info < (2, 6, 0):
                    # Port to Python 2.4
                    p = re.match("^.*?no such option:\s*(.*?)$", e.msg)
                    if p:
                        largs.append(p.group(1))
                else:
                    largs.append(e.opt_str)


    # This version of the function successfully passes through negative numbers
    def _process_args2(self, largs, rargs, values):
        """_process_args(largs : [string],
                         rargs : [string],
                         values : Values)

        Process command-line arguments and populate 'values', consuming
        options and arguments from 'rargs'.  If 'allow_interspersed_args' is
        false, stop at the first non-option argument.  If true, accumulate any
        interspersed non-option arguments in 'largs'.
        """
        while rargs:
            arg = rargs[0]

            p = re.match('^-[0-9.]+$', arg) # Identify a numeric argument
            if p:
                del rargs[0]
                raise BadOptionError(arg)
                #self.error(_("%s unrecognized number in arguments") % arg)

            # We handle bare "--" explicitly, and bare "-" is handled by the
            # standard arg handler since the short arg case ensures that the
            # len of the opt string is greater than 1.
            if arg == "--":
                del rargs[0]
                return
            elif arg[0:2] == "--":
                # process a single long option (possibly with value(s))
                OptionParser._process_long_opt(self, rargs, values)
            elif arg[:1] == "-" and len(arg) > 1:
                # process a cluster of short options (possibly with
                # value(s) for the last one only)
                OptionParser._process_short_opts(self, rargs, values)
            elif self.allow_interspersed_args:
                largs.append(arg)
                del rargs[0]
            else:
                return                  # stop now, leave this arg in rargs

        # Say this is the original argument list:
        # [arg0, arg1, ..., arg(i-1), arg(i), arg(i+1), ..., arg(N-1)]
        #                            ^
        # (we are about to process arg(i)).
        #
        # Then rargs is [arg(i), ..., arg(N-1)] and largs is a *subset* of
        # [arg0, ..., arg(i-1)] (any options and their arguments will have
        # been removed from largs).
        #
        # The while loop will usually consume 1 or more arguments per pass.
        # If it consumes 1 (eg. arg is an option that takes no arguments),
        # then after _process_arg() is done the situation is:
        #
        #   largs = subset of [arg0, ..., arg(i)]
        #   rargs = [arg(i+1), ..., arg(N-1)]
        #
        # If allow_interspersed_args is false, largs will always be
        # *empty* -- still a subset of [arg0, ..., arg(i-1)], but
        # not a very interesting subset!
