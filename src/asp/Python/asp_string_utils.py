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

"""
   Functions to help with string parsing.
"""

import sys, os, re, string, time, shlex

def isNumber(text):
    """Returns True if the text is a number"""
    
    try:
        float(text)
    except ValueError: # Not a number
        return False
    return True # No error, the number must have converted

def convertToFloatIfNumber(text):
    """If the text is a number returns float(text), otherwise just text"""
    
    try:
        a = float(text)
    except ValueError: # Not a number
        return text
    return a # No error, the number must have converted

def getLineAfterText(text, prefix, startPos=0, includeText=False):
    """Extracts the remainder of a line after a prefix"""
    # StartPos = index in the text to start looking at
    
    # Find the prefix
    prefixLoc = text.find(prefix, startPos)
    if prefixLoc < 0:
        raise Exception('Desired text not found: ' + prefix)
    
    # Find the bounds after the prefix
    prefixEnd = prefixLoc + len(prefix)
    nextEnd   = text.find('\n', prefixEnd)
    if nextEnd == -1: # If there was no trailing \n, use one past last character.
        nextEnd = len(text) 

    # Check that we found the bounds
    if (prefixEnd < startPos) or (nextEnd <= prefixEnd):
        raise Exception('Desired text not found: ' + prefix)
    
    # Extract the text
    if includeText:
        return text[prefixLoc:nextEnd]
    else:
        return text[prefixEnd:nextEnd]
    
def getNumbersInParentheses(text):
    """Returns all sets of numbers in parentheses"""
    
    numberSets = []
    
    # Find all number sets in the text
    parenGroups = re.findall( r'\([ \d.,-]*\)', text) 

    for p in parenGroups:
        numberText = p[1:-1] # Remove ()
        numberList = numberText.split(',')
        numbers = []
        for n in numberList: # Convert strings to floats
            numbers.append(float(n))
        numberSets.append(numbers) # Add this set of numbers to output list

    # If only one set of numbers return a list
    if len(numberSets) == 1:
        return numberSets[0]
    else: # Otherwise return a list of lists
        return numberSets
    

def getNumberAfterEqualSign(text, lineStart=0):
    """Extracts a number after an '=' sign on a line"""
    # LineStart = index in the text to start looking at
    
    numberText = getLineAfterText(text, '=', lineStart, False)

    return convertToFloatIfNumber(numberText)


# The following functions are useful for going between string and list
#  representations of command line arguments
def isNotString(a):
    """Returns true if the object is not a string"""
    return (not isinstance(a, basestring))

def argListToString(argList):
    """Converts a list of arguments into a single argument string"""

    string = ""
    for arg in argList:
        stringVersion = str(arg)

        # Wrap arguments with spaces in them in "" so they stay together
        if stringVersion.find(' ') >= 0:
            stringVersion = '"' + stringVersion + '" '

        if string == "":
            string = stringVersion
        else:
            string = string + ' ' + stringVersion
        
    return string

def stringToArgList(string):
    """Converts a single argument string into a list of arguments"""
    return shlex.split(string)

