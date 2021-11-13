#!/usr/bin/env python
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

# Thanks to Amaury Dehecq for contributing this tool.

import argparse, os, struct
import numpy as np

def read_ip_record(mf):
    """
    Read one IP record from the binary match file.
    Information comtained are x, y, xi, yi, orientation, scale, interest, polarity, octave, scale_lvl, desc 
    Input: - mf, file handle to the in put binary file (in 'rb' mode)
    Output: - iprec, array containing the IP record
    """
    x, y = np.frombuffer(mf.read(8), dtype=np.float32)
    xi, yi = np.frombuffer(mf.read(8), dtype=np.int32)
    orientation, scale, interest = np.frombuffer(mf.read(12), dtype=np.float32)
    polarity, = np.frombuffer(mf.read(1), dtype=np.int8)  # or np.bool?
    octave, scale_lvl = np.frombuffer(mf.read(8), dtype=np.uint32)
    ndesc, = np.frombuffer(mf.read(8), dtype=np.uint64)
    desc = np.frombuffer(mf.read(int(ndesc * 4)), dtype=np.float32)
    iprec = [x, y, xi, yi, orientation, scale, interest, polarity, octave, scale_lvl, ndesc]
    iprec.extend(desc)
    return iprec


def write_ip_record(out, iprec):
    """
    Just the reverse operation of read_ip_record.
    Inputs:
    - out: file handle to the output binary file (in 'wb' mode)
    - iprec: 1D array containing one IP record
    """
    out.write(struct.pack('f',iprec[0]))  # x, y
    out.write(struct.pack('f',iprec[1]))  
    out.write(struct.pack('i',iprec[2]))  # xi, yi
    out.write(struct.pack('i',iprec[3]))
    out.write(struct.pack('f',iprec[4]))  # orientation, scale, interest
    out.write(struct.pack('f',iprec[5]))
    out.write(struct.pack('f',iprec[6]))
    out.write(struct.pack('?',iprec[7]))  # polarity  # use fmt ? instead?
    out.write(struct.pack('I',iprec[8]))  # octave, scale_lvl
    out.write(struct.pack('I',iprec[9]))

    ndesc = iprec[10]

    # If the descriptors were not read, their true number
    # is less than specified.
    ndesc = len(iprec) - 11
    
    out.write(struct.pack('Q', ndesc))  # ndesc
    
    for k in range(ndesc):
        out.write(struct.pack('f',iprec[11+k]))  # desc
    return

        
def read_match_file(match_file):
    """
    Read a full binary match file. First two 8-bits contain the number of IPs in each image. Then contains the record for each IP, image1 first, then image2.
    Input: 
    - match_file: str, path to the match file
    Outputs:
    - two arrays, containing the IP records for image1 and image2.
    """

    # Open binary file in read mode
    print("Reading: " + match_file)
    mf = open(match_file,'rb')

    # Read record length
    size1 = np.frombuffer(mf.read(8), dtype=np.uint64)[0]
    size2 = np.frombuffer(mf.read(8), dtype=np.uint64)[0]

    # Read record for each image
    im1_ip = [read_ip_record(mf) for i in range(size1)]
    im2_ip = [read_ip_record(mf) for i in range(size2)]

    # Close file
    mf.close()
    
    return im1_ip, im2_ip


def write_match_file(outfile, im1_ip, im2_ip):
    """
    Write the full binary match file. 
    Inputs:
    - outfile: str, path to the output match file
    - im1_ip: array containing all the records for image1
    - im2_ip: array containing all the records for image2
    """

    # Open binary file in write mode
    print("Writing: " + outfile)

    try:
        os.makedirs(os.path.dirname(outfile))
    except OSError:
        pass
    
    out = open(outfile, 'wb')

    # Read records lengths
    size1 = len(im1_ip)
    size2 = len(im2_ip)

    # Write record length
    out.write(struct.pack('q',size1))
    out.write(struct.pack('q',size2))

    # Write records for both images
    for k in range(size1):
        write_ip_record(out, im1_ip[k])
    for k in range(size2):
        write_ip_record(out, im2_ip[k])
    return
        

if __name__ == '__main__':

    #Set up arguments
    parser = argparse.ArgumentParser(description='Convert an ASP (binary) match file into a text file. Use option -rev to do the reverse operation.')
    parser.add_argument('infile', type=str, help='Path to the input file.')
    parser.add_argument('outfile', type=str, help='Path to the output file.')
    parser.add_argument('-rev', dest='rev', help='Convert a text file into an ASP match file.',
                        action='store_true')
    args = parser.parse_args()

    
    if args.rev==False:

        # Read match file
        im1_ip, im2_ip = read_match_file(args.infile)

        # Save to text file
        print("Writing: " + args.outfile)

        try:
            os.makedirs(os.path.dirname(args.outfile))
        except OSError:
            pass
        
        with open(args.outfile, 'w') as out:

            # Write number of records
            out.write('%i %i\n' %(len(im1_ip), len(im2_ip)))

            # Write IPs for image1
            for i in range(len(im1_ip)):
                iprec = im1_ip[i]
                iprec_str = [str(a) for a in iprec]
                out.write(' '.join(iprec_str) + '\n')

            # Write IPs for image2
            for i in range(len(im2_ip)):
                iprec = im2_ip[i]
                iprec_str = [str(a) for a in iprec]
                out.write(' '.join(iprec_str) + '\n')

    else:

        # Read number of records
        f = open(args.infile,'r')
        print("Reading: " + args.infile)
        out = f.readline()
        size1, size2 = np.uint64(out.strip().split(' '))
        f.close()

        # Read each image IPs records. Note how we don't bother to read
        # the descriptors, which are a handful of values on each row beyond
        # the specified fields.
        try:
            im1_ipb = np.genfromtxt(args.infile,skip_header=1,dtype='float32, float32, int32, int32, float32, float32, float32, int8, uint32, uint32, uint64', max_rows=size1)
            im1_ipb = im1_ipb.reshape((size1,))
            im2_ipb = np.genfromtxt(args.infile,skip_header=1+int(size1),dtype='float32, float32, int32, int32, float32, float32, float32, int8, uint32, uint32, uint64', max_rows=size2)
            im2_ipb = im2_ipb.reshape((size2,))
        except Exception as e:
            print("Your numpy version (" + str(np.__version__) + ") may be too old. " +
                  "Got the error: " + str(e))

        # Save to output file
        write_match_file(args.outfile, im1_ipb, im2_ipb)

