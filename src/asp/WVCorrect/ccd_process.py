#!/usr/bin/python

# Process the average disparity to correct for ccd artifacts. Use as
# inputs the outputs of disp_avg. It is very important to note that
# the output of disp_avg will have two extra elements at the
# beginning, which are metadata, so the total number of values in that
# array is two more than the number of image columns. During
# processing this metadata will be removed.

import sys, os, re, argparse
import numpy as np
import matplotlib.pyplot as plt

usage = "python ccd_process.py <options> <inputs>-dx.txt"

parser = argparse.ArgumentParser(usage = usage,
                                 formatter_class = argparse.RawTextHelpFormatter)

parser.add_argument("--exclude",  dest = "exclude", type = int, default = 40,
                    help = "The number of (presumably inaccuate) values to exclude at "      + \
                    "the endpoints (they will be replaced with the value at the immedate " + \
                    "non-excluded value).")

parser.add_argument("--output-prefix",  dest = "output_prefix", default = None, 
                    help = "Save the processed results with this output prefix.")

parser.add_argument("--plot-final-data", action = "store_true", default = False,
                    dest = "plot_final_data",
                    help = "If instead of reading the outputs of disp_avg to process, " + \
                    "read the outputs of previous invocations of this tool, purely to plot them.")

(options, files) = parser.parse_known_args(sys.argv)

def load_and_process(f, options):

    # Load the text file 
    x = np.loadtxt(f)

    if options.plot_final_data:
        return x
    
    # Remove the first two elements, those are metadata
    x = x[2:]

    x_len = len(x)

    # Replace the values at the end points as there correlation
    # does not do too well.
    for i in range(options.exclude):
        x[i] = x[options.exclude]
        x[x_len - i - 1] = x[x_len - options.exclude - 1]
        
    return x

def plot_data(files, options):

    do_plot = True
    colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k']

    num_colors = len(colors)

    if do_plot:
        plt.figure(1, figsize=(16, 5)) # inches
        plt.subplot(211)

    # Process x
    print("\nProcessing the x average disparity")
    mean_x = []
    num_vals = 0
    start = True
    for i in range(len(files)):
        f = files[i]
        print("Loading: " + f)
        x = load_and_process(f, options)

        num_vals = num_vals + 1

        if not options.plot_final_data:
            # Compensate for the fact that the PAN images have exra 50
            # columns on the sides compared to the MS images at the PAN
            # resolution, which is 12.5, so 1/4, at the MS resolution
            x = x - 12.5
            
            # Flip, since we will apply to the MS images to match PAN
            # rather as in reverse as computed.
            x = -x
            
            if start:
                # Initialize the mean
                mean_x = x[:]
                start = False
            else:
                mean_x = mean_x + x
            
        if do_plot:
            plt.plot(x, colors[i % num_colors])

    if do_plot:
        # Label the x plot
        #plt.ylim(-2, 2)
        plt.title('dx')

    if do_plot:
        # Set up the y plot
        plt.subplot(212)

    # Process y
    print("\nProcessing the y average disparity")
    mean_y = []
    start = True
    for i in range(len(files)):
        f = files[i]
        # switch to the y average disparity
        f = f.replace('x.txt', 'y.txt') 
        print("Loading: " + f)
        y = load_and_process(f, options)

        if not options.plot_final_data:
            
            # flip as before
            y = -y

            if start:
                # Initialize the mean
                mean_y = y[:]
                start = False
            else:
                mean_y = mean_y + y
            
        if do_plot:
            plt.plot(y, colors[i % num_colors])
            
    if do_plot:
        # Label the y plot
        #plt.ylim(-2, 2)
        plt.title('dy')
        
    if not options.plot_final_data:

        print("\nWill average " + str(num_vals) + " datasets")
        mean_x = mean_x / num_vals
        mean_y = mean_y / num_vals
        
        fx = options.output_prefix + '-dx.txt'
        print("Saving the processed x data at: " + fx)
        np.savetxt(fx, mean_x)
        
        fy = options.output_prefix + '-dy.txt'
        print("Saving the processed y data at: " + fy)
        np.savetxt(fy, mean_y)

    if do_plot:
        plt.show()

# Main
if len(files) <= 1:
    parser.print_help()
    sys.exit(1)

if (not options.plot_final_data) and (options.output_prefix is None):
    print("Must specify an output prefix")
    parser.print_help()
    sys.exit(1)
    
plot_data(files[1:], options)
