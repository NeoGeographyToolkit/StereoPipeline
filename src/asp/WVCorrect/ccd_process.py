#!/usr/bin/python

# Process the average disparity to correct for ccd artifacts. Use as
# inputs the outputs of disp_avg. It is very important to note that
# the output of disp_avg will have two extra elements at the
# beginning, which are metadata, so the total number of values in that
# array is two more than the number of image columns. During
# processing this metadata will be removed.

# This expects numpy and matplotlib to be installed.

import sys, os, re, argparse
import numpy as np

usage = "python ccd_process.py <options> <inputs>-dx.txt"

parser = argparse.ArgumentParser(usage = usage,
                                 formatter_class = argparse.RawTextHelpFormatter)

parser.add_argument("--exclude",  dest = "exclude", type = int, default = 40,
                    help = "The number of (presumably inaccuate) values to exclude at "    + \
                    "the endpoints (they will be replaced with the value at the immedate " + \
                    "non-excluded value).")

parser.add_argument("--output-prefix",  dest = "output_prefix", default = None, 
                    help = "Save the processed results with this output prefix.")

parser.add_argument("--no-plot", action = "store_true", default = False,
                    dest = "no_plot",
                    help = "Do not plot the computed corrections.")

parser.add_argument("--plot-only", action = "store_true", default = False,
                    dest = "plot_only",
                    help = "Do not compute the CCD corrections, only " + \
                    "plot any corrections that were computed before.")

(options, files) = parser.parse_known_args(sys.argv)

def load_and_process(f, options):

    # Load the text file 
    x = np.loadtxt(f)

    if options.plot_only:
        return x
    
    x_len = len(x)

    # Replace the values at the end points as there correlation
    # does not do too well.
    for i in range(options.exclude):
        x[i] = x[options.exclude]
        x[x_len - i - 1] = x[x_len - options.exclude - 1]
        
    return x

def ccd_process(files, options):

    do_plot = (not options.no_plot)

    if do_plot:
        import matplotlib.pyplot as plt

    colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k']

    num_colors = len(colors)

    if do_plot:
        try:
            plt.figure(1, figsize=(16, 5)) # inches
            plt.subplot(211)
        except Exception as e:
            print("Could not launch a plot. Consider using this script with '--no-plot'.\n")
            print("Error was: " + str(e))
            sys.exit(1)

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

        if not options.plot_only:

            # Subtract the mean to overall not disturb the pixels too much,
            # with the correction, hence let the mean correction be zero.
            x = x - np.mean(x)

            # Compensate for the fact that the PAN images have exra 50
            # columns on the sides compared to the MS images at the PAN
            # resolution, which is 12.5, so 1/4, at the MS resolution
            # Don't do this given that we subtract the mean.
            # (x = x - 12.5)
            
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

        if not options.plot_only:
            
            # Subtract the mean as before
            y = y - np.mean(y)

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
        
    if not options.plot_only:

        print("\nWill average " + str(num_vals) + " dataset(s).")
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

if (not options.plot_only) and (options.output_prefix is None):
    print("Must specify an output prefix")
    parser.print_help()
    sys.exit(1)
    
ccd_process(files[1:], options)
