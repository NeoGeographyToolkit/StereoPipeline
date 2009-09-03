#!/usr/bin/perl 
#===============================================================================
#
#         FILE:  run_ba_tests.pl
#
#        USAGE:  ./run_ba_tests.pl 
#
#  DESCRIPTION:  Runs a set of bundle adjustment tests
#
#      OPTIONS:  ---
# REQUIREMENTS:  ---
#         BUGS:  ---
#        NOTES:  ---
#       AUTHOR:  Michael Styer, <michael@styer.net>
#      COMPANY:  
#      VERSION:  1.0
#      CREATED:  09/03/2009 02:24:05 PM PDT
#     REVISION:  ---
#===============================================================================

use strict;
use warnings;

my $make_data          = './make_ba_test_data';
my $ba_test            = './ba_test';
my $ba_test_cfg        = 'ba_test.cfg';
my $cnet_file          = "noisy_control.cnet";
#my $camera_prefix      = "noisy_camera";
my $camera_prefix      = "camera";
my $camera_ext         = "tsai";
my $lambda             = 1;
my $iterations         = 10;
my $num_cameras        = 10;
my $min_tiepoints      = 1000;
my $cam_position_sigma = 1;
my $cam_pose_sigma     = 1e-16;
my $max_iterations     = 10;
my $ba_type            = "sparse";
my $num_tests          = 1;

my %test_config = (
    "pixel-inlier-noise-type"  => "normal",
    "pixel-inlier-df"          => 1,
    "pixel-inlier-sigma"       => 1,
    "pixel-outlier-noise-type" => "normal",
    "pixel-outlier-df"         => 1,
    "pixel-outlier-sigma"      => 1,
    "xyz-inlier-noise-type"    => "normal",
    "xyz-inlier-df"            => 1,
    "xyz-inlier-sigma"         => 1,
    "xyz-outlier-noise-type"   => "normal",
    "xyz-outlier-df"           => 1,
    "xyz-outlier-sigma"        => 1,
    "euler-inlier-noise-type"  => "normal",
    "euler-inlier-df"          => 1,
    "euler-inlier-sigma"       => 1,
    "euler-outlier-noise-type" => "normal",
    "euler-outlier-df"         => 1,
    "euler-outlier-sigma"      => 1,
    "min-tiepoints-per-image"  => 20,
    "number-of-cameras"        => $num_cameras,
    "lambda"                   => $lambda,
    "camera-position-sigma"    => $cam_position_sigma,
    "camera-pose-sigma"        => $cam_pose_sigma,
    "max-iterations"           => $max_iterations,
    "cnet"                     => $cnet_file,
    "bundle-adjustment-type"   => $ba_type
);
    
open(CONFIG, ">$ba_test_cfg") or die "open $ba_test_cfg failed: $?\n"; 
foreach (sort keys %test_config) {
    print CONFIG "$_=$test_config{$_}\n";
}
close CONFIG;

# create a set of test data
my @make_data = ($make_data);
system(@make_data) == 0 or die "@make_data failed: $?";

my @camera_files = ();
foreach (0 .. $num_cameras-1) {
    push(@camera_files, "$camera_prefix$_.$camera_ext");
}

# run ba_test
my @ba_test_opts = ('-s', '-v');
my @ba_test = ($ba_test, @ba_test_opts, @camera_files);
system(@ba_test) == 0 or die "@ba_test failed: $?";




