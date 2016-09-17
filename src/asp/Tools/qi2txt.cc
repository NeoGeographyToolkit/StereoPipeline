/*
* Initial version was received at NSIDC from Serdar Manizade.  Modified
* significantly by Bruce Raup (braup@nsidc.org).
* 7-8-16 Modified by Cade "The Intern" Haley to accommodate 10 and 14-word data
* 	plus other output modes
* 
* qi2txt()  creates ascii output (to standard output) from qfit binary files
* (12-word input).  Output stream contains the laser lat/lon/elev/hhmmss
* (removed output of passive data and laser srt).  Originally written by Bob
* Swift/EG&G, Modified by Serdar Manizade 08-Feb-2010.  Output format includes
* title for each column, comma delimited.
*
* Input data are in big-endian format.  This reader automatically detects the
* endianness of the host machine and swaps if needed.
* 
*
* The output columns are different depending on whether the ATM records
* are 10, 12, or 14 words long (hence the need for 'scale' and 'multiply_table'
* below). The corresponding column information will be printed to standard output.
*/

/* Copied from qi2txt-readme.txt

OVERVIEW

This readme accompanies the IceBridge QFIT data reader: qi2txt

The qi2txt program reads binary data files from the Operation IceBridge ATM
instrument, which are available as the ILATM1B and BLATM1B product at the
National Snow and Ice Data Center (NSIDC), at

http://nsidc.org/data/ilatm1b.html

This software is available at

http://nsidc.org/data/icebridge/tools.html

DISCLAIMER

This software is provided as-is as a service to the user community in the
hope that it will be useful, but without any warranty of fitness for any
particular purpose or correctness.  Bug reports, comments, and suggestions
for improvement are welcome; please send to nsidc@nsidc.org.

CHANGELOG

v0.4 >> 7-8-16 Modified to accommodate 10 and 14-word data outputs
       plus more output modes:
        - Short output  -Coordinates only -First and Last -Print all

The program assumes by default that the input binary QFIT file is in big
endian format.  It tests the endianness of the host machine and swaps the
data to match that of the host machine.  To have the program assume the
data format is little endian, use the -L option.


Examples of using the reader:

Convert an entire binary input file to a (possibly huge) text file:
  $ ./qi2txt inputfile.qi > outfile_ascii.txt

Extract lat, lon, elevation only, skipping over the header line:
  $ ./qi2txt -S inputfile.qi > xyz.txt

Print the first few lines, and tell the program that the input file is in
little endian format:
  $ ./qi2txt -L inputfile.qi | head -n10

*/


#include <stdio.h>
#include <stdlib.h>



/*----------------------------------------------------------------------
 * Contents of define.h - operating system dependent stuff
 *----------------------------------------------------------------------*/

static const char define_h_rcsid[] = "$Id: define.h 16072 2010-01-30 19:39:09Z brodzik $";

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>

#ifndef DEBUG
#define NDEBUG
#endif
#include <assert.h>

#ifdef DEBUG_MALLOC
#include "dbmalloc.h"
#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#define NEVER FALSE

#ifndef PI
#define PI 3.141592653589793
#endif

#define radians(t) ( (t) * PI / 180.0)
#define degrees(t) ( (t) * 180.0 / PI)

#define nint(x) ((int)((x)+.5))

#define sign(x) ((x) < 0 ? -1 : 1)

#define streq(s1,s2) (strcmp(s1,s2) == 0)

#ifdef NEED_STRDUP
static char *t_sd_p;
#define strdup(string) \
  t_sd_p = (char *)malloc(strlen(string)+1); \
  if (t_sd_p) strcpy(t_sd_p, string); 
#endif

#define NUMBER(a) ((int)(sizeof(a)/sizeof(a[0])))

#define ABORT EXIT_FAILURE

#define error_exit(msg) {fprintf(stderr,"%s\n",msg); exit(ABORT);}

#define repeat do
#define until(condition) while(!(condition))

#define MAX_STRING 256

//typedef int bool;

typedef unsigned char byte1;
typedef unsigned short int byte2;
typedef unsigned int byte4;
typedef unsigned long long NSIDCbyte8;

typedef char int1;
typedef short int int2;
typedef int int4;
typedef long long NSIDCint8;

#define BYTE1_BITS CHAR_BIT
#define BYTE1_MAX UCHAR_MAX
#define BYTE2_MAX USHRT_MAX
#define BYTE4_MAX UINT_MAX
#define NSIDCBYTE8_MAX ULLONG_MAX

#define INT1_MAX SCHAR_MAX
#define INT2_MAX SHRT_MAX
#define INT4_MAX INT_MAX
#define NSIDCINT8_MAX LLONG_MAX

#define BYTE1_MIN 0
#define BYTE2_MIN 0
#define BYTE4_MIN 0
#define NSIDCBYTE8_MIN 0

#define INT1_MIN SCHAR_MIN
#define INT2_MIN SHRT_MIN
#define INT4_MIN INT_MIN
#define NSIDCINT8_MIN LLONG_MIN


/*----------------------------------------------------------------------
 * END contents of define.h
 *----------------------------------------------------------------------*/



#define SEEK_SET        0
#define MAXARG          14
#define LAT_MIN         0
#define LAT_MAX         90

/* There are system endian.h definitions, but I'm not sure these are available
* on all potential platforms.  This source file includes a small routine to
* test the endianness of the machine it's running on.
*/

#define MY_BIG_ENDIAN      0
#define MY_LITTLE_ENDIAN   1
#define MAX_NAME_LENGTH    1024
#define VERSION            0.4


int short_output = 0;
int coordinates_output = 0;
int first_n_last = 0;
int printall = 0;

int host_endianness;
int data_endianness;

/*======================================================================*/
/* byte swap function myswap */
int4 myswap(char *in, char* out, int4 len, int4 cnt) {
    int4 i,  j,  k, sp, ep;

    for (i=0; i<cnt; ++i) {

        sp = i*len;
        ep = sp+len-1;     /* ((i+1) * len)-1; */

        for (j=sp, k=0; j<=ep; ++j, ++k) {
            out[j] = in[ep-k];
        }
    }
    return(i);
}

/*======================================================================*/
int testendianness(void) {

    typedef union
    {
        int i;
        char c[4];
    } u;

    u temp;
    temp.i = 0x12345678;

    if (temp.c[0] == 0x12) {
      return(MY_BIG_ENDIAN);
    }
    else {
      return(MY_LITTLE_ENDIAN);
    }
}


/*======================================================================*/
int4 get_record_length(int4 *value, int4 *svalue, FILE *infile) {

    int4 nvar, ipart;

    fread((char *)value,sizeof(*value),1,infile);//Read single int4
 
    /* swap bytes if host machine is little-endian (e.g. PC) */
    if (host_endianness != data_endianness) {
      ipart = myswap((char*)value,(char*)svalue,4,1); // Swap the bytes in that int4
      nvar = *(svalue) / 4;
    }
    else {
      /* Sun Workstations, etc. */
      nvar = *(value) / 4;
    }

    /*  rewind file */
    fseek (infile,0L,SEEK_SET);

    /*  read past first record */	 
    fread((char *)value,sizeof(*value),nvar,infile); // Read again but length of that first integer into infile

    return( nvar );
}


/*======================================================================*/
/* Print function which accommodates a variety of modes  */
// word_format = values 0-2 representing 10, 12, and 14 word modes, respectively
// mode = Different print modes. 'n' = normal, 'h' = header only
// bufout = Pointer to the array containing the multiplied results

void printData(int word_format, char mode, double * bufout){
		
	if (short_output){
		if (mode == 'h'){
			fprintf(stdout,"# LATITUDE,LONGITUDE,ELEVATION,TIME-HHMMSS\n");
			return;
		}
		switch(word_format){
			case 0:
				fprintf(stdout,"%10.7f  %11.7f  %8.3f  %011.4f\n",
				bufout[1], bufout[2],bufout[3],bufout[9]);
			break;
			case 1:	
				fprintf(stdout,"%10.7f  %11.7f  %8.3f  %011.4f\n",
				bufout[1], bufout[2],bufout[3],bufout[11]);
			break;
			case 2:
			
				fprintf(stdout,"%10.7f  %11.7f  %8.3f  %011.4f\n",
				bufout[1], bufout[2],bufout[3],bufout[13]);
			break;
		}	
	}
	else if (coordinates_output){ //Longitude, latitude
		if (mode == 'n'){
			fprintf(stdout,"%.6f %.6f\n", bufout[2], bufout[1]);
		}
	}

	else { /* full output */
		switch(word_format){
			case 0: // 10 word
				if (mode == 'h'){
					fprintf(stdout,"# REL_TIME,LATITUDE,LONGITUDE,ELEVATION,strt_pulse_sigstr,ref_sigstr,azi,pitch,roll,time-hhmmss\n");
					return;
				}
				
				fprintf(stdout,
				"%10.6f %10.7f %11.7f %8.3f %7.0f %5.0f %5.0f %10.3f %11.3f %011.4f\n",
				bufout[0], bufout[1], bufout[2], bufout[3], bufout[4], bufout[5],
				bufout[6], bufout[7], bufout[8], bufout[9]);
			break;
			case 1:	// 12 word
				if (mode == 'h'){
					fprintf(stdout,"# REL_TIME,LATITUDE,LONGITUDE,ELEVATION,strt_pulse_sigstr,ref_sigstr,azi,pitch,roll,gps_dil_prec,pulse_width,time-hhmmss\n");
					return;
				}

				fprintf(stdout,
				"%10.6f %10.7f %11.7f %8.3f %7.0f %5.0f %5.0f %10.3f %11.3f %8.1f %10.1f %011.4f\n",
				bufout[0], bufout[1], bufout[2], bufout[3], bufout[4], bufout[5],
				bufout[6], bufout[7], bufout[8], bufout[9], bufout[10], bufout[11]);
			break;
			case 2: // 14 word
				if (mode == 'h'){
					fprintf(stdout,"# REL_TIME,LATITUDE,LONGITUDE,ELEVATION,strt_pulse_sigstr,ref_sigstr,azi,pitch,roll,passive_sig,pass_foot_lat,pass_foot_long,pass_foot_synth_elev,time-hhmmss\n");
					return;
				}

				fprintf(stdout,
				"%10.6f %10.7f %11.7f %8.3f %7.0f %5.0f %5.0f %10.3f %11.3f %7.0f %10.7f %10.7f %8.3f %011.4f\n",
				bufout[0], bufout[1], bufout[2], bufout[3], bufout[4], bufout[5], bufout[6],
				bufout[7], bufout[8], bufout[9], bufout[10], bufout[11], bufout[12], bufout[13]);
			break;
		}	
	}
}


int main(int argc, char *argv[]) {
    char infilename[MAX_NAME_LENGTH]; // 100 long c string
    FILE *infile; // Pointer to infile c string
    int4 value[MAXARG], svalue[MAXARG], gvalue[MAXARG];// value, s, and g w/ 14 spots each
    long unsigned int in_rec = 0, out_rec = 0;
    int4 neg_rec_count = 0;
    int4 j, nvar, ipart, multiply_rule;
    // This scale represents a combination of word sizes,
    // appropriately multiplied by 'multiply_matrix'
    double scale[MAXARG] = {
                              1.0e3,
                              1.0e6,
                              1.0e6,
                              1.0e3,
                              1.0,
                              1.0,
                              1.0e3,
                              1.0e3,
                              1.0e3,
                              1.0e1,
                              1.0e6,
                              1.0e6,
			      1.0e3,
			      1.0
                           };
    int multiply_table[3][MAXARG] = // 3 word formats, 14 max words
				    // 0 = don't multiply, 1 = multiply by scale[i], 2 = is timestamp
	{{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 0, 0, 0, 0},
	 { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 2, 0, 0},
	 { 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 2}};
    int word_format;
    double bufout[MAXARG];// 14 entries
    char *endian[] = { "big", "little" };
    float version = VERSION;

    host_endianness = testendianness();

    // Had to make some edits here to accept more than one argument at once!

    if (argc < 2) {
      fprintf(stderr, "%s version %g\n", argv[0], version );
      fprintf( stderr, "Usage:  %s [-S/-C/-L/-F/-P] inputfile\n", argv[0] );
      fprintf( stderr, "  Use -S to print shortened version (only LAT,LONG,ELEVATION,TIME)\n" );
      fprintf( stderr, "  Use -C to print coordinates only (for generating .spatial files)\n" );
      fprintf( stderr, "  Use -L to assume data is little-endian\n" );
      fprintf( stderr, "  Use -F to print only first and last valid lines in short form\n" );
      fprintf( stderr, "  Use -P to forceably print all lines, even if garbage (for debug)\n" );
      fprintf( stderr, "  By default the program assumes the data are big-endian\n" );
      fprintf( stderr, "  In either case, it tests the host architecture and will\n" );
      fprintf( stderr, "  try to match the data to that endianness.\n" );
      fprintf( stderr, "(Enter any key to exit)\n" );
      getchar();
      exit(1);
    }

    // To keep this code simple, this must be the last argument!
    int file_index = argc - 1;
    data_endianness = MY_BIG_ENDIAN;

    // Check for flags
    for (int i=1; i<argc-1; ++i) {
      if (strcmp(argv[i], "-S")==0) // Short mode
        short_output = 1;
      if (strcmp(argv[i], "-C")==0) // Coordinates only
        coordinates_output = 1;
      if (strcmp(argv[i], "-L")==0) // Little endian mode
        data_endianness = MY_LITTLE_ENDIAN;
      if (strcmp(argv[i], "-F")==0){ // Print first and last records only
        first_n_last = 1;
        short_output = 1;
      }
      if (strcmp(argv[i], "-P")==0) // Print all records, even negative
        printall = 1;
    }
    
    strncpy(infilename, argv[file_index], MAX_NAME_LENGTH); // Dump first arg name into 1024 long c string

    fprintf(stderr, "%s version %g\n", argv[0], version );
    fprintf(stderr, " Input file:  %s\n", infilename);
    fprintf(stderr, "This machine is %s endian.\n", endian[host_endianness]);
    fprintf(stderr, "Assuming data are %s endian.\n", endian[data_endianness]);
 
    infile=fopen(infilename,"rb"); // Open the filename in binary read mode, give pointer to file object
    if (infile==NULL) {
        fprintf(stderr, "cannot open input file\n");
        exit(1);
    }

    /*  read first record and verify fixed record length */
    nvar = get_record_length(value, svalue, infile);
    word_format = (nvar-10)/2; // 10word = 0, 12word = 1, 14word = 2
    // Quick error check
    if (word_format > 2){
	fprintf(stderr, "ERROR: Unexpected words/record %d\n", nvar);
	exit(1);
    }
    fprintf(stderr, "Number of words/record = %d\n", nvar);
    fprintf(stderr, "Skipping records with negative time values.\n");
      
 //    // OUTPUT FILE instead of stdout
 //   FILE *output = fopen("output.txt","w+");
 //   if (output==NULL){
 //       fprintf(stderr, "cant create output file");
 //       exit (1);
 //   }
    int found_first = 0;
    int found_last = 0;
    int nvar_mult = 0;

    printData(word_format, 'h', NULL); // Print headers
    
    
    // Begin fairly slow stdout print. Want speedup? fputs out to file instead!
    while (fread((char *)value,sizeof(*value),nvar,infile) != 0) {
        ++in_rec;


        /* swap bytes if host machine is little-endian (e.g. PC) */
        if (host_endianness != data_endianness) {
      	  ipart = myswap((char*)value,(char*)svalue,sizeof(*value),nvar);
        }
        else {
          /* host machine is big-endian (e.g. Sun Workstation) */
          for (j=0; j < nvar; j++) svalue[j] = value[j]; 
        }


        /*  skip header records which begin with negative integers */
        // if (*(svalue) >= 0 && (in_rec%20000)==0) {
        if (*(svalue) >= 0 || printall == 1) {
            /*  convert scaled integers to double precision reals */
            for (j=0; j < nvar; j++){
		multiply_rule=multiply_table[word_format][j];
		if (multiply_rule == 1){ // Regular divide (value / scale)
                    bufout[j] = svalue[j] / scale[j];
		}
		else if (multiply_rule == 2){ // Is an HHMMSS timestamp
		    bufout[j] = svalue[j] / 1.0e3;
		}
		else{ // Doesn't need to be scaled
		    bufout[j] = svalue[j];
		}
	    }
            /*  convert positive east longitude to negative when value exists */
            if (bufout[2] > 180) bufout[2] -= 360;
	             /*  xyz limited output for laser spot */   
            if (bufout[1] != 0.0 && bufout[3] > -9999){
		
		printData(word_format, 'n', bufout); ///////PRINT DATA
		
		/* SPECIAL: Track when first line, then last line is printed*/
		(found_first) ? (found_last=1) : (found_first=1);
                ++out_rec;
	    }
        }
        else {
          ++neg_rec_count;
        }

	/* SPECIAL: Print first and last lines only */
	if (first_n_last && found_first){
		nvar_mult++;
		if (found_last){
			break;
		}
		// Rewind line-by-line to find the last data point
		fseek(infile,-nvar*nvar_mult*sizeof(*value), SEEK_END);
	}
    }
    
    
    
    
    
 
    fprintf(stderr, "Number of records read = %ld\nNumber of records written = %ld\n", in_rec, out_rec);
    fprintf(stderr, "Number of negative time records skipped = %d\n", neg_rec_count);
    fclose(infile);

    
    if (out_rec == 0){
	fprintf(stderr, "\n\tWarning: No valid records found.");
	exit (2);
    }
//    fclose(output);

    return 0;
}

