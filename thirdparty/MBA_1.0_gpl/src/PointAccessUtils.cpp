//===========================================================================
// GoTools - SINTEF Geometry Tools version 1.0
//
// Multilevel B-spline Approximation module
//
// Copyright (C) 2000-2005 SINTEF ICT, Applied Mathematics, Norway.
//
// This program is free software; you can redistribute it and/or          
// modify it under the terms of the GNU General Public License            
// as published by the Free Software Foundation version 2 of the License. 
//
// This program is distributed in the hope that it will be useful,        
// but WITHOUT ANY WARRANTY; without even the implied warranty of         
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          
// GNU General Public License for more details.                           
//
// You should have received a copy of the GNU General Public License      
// along with this program; if not, write to the Free Software            
// Foundation, Inc.,                                                      
// 59 Temple Place - Suite 330,                                           
// Boston, MA  02111-1307, USA.                                           
//
// Contact information: e-mail: tor.dokken@sintef.no                      
// SINTEF ICT, Department of Applied Mathematics,                         
// P.O. Box 124 Blindern,                                                 
// 0314 Oslo, Norway.                                                     
//
// Other licenses are also available for this software, notably licenses
// for:
// - Building commercial software.                                        
// - Building software whose source code you wish to keep private.        
//===========================================================================
#include <PointAccessUtils.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
using namespace std;


#ifdef SGI
#include <math.h>
#else
#include <cmath>
#endif
#ifdef WIN32
#include <minmax.h>
#endif

#include <stdio.h>


namespace UCBspl { 
int numberOfPoints(const char filename[]) {
  double x,y,z;
  int no=0;
  ifstream ifile(filename);
  if (!ifile) {
    cout << "ERROR: File does not exist (or empty?)\n" << endl;
    exit(-1);
  }
  
  do {
    ifile >> x >> y >> z;
    if (ifile)
      no++;
  } while(ifile);
#ifdef MBA_DEBUG
  cout << "No. of points = " << no << endl;
#endif
  return no;
}


void readScatteredData(const char filename[], std::vector<double>& X, std::vector<double>& Y, std::vector<double>& Z, int incr) {
  
#ifdef MBA_DEBUG
  cout << "\nReading data from file (every 1000 point marked with .)" << endl;
#endif
  
#ifdef MBA_DEBUG
  if (incr != 1)
    cout << "Using every " << incr << " point" << endl;
#endif
  
  ifstream ifile(filename);
  if (!ifile) {
    cout << "ERROR: File does not exist (or empty?)\n" << endl;
    exit(-1);
  }
  
  double x,y,z;
  int no=0;
  int noUsed = 0;
  
  do {
    ifile >> x >> y >> z;
    no++;
    if (ifile) {
#ifdef MBA_DEBUG
      if ((no%1000)==0)
        cout << ".";
#endif
      if ((no%incr) == 0) {
        noUsed++;
        X.push_back(x);
        Y.push_back(y);
        Z.push_back(z);
      }
    }
    
  } while(ifile);
  
  
  
#ifdef MBA_DEBUG
  cout << endl;
  
  cout << "No. of points in file = " << no-1;
  cout << ", No used = " << noUsed << endl << endl;
#endif
}

void gisGeo2Planar(std::vector<double>& X, std::vector<double>& Y) {
  
#ifdef MBA_DEBUG
  cout << "Converting GIS-data..." << endl;
#endif
  
  double umin, vmin, umax, vmax;
  umin = vmin =  1.0e+20;
  umax = vmax = -1.0e+20;
  
  std::vector<double>::iterator itx;
  std::vector<double>::iterator ity;
  for (itx = X.begin(), ity = Y.begin(); itx != X.end(); ++itx, ++ity) {
    umin = min(*itx, umin);
    vmin = min(*ity, vmin);
    
    umax = max(*itx, umax);
    vmax = max(*ity, vmax);
  }
  
  double earth_radius = 6378000.0;
  double deg_to_rad = atan(1.0)*4.0/180.;
  
  for (itx = X.begin(), ity = Y.begin(); itx != X.end(); ++itx, ++ity) {
    *itx = earth_radius*(*itx - umin)*deg_to_rad * cos(vmin*deg_to_rad);
    *ity = earth_radius*(*ity - vmin)*deg_to_rad;
  }
}

void printVRMLpoints(const char filename[], const vector<double>& X, const vector<double>& Y, const vector<double>& Z,
                               int incr,  
                               double scale) {
  
  
#ifdef MBA_DEBUG
  cout << "printVRMLpoints to: " << filename << endl; 
#endif
  
  bool bscale = false;
  if (scale != 1.0)
    bscale = true;
  
  ofstream os(filename);
  
  
  os << "#VRML V1.0 ascii" << endl;
  os << endl;
  
  os << "Separator {" << endl; 
  os << "  Coordinate3 {" << endl;
  os << "    point [" << endl;
  
  double xmin = *(std::min_element(X.begin(), X.end())); 
  double ymin = *(std::min_element(Y.begin(), Y.end()));
  
  
  std::vector<double>::const_iterator itx;
  std::vector<double>::const_iterator ity;
  std::vector<double>::const_iterator itz;
  int noPoints = X.size();
  int i;
  for (itx = X.begin(), ity = Y.begin(), itz = Z.begin(), i=0; i < noPoints; itx += incr, ity += incr, itz += incr, i+= incr) {
    double z = *itz;
    if (bscale)
      z *= scale;
    os << *itx - xmin << " " << *ity - ymin << " " << z << ",\n"; 
  }
  
  os << "    ]\n"; 
  os << "  }\n"; 
  
  os << "  PointSet {" << endl;
  os << "    startIndex 0" << endl;
  os << "    numPoints  -1" << endl;
  os << "  }" << endl;
  os << "}" << endl; 
}

void printVTKpoints(const char filename[], const vector<double>& X, const vector<double>& Y, const vector<double>& Z,
                              double scale) {

  ofstream os(filename);

  os << "# vtk DataFile Version 3.0" << endl;
  os << "This file was generated by class UCButils and contains vertices only" << endl;
  os << "ASCII" << endl;
  os << "DATASET POLYDATA" << endl;
  int noPoints = X.size();
  os << "POINTS " << noPoints << " float" << endl;

  for(int i = 0; i < noPoints; i++)
    os << X[i] << " " << Y[i] << " " << Z[i]*scale << '\n';

  os << "VERTICES " << 1 << " " << noPoints+1 << endl;
  os << noPoints;
  for (int i = 0; i < noPoints; i++)
    os << " " << i;
  
  os << endl;
}

void asciiXYZ2Bin(const char infile[], const char outfile[], int incr) {
  
#ifdef MBA_DEBUG
  cout << "AsciXYZ2Bin...to XYZ... arr ..." << endl;
#endif
  vector<double> x_arr;
  vector<double> y_arr;
  vector<double> z_arr;
  readScatteredData(infile, x_arr, y_arr, z_arr, incr);
  
  FILE* fp = fopen(outfile,"wb");
  
  int noPoints = x_arr.size();
  
  double* itx = &x_arr[0];   
  double* ity = &y_arr[0];   
  double* itz = &z_arr[0];   


  for (int i = 0; i < noPoints; i++, ++itx, ++ity, ++itz) {
    fwrite(itx,sizeof(double),1,fp);
    fwrite(ity,sizeof(double),1,fp);
    fwrite(itz,sizeof(double),1,fp);
  }
  fclose(fp);
}

void grid2scat(const char infile[], const char outfile[]) {
#ifdef MBA_DEBUG
  cout << "grid2scat..." << endl;
#endif
  int noX, noY;
  double xorig, yorig, dx, dy;
  
  ifstream is(infile);
  is >> noX >> noY >> xorig >> yorig >> dx >> dy;
  
  double z;
  ofstream os(outfile);
  
  for (int i = 0; i < noX; i++) {
    double x = xorig + (double)i*dx;
    for (int j = 0; j < noY; j++) {
      double y = yorig + (double)j*dy;
      is >> z;
      os << x << " " << y << " " << z << endl;
    }
  }
}

void asciiXYZ2Bin2(const char infile[], const char outfile[], int incr) {
  
#ifdef MBA_DEBUG
  cout << "AsciXYZ2Bin...to XarrYarrZarr !!! ..." << endl;
#endif
  vector<double> x_arr;
  vector<double> y_arr;
  vector<double> z_arr;
  readScatteredData(infile, x_arr, y_arr, z_arr, incr);
  
  FILE* fp = fopen(outfile,"wb");
  
  int noPoints = x_arr.size();
  
  double* itx = &x_arr[0];  
  double* ity = &y_arr[0];  
  double* itz = &z_arr[0];  


  fwrite(itx, sizeof(double), noPoints, fp);  
  fwrite(ity, sizeof(double), noPoints, fp);  
  fwrite(itz, sizeof(double), noPoints, fp);  

  fclose(fp);
}

void readScatteredDataBin(const char filename[], std::vector<double>& X, std::vector<double>& Y, std::vector<double>& Z, bool invZ) {
  
  
#ifdef MBA_DEBUG
  cout << "Read binary data from " << filename << " XYZarr..." << endl;
#endif
  
  FILE* fp = fopen(filename,"rb");
  
#ifdef MBA_DEBUG
  MBAclock rolex;
#endif
  long offset = 0L;
  int whence = SEEK_END;
  int status = fseek(fp, offset, whence);
  long pos = ftell(fp);
  
  rewind(fp);
  
  int noPoints = pos/24;
  X.resize(noPoints);
  Y.resize(noPoints);
  Z.resize(noPoints);
  

  double* itx = &X[0];  
  double* ity = &Y[0];  
  double* itz = &Z[0];  
  
  for (int i = 0; i < noPoints; i++, ++itx, ++ity, ++itz) {
    fread(itx,sizeof(double),1,fp);
    fread(ity,sizeof(double),1,fp);
    fread(itz,sizeof(double),1,fp);
    if (invZ)
      *itz *= -1.0;
  }
  
#ifdef MBA_DEBUG
  cout << "Time used on reading binary data = " << rolex.getInterval() << endl;
  cout << "No. of points read = " << noPoints << endl;
#endif
  
  fclose(fp);
}






void readScatteredDataBin2(const char filename[], std::vector<double>& X, std::vector<double>& Y, std::vector<double>& Z) {
  
  
#ifdef MBA_DEBUG
  cout << "Read binary data from " << filename << " XarrYarrZarr !!!..." << endl;
#endif
  
  FILE* fp = fopen(filename,"rb");
#ifdef MBA_DEBUG
  MBAclock rolex;
#endif
  long offset = 0L;
  int whence = SEEK_END;
  int status = fseek(fp, offset, whence);
  long pos = ftell(fp);
  
  rewind(fp);
  
  
  int noPoints = pos/24;
  X.resize(noPoints);
  Y.resize(noPoints);
  Z.resize(noPoints);
  
  double* itx = &X[0];  
  double* ity = &Y[0];  
  double* itz = &X[0];  


  fread(itx, sizeof(double), noPoints, fp);  
  fread(ity, sizeof(double), noPoints, fp);  
  fread(itz, sizeof(double), noPoints, fp);  


#ifdef MBA_DEBUG
  cout << "Time used on reading binary data = " << rolex.getInterval() << endl;
#endif
  
  fclose(fp);
}





        
static vector<string>* readFileList(const char metafile[]) {
  
  ifstream is(metafile);
  
  char cbuf[200];
  int noFiles = 0;
  while(is >> cbuf) {
    cout << "File name " << ++noFiles << ": " << cbuf << endl;
  }
  is.clear();
  is.seekg(0);
  
  cout << endl;
  
  vector<string>* infiles = new vector<string>(noFiles);
  
  for (int i = 0; i < noFiles; ++i) {
    is >> cbuf;
    (*infiles)[i] = cbuf;
    cout << "File name " << i+1 << ": " << (*infiles)[i].c_str() << endl;
  }
  
  return infiles;
}


static int noPointsInBinFile(const char* filename) {
  
  FILE* fp = fopen(filename,"rb");
  
  long offset = 0L;
  int whence = SEEK_END;
  int status = fseek(fp, offset, whence);
  long pos = ftell(fp);
  fclose(fp);
  
  return  ((int)pos)/24;
}

static void limits(std::vector<double>& X, std::vector<double>& Y, std::vector<double>& Z,
                   double& umin, double& vmin, double& zmin,
                   double& umax, double& vmax, double& zmax) {
  
  umin = vmin = zmin =  1.0e+20;
  umax = vmax = zmax = -1.0e+20;
  
  std::vector<double>::iterator itx;
  std::vector<double>::iterator ity;
  std::vector<double>::iterator itz;
  for (itx = X.begin(), ity = Y.begin(), itz = Z.begin(); itx != X.end(); ++itx, ++ity, ++itz) {
    umin = min(*itx, umin);
    vmin = min(*ity, vmin);
    zmin = min(*itz, zmin);
    
    umax = max(*itx, umax);
    vmax = max(*ity, vmax);
    zmax = max(*itz, zmax);
  }
}



void readScatteredDataFileSetBin(const char metafile[], std::vector<double>& X, std::vector<double>& Y, std::vector<double>& Z) {
  
#ifdef MBA_DEBUG
  cout << "Reading metafile with a list of files (full path)..." << endl;
#endif
  vector<string>* infiles = readFileList(metafile);
  int noFiles = infiles->size();
  
#ifdef MBA_DEBUG
  MBAclock rolex;
#endif
  int noTot = 0;
  int i;  
  for (i = 0; i < noFiles; i++) {  
    string filename = (*infiles)[i];
    int no = noPointsInBinFile(filename.c_str());
    noTot += no;
  }
#ifdef MBA_DEBUG
  cout << "Total no. of points = " << noTot << endl;
#endif
  
  X.resize(noTot);
  Y.resize(noTot);
  Z.resize(noTot);
  

  double* itx = &X[0];  
  double* ity = &Y[0];  
  double* itz = &Z[0];  
  
  for (i = 0; i < noFiles; i++) {
    
    string filename = (*infiles)[i];
    int no = noPointsInBinFile(filename.c_str());
#ifdef MBA_DEBUG
    cout << "No. points in file = " << no << endl;
#endif
    
    FILE* fp = fopen(filename.c_str(),"rb");
    
    for (int i = 0; i < no; i++, ++itx, ++ity, ++itz) {
      fread(itx,sizeof(double),1,fp);
      fread(ity,sizeof(double),1,fp);
      fread(itz,sizeof(double),1,fp);
    }
    fclose(fp);
  }
  
#ifdef MBA_DEBUG
  cout << "Time used on reading binary data = " << rolex.getInterval() << endl;
#endif
}

};
 
