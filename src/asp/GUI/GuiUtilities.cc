// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file GuiUtilities.cc
///
///

#include <string>
#include <vector>
#include <QPolygon>
#include <QtGui>
#include <QtWidgets>
#include <ogrsf_frmts.h>

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Image/Algorithms.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/tools/hillshade.h>
#include <vw/Core/RunOnce.h>
#include <asp/GUI/GuiUtilities.h>

using namespace vw;
using namespace vw::gui;
using namespace std;

namespace vw { namespace gui {

vw::RunOnce temporary_files_once = VW_RUNONCE_INIT;
boost::shared_ptr<TemporaryFiles> temporary_files_ptr;
void init_temporary_files() {
  temporary_files_ptr = boost::shared_ptr<TemporaryFiles>(new TemporaryFiles());
}

TemporaryFiles& temporary_files() {
  temporary_files_once.run( init_temporary_files);
  return *temporary_files_ptr;
}

bool isPolyZeroDim(const QPolygon & pa){
  
  int numPts = pa.size();
  for (int s = 1; s < numPts; s++){
    if (pa[0] != pa[s]) return false;
  }
  
  return true;
}
  
void popUp(std::string msg){
  QMessageBox msgBox;
  msgBox.setText(msg.c_str());
  msgBox.exec();
  return;
}

bool getStringFromGui(QWidget * parent,
		      std::string title, std::string description,
		      std::string inputStr,
		      std::string & outputStr){ // output
  outputStr = "";

  bool ok = false;
  QString text = QInputDialog::getText(parent, title.c_str(), description.c_str(),
				       QLineEdit::Normal, inputStr.c_str(),
				       &ok);

  if (ok) outputStr = text.toStdString();

  return ok;
}

bool supplyOutputPrefixIfNeeded(QWidget * parent, std::string & output_prefix){

  if (output_prefix != "") return true;

  bool ans = getStringFromGui(parent,
			      "Enter the output prefix to use for the interest point match file.",
			      "Enter the output prefix to use for the interest point match file.",
			      "",
				output_prefix);

  if (ans)
    vw::create_out_dir(output_prefix);

  return ans;
}

std::string fileDialog(std::string title, std::string start_folder){

  std::string fileName = QFileDialog::getOpenFileName(0,
                                      title.c_str(),
                                      start_folder.c_str()).toStdString();

  return fileName;
}

QRect bbox2qrect(BBox2 const& B){
  // Need some care here, an empty BBox2 can have its corners
  // as the largest double, which can cause overflow.
  if (B.empty()) 
    return QRect();
  return QRect(round(B.min().x()), round(B.min().y()),
               round(B.width()), round(B.height()));
}

bool write_hillshade(vw::cartography::GdalWriteOptions const& opt,
                     double azimuth, double elevation,
                     std::string const& input_file,
                     std::string      & output_file) {

  // Sanity check: Must have a georeference
  cartography::GeoReference georef;
  bool has_georef = vw::cartography::read_georeference(georef, input_file);
  if (!has_georef) {
    popUp("No georeference present in: " + input_file + ".");
    return false;
  }

  double scale       = 0.0;
  double blur_sigma  = std::numeric_limits<double>::quiet_NaN();
  double nodata_val  = std::numeric_limits<double>::quiet_NaN();
  vw::read_nodata_val(input_file, nodata_val);
  std::ostringstream oss;
  oss << "_hillshade_a" << azimuth << "_e" << elevation << ".tif"; 
  std::string suffix = oss.str();

  output_file = vw::mosaic::filename_from_suffix1(input_file, suffix);
  try {
    DiskImageView<float> input(input_file);
    try{
      bool will_write = vw::mosaic::overwrite_if_no_good(input_file, output_file,
                                             input.cols(), input.rows());
      if (will_write){
        vw_out() << "Writing: " << output_file << std::endl;
        vw::do_multitype_hillshade(input_file, output_file, azimuth, elevation, scale,
                                   nodata_val, blur_sigma);
      }
    }catch(...){
      // Failed to write, presumably because we have no write access.
      // Write the file in the current dir.
      vw_out() << "Failed to write: " << output_file << "\n";
      output_file = vw::mosaic::filename_from_suffix2(input_file, suffix);
      bool will_write = vw::mosaic::overwrite_if_no_good(input_file, output_file,
                                             input.cols(), input.rows());
      if (will_write){
        vw_out() << "Writing: " << output_file << std::endl;
        vw::do_multitype_hillshade(input_file,  output_file, azimuth, elevation, scale,
                                   nodata_val, blur_sigma);
      }
    }
  } catch (const Exception& e) {
    popUp(e.what());
    return false;
  }

  return true;
}

void readPolyFromOGR(OGRPolygon *poPolygon, std::string const& layer_str,
		     std::string const& poly_color,
		     vw::geometry::dPoly & poly){

  bool isPolyClosed = true; // only closed polygons are supported
  
  poly.reset();
  
  int numInteriorRings = poPolygon->getNumInteriorRings();
  
  // Read exterior and interior rings
  int count = -1;
  while (1){
    
    count++;
    OGRLinearRing *ring;
    
    if (count == 0){
      // Exterior ring
      ring = poPolygon->getExteriorRing();
      if (ring == NULL || ring->IsEmpty ()){
	// No exterior ring, that means no polygon
	break;
      }
    }else{
      // Interior rings
      int iRing = count - 1;
      if (iRing >= numInteriorRings)
	break; // no more rings
      ring = poPolygon->getInteriorRing(iRing);
      if (ring == NULL || ring->IsEmpty ()) continue; // go to the next ring
    }
    
    int numPoints = ring->getNumPoints();
    std::vector<double> x, y;
    x.clear(); y.clear();
    for (int iPt = 0; iPt < numPoints; iPt++){
      OGRPoint poPoint;
      ring->getPoint(iPt, &poPoint);
      x.push_back(poPoint.getX());
      y.push_back(poPoint.getY());
    }
    poly.appendPolygon(x.size(), vw::geometry::vecPtr(x), vw::geometry::vecPtr(y),
		       isPolyClosed, poly_color, layer_str);
    
  }
}
  
void read_shapefile(std::string const& file,
		    std::string const& poly_color,
		    bool & has_geo, 
		    vw::cartography::GeoReference & geo,
		    std::vector<vw::geometry::dPoly> & polyVec){
  
  // Make sure the outputs are initialized
  has_geo = false;
  geo = vw::cartography::GeoReference();
  polyVec.clear();
  
  std::string layer_str = fs::path(file).stem().string();

  vw_out() << "Reading layer: " << layer_str << " from: " << file << "\n";
  
  GDALAllRegister();
  GDALDataset * poDS;
  poDS = (GDALDataset*) GDALOpenEx(file.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
  if (poDS == NULL) 
    vw_throw(ArgumentErr() << "Could not open file: " << file << ".\n");
  
  OGRLayer  *poLayer;
  poLayer = poDS->GetLayerByName( layer_str.c_str() );
  if (poLayer == NULL)
    vw_throw(ArgumentErr() << "Could not find layer " << layer_str << " in file: "
	     << file << ".\n");

  // Read the georef.
  int nGeomFieldCount = poLayer->GetLayerDefn()->GetGeomFieldCount();
  char *pszWKT = NULL;
  if (nGeomFieldCount > 1) {
    for(int iGeom = 0; iGeom < nGeomFieldCount; iGeom ++ ){
      OGRGeomFieldDefn* poGFldDefn =
	poLayer->GetLayerDefn()->GetGeomFieldDefn(iGeom);
      OGRSpatialReference* poSRS = poGFldDefn->GetSpatialRef();
      if( poSRS == NULL )
	pszWKT = CPLStrdup( "(unknown)" );
      else {
	has_geo = true;
	poSRS->exportToPrettyWkt( &pszWKT );
	// Stop at the first geom
	break;
      }
    }
  }else{
    if( poLayer->GetSpatialRef() == NULL )
      pszWKT = CPLStrdup( "(unknown)" );
    else{
      has_geo = true;
      poLayer->GetSpatialRef()->exportToPrettyWkt( &pszWKT );
    }
  }
  geo.set_wkt(pszWKT);
  if (pszWKT != NULL)
    CPLFree( pszWKT );

  // There is no georef per se, as there is no image. The below forces
  // that the map from projected coordinates to pixel coordinates (point_to_pixel())
  // to be the identity.
  geo.set_pixel_interpretation(vw::cartography::GeoReference::PixelAsPoint);
  
  OGRFeature *poFeature;
  poLayer->ResetReading();
  while ( (poFeature = poLayer->GetNextFeature()) != NULL ) {

    OGRGeometry *poGeometry;
    poGeometry = poFeature->GetGeometryRef();
    
    if( poGeometry == NULL) {
      
      // nothing to do
      
    } else if (wkbFlatten(poGeometry->getGeometryType()) == wkbPoint ) {

      // Create a polygon with just one point
      
      OGRPoint *poPoint = (OGRPoint *) poGeometry;
      std::vector<double> x, y;
      x.push_back(poPoint->getX());
      y.push_back(poPoint->getY());
      
      vw::geometry::dPoly poly;
      bool isPolyClosed = true; // only closed polygons are supported
      poly.setPolygon(x.size(), vw::geometry::vecPtr(x), vw::geometry::vecPtr(y),
		      isPolyClosed, poly_color, layer_str);
      polyVec.push_back(poly);
      
    } else if (wkbFlatten(poGeometry->getGeometryType()) == wkbMultiPolygon){

      OGRMultiPolygon *poMultiPolygon = (OGRMultiPolygon *) poGeometry;
      int numGeom = poMultiPolygon->getNumGeometries();
      for (int iGeom = 0; iGeom < numGeom; iGeom++){
	
	const OGRGeometry *currPolyGeom = poMultiPolygon->getGeometryRef(iGeom);
	if (wkbFlatten(currPolyGeom->getGeometryType()) != wkbPolygon) continue;
	OGRPolygon *poPolygon = (OGRPolygon *) currPolyGeom;
	vw::geometry::dPoly poly;
	readPolyFromOGR(poPolygon, poly_color, layer_str, poly);

	polyVec.push_back(poly);
	
      }
      
    } else if (wkbFlatten(poGeometry->getGeometryType()) == wkbPolygon){
    
      OGRPolygon *poPolygon = (OGRPolygon *) poGeometry;
      vw::geometry::dPoly poly;
      readPolyFromOGR(poPolygon, poly_color, layer_str, poly);
      polyVec.push_back(poly);
    }
    
    OGRFeature::DestroyFeature( poFeature );
  }
  
  GDALClose( poDS );
}

void write_shapefile(std::string const& file,
		       bool has_geo,
		       vw::cartography::GeoReference const& geo, 
		       std::vector<vw::geometry::dPoly> const& polyVec){

  std::string layer_str = fs::path(file).stem().string();

  vw_out() << "Writing layer: " << layer_str << " to: " << file << "\n";

  const char *pszDriverName = "ESRI Shapefile";
  GDALDriver *poDriver;
  GDALAllRegister();
  poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
  if (poDriver == NULL ) 
    vw_throw(ArgumentErr() << "Could not find driver: " << pszDriverName << ".\n");

  GDALDataset *poDS;
  poDS = poDriver->Create(file.c_str(), 0, 0, 0, GDT_Unknown, NULL );
  if (poDS == NULL) 
    vw_throw(ArgumentErr() << "Failed writing file: " << file << ".\n");
  
  // Write the georef
  OGRSpatialReference spatial_ref;
  OGRSpatialReference * spatial_ref_ptr = NULL;
  if (has_geo){
    std::string srs_string = geo.get_wkt();
    if (spatial_ref.SetFromUserInput( srs_string.c_str() ))
      vw_throw( ArgumentErr() << "Failed to parse: \"" << srs_string << "\"." );
    spatial_ref_ptr = &spatial_ref;
  }
  
  OGRLayer *poLayer = poDS->CreateLayer(layer_str.c_str(),
					spatial_ref_ptr, wkbPolygon, NULL );
  if (poLayer == NULL)
    vw_throw(ArgumentErr() << "Failed creating layer: " << layer_str << ".\n");

#if 0
  OGRFieldDefn oField( "Name", OFTString );
  oField.SetWidth(32);
  if( poLayer->CreateField( &oField ) != OGRERR_NONE ) 
    vw_throw(ArgumentErr() << "Failed creating name field for layer: " << layer_str
	     << ".\n");
#endif
  
  for (size_t vecIter = 0; vecIter < polyVec.size(); vecIter++){

    vw::geometry::dPoly const& poly = polyVec[vecIter]; // alias
    if (poly.get_totalNumVerts() == 0) continue;
      
    OGRFeature *poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );
#if 0
    poFeature->SetField( "Name", "ToBeFilledIn" );
#endif
    
    const double * xv           = poly.get_xv();
    const double * yv           = poly.get_yv();
    const int    * numVerts     = poly.get_numVerts();
    int numPolys                = poly.get_numPolys();

    // Iterate over polygon rings, adding them one by one
    OGRPolygon P;
    int start = 0;
    for (int pIter = 0; pIter < numPolys; pIter++){

      if (pIter > 0) start += numVerts[pIter - 1];
      int pSize = numVerts[pIter];

      OGRLinearRing R;
      for (int vIter = 0; vIter < pSize; vIter++){
	double x = xv[start + vIter], y = yv[start + vIter];
	R.addPoint(x, y);
      }
    
      if (P.addRing(&R) != OGRERR_NONE ){
	// Presumably not much can be done if this failed
      }

    }
    
    poFeature->SetGeometry(&P); 
    
    if (poLayer->CreateFeature( poFeature ) != OGRERR_NONE)
      vw_throw(ArgumentErr() << "Failed to create feature in shape file.\n");
    
    OGRFeature::DestroyFeature( poFeature );
  }
  
  GDALClose( poDS );
}

void shapefile_bdbox(const std::vector<vw::geometry::dPoly> & polyVec,
		     // outputs
		     double & xll, double & yll,
		     double & xur, double & yur){
  
  double big = std::numeric_limits<double>::max();
  xll = big; yll = big; xur = -big; yur = -big;
  for (size_t p = 0; p < polyVec.size(); p++){
    if (polyVec[p].get_totalNumVerts() == 0) continue;
    double xll0, yll0, xur0, yur0;
    polyVec[p].bdBox(xll0, yll0, xur0, yur0);
    xll = std::min(xll, xll0); xur = std::max(xur, xur0);
    yll = std::min(yll, yll0); yur = std::max(yur, yur0);
  }

  return;
}

// This will tweak the georeference so that point_to_pixel() is the identity.
bool read_georef_from_shapefile(vw::cartography::GeoReference & georef,
				std::string const& file){
  
  if (!asp::has_shp_extension(file))
    vw_throw(ArgumentErr() << "Expecting a shapefile as input, got: " << file << ".\n");
  
  bool has_georef;
  std::vector<vw::geometry::dPoly> polyVec;
  std::string poly_color;
  read_shapefile(file, poly_color, has_georef, georef, polyVec);
  
  return has_georef;
}

bool read_georef_from_image_or_shapefile(vw::cartography::GeoReference & georef,
					 std::string const& file){
  
  if (asp::has_shp_extension(file)) 
    return read_georef_from_shapefile(georef, file);
  
  return vw::cartography::read_georeference(georef, file);
}
  
// Find the closest point in a given vector of polygons to a given point.
void findClosestPolyVertex(// inputs
			   double x0, double y0,
			   const std::vector<vw::geometry::dPoly> & polyVec,
			   // outputs
			   int & polyVecIndex,
			   int & polyIndexInCurrPoly,
			   int & vertIndexInCurrPoly,
			   double & minX, double & minY,
			   double & minDist
			   ){
  
  polyVecIndex = -1; polyIndexInCurrPoly = -1; vertIndexInCurrPoly = -1;
  minX = x0; minY = y0; minDist = std::numeric_limits<double>::max();
  
  for (int s = 0; s < (int)polyVec.size(); s++){
    
    double minX0, minY0, minDist0;
    int polyIndex, vertIndex;
    polyVec[s].findClosestPolyVertex(// inputs
                                     x0, y0,
                                     // outputs
                                     polyIndex, vertIndex, minX0, minY0, minDist0
                                     );
    
    if (minDist0 <= minDist){
      polyVecIndex  = s;
      polyIndexInCurrPoly = polyIndex;
      vertIndexInCurrPoly = vertIndex;
      minDist       = minDist0;
      minX          = minX0;
      minY          = minY0;
    }

  }

  return;
}

// Find the closest edge in a given vector of polygons to a given point.
void findClosestPolyEdge(// inputs
			 double x0, double y0,
			 const std::vector<vw::geometry::dPoly> & polyVec,
			 // outputs
			 int & polyVecIndex,
			 int & polyIndexInCurrPoly,
			 int & vertIndexInCurrPoly,
			 double & minX, double & minY,
			 double & minDist){
  
  polyVecIndex = -1; polyIndexInCurrPoly = -1; vertIndexInCurrPoly = -1;
  minX = x0; minY = y0; minDist = std::numeric_limits<double>::max();
  
  for (int s = 0; s < (int)polyVec.size(); s++){
    
    double minX0, minY0, minDist0;
    int polyIndex, vertIndex;
    polyVec[s].findClosestPolyEdge(// inputs
				   x0, y0,
				   // outputs
				   polyIndex, vertIndex, minX0, minY0, minDist0
				   );
    
    if (minDist0 <= minDist){
      polyVecIndex  = s;
      polyIndexInCurrPoly = polyIndex;
      vertIndexInCurrPoly = vertIndex;
      minDist       = minDist0;
      minX          = minX0;
      minY          = minY0;
    }

  }

  return;
}


void imageData::read(std::string const& name_in,
		     vw::cartography::GdalWriteOptions const& opt,
                     bool use_georef){
  m_opt = opt;
  name = name_in;
  std::string poly_color = "red";
  
  if (asp::has_shp_extension(name)){
    read_shapefile(name, poly_color, has_georef, georef, polyVec);
    
    double xll, yll, xur, yur;
    shapefile_bdbox(polyVec,  
		    xll, yll, xur, yur // outputs
		   );
    BBox2 world_bbox;
    world_bbox.min() = Vector2(xll, yll);
    world_bbox.max() = Vector2(xur, yur);

    // There is no definition of pixel for shapefiles. 
    image_bbox = world_bbox;
    
  }else{
    
    int top_image_max_pix = 1000*1000;
    int subsample = 4;
    img = DiskImagePyramidMultiChannel(name, m_opt, top_image_max_pix, subsample);
    
    has_georef = vw::cartography::read_georeference(georef, name);
    
    if (use_georef && !has_georef){
      popUp("No georeference present in: " + name + ".");
      vw_throw(ArgumentErr() << "Missing georeference.\n");
    }
    
    image_bbox = BBox2(0, 0, img.cols(), img.rows());
  }
}

vw::Vector2 QPoint2Vec(QPoint const& qpt) {
  return vw::Vector2(qpt.x(), qpt.y());
}

QPoint Vec2QPoint(vw::Vector2 const& V) {
  return QPoint(round(V.x()), round(V.y()));
}

// Allow the user to choose which files to hide/show in the GUI.
// User's choice will be processed by MainWidget::showFilesChosenByUser().
chooseFilesDlg::chooseFilesDlg(QWidget * parent):
  QWidget(parent){

  setWindowModality(Qt::ApplicationModal);
  
  int spacing = 0;
  
  QVBoxLayout * vBoxLayout = new QVBoxLayout(this);
  vBoxLayout->setSpacing(spacing);
  vBoxLayout->setAlignment(Qt::AlignLeft);
  
  // The layout having the file names. It will be filled in
  // dynamically later.
  m_filesTable = new QTableWidget();
  
  //m_filesTable->horizontalHeader()->hide();
  m_filesTable->verticalHeader()->hide();
    
  vBoxLayout->addWidget(m_filesTable);
  
  return;
}
  
chooseFilesDlg::~chooseFilesDlg(){}

void chooseFilesDlg::chooseFiles(const std::vector<imageData> & images){

  // See the top of this file for documentation.

  int numFiles = images.size();
  int numCols = 2;
  m_filesTable->setRowCount(numFiles);
  m_filesTable->setColumnCount(numCols);

  for (int fileIter = 0; fileIter < numFiles; fileIter++){

    // Checkbox
    QTableWidgetItem *item = new QTableWidgetItem(1);
    item->data(Qt::CheckStateRole);
    item->setCheckState(Qt::Checked);
    m_filesTable->setItem(fileIter, 0, item);

    // Set the filename in the table
    string fileName = images[fileIter].name;
    item = new QTableWidgetItem(fileName.c_str());
    item->setFlags(Qt::NoItemFlags);
    item->setForeground(QColor::fromRgb(0, 0, 0));
    m_filesTable->setItem(fileIter, numCols - 1, item);

  }

  QStringList rowNamesList;
  for (int fileIter = 0; fileIter < numFiles; fileIter++) rowNamesList << "";
  m_filesTable->setVerticalHeaderLabels(rowNamesList);

  QStringList colNamesList;
  for (int colIter = 0; colIter < numCols; colIter++) colNamesList << "";
  m_filesTable->setHorizontalHeaderLabels(colNamesList);
  QTableWidgetItem * hs = m_filesTable->horizontalHeaderItem(0);
  hs->setBackground(QBrush(QColor("lightgray")));

  m_filesTable->setSelectionMode(QTableWidget::ExtendedSelection);
  string style = string("QTableWidget::indicator:unchecked ")
    + "{background-color:white; border: 1px solid black;}; " +
    "selection-background-color: rgba(128, 128, 128, 40);";

  m_filesTable->setSelectionMode(QTableWidget::NoSelection);
  m_filesTable->setStyleSheet(style.c_str());

  // Horizontal header caption
   QTableWidgetItem *item = new QTableWidgetItem("Hide/show all");
  item->setFlags(Qt::NoItemFlags);
  item->setForeground(QColor::fromRgb(0, 0, 0));
  m_filesTable->setHorizontalHeaderItem(1, item);
  
  m_filesTable->resizeColumnsToContents();
  m_filesTable->resizeRowsToContents();

  // The processing of user's choice happens in MainWidget::showFilesChosenByUser()

  return;
}


DiskImagePyramidMultiChannel::DiskImagePyramidMultiChannel(std::string const& base_file,
                             vw::cartography::GdalWriteOptions const& opt,
                             int top_image_max_pix,
                             int subsample):m_opt(opt),
                                                m_num_channels(0),
                                                m_rows(0), m_cols(0),
                                                m_type(UNINIT){
  if (base_file == "") return;

  // Instantiate the correct DiskImagePyramid then record information including
  //  the list of temporary files it created.
  try {
    m_num_channels = get_num_channels(base_file);
    if (m_num_channels == 1) {
      // Single channel image with float pixels.
      m_img_ch1_double = vw::mosaic::DiskImagePyramid<double>(base_file, m_opt);
      m_rows = m_img_ch1_double.rows();
      m_cols = m_img_ch1_double.cols();
      m_type = CH1_DOUBLE;
      temporary_files().files.insert(m_img_ch1_double.get_temporary_files().begin(), 
                                     m_img_ch1_double.get_temporary_files().end());
    }else if (m_num_channels == 2){
      // uint8 image with an alpha channel.
      m_img_ch2_uint8 = vw::mosaic::DiskImagePyramid< Vector<vw::uint8, 2> >(base_file, m_opt);
      m_num_channels = 2; // we read only 1 channel
      m_rows = m_img_ch2_uint8.rows();
      m_cols = m_img_ch2_uint8.cols();
      m_type = CH2_UINT8;
      temporary_files().files.insert(m_img_ch2_uint8.get_temporary_files().begin(), 
                                     m_img_ch2_uint8.get_temporary_files().end());
    } else if (m_num_channels == 3){
      // RGB image with three uint8 channels.
      m_img_ch3_uint8 = vw::mosaic::DiskImagePyramid< Vector<vw::uint8, 3> >(base_file, m_opt);
      m_num_channels = 3;
      m_rows = m_img_ch3_uint8.rows();
      m_cols = m_img_ch3_uint8.cols();
      m_type = CH3_UINT8;
      temporary_files().files.insert(m_img_ch3_uint8.get_temporary_files().begin(), 
                                     m_img_ch3_uint8.get_temporary_files().end());
    } else if (m_num_channels == 4){
      // RGB image with three uint8 channels and an alpha channel
      m_img_ch4_uint8 = vw::mosaic::DiskImagePyramid< Vector<vw::uint8, 4> >(base_file, m_opt);
      m_num_channels = 4;
      m_rows = m_img_ch4_uint8.rows();
      m_cols = m_img_ch4_uint8.cols();
      m_type = CH4_UINT8;
      temporary_files().files.insert(m_img_ch4_uint8.get_temporary_files().begin(), 
                                     m_img_ch4_uint8.get_temporary_files().end());
    }else{
      vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands.\n");
    }
  } catch (const Exception& e) {
      popUp(e.what());
      return;
  }
}

double DiskImagePyramidMultiChannel::get_nodata_val() const {
  
  // Extract the clip, then convert it from VW format to QImage format.
  if (m_type == CH1_DOUBLE) {
    return m_img_ch1_double.get_nodata_val();
  } else if (m_type == CH2_UINT8) {
    return m_img_ch2_uint8.get_nodata_val();
  } else if (m_type == CH3_UINT8) {
    return m_img_ch3_uint8.get_nodata_val();
  } else if (m_type == CH4_UINT8) {
    return m_img_ch4_uint8.get_nodata_val();
  }else{
    vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }
}
  
void DiskImagePyramidMultiChannel::get_image_clip(double scale_in, vw::BBox2i region_in,
                  bool highlight_nodata,
                  QImage & qimg, double & scale_out, vw::BBox2i & region_out) const{

  bool scale_pixels = (m_type == CH1_DOUBLE);
  vw::Vector2 bounds;

  // Extract the clip, then convert it from VW format to QImage format.
  if (m_type == CH1_DOUBLE) {

    bounds = m_img_ch1_double.get_approx_bounds();
    
    ImageView<double> clip;
    m_img_ch1_double.get_image_clip(scale_in, region_in, clip,
				    scale_out, region_out);
    formQimage(highlight_nodata, scale_pixels, m_img_ch1_double.get_nodata_val(), bounds,
	       clip, qimg);
  } else if (m_type == CH2_UINT8) {
    ImageView<Vector<vw::uint8, 2> > clip;
    m_img_ch2_uint8.get_image_clip(scale_in, region_in, clip,
                                 scale_out, region_out);
    formQimage(highlight_nodata, scale_pixels, m_img_ch2_uint8.get_nodata_val(), bounds,
	       clip, qimg);
  } else if (m_type == CH3_UINT8) {
    ImageView<Vector<vw::uint8, 3> > clip;
    m_img_ch3_uint8.get_image_clip(scale_in, region_in, clip,
                                 scale_out, region_out);
    formQimage(highlight_nodata, scale_pixels, m_img_ch3_uint8.get_nodata_val(), bounds,
	       clip, qimg);
  } else if (m_type == CH4_UINT8) {
    ImageView<Vector<vw::uint8, 4> > clip;
    m_img_ch4_uint8.get_image_clip(scale_in, region_in, clip,
          scale_out, region_out);
    formQimage(highlight_nodata, scale_pixels, m_img_ch4_uint8.get_nodata_val(), bounds,
	       clip, qimg);
  }else{
    vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }
}

std::string DiskImagePyramidMultiChannel::get_value_as_str(int32 x, int32 y) const {

  // Below we cast from Vector<uint8> to Vector<double>, as the former
  // refuses to print well.
  std::ostringstream os;
  if (m_type == CH1_DOUBLE) {
    os << m_img_ch1_double.bottom()(x, y, 0);
  } else if (m_type == CH2_UINT8) {
    os << Vector2(m_img_ch2_uint8.bottom()(x, y, 0));
  } else if (m_type == CH3_UINT8) {
    os << Vector3(m_img_ch3_uint8.bottom()(x, y, 0));
  } else if (m_type == CH4_UINT8) {
    os << Vector4(m_img_ch4_uint8.bottom()(x, y, 0));
  }else{
    vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }
  
  return os.str();
}
  
double DiskImagePyramidMultiChannel::get_value_as_double(int32 x, int32 y) const {
  if (m_type == CH1_DOUBLE) {
    return m_img_ch1_double.bottom()(x, y, 0);
  }else if (m_type == CH2_UINT8){
    return m_img_ch2_uint8.bottom()(x, y, 0)[0];
  }else{
    vw_throw(ArgumentErr() << "Unsupported image with " << m_num_channels << " bands\n");
  }
  return 0;
}

void PointList::push_back(std::list<vw::Vector2> pts) {
  std::list<vw::Vector2>::iterator iter  = pts.begin();
  while (iter != pts.end()) {
    m_points.push_back(*iter);
    ++iter;
  }
}

}} // namespace vw::gui
