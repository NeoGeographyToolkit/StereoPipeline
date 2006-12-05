#include <cxxtest/TestSuite.h>
#include "stereo.h"
#include "GeoRef.h"
#include "vw/vw.h"
#include "vw/FileIO.h"

using namespace std;
using namespace vw;

class TestGDAL : public CxxTest::TestSuite {
 public:
  
  void testGDAL() {

    vil_image_view<vw_float> testImg(1024,1024,3);
    vnl_matrix<double> transform(3,3);

    for (unsigned int i = 0; i < testImg.ni(); i++) {
      for (unsigned int j = 0; j < testImg.nj(); j++) {
	testImg(i,j,0) = i;
	testImg(i,j,1) = j;
	testImg(i,j,2) = 0;
      }
    }

    transform.set_identity();
    
    printf("\n\nWriting floating point image to disk...\n");
    write_georef_file(testImg, 
		      "test.tif", 
		      "GTiff",
		      transform);

  }

};
