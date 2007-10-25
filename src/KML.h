#include <fstream>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>

class KMLStateVectorViz {
  std::ofstream m_output_file;
  double m_scale;
public:
  KMLStateVectorViz(std::string filename, std::string name, double scale = 1.0);
  ~KMLStateVectorViz();
  void close();
  void append_body_state(std::string name, vw::Vector3 position, vw::Quaternion<double> pose);

};



