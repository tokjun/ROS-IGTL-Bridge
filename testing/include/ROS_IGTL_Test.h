#ifndef ROS_IGTL_TEST_H
#define ROS_IGTL_TEST_H
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Transform.h"
#include "image_transport/image_transport.h"

#include <ROS_IGTL_Bridge.h>

//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//// VTK Includes
//#include <vtkCellArray.h>
//#include <vtkIdList.h>
//#include <vtkPolyData.h>
//#include <vtkPoints.h>
//#include <vtkSmartPointer.h>
//#include <vtkPolyData.h>
//#include <vtkPolygon.h>
//#include <vtkCubeSource.h>
//#include <vtkVertex.h>
//#include <vtkPolyLine.h>
//#include <vtkTriangleStrip.h>
//#include <vtkFloatArray.h>
//#include <vtkTransform.h>
//#include <vtkPolyDataReader.h>
// C++ Includes
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdint.h>

// OCT Paramter---------------------------------------------------------
typedef struct octScanParameter{
	float   x_range;    /**< x scan dimension */
    float   y_range;    /**< y scan dimension */
    float   z_range;    /**< z scan dimension */
    float   x_spacing;    /**< x spacing */
    float   y_spacing;    /**< y spacing */
    float   z_spacing;    /**< z spacing */
    int32_t x_steps;    /**< scan steps in x */
    int32_t y_steps;    /**< scan steps in y */
    int32_t z_steps;    /**< scan steps in z */
    float   x_offset;   /**< scan area offset in x */
    float   y_offset;   /**< scan area offset in y */
}octScanParameter_t;

class ROS_IGTL_Test
{
public:
	ROS_IGTL_Test(int argc, char *argv[], const char* node_name);
	~ROS_IGTL_Test();
	void Run();
	void PublishOCTScan(std::string filepath);
	void tfunction();
	void test_sending();
	
private:
	ros::NodeHandle *nh;
	ros::Publisher point_pub;
	ros::Publisher transform_pub;
	//ros::Publisher polydata_pub;
	ros::Publisher image_pub;
	ros::Publisher string_pub;
	ros::Publisher pointcloud_pub;
	
	ros::Subscriber sub_point;
	ros::Subscriber sub_transform;
	//ros::Subscriber sub_polydata;
	ros::Subscriber sub_image;
	ros::Subscriber sub_string;
	
	virtual void pointCallback(const ros_igtl_bridge::igtlpoint::ConstPtr& msg);
	virtual void transformCallback(const ros_igtl_bridge::igtltransform::ConstPtr& msg);
	virtual void stringCallback(const ros_igtl_bridge::igtlstring::ConstPtr& msg);
	virtual void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
	//virtual void polydataCallback(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg);
};
#endif 
