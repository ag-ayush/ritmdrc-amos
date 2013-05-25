#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>

int width, height;
sensor_msgs::CameraInfo camera_info;
std::string cameraName, cfgPath, frameId;

std::string configFile() {
    return cfgPath + cameraName + ".txt";
}

/**
 * The ROS service responsible for receiving camera calibration parameters and reporting
 * them with the image transport.
 *
 * @param req The service request containing the new camera calibration.
 * @param rsp Unused
 * @return true if the parameters were successfully saved to the file, false on error.
 */
bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req,
		sensor_msgs::SetCameraInfo::Response &rsp) {

	ROS_INFO("New camera info received");
	camera_info = req.camera_info;

	if (camera_calibration_parsers::writeCalibrationIni(configFile(),
			cameraName, camera_info)) {
		ROS_INFO("Camera information written to %s.txt", configFile().c_str());
		return true;
	} else {
		ROS_ERROR("Could not write camera_parameters.txt");
		return false;
	}
}

int main(int argc, char** argv) {

	//Init the node with its default name
	ros::init(argc, argv, "opencv_cam");

	//Put the node in a private namespace to make it easier to run multiple camera sources.
	ros::NodeHandle nh("~");

	//Determine which video device number to use. Defaults to 0 (/dev/video0)
	int videoDev = -1;
	nh.param("videoDev", videoDev, 0);
	ROS_INFO("Using video device %d", videoDev);

	bool showCross;
	nh.param("showCross", showCross, false);

	//establish the width and height to use for the camera frames
	nh.param("width", width, 320);
	nh.param("height", height, 240);
    nh.param("cfgPath", cfgPath, std::string());
    nh.param("frameId", frameId, std::string("camera"));

	ROS_INFO("Using %dx%d resolution for camera", width, height);

	//The calibration file name is based on the namespace of the node instance.
	//First it removes the leading / and then replaces subsequent / with _
	cameraName = nh.getNamespace();
	if (cameraName.at(0) == '/') {
		cameraName = cameraName.substr(1, cameraName.length() - 1);
	}
	size_t idx = cameraName.find('/', 0);
	while (idx < std::string::npos) {
		cameraName.replace(idx, 1, "_");
	}

	ROS_INFO("Using %s as camera name.", cameraName.c_str());
    ROS_INFO("Using %s as configuration path.", cfgPath.c_str());
    ROS_INFO("Using %s as the frame_id.", frameId.c_str());

	//Initialize the OpenCV camera source and set the frame size.
	cv::VideoCapture cam(videoDev);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, width);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, height);

	//cv_bridge is used to handle converting the OpenCV image into the ROS image message.
	cv_bridge::CvImage cvIm;
	sensor_msgs::Image rosIm;

	std::string read_name;
	//Attempt to read in the camera parameters using the above camera name
	if (camera_calibration_parsers::readCalibrationIni(configFile(),
			read_name, camera_info)) {
		ROS_INFO(
				"Successfully read camera calibration.  Rerun camera calibrator if it is incorrect.");
	} else {
		ROS_ERROR(
				"No camera parameters file found.  Use default file if no other is available.");
	}

	//image_transport allows easy publishing of a video stream along with camera calibration and TF.
	//image_transport publishers will also use compression if a subscriber attaches to one of the
	//compressed transports, albeit at a significant computational cost.
	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher pub = it.advertiseCamera("image", 1);

	//Advertise the service that receives camera calibrations.
	ros::ServiceServer set_camera_info = nh.advertiseService("set_camera_info",
			setCameraInfo);

	while (ros::ok()) {
		//Extract a frame from the camera
		cam >> cvIm.image;

		//Make sure we got a frame and publish the image.
		if (!cvIm.image.empty()) {

            if (showCross) {
            	size_t w_2 = width / 2;
            	size_t h_2 = height / 2;

                cv::line(cvIm.image, cv::Point(0, h_2), cv::Point(width, h_2), cv::Scalar(0, 0, 0), 2, 8);
                cv::line(cvIm.image, cv::Point(w_2, 0), cv::Point(w_2, height), cv::Scalar(0, 0, 0), 2, 8);
            }

			//Again, using cv_bridge's CvImage type to handle the conversion to a ROS message
			cvIm.toImageMsg(rosIm);

			//Apparently the cv_bridge function doesn't correctly set the encoding, so do it here.
			rosIm.encoding = "bgr8";

			//Also stamp the image to the current time for logging / synchronization
			rosIm.header.stamp = ros::Time::now();

            rosIm.header.frame_id = frameId;
            camera_info.header.frame_id = frameId;

			//And release the image to the wild
			pub.publish(rosIm, camera_info);
		}

		ros::spinOnce();
	}

	return 0;
}
