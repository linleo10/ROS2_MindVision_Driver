#include "../include/cam/cam_node.h"

void my_parameter_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
	// Check if the event is a parameter set event
	if (event->event_type == rcl_interfaces::msg::ParameterEvent::EVENT_TYPE_SET) {
		// Trigger your desired function here
		// ...
		CamParams camParams;
		camParams.usingAe = get_parameter("usingAe").as_bool();
		camParams.usingAutoWb = get_parameter("usingAutoWb").as_bool();
		camParams.exposureTimeValue = get_parameter("exposureTimeValue").as_int();
		camParams.sharpnessValue = get_parameter("sharpnessValue").as_int();
		camParams.saturationValue = get_parameter("saturationValue").as_int();
	}
}

std::unique_ptr<sensor_msgs::msg::Image> DahengCamNode::convert_frame_to_msg(cv::Mat frame)
{
	std_msgs::msg::Header header;
	sensor_msgs::msg::Image ros_image;
	
	if(frame.size().width != image_width || frame.size().height != image_height)
	{
		cv::resize(frame, frame, cv::Size(image_width, image_height));
		RCLCPP_WARN(this->get_logger(), "Resize frame: width:%d height:%d", image_width, image_height);
	}

	ros_image.header.frame_id = this->frame_id;
	ros_image.header.stamp = this->get_clock()->now();
	ros_image.width = frame.size().width;
	ros_image.height = frame.size().height;
	ros_image.encoding = "bgr8";
		
	ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);  
	ros_image.is_bigendian = false;
	ros_image.data.assign(frame.datastart, frame.dataend);

	auto msg_ptr = std::make_unique<sensor_msgs::msg::Image>(ros_image);	  
	return msg_ptr;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto img_publisher = node->create_publisher<sensor_msgs::msg::Image>("mvs_img", 10);

	Camera camera;

	if (camera.initCamera() != CAMERA_STATUS_SUCCESS) {
		RCLCPP_INFO(this->get_logger(), "INIT CAMERA ERROR!");
	}

	auto node = rclcpp::Node::make_shared("publisher_node");
	
	node->declare_parameter<bool>("usingAe", false);
	node->declare_parameter<int>("exposureTimeValue", 9999);
	node->declare_parameter<int>("sharpnessValue", 0);
	node->declare_parameter<bool>("usingAutoWb", true);
	node->declare_parameter<int>("saturationValue", 0);

	auto parameter_event_subscriber = node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
		"/cam", 10, my_parameter_callback
	);

	camera.settingParams(false, 9999, 0, true, 50);
	camera.prepareCamera();
	while(rclcpp::ok()) {
		camera.convertToMat();
		sensor_msgs::msg::Image::UniquePtr msg = convert_frame_to_msg(camera.frame);
		img_publisher->publish(std::move(msg));
	}

	rclcpp::spin();
	rclcpp::shutdown();
	return 0;
}
