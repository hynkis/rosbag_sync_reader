#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/simple_filter.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

// Global variable
bool stop_parsing_bag = false;

// A struct to hold the synchronized camera data 
// Struct to store stereo data
class CameraLidarData
{
public:
  sensor_msgs::CompressedImage::ConstPtr image_center, image_left, image_right;
  sensor_msgs::PointCloud2::ConstPtr pointclouds;
  
  CameraLidarData(const sensor_msgs::CompressedImage::ConstPtr &center_img, 
                  const sensor_msgs::CompressedImage::ConstPtr &left_img, 
                  const sensor_msgs::CompressedImage::ConstPtr &right_img,
                  const sensor_msgs::PointCloud2::ConstPtr &points) :
    image_center(center_img),
    image_left(left_img),
    image_right(right_img),
    pointclouds(points)
  {}
};

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    // message_filters::SimpleFilter::signalMessage(msg);
    this->signalMessage(msg);
  }
};

// Callback for synchronized messages
void callback(const sensor_msgs::CompressedImage::ConstPtr &msg_center_img,
                               const sensor_msgs::CompressedImage::ConstPtr &msg_left_img,
                               const sensor_msgs::CompressedImage::ConstPtr &msg_right_img,
                               const sensor_msgs::PointCloud2::ConstPtr &msg_points)
{
  CameraLidarData camLidar(msg_center_img, msg_left_img, msg_right_img, msg_points);
  
  // Do what you want! -----------------------------

  // Get CV Image (from compressed image message)
  cv::Mat img_center = cv::imdecode(cv::Mat(msg_center_img->data), 1); //convert compressed image data to cv::Mat
  cv::Mat img_left   = cv::imdecode(cv::Mat(msg_left_img->data), 1);
  cv::Mat img_right  = cv::imdecode(cv::Mat(msg_right_img->data), 1);

  cv::imshow("cam_center", img_center);
  cv::imshow("cam_left", img_left);
  cv::imshow("cam_right", img_right);
  
  // if push ESC key, destroy all windows & stop parsing bag
  char key = cv::waitKey(0);
  if (key == 27)
  {
    cv::destroyAllWindows();
    stop_parsing_bag = true;
    return;
  }
}
 
// Load bag
void loadBag(const std::string filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);

  // Topic names for reading
  std::string topic_center_camera = "/gmsl_camera1/compressed";
  std::string topic_left_camera = "/gmsl_camera3/compressed";
  std::string topic_right_camera = "/gmsl_camera2/compressed";
  std::string topic_lidar = "/velodyne_points";

  std::vector<std::string> topics;
  topics.push_back(topic_center_camera);
  topics.push_back(topic_left_camera);
  topics.push_back(topic_right_camera);
  topics.push_back(topic_lidar);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  // Set up fake subscribers to capture messages
  BagSubscriber<sensor_msgs::CompressedImage> sub_center_img, sub_left_img, sub_right_img;
  BagSubscriber<sensor_msgs::PointCloud2> sub_points;
  
  // Use time synchronizer to make sure we get properly synchronized images & pointclouds
  typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer <MySyncPolicy> sync(MySyncPolicy(10), sub_center_img, sub_left_img, sub_right_img, sub_points);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  // Load all messages
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    // Check stopping condition
    if (stop_parsing_bag)
    {
      bag.close();
      return;
    }

    // Read camera messages (Center)
    if (m.getTopic() == topic_center_camera || ("/" + m.getTopic() == topic_center_camera))
    {
      sensor_msgs::CompressedImage::ConstPtr center_img = m.instantiate<sensor_msgs::CompressedImage>();
      if (center_img != NULL)
      {
        sub_center_img.newMessage(center_img);
        // std::cout << "get compressed image center" << std::endl;
      }
    }
    // Read camera messages (Left)
    if (m.getTopic() == topic_left_camera || ("/" + m.getTopic() == topic_left_camera))
    {
      sensor_msgs::CompressedImage::ConstPtr left_img = m.instantiate<sensor_msgs::CompressedImage>();
      if (left_img != NULL)
      {
        sub_left_img.newMessage(left_img);
        // std::cout << "get compressed image left" << std::endl;
      }
    }
    // Read camera messages (Right)
    if (m.getTopic() == topic_right_camera || ("/" + m.getTopic() == topic_right_camera))
    {
      sensor_msgs::CompressedImage::ConstPtr right_img = m.instantiate<sensor_msgs::CompressedImage>();
      if (right_img != NULL)
      {
        sub_right_img.newMessage(right_img);
        // std::cout << "get compressed image right" << std::endl;
      }
    }
    // Read LiDAR messages
    if (m.getTopic() == topic_lidar || ("/" + m.getTopic() == topic_lidar))
    {
      sensor_msgs::PointCloud2::ConstPtr points = m.instantiate<sensor_msgs::PointCloud2>();
      if (points != NULL)
      {
        sub_points.newMessage(points);
        // std::cout << "get pointcloud" << std::endl;
      }
    }
  }
  bag.close();
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "RosbagSyncReader");
  ros::NodeHandle _nh;
  std::string pkg_path = ros::package::getPath("rosbag_sync_reader");
  std::string bag_path = pkg_path + "/bags/example.bag";

  // Start to read messages
  loadBag(bag_path);
  
  return 0;
}

