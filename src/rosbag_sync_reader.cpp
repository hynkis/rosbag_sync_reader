#include <iostream>

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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

// Global parameters
bool SHOW_IMG = false;
bool SAVE_RAW_IMG = true;
bool SAVE_FUSION_IMG = true;
bool SAVE_DEPTH_IMG = true;
bool SAVE_POINTCLOUD_BIN = true;

std::string BAG_FILE_PATH = "/media/usrg/Samsung_T51/Carnival/21.12.16_sensor_data/kaist/2021-12-16-16-26-41-KAIST-CW.bag";

// Global variable
bool stop_parsing_bag = false;
// - For transformation
Eigen::Matrix4f trans_center, trans_left, trans_right;
std::vector<float> vec_center, vec_left, vec_right;

// Global function
void SavePNGImage(const cv::Mat img, std::string save_path, bool is_depth)
{
    if (is_depth)
    {
      img.convertTo(img, CV_16UC1);
    }
    cv::imwrite(save_path, img);
}

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

  // For file path
  std::string pkg_path = ros::package::getPath("rosbag_sync_reader");
  std::string file_name = std::to_string(msg_center_img->header.stamp.sec + msg_center_img->header.stamp.nsec * 1e-9);

  std::string save_path_image_center = pkg_path + "/image_center/" + file_name + ".png";
  std::string save_path_image_left   = pkg_path + "/image_left/" + file_name + ".png";
  std::string save_path_image_right  = pkg_path + "/image_right/" + file_name + ".png";

  std::string save_path_fusion_center = pkg_path + "/fusion_center/" + file_name + ".png";
  std::string save_path_fusion_left   = pkg_path + "/fusion_left/" + file_name + ".png";
  std::string save_path_fusion_right  = pkg_path + "/fusion_right/" + file_name + ".png";

  std::string save_path_depth_center = pkg_path + "/depth_center/" + file_name + ".png";
  std::string save_path_depth_left   = pkg_path + "/depth_left/" + file_name + ".png";
  std::string save_path_depth_right  = pkg_path + "/depth_right/" + file_name + ".png";

  std::string save_path_pointcloud_bin = pkg_path + "/point_cloud/" + file_name + ".bin";

  // Get CV Image (from compressed image message)
  cv::Mat img_center = cv::imdecode(cv::Mat(msg_center_img->data), 1); //convert compressed image data to cv::Mat
  cv::Mat img_left   = cv::imdecode(cv::Mat(msg_left_img->data), 1);
  cv::Mat img_right  = cv::imdecode(cv::Mat(msg_right_img->data), 1);

  // Get Pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // - message to PointCloud
  pcl::fromROSMsg(*msg_points, *cloud);
  // - create .bin file
  //  - reference: https://github.com/leofansq/Tools_RosBag2KITTI/blob/master/pcd2bin/pcd2bin.cpp
  //  - reference: https://github.com/PRBonn/lidar-bonnetal/issues/78
  
  // -- std::ios (out: mode to write file only / binary: mode for binary / app: add data from the end of the file)
  //  -- reference: http://tcpschool.com/cpp/cpp_io_fileMode
  std::ofstream bin_file(save_path_pointcloud_bin.c_str(), std::ios::out|std::ios::binary|std::ios::app);
  if(!bin_file.good())
  {
    throw std::runtime_error(std::string("Failed to open file: ") + save_path_pointcloud_bin);
  }

  // Fuse image and pointcloud
  cv::Mat fusion_img_center, fusion_img_left, fusion_img_right;
  img_center.copyTo(fusion_img_center);
  img_left.copyTo(fusion_img_left);
  img_right.copyTo(fusion_img_right);

  // Project points to depth image
  // - Initialize depth image (16 bit)
  cv::Mat depth_img_center(img_center.rows, img_center.cols, CV_16UC1, cv::Scalar(0));
  cv::Mat depth_img_left(img_left.rows, img_left.cols, CV_16UC1, cv::Scalar(0));
  cv::Mat depth_img_right(img_right.rows, img_right.cols, CV_16UC1, cv::Scalar(0));

  // Iteration w.r.t. points (for Visualization of the Camera-LiDAR fusion)
  int i = 0; // for iterator
  for (auto &point : cloud->points)
  {
      pcl::PointXYZI point_buf_center, point_buf_left, point_buf_right;
      
      // Depth for coloring
      double depth = sqrt(pow(point.x, 2.0) + pow(point.y, 2));
      int point_to_color = (1 - std::min(depth, 50.0) / 50.0) * 255.0;
      int depth_to_color = std::min(depth, 255.0) * 255.0;

      // =================================== //
      // ========== Center Camera ========== //
      // =================================== //
      // for cam + lidar
      point_buf_center.x = ((vec_center.at(0) * point.x + vec_center.at(1) * point.y + vec_center.at(2) * point.z + vec_center.at(3)) / (vec_center.at(8) * point.x + vec_center.at(9) * point.y + vec_center.at(10) * point.z + vec_center.at(11)));
      point_buf_center.y = ((vec_center.at(4) * point.x + vec_center.at(5) * point.y + vec_center.at(6) * point.z + vec_center.at(7)) / (vec_center.at(8) * point.x + vec_center.at(9) * point.y + vec_center.at(10) * point.z + vec_center.at(11)));
      point_buf_center.z = (vec_center.at(8) * point.x + vec_center.at(9) * point.y + vec_center.at(10) * point.z + vec_center.at(11));
      
      // - for camera-lidar fusion image
      // -- circle(InputOutputArray img, Point center, int radius, Scalar color, int thickness, int lineType, int shift)
      circle(fusion_img_center, cv::Point(point_buf_center.x, point_buf_center.y), 5, cv::Scalar(0, point_to_color, 0), -1, 8, 0);

      // - for depth image
      // -- circle(InputOutputArray img, Point center, int radius, Scalar color, int thickness, int lineType, int shift)
      // int depth_to_color = (1 - std::min(depth, 100.0) / 100.0) * 255.0 * 255.0;
      circle(depth_img_center, cv::Point(point_buf_center.x, point_buf_center.y), 5, cv::Scalar(depth_to_color), -1, 8, 0);

      // ================================= //
      // ========== Left Camera ========== //
      // ================================= //
      // for cam + lidar
      point_buf_left.x = ((vec_left.at(0) * point.x + vec_left.at(1) * point.y + vec_left.at(2) * point.z + vec_left.at(3)) / (vec_left.at(8) * point.x + vec_left.at(9) * point.y + vec_left.at(10) * point.z + vec_left.at(11)));
      point_buf_left.y = ((vec_left.at(4) * point.x + vec_left.at(5) * point.y + vec_left.at(6) * point.z + vec_left.at(7)) / (vec_left.at(8) * point.x + vec_left.at(9) * point.y + vec_left.at(10) * point.z + vec_left.at(11)));
      point_buf_left.z = (vec_left.at(8) * point.x + vec_left.at(9) * point.y + vec_left.at(10) * point.z + vec_left.at(11));
      
      // - for camera-lidar fusion image
      // -- circle(InputOutputArray img, Point center, int radius, Scalar color, int thickness, int lineType, int shift)
      circle(fusion_img_left, cv::Point(point_buf_left.x, point_buf_left.y), 5, cv::Scalar(0, point_to_color, 0), -1, 8, 0);

      // - for depth image
      // -- circle(InputOutputArray img, Point center, int radius, Scalar color, int thickness, int lineType, int shift)
      // int depth_to_color = (1 - std::min(depth, 100.0) / 100.0) * 255.0 * 255.0;
      circle(depth_img_left, cv::Point(point_buf_left.x, point_buf_left.y), 5, cv::Scalar(depth_to_color), -1, 8, 0);

      // ================================== //
      // ========== Right Camera ========== //
      // ================================== //
      // for cam + lidar
      point_buf_right.x = ((vec_right.at(0) * point.x + vec_right.at(1) * point.y + vec_right.at(2) * point.z + vec_right.at(3)) / (vec_right.at(8) * point.x + vec_right.at(9) * point.y + vec_right.at(10) * point.z + vec_right.at(11)));
      point_buf_right.y = ((vec_right.at(4) * point.x + vec_right.at(5) * point.y + vec_right.at(6) * point.z + vec_right.at(7)) / (vec_right.at(8) * point.x + vec_right.at(9) * point.y + vec_right.at(10) * point.z + vec_right.at(11)));
      point_buf_right.z = (vec_right.at(8) * point.x + vec_right.at(9) * point.y + vec_right.at(10) * point.z + vec_right.at(11));
      
      // - for camera-lidar fusion image
      // -- circle(InputOutputArray img, Point center, int radius, Scalar color, int thickness, int lineType, int shift)
      circle(fusion_img_right, cv::Point(point_buf_right.x, point_buf_right.y), 5, cv::Scalar(0, point_to_color, 0), -1, 8, 0);

      // - for depth image
      // -- circle(InputOutputArray img, Point center, int radius, Scalar color, int thickness, int lineType, int shift)
      // int depth_to_color = (1 - std::min(depth, 100.0) / 100.0) * 255.0 * 255.0;
      circle(depth_img_right, cv::Point(point_buf_right.x, point_buf_right.y), 5, cv::Scalar(depth_to_color), -1, 8, 0);

      // =========================== //
      // ========== LiDAR ========== //
      // =========================== //
      // - from pointcloud msg to bin
      if (SAVE_POINTCLOUD_BIN)
      {
        bin_file.write((char*)&cloud->points[i].x, sizeof(float));
        bin_file.write((char*)&cloud->points[i].y, sizeof(float));
        bin_file.write((char*)&cloud->points[i].z, sizeof(float));
        bin_file.write((char*)&cloud->points[i].intensity, sizeof(float));
        // std::cout << "x, y, z, i : \n";
        // std::cout << cloud->points[i].x << " ";
        // std::cout << cloud->points[i].y << " ";
        // std::cout << cloud->points[i].z << " ";
        // std::cout << cloud->points[i].intensity;
        // std::cout << std::endl;
      }

      i++; // for iterator
  }

  // close bin file
  bin_file.close();

  if (img_center.empty() || img_left.empty() || img_right.empty())
  {
      std::cout << "image is empty" << std::endl;
  }
  else
  {
      // Show current images
      if (SHOW_IMG)
      {
        cv::imshow("cam_center", img_center);
        cv::imshow("cam_left", img_left);
        cv::imshow("cam_right", img_right);
        // if push ESC key, destroy all windows & stop parsing bag
        char key = cv::waitKey(5);
        if (key == 27)
        {
          cv::destroyAllWindows();
          stop_parsing_bag = true;
          return;
        }
      }

      // Save image data     
      // - save raw image
      if (SAVE_RAW_IMG)
      {
        SavePNGImage(img_center, save_path_image_center, false);
        SavePNGImage(img_left, save_path_image_left, false);
        SavePNGImage(img_right, save_path_image_right, false);
      }
      // - save fuse image
      if (SAVE_FUSION_IMG)
      {
        SavePNGImage(fusion_img_center, save_path_fusion_center, false);
        SavePNGImage(fusion_img_left, save_path_fusion_left, false);
        SavePNGImage(fusion_img_right, save_path_fusion_right, false);
      }
      // - save depth image
      if (SAVE_DEPTH_IMG)
      {
        SavePNGImage(depth_img_center, save_path_depth_center, true);
        SavePNGImage(depth_img_left, save_path_depth_left, true);
        SavePNGImage(depth_img_right, save_path_depth_right, true);
      }

      // - save pointcloud bin (inside above pointcloud for-loop)


      std::cout << "Save data: " << file_name << std::endl;
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
  // std::string pkg_path = ros::package::getPath("rosbag_sync_reader");
  // std::string bag_path = pkg_path + "/bags/example.bag";
  std::string bag_path = BAG_FILE_PATH;

  // for center cam (4x4 LiDAR-to-Camera transformation matrix)
  trans_center << -0.197307, 0.393827, -0.00539561, -0.442059,
                  -0.159261, 0.00514399, 0.427143, -0.634573,
                  -0.000410322, -1.00294e-05, 1.87862e-05, -0.000903967,
                  0, 0, 0, 1;
  vec_center = {-0.197307, 0.393827, -0.00539561, -0.442059,
                -0.159261, 0.00514399, 0.427143, -0.634573,
                -0.000410322, -1.00294e-05, 1.87862e-05, -0.000903967,
                0, 0, 0, 1};;

  // for left cam (4x4 LiDAR-to-Camera transformation matrix)
  trans_left << 0.349557, -0.162533, 0.00420289, 0.666721,
                0.0997905, 0.0614498, -0.361939, 0.51198,
                0.000273125, 0.000207988, -1.23612e-05, 0.000656519,   
                0, 0, 0, 1;
  vec_left = {0.349557, -0.162533, 0.00420289, 0.666721,
              0.0997905, 0.0614498, -0.361939, 0.51198,
              0.000273125, 0.000207988, -1.23612e-05, 0.000656519, 
              0, 0, 0, 1};

  // for right cam (4x4 LiDAR-to-Camera transformation matrix)
  trans_right << -0.0885233, -0.517636, -0.0176861, -0.13794,
                  0.163749, -0.0916227, -0.511527, 0.638755,
                  0.000421721, -0.000293506, -9.07933e-06, 0.00073718,  
                  0, 0, 0, 1;
  vec_right = {-0.0885233, -0.517636, -0.0176861, -0.13794,
                0.163749, -0.0916227, -0.511527, 0.638755,
                0.000421721, -0.000293506, -9.07933e-06, 0.00073718,
                0, 0, 0, 1};
                
  // Start to read messages
  loadBag(bag_path);
  
  return 0;
}

