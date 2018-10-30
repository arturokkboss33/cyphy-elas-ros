/*
 Copywrite 2012. All rights reserved.
 Cyphy Lab, https://wiki.qut.edu.au/display/cyphy/Robotics,+Vision+and+Sensor+Networking+at+QUT
 Queensland University of Technology
 Brisbane, Australia

 Author: Patrick Ross
 Contact: patrick.ross@connect.qut.edu.au

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <elas_ros/ElasFrameData.h>

#include <elas.h>

//#define DOWN_SAMPLE

class Elas_Proc
{
public:
  Elas_Proc(const std::string& transport)
  {
    this->_local_nh = ros::NodeHandle("~");
    this->_local_nh.param("queue_size", queue_size_, 5);

    // Topics
    std::string stereo_ns = this->nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + this->nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + this->nh.resolveName("image"));
    std::string left_info_topic = stereo_ns + "/left/camera_info";
    std::string right_info_topic = stereo_ns + "/right/camera_info";

    // Create the elas processing class
    this->_local_nh.param("robotics_stereo_params", robotics_stereo_params_,false);
    std::string robotics_stereo_params = "robotics";
    if(robotics_stereo_params_){
      param.reset(new Elas::parameters(Elas::ROBOTICS));
    }else{
      param.reset(new Elas::parameters(Elas::MIDDLEBURY));
    }
#ifdef DOWN_SAMPLE
    param->subsampling = true;
#endif
    elas_.reset(new Elas(*param));

    // Check for user stereo camera model
    this->_local_nh.param("user_defined_camera_model", user_defined_camera_model_,false);
    ROS_INFO("User camera model: %d", user_defined_camera_model_);
    if(user_defined_camera_model_){

      int cam_info_height;
      int cam_info_width;
      std::string cam_info_distortion_model = "";
      std::vector<double> D_params;
      std::vector<double> K_params;
      std::vector<double> R_params;
      std::vector<double> P_params;
      int cam_info_binning_x;
      int cam_info_binning_y;

      ros::param::param("~cam_info_height", cam_info_height, cam_info_height);
      ros::param::param("~cam_info_width", cam_info_width, cam_info_width);
      ros::param::param("~cam_info_distortion_model", cam_info_distortion_model, cam_info_distortion_model);
      this->getArrayParameter<double>("D_params",D_params);
      this->getArrayParameter<double>("K_params",K_params);
      this->getArrayParameter<double>("R_params",R_params);
      this->getArrayParameter<double>("P_params",P_params);
      ros::param::param("~cam_info_binning_x", cam_info_binning_x, cam_info_binning_x);
      ros::param::param("~cam_info_binning_y", cam_info_binning_y, cam_info_binning_y);

      ROS_INFO("User defined stereo camera params ...");
      user_left_cam_info_.height = user_right_cam_info_.height = cam_info_height;
      ROS_INFO("Cam height: %d",cam_info_height);
      user_left_cam_info_.width = user_right_cam_info_.width = cam_info_width;
      ROS_INFO("Cam width: %d",cam_info_width);
      user_left_cam_info_.distortion_model = user_right_cam_info_.distortion_model = cam_info_distortion_model;
      ROS_INFO("Cam distortion model: %s",cam_info_distortion_model.c_str());
      ROS_INFO("Cam model D matrx:");
      for(unsigned int i_elem = 0; i_elem < D_params.size(); ++i_elem){
        user_left_cam_info_.D.push_back(D_params[i_elem]); user_right_cam_info_.D.push_back(D_params[i_elem]);
        ROS_INFO("* %.2f", D_params[i_elem]);
      }
      ROS_INFO("Cam model K matrx:");
      for(unsigned int i_elem = 0; i_elem < K_params.size(); ++i_elem){
        user_left_cam_info_.K[i_elem] = K_params[i_elem]; user_right_cam_info_.K[i_elem] = K_params[i_elem];
        ROS_INFO("* %.2f", K_params[i_elem]);
      }
      ROS_INFO("Cam model R matrx:");
      for(unsigned int i_elem = 0; i_elem < R_params.size(); ++i_elem){
        user_left_cam_info_.R[i_elem] = R_params[i_elem]; user_right_cam_info_.R[i_elem] = R_params[i_elem];
        ROS_INFO("* %.2f", R_params[i_elem]);
      }
      ROS_INFO("Cam model P matrx:");
      for(unsigned int i_elem = 0; i_elem < P_params.size(); ++i_elem){
        user_left_cam_info_.P[i_elem] = P_params[i_elem]; user_right_cam_info_.P[i_elem] = P_params[i_elem];
        ROS_INFO("* %.2f", P_params[i_elem]);
      }
      user_left_cam_info_.binning_x = user_right_cam_info_.binning_x = cam_info_binning_x;
      ROS_INFO("Cam binning X: %d",cam_info_binning_x);
      user_left_cam_info_.binning_y = user_right_cam_info_.binning_y = cam_info_binning_y;
      ROS_INFO("Cam binning Y: %d",cam_info_binning_y);
    }

    image_transport::ImageTransport it(this->nh);
    left_sub_.subscribe(it, left_topic, 1, transport);
    right_sub_.subscribe(it, right_topic, 1, transport);
    left_info_sub_.subscribe(this->nh, left_info_topic, 1);
    right_info_sub_.subscribe(this->nh, right_info_topic, 1);

    ROS_INFO("Subscribing to:\n%s\n%s\n%s\n%s",left_topic.c_str(),right_topic.c_str(),left_info_topic.c_str(),right_info_topic.c_str());

    image_transport::ImageTransport local_it(this->_local_nh);
    std::string stereo_frame = ros::names::clean(stereo_ns + "/left/");
    disp_pub_.reset(new Publisher(local_it.advertise(stereo_frame + "/image_disparity", 1)));
    depth_pub_.reset(new Publisher(local_it.advertise(stereo_frame + "/depth", 1)));
    pc_pub_.reset(new ros::Publisher(this->_local_nh.advertise<PointCloud>(stereo_frame + "/pointcloud", 1)));
    elas_fd_pub_.reset(new ros::Publisher(this->_local_nh.advertise<elas_ros::ElasFrameData>(stereo_frame + "/frame_data", 1)));

    pub_disparity_ = this->_local_nh.advertise<stereo_msgs::DisparityImage>(stereo_frame + "/disparity", 1);

    // Synchronize input topics. Optionally do approximate synchronization.
    bool approx;
    this->_local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),
                                                  left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
      approximate_sync_->registerCallback(boost::bind(&Elas_Proc::process, this, _1, _2, _3, _4));
    }
    else
    {
      exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
                                      left_sub_, right_sub_, left_info_sub_, right_info_sub_) );
      exact_sync_->registerCallback(boost::bind(&Elas_Proc::process, this, _1, _2, _3, _4));
    }

  }

  typedef image_transport::SubscriberFilter Subscriber;
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
  typedef image_transport::Publisher Publisher;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  template<typename ParameterType>
  bool getArrayParameter(const std::string &key, std::vector<ParameterType> &vec) {
      XmlRpc::XmlRpcValue list_;
      this->_local_nh.getParam(key, list_);
      ROS_ASSERT(list_.getType() == XmlRpc::XmlRpcValue::TypeArray);

      return getArrayParameter<ParameterType>(list_, vec);
  }

  template<typename ParameterType>
  bool getArrayParameter(XmlRpc::XmlRpcValue &list, std::vector<ParameterType> &vec) {

      if (!(list.getType() == XmlRpc::XmlRpcValue::TypeArray))
          return false;

      for (int i = 0; i < list.size(); ++i) {
          if ((list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble && boost::is_same<ParameterType, double>::value) || (list[i].getType() == XmlRpc::XmlRpcValue::TypeInt && boost::is_same<ParameterType, int>::value)
                  || (list[i].getType() == XmlRpc::XmlRpcValue::TypeString && boost::is_same<ParameterType, std::string>::value)) {
              vec.push_back(static_cast<ParameterType>(list[i]));
          } else if (list[i].getType() == XmlRpc::XmlRpcValue::TypeInt && boost::is_same<ParameterType, double>::value) {
              vec.push_back(static_cast<ParameterType>(list[i]));
          } else if (list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble && boost::is_same<ParameterType, int>::value) {
              vec.push_back(static_cast<ParameterType>(list[i]));
          } else {
              return false;
          }
      }
      return true;
  }

  void publish_point_cloud(const sensor_msgs::ImageConstPtr& l_image_msg, 
                           float* l_disp_data, const std::vector<int32_t>& inliers,
                           int32_t l_width, int32_t l_height,
                           const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
                           const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      pcl::PCLHeader l_info_header = pcl_conversions::toPCL(l_info_msg->header);

      PointCloud::Ptr point_cloud(new PointCloud());
      point_cloud->header.frame_id = l_info_header.frame_id;
      point_cloud->header.stamp = l_info_header.stamp;
      point_cloud->width = l_width;
      point_cloud->height = l_height;
      point_cloud->points.resize(l_width * l_height);
      point_cloud->is_dense = false;

      elas_ros::ElasFrameData data;
      data.header.frame_id = l_info_msg->header.frame_id;
      data.header.stamp = l_info_msg->header.stamp;
      data.width = l_width;
      data.height = l_height;
      data.disparity.resize(l_width * l_height);
      data.r.resize(l_width * l_height);
      data.g.resize(l_width * l_height);
      data.b.resize(l_width * l_height);
      data.x.resize(l_width * l_height);
      data.y.resize(l_width * l_height);
      data.z.resize(l_width * l_height);
      data.left = *l_info_msg;
      data.right = *r_info_msg;

      // Copy into the data
      for (int32_t u=0; u<l_width; u++)
      {
        for (int32_t v=0; v<l_height; v++)
        {
          int index = v*l_width + u;
          data.disparity[index] = l_disp_data[index];
#ifdef DOWN_SAMPLE
          cv::Vec3b col = cv_ptr->image.at<cv::Vec3b>(v*2,u*2);
#else
          cv::Vec3b col = cv_ptr->image.at<cv::Vec3b>(v,u);
#endif
          data.r[index] = col[0];
          data.g[index] = col[1];
          data.b[index] = col[2];
        }
      }

      for (size_t i=0; i<l_width*l_height; i++)
      {
        cv::Point2d left_uv;
        int32_t index = i;
#ifdef DOWN_SAMPLE
        left_uv.x = (index % l_width) * 2;
        left_uv.y = (index / l_width) * 2;
#else
        left_uv.x = index % l_width;
        left_uv.y = index / l_width;
#endif
        cv::Point3d point;
        if(l_disp_data[index] > 0)
        {
          model.projectDisparityTo3d(left_uv, l_disp_data[index], point);
          point_cloud->points[i].x = point.x;
          point_cloud->points[i].y = point.y;
          point_cloud->points[i].z = point.z;
          point_cloud->points[i].r = data.r[index];
          point_cloud->points[i].g = data.g[index];
          point_cloud->points[i].b = data.b[index];
        }else{
          point_cloud->points[i].x = point_cloud->points[i].y = point_cloud->points[i].z = nanf("") ;
          point_cloud->points[i].r = point_cloud->points[i].g = point_cloud->points[i].b = (int)nan("");
        }

        data.x[index] = point.x;
        data.y[index] = point.y;
        data.z[index] = point.z;
      }

      pc_pub_->publish(point_cloud);
      elas_fd_pub_->publish(data);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void process(const sensor_msgs::ImageConstPtr& l_image_msg, 
               const sensor_msgs::ImageConstPtr& r_image_msg,
               const sensor_msgs::CameraInfoConstPtr& l_info_msg, 
               const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    ROS_DEBUG("Received images and camera info.");
    // Update the camera model;
    sensor_msgs::CameraInfoConstPtr new_l_info_msg;
    sensor_msgs::CameraInfoConstPtr new_r_info_msg;
    if(user_defined_camera_model_){
      user_left_cam_info_.header = l_info_msg->header;
      user_right_cam_info_.header = r_info_msg->header;
      new_l_info_msg = sensor_msgs::CameraInfoConstPtr( new sensor_msgs::CameraInfo(user_left_cam_info_) );
      new_r_info_msg = sensor_msgs::CameraInfoConstPtr( new sensor_msgs::CameraInfo(user_right_cam_info_) );
      model_.fromCameraInfo(new_l_info_msg,new_r_info_msg);      
    }else{
      model_.fromCameraInfo(l_info_msg, r_info_msg);
    }

    // Allocate new disparity image message
    stereo_msgs::DisparityImagePtr disp_msg =
    boost::make_shared<stereo_msgs::DisparityImage>();
    disp_msg->header         = l_info_msg->header;
    disp_msg->image.header   = l_info_msg->header;
    disp_msg->image.height   = l_image_msg->height;
    disp_msg->image.width    = l_image_msg->width;
    disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    disp_msg->image.step     = disp_msg->image.width * sizeof(float);
    disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);
    disp_msg->min_disparity = param->disp_min;
    disp_msg->max_disparity = param->disp_max;

    // Stereo parameters
    float f = model_.right().fx();
    float T = model_.baseline();
    float depth_fact = T*f*1000.0f;
    uint16_t bad_point = std::numeric_limits<uint16_t>::max();

    // Have a synchronised pair of images, now to process using elas
    // convert images if necessary
    uint8_t *l_image_data, *r_image_data;
    int32_t l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    if (l_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      l_image_data = const_cast<uint8_t*>(&(l_image_msg->data[0]));
      l_step = l_image_msg->step;
    }
    else
    {
      l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
      l_image_data = l_cv_ptr->image.data;
      l_step = l_cv_ptr->image.step[0];
    }
    if (r_image_msg->encoding == sensor_msgs::image_encodings::MONO8)
    {
      r_image_data = const_cast<uint8_t*>(&(r_image_msg->data[0]));
      r_step = r_image_msg->step;
    }
    else
    {
      r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
      r_image_data = r_cv_ptr->image.data;
      r_step = r_cv_ptr->image.step[0];
    }

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

#ifdef DOWN_SAMPLE
    int32_t width = l_image_msg->width/2;
    int32_t height = l_image_msg->height/2;
#else
    int32_t width = l_image_msg->width;
    int32_t height = l_image_msg->height;
#endif

    // Allocate
    const int32_t dims[3] = {l_image_msg->width,l_image_msg->height,l_step};
    //float* l_disp_data = new float[width*height*sizeof(float)];
    float* l_disp_data = reinterpret_cast<float*>(&disp_msg->image.data[0]);
    float* r_disp_data = new float[width*height*sizeof(float)];

    // Process
    elas_->process(l_image_data, r_image_data, l_disp_data, r_disp_data, dims);

    // Find the max for scaling the image colour
    float disp_max = 0;
    for (int32_t i=0; i<width*height; i++)
    {
      if (l_disp_data[i]>disp_max) disp_max = l_disp_data[i];
      if (r_disp_data[i]>disp_max) disp_max = r_disp_data[i];
    }

    cv_bridge::CvImage out_depth_msg;
    out_depth_msg.header = l_image_msg->header;
    out_depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
    out_depth_msg.image = cv::Mat(height, width, CV_16UC1);
    uint16_t * out_depth_msg_image_data = reinterpret_cast<uint16_t*>(&out_depth_msg.image.data[0]);

    cv_bridge::CvImage out_msg;
    out_msg.header = l_image_msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = cv::Mat(height, width, CV_8UC1);
    std::vector<int32_t> inliers;
    for (int32_t i=0; i<width*height; i++)
    {
      out_msg.image.data[i] = (uint8_t)std::max(255.0*l_disp_data[i]/disp_max,0.0);
      disp_msg->image.data[i] = l_disp_data[i];
      //disp_msg->image.data[i] = out_msg.image.data[i]

      float disp =  l_disp_data[i];
      // In milimeters
      //out_depth_msg_image_data[i] = disp;
      out_depth_msg_image_data[i] = disp <= 0.0f ? bad_point : (uint16_t)(depth_fact/disp);

      if (l_disp_data[i] > 0) inliers.push_back(i);
    }

    // Publish
    disp_pub_->publish(out_msg.toImageMsg());
    depth_pub_->publish(out_depth_msg.toImageMsg());
    if(user_defined_camera_model_)
      publish_point_cloud(l_image_msg, l_disp_data, inliers, width, height, new_l_info_msg, new_r_info_msg);
    else
      publish_point_cloud(l_image_msg, l_disp_data, inliers, width, height, l_info_msg, r_info_msg);

    pub_disparity_.publish(disp_msg);

    // Cleanup data
    //delete l_disp_data;
    delete r_disp_data;
  }

private:

  ros::NodeHandle nh;
  ros::NodeHandle _local_nh;
  Subscriber left_sub_, right_sub_;
  InfoSubscriber left_info_sub_, right_info_sub_;
  boost::shared_ptr<Publisher> disp_pub_;
  boost::shared_ptr<Publisher> depth_pub_;
  boost::shared_ptr<ros::Publisher> pc_pub_;
  boost::shared_ptr<ros::Publisher> elas_fd_pub_;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  boost::shared_ptr<Elas> elas_;
  int queue_size_;
  bool user_defined_camera_model_;
  sensor_msgs::CameraInfo user_left_cam_info_;
  sensor_msgs::CameraInfo user_right_cam_info_;
  bool robotics_stereo_params_;


  image_geometry::StereoCameraModel model_;
  ros::Publisher pub_disparity_;
  boost::scoped_ptr<Elas::parameters> param;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elas_ros");
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun viso2_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
  }
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("stereo_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  Elas_Proc processor(transport);

  ros::spin();
  return 0;
}
