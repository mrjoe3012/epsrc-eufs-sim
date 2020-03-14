/*MIT License
*
* Copyright (c) 2019 Edinburgh University Formula Student (EUFS)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
*         of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
*         to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*         copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
*         copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.*/

/**
 * @file gazebo_cone_ground_truth.h
 * @author Niklas Burggraaff <s1902977@ed.ac.uk>
 * @date Mar 10, 2020
 * @copyright 2020 Edinburgh University Formula Student (EUFS)
 * @brief ground truth cone Gazebo plugin
 *
 * @details TODO:
 * Provides ground truth state in simulation in the form of nav_msgs/Odometry and
 * eufs_msgs/CarState. Additionally can publish transform.P
 **/


#ifndef EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H
#define EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H

#include <math.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>

#include <eufs_msgs/ConeArray.h>
#include <eufs_msgs/PointArray.h>
#include <eufs_msgs/CarState.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace gazebo {

  class GazeboConeGroundTruth : public ModelPlugin {

  public:

    enum ConeType {
      blue, yellow, orange, big_orange
    };

    GazeboConeGroundTruth();

    // Gazebo plugin functions

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    void UpdateChild();

    // Getting the cone arrays
    eufs_msgs::ConeArray getConeArrayMessage();

    void addConeToConeArray(eufs_msgs::ConeArray &ground_truth_cone_array, physics::LinkPtr link);

    void processCones(std::vector <geometry_msgs::Point> &points);

    GazeboConeGroundTruth::ConeType getConeType(physics::LinkPtr link);

    // Getting the cone marker array
    visualization_msgs::MarkerArray getConeMarkerArrayMessage(eufs_msgs::ConeArray &cone_array_message);

    int addConeMarkers(std::vector <visualization_msgs::Marker> &marker_array, int marker_id,
                       std::vector <geometry_msgs::Point> cones, float red, float green, float blue, bool big);

    // Add noise to the cone arrays
    eufs_msgs::ConeArray getConeArrayMessageWithNoise(eufs_msgs::ConeArray &ground_truth_cone_array_message, ignition::math::Vector3d noise);

    void addNoiseToConeArray(std::vector<geometry_msgs::Point> &cone_array, ignition::math::Vector3d noise);

    double GaussianKernel(double mu, double sigma);

      // Helper function for parameters
    bool getBoolParameter(sdf::ElementPtr _sdf, const char* element, bool default_value, const char* default_description);
    double getDoubleParameter(sdf::ElementPtr _sdf, const char* element, double default_value, const char* default_description);
    std::string getStringParameter(sdf::ElementPtr _sdf, const char* element, std::string default_value, const char* default_description);
    ignition::math::Vector3d getVector3dParameter(sdf::ElementPtr _sdf, const char* element, ignition::math::Vector3d default_value, const char* default_description);

    // Publishers
    ros::Publisher ground_truth_cone_pub_;
    ros::Publisher ground_truth_cone_marker_pub_;

    ros::Publisher camera_cone_pub_;
    ros::Publisher camera_cone_marker_pub_;

    ros::Publisher lidar_cone_pub_;
    ros::Publisher lidar_cone_marker_pub_;

    // Gazebo variables
    physics::ModelPtr track_model;
    physics::LinkPtr car_link;
    ignition::math::Pose3d car_pos;

    // Parameters

    double view_distance;
    double fov;

    double update_rate_;
    ros::Time time_last_published;

    std::string cone_frame_;

    bool simulate_camera_;
    bool simulate_lidar_;

    ignition::math::Vector3d camera_noise_;
    ignition::math::Vector3d lidar_noise_;

    // Required ROS gazebo plugin variables
    event::ConnectionPtr update_connection_;

    ros::NodeHandle *rosnode_;

    unsigned int seed;

  };
}

#endif //EUFS_SIM_GAZEBO_CONE_GROUND_TRUTH_H
