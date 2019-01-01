#include <cmath>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>

#include <smart_car/FrameToFrame.h>
#include <smart_car/KinematicsForward.h>
#include <smart_car/KinematicsInverse.h>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

long double L;
long double L3;
long double sqrt3;

long double theta = 0;

void mobileToWorldCore(double Vxm, double Vym, double& Vxw, double& Vyw) {
    Vxw = (std::cos(theta) * Vxm) - (std::sin(theta) * Vym);
    Vyw = (std::sin(theta) * Vxm) + (std::cos(theta) * Vym);
}

bool mobileToWorld(smart_car::FrameToFrame::Request &request, smart_car::FrameToFrame::Response &response) {
    mobileToWorldCore(request.input.x, request.input.y, response.output.x, response.output.y);
}

void worldToMobileCore(double Vxw, double Vyw, double& Vxm, double& Vym) {
    Vxm =   (std::cos(theta) * Vxw) + (std::sin(theta) * Vyw);
    Vym = - (std::sin(theta) * Vxw) + (std::cos(theta) * Vyw);
}

bool worldToMobile(smart_car::FrameToFrame::Request &request, smart_car::FrameToFrame::Response &response) {
    worldToMobileCore(request.input.x, request.input.y, response.output.x, response.output.y);
}

bool forwardMobile(smart_car::KinematicsForward::Request &request, smart_car::KinematicsForward::Response &response) {
    response.output.x     = ((2.0L * request.input.v_back) - request.input.v_left - request.input.v_right) / 3.0L;
    response.output.y     = ((sqrt3 * request.input.v_right) - (sqrt3 * request.input.v_left)) / 3.0L;
    response.output.theta = (request.input.v_left + request.input.v_back + request.input.v_right) / L3;
    return true;
}

bool forwardWorld(smart_car::KinematicsForward::Request &request, smart_car::KinematicsForward::Response &response) {
    forwardMobile(request, response);
    mobileToWorldCore(response.output.x, response.output.y, response.output.x, response.output.y);
    return true;
}

bool inverseMobile(smart_car::KinematicsInverse::Request &request, smart_car::KinematicsInverse::Response &response) {
    long double V__m_x2 = - request.input.x / 2.0L;
    long double sqrt3V__m_y2 = (sqrt3 * request.input.y) / 2.0L;
    long double Lomega_p = L * request.input.theta;
    response.output.v_left  = V__m_x2 - sqrt3V__m_y2 + Lomega_p;
    response.output.v_back  = request.input.x        + Lomega_p;
    response.output.v_right = V__m_x2 + sqrt3V__m_y2 + Lomega_p;
    return true;
}

bool inverseWorld(smart_car::KinematicsInverse::Request &request, smart_car::KinematicsInverse::Response &response) {
    worldToMobileCore(request.input.x, request.input.y, request.input.x, request.input.y);
    inverseMobile(request, response);
    return true;
}

void onPoseWorldMessage(const geometry_msgs::Pose2D::ConstPtr& input){
    theta = input->theta;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle node;
    {
        std::string description;
        if(!node.getParam("robot_description",description)) {
            ROS_ERROR("Could not find '/robot_description'.");
            return -1;
        }
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(description, tree)) {
            ROS_ERROR("Failed to construct KDL tree.");
            return -1;
        }
        KDL::Chain chain;
        if (!tree.getChain("base_link", "rim_back_link", chain)) {
            ROS_ERROR("Failed to get chain from KDL tree.");
            return -1;
        }
        KDL::Frame frame = chain.getSegment(0).pose(0);
        L = std::sqrt(std::pow(frame.p.x() - 0.0L, 2.0L) + std::pow(frame.p.y() - 0.0L, 2.0L));
        node.setParam("parameter/wheel/distance", (double) L);
        L3 = 3.0L * L;
        sqrt3 = std::sqrt(3.0L);
        double parameter;
        if (!node.getParam("parameter/initial/theta", parameter)) {
            parameter = 0;
        }
        theta = parameter;
    }
    ros::ServiceServer forwardMobileService = node.advertiseService("kinematics_forward_mobile", forwardMobile);
    ros::ServiceServer forwardWorldService  = node.advertiseService("kinematics_forward_world" , forwardWorld );
    ros::ServiceServer inverseMobileService = node.advertiseService("kinematics_inverse_mobile", inverseMobile);
    ros::ServiceServer inverseWorldService  = node.advertiseService("kinematics_inverse_world" , inverseWorld );
    ros::ServiceServer mobileToWorldService = node.advertiseService("kinematics_mobile_to_world" , mobileToWorld);
    ros::ServiceServer worldToMobileService = node.advertiseService("kinematics_world_to_mobile" , worldToMobile);
    ros::Subscriber subscriber = node.subscribe("pose/world", 1, onPoseWorldMessage);
    ros::spin();
    return 0;
}
