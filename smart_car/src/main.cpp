#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>

#include <smart_car/FrameToFrame.h>
#include <smart_car/KinematicsInverse.h>
#include <smart_car/Movement.h>
#include <smart_car/MovementGeneric.h>
#include <smart_car/Velocity.h>

#define pi30 0.1047197551196597705355242034774843062905347323976457118988037109375 // long double thirty = 30; long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne) / thirty);
#define pi 3.141592653589793115997963468544185161590576171875 // long double mOne = -1; printf("%1.70Lf\n", (long double) acos(mOne));
#define pi2 6.28318530717958623199592693708837032318115234375 // long double two = 2; long double mOne = -1; printf("%1.70Lf\n", (long double) two * acos(mOne));

enum Movements {
    MOVEMENT_DIRECT,
    MOVEMENT_DIRECT_M,
    MOVEMENT_DIRECT_W,
    MOVEMENT_NONE,
    MOVEMENT_ABSOLUTE_M,
    MOVEMENT_ABSOLUTE_W,
};
Movements movement = MOVEMENT_NONE;

long long binomialCoefficient_c;

long long binomialCoefficient_i;

geometry_msgs::Pose2D error;

geometry_msgs::Pose2D errorI;

smart_car::FrameToFrame mobileToWorld;

long double I;

smart_car::KinematicsInverse kinematicsInverse;

ros::ServiceClient kinematicsInverseMobileClient;

ros::ServiceClient kinematicsInverseWorldClient;

ros::ServiceClient kinematicsMobileToWorldClient;

double offsetTheta;

double offsetX;

double offsetY;

long double omega_max;

long double P;

geometry_msgs::Pose2D poseMobile;

geometry_msgs::Pose2D poseWorld;

geometry_msgs::Pose2D poseTarget;

long double radius;

long double scale;
long double stopAngle;

long double stopDistance;

std::vector<double> targetRotationArray;

std::vector<geometry_msgs::Pose2D> targetTranslationArray;

geometry_msgs::Pose2D v_abs;

ros::Publisher v_leftCommand ;
ros::Publisher v_backCommand ;
ros::Publisher v_rightCommand;

long double v_max;

std_msgs::Float64 value;

smart_car::Velocity velocity;

/**
 * source:
 * https://en.wikipedia.org/wiki/Binomial_coefficient
 * @param n the number that goes above
 * @param k the number that goes below
 * @return
 */
long long binomialCoefficient(long long n, long long k) {
    if ((k < 0) || (k > n))
        return 0;
    if ((k == 0) || (k == n))
        return 1;
    k = std::min(k, n - k);// take advantage of symmetry
    binomialCoefficient_c = 1;
    for (long binomialCoefficient_i = 0;
     binomialCoefficient_i < k; binomialCoefficient_i++)
        binomialCoefficient_c = binomialCoefficient_c
         * (n - binomialCoefficient_i) / (binomialCoefficient_i + 1);
    return binomialCoefficient_c;
}

void geometryMsgsPose2DOffset(geometry_msgs::Pose2D& pose, double x, double y) {
    pose.x += x;
    pose.y += y;
}

void geometryMsgsPose2DRotate(geometry_msgs::Pose2D& pose) {
    mobileToWorld.request.input.x = pose.x;
    mobileToWorld.request.input.y = pose.y;
    kinematicsMobileToWorldClient.call(mobileToWorld);
    pose.x = mobileToWorld.response.output.x;
    pose.y = mobileToWorld.response.output.y;
}

bool isStopPose() {
    return (std::abs(error.x) < stopDistance)
     && (std::abs(error.y) < stopDistance)
     && (std::abs(error.theta) < stopAngle);
}

long double normalizeRadian(long double radian) {
    radian = fmod(radian, pi2);
    if (radian < 0) radian += pi2;
    return radian;
}

void onCommandMessage(const smart_car::Movement::ConstPtr& input){
    ROS_INFO("Hered command!");
    if (input->movement == smart_car::Movement::WHEEL) {
        ROS_INFO("Movement::WHEEL");
        // direct wheel
        velocity.v_left  = input->wheel.v_left ;
        velocity.v_back  = input->wheel.v_back ;
        velocity.v_right = input->wheel.v_right;
        movement = MOVEMENT_DIRECT;

    } else if (input->movement == smart_car::Movement::GENERIC) {
        ROS_INFO("Movement::GENERIC");
        if (input->generic.type == smart_car::Movement::POSITION_ABSOLUTE) {
            if (input->generic.frame == smart_car::Movement::WORLD) {

                // absolute world
                poseTarget.x     = input->generic.target.x;
                poseTarget.y     = input->generic.target.y;
                poseTarget.theta = input->generic.target.theta;
                movement = MOVEMENT_ABSOLUTE_W;

            } else if (input->generic.frame == smart_car::Movement::MOBILE) {
                ROS_INFO("Movement::MOBILE");
                // absolute mobile
                // distance to be traversed in the mobile platform's frame
                mobileToWorld.request.input.x = input->generic.target.x - poseMobile.x;
                mobileToWorld.request.input.y = input->generic.target.y - poseMobile.y;
                // distance rotated to the direction
                // to be traversed in the world's frame
                kinematicsMobileToWorldClient.call(mobileToWorld);
                // rotated distance added to current world's frame position
                poseTarget.x     = poseWorld.x + mobileToWorld.response.output.x;
                poseTarget.y     = poseWorld.y + mobileToWorld.response.output.y;
                poseTarget.theta = input->generic.target.theta;
                movement = MOVEMENT_ABSOLUTE_W;

            } else {//input->generic.frame == smart_car::Movement::MOBILE_RAW

                // absolute mobile raw
                poseTarget.x     = input->generic.target.x;
                poseTarget.y     = input->generic.target.y;
                poseTarget.theta = input->generic.target.theta;
                movement = MOVEMENT_ABSOLUTE_M;
            }
        } else if (input->generic.type == smart_car::Movement::POSITION_RELATIVE) {
                ROS_INFO("Movement::POSITION_RELATIVE");
            if (input->generic.frame == smart_car::Movement::WORLD) {
                
                // relative world
                poseTarget.x     = poseWorld.x + input->generic.target.x;
                poseTarget.y     = poseWorld.y + input->generic.target.y;
                poseTarget.theta = normalizeRadian(poseWorld.theta + input->generic.target.theta);
                movement = MOVEMENT_ABSOLUTE_W;

            } else if (input->generic.frame == smart_car::Movement::MOBILE) {

                // relative mobile
                // distance to be traversed in the mobile platform's frame
                mobileToWorld.request.input.x = input->generic.target.x;
                mobileToWorld.request.input.y = input->generic.target.y;
                // distance rotated to the direction
                // to be traversed in the world's frame
                kinematicsMobileToWorldClient.call(mobileToWorld);
                // rotated distance added to current world's frame position
                poseTarget.x     = poseWorld.x + mobileToWorld.response.output.x;
                poseTarget.y     = poseWorld.y + mobileToWorld.response.output.y;
                poseTarget.theta = normalizeRadian(poseWorld.theta + input->generic.target.theta);
                movement = MOVEMENT_ABSOLUTE_W;

            } else {//input->generic.frame == smart_car::Movement::MOBILE_RAW

                // relative mobile raw
                poseTarget.x     = poseMobile.x + input->generic.target.x;
                poseTarget.y     = poseMobile.y + input->generic.target.y;
                poseTarget.theta = normalizeRadian(poseMobile.theta + input->generic.target.theta);
                movement = MOVEMENT_ABSOLUTE_M;

            }
        } else {//input->generic.type == smart_car::Movement::VELOCITY
            if (input->generic.frame == smart_car::Movement::MOBILE) {

                // direct mobile
                kinematicsInverse.request.input.x     = input->generic.target.x;
                kinematicsInverse.request.input.y     = input->generic.target.y;
                kinematicsInverse.request.input.theta = input->generic.target.theta;
                movement = MOVEMENT_DIRECT_M;

            } else if (input->generic.frame == smart_car::Movement::WORLD) {

                // direct world
                kinematicsInverse.request.input.x     = input->generic.target.x;
                kinematicsInverse.request.input.y     = input->generic.target.y;
                kinematicsInverse.request.input.theta = input->generic.target.theta;
                movement = MOVEMENT_DIRECT_W;

            } else {//input->generic.frame == smart_car::Movement::HYBRID

                // direct hybrid
                mobileToWorld.request.input.x = input->generic.target.x;
                mobileToWorld.request.input.y = input->generic.target.y;
                kinematicsMobileToWorldClient.call(mobileToWorld);
                kinematicsInverse.request.input.x     = mobileToWorld.response.output.x;
                kinematicsInverse.request.input.y     = mobileToWorld.response.output.y;
                kinematicsInverse.request.input.theta = input->generic.target.theta;
                movement = MOVEMENT_DIRECT_W;

            }
        }
    } else {
        movement = MOVEMENT_NONE;
    }
}

void onPoseMobileMessage(const geometry_msgs::Pose2D::ConstPtr& input){
    poseMobile.x     = input->x    ;
    poseMobile.y     = input->y    ;
    poseMobile.theta = input->theta;
}

void onPoseWorldMessage(const geometry_msgs::Pose2D::ConstPtr& input){
    poseWorld.x     = input->x    ;
    poseWorld.y     = input->y    ;
    poseWorld.theta = input->theta;
}

void resetErrors() {
    errorI.x     = 0;
    errorI.y     = 0;
    errorI.theta = 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle node;
    {
        double parameter;
        if (!node.getParam("parameter/wheel/radius", parameter)) {
            ROS_ERROR("Could not get wheel radius from parameter server.");
            return -1;
        }
        radius = parameter;
        if (!node.getParam("parameter/stop/distance", parameter)) {
            parameter = 0.001;
        }
        stopDistance = parameter;
        if (!node.getParam("parameter/stop/angle", parameter)) {
            parameter = pi30;
        }
        stopAngle = parameter;
        if (!node.getParam("parameter/max/v", parameter)) {
            parameter = 1;
        }
        v_max = parameter;
        if (!node.getParam("parameter/max/omega", parameter)) {
            parameter = 50;
        }
        omega_max = parameter;
        if (!node.getParam("parameter/control/P", parameter)) {
            parameter = 5;
        }
        P = parameter;
        if (!node.getParam("parameter/control/I", parameter)) {
            parameter = 0.0005;
        }
        I = parameter;
    }
    kinematicsInverseMobileClient = node.serviceClient<smart_car::KinematicsInverse>("kinematics_inverse_mobile");
    kinematicsInverseWorldClient  = node.serviceClient<smart_car::KinematicsInverse>("kinematics_inverse_world" );
    kinematicsMobileToWorldClient = node.serviceClient<smart_car::FrameToFrame>("kinematics_mobile_to_world");
    v_leftCommand  = node.advertise<std_msgs::Float64>("left_joint_velocity_controller/command", 1);
    v_backCommand  = node.advertise<std_msgs::Float64>("back_joint_velocity_controller/command", 1);
    v_rightCommand = node.advertise<std_msgs::Float64>("right_joint_velocity_controller/command", 1);
    ros::Subscriber poseWorldSubscriber  = node.subscribe("pose/world" , 1, onPoseWorldMessage );
    ros::Subscriber poseMobileSubscriber = node.subscribe("pose/mobile", 1, onPoseMobileMessage);
    ros::Subscriber command = node.subscribe("command", 1, onCommandMessage);

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        switch (movement) {
            case MOVEMENT_ABSOLUTE_M:
                error.x     = poseTarget.x     - poseMobile.x    ;
                error.y     = poseTarget.y     - poseMobile.y    ;
                error.theta = poseTarget.theta - poseMobile.theta;
                // method parameter: theta plus shift to place thetaTarget at pi
                error.theta = pi - normalizeRadian(poseMobile.theta + pi - poseTarget.theta);
                if (isStopPose()) {
                    movement = MOVEMENT_NONE;
                    break;
                }
                errorI.x     += error.x    ;
                errorI.y     += error.y    ;
                errorI.theta += error.theta;
                kinematicsInverse.request.input.x = (error.x * P) + (errorI.x * I);
                kinematicsInverse.request.input.y = (error.y * P) + (errorI.y * I);
                v_abs.x = std::abs(kinematicsInverse.request.input.x);
                v_abs.y = std::abs(kinematicsInverse.request.input.y);
                if ((v_abs.x > v_max) || (v_abs.y > v_max)) {
                    scale = v_max / ((v_abs.x > v_abs.y) ? v_abs.x : v_abs.y);
                    kinematicsInverse.request.input.x *= scale;
                    kinematicsInverse.request.input.y *= scale;
                }
                kinematicsInverse.request.input.theta = (error.theta * P) + (errorI.theta * I);
                if (kinematicsInverse.request.input.theta > omega_max) {
                    kinematicsInverse.request.input.theta = omega_max;
                }
                kinematicsInverseMobileClient.call(kinematicsInverse);
                velocity.v_left  = kinematicsInverse.response.output.v_left ;
                velocity.v_back  = kinematicsInverse.response.output.v_back ;
                velocity.v_right = kinematicsInverse.response.output.v_right;
                break;
                // TO DO
                // break; // fall through intended
            case MOVEMENT_ABSOLUTE_W:
                error.x     = poseTarget.x     - poseWorld.x    ;
                error.y     = poseTarget.y     - poseWorld.y    ;
                error.theta = poseTarget.theta - poseWorld.theta;
                // method parameter: theta plus shift to place thetaTarget at pi
                error.theta = pi - normalizeRadian(poseWorld.theta + pi - poseTarget.theta);
                if (isStopPose() && (movement == MOVEMENT_ABSOLUTE_W)) {
                    movement = MOVEMENT_NONE;
                    break;
                }
                errorI.x     += error.x    ;
                errorI.y     += error.y    ;
                errorI.theta += error.theta;
                kinematicsInverse.request.input.x = (error.x * P) + (errorI.x * I);
                kinematicsInverse.request.input.y = (error.y * P) + (errorI.y * I);
                v_abs.x = std::abs(kinematicsInverse.request.input.x);
                v_abs.y = std::abs(kinematicsInverse.request.input.y);
                if ((v_abs.x > v_max) || (v_abs.y > v_max)) {
                    scale = v_max / ((v_abs.x > v_abs.y) ? v_abs.x : v_abs.y);
                    kinematicsInverse.request.input.x *= scale;
                    kinematicsInverse.request.input.y *= scale;
                }
                kinematicsInverse.request.input.theta = (error.theta * P) + (errorI.theta * I);
                if (kinematicsInverse.request.input.theta > omega_max) {
                    kinematicsInverse.request.input.theta = omega_max;
                }
                kinematicsInverseWorldClient.call(kinematicsInverse);
                velocity.v_left  = kinematicsInverse.response.output.v_left ;
                velocity.v_back  = kinematicsInverse.response.output.v_back ;
                velocity.v_right = kinematicsInverse.response.output.v_right;
                break;
            case MOVEMENT_DIRECT:
                break;
            case MOVEMENT_DIRECT_M:
                kinematicsInverseMobileClient.call(kinematicsInverse);
                velocity.v_left  = kinematicsInverse.response.output.v_left ;
                velocity.v_back  = kinematicsInverse.response.output.v_back ;
                velocity.v_right = kinematicsInverse.response.output.v_right;
                break;
            case MOVEMENT_DIRECT_W:
                kinematicsInverseWorldClient.call(kinematicsInverse);
                velocity.v_left  = kinematicsInverse.response.output.v_left ;
                velocity.v_back  = kinematicsInverse.response.output.v_back ;
                velocity.v_right = kinematicsInverse.response.output.v_right;
                break;
            default:
                resetErrors();
                velocity.v_left  = 0;
                velocity.v_back  = 0;
                velocity.v_right = 0;
        }
        value.data = velocity.v_left  / radius; 
        v_leftCommand .publish(value);
        value.data = velocity.v_back  / radius; 
        v_backCommand .publish(value);
        value.data = velocity.v_right / radius; 
        v_rightCommand.publish(value);

        if (MOVEMENT_NONE != movement)
            ROS_INFO("The speed is left:%f back:%f right:%f",velocity.v_left,velocity.v_back,velocity.v_right);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
