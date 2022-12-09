#ifndef CAR_INTERFACE_HPP
#define CAR_INTERFACE_HPP

#include <ros/ros.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>

#include "car_interface/CarCommandValue.h"
#include "car_interface/CarCommand.h"


class CarInterface
{
public:
    CarInterface(ros::NodeHandle &node, ros::NodeHandle &nodePriv);

private:
    void commandCallback(const car_interface::CarCommandConstPtr &msg);
    void resetCallback(const std_msgs::TimeConstPtr &msg);
    void trimCallback(car_interface::TrimConfig &cfg, uint32_t);
    void safetyCallback(const std_msgs::BoolConstPtr &msg);

    void publishStop();

    ros::NodeHandle node;
    ros::NodeHandle nodePriv;

    ros::Subscriber commandSub;
    ros::Subscriber resetSub;
    ros::Subscriber safetySub;
    //ros::Publisher commandValuePub;
    ros::Timer serialwriteTimer; //run this at the frequency of 
    /**
     * you will have to manage the different problems faced with dequing and finding nothing in the queue.
     * Might have to just return from the function if nothing in there. 
     * both threads could call timer callbacks 
     */

    bool safe;

    uint16_t steerNeutral;
    uint16_t steerMaxOffset;
    bool steerReverse;
    uint16_t throttleNeutral;
    uint16_t throttleMax;
    int16_t steerTrim;
    int16_t throttleTrim;

};

#endif
