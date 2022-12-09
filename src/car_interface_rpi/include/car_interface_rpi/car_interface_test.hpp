#include <ros/ros.h>
#include "car_interface_rpi/CarCommand.h"
#include "car_interface_rpi/threadsafe_queue.hpp"

class car_interface_rpi_test {
    public:
        car_interface_rpi_test(ros::NodeHandle &node);

    private:

        void commandCallback(const car_interface_rpi::CarCommandConstPtr  msg);
        void canTimerCallback(const ros::TimerEvent& event);
        void commandTimerCallback(const ros::TimerEvent& event);

        ros::NodeHandle node;

        ros::Timer canTimer;
        ros::Subscriber commandSub;
        ros::Publisher commandPub;
        ros::Timer commandTimer;

        threadsafe_queue<car_interface_rpi::CarCommand> carcommandQ;

        int count;

};