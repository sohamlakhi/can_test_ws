#include "car_interface_rpi/car_interface_test.hpp"
#include "car_interface_rpi/ros2can.hpp"

car_interface_rpi_test::car_interface_rpi_test(ros::NodeHandle &node) 
    :   node(node), 
        commandSub(node.subscribe("command", 10, &car_interface_rpi_test::commandCallback, this)), 
        canTimer(node.createTimer(ros::Duration(1/30), &car_interface_rpi_test::canTimerCallback, this)),
        commandTimer(node.createTimer(ros::Duration(1/20), &car_interface_rpi_test::commandTimerCallback, this)),
        commandPub(node.advertise<car_interface_rpi::CarCommand> ("command", 10)),
        count(0){} 

void car_interface_rpi_test::commandCallback(const car_interface_rpi::CarCommandConstPtr msg) {
    // uint8_t data_0 = 0xFF;
    // uint8_t data_1 = 0xAA;
    
    carcommandQ.push(*msg);
    ROS_INFO("data pushed: ");
}

void car_interface_rpi_test::canTimerCallback(const ros::TimerEvent& event) {
    std::queue<car_interface_rpi::CarCommand> queue = carcommandQ.dequeue_all();

    while(queue.size() > 0) {
        car_interface_rpi::CarCommand popped = queue.front();
        queue.pop();
        ros2can canObj;

        if (popped.steer == 100) {
            canObj.data_frame.data[0] = 0xBB;
            canObj.data_frame.can_id = 0x10;
        }

        else if (popped.steer == 200) {
            canObj.data_frame.data[0] = 0xCC;
            canObj.data_frame.can_id = 0x20;
        }

        
        canObj.data_frame.can_dlc = 8;
        // canObj.data_frame.can_id = 0x20;
        // canObj.data_frame.data[0] = 0xFF;
        canObj.data_frame.data[1] = 0xAA;
        for (int i = 2; i<9; i++) {
            canObj.data_frame.data[i] = 0x00;
        }
        
        canObj.send();
        
    }
}

void car_interface_rpi_test::commandTimerCallback(const ros::TimerEvent& event) {
    car_interface_rpi::CarCommand msg;

    if (count == 1) {
        msg.steer = 100;
        msg.throttle = 0;
        count = 0;
    }
    
    else if (count == 0) {
        msg.steer = 200;
        msg.throttle = 0;
        count = 1;
    }

    commandPub.publish(msg);
}



/**
 * TODO: 
 * setup multithreaded CAN pubsub
 * make rpi docker container and upload everything to that
 * debug CAN firmware
 * run f1tenth and collect videos
 */