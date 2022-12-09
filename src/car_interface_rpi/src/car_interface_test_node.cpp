#include <ros/ros.h>

#include "car_interface_rpi/car_interface_test.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "car_interface_rpi_test");
    ros::NodeHandle node;

    car_interface_rpi_test interface(node);

    /**
     * NOTE: you cannot call the same callback function body on different thread stacks at the same time as there is a mutex. 
     * Hence, that thread will block until it acquires the mutex and then continues
     * However, it can process different callbacks concurrently on separate/same cores (depending on how the linux kernel timeslices
     * You can go down one more level to manage it through the C linux system libraries and manipulating the relevant structs and bits)
     *  
     * 
     * Use thread ids to troubleshoot your code!
     */
    ros::MultiThreadedSpinner spinner(2); //use two threads to dequeue from the global callback queue
    spinner.spin();

    return 0;
}
