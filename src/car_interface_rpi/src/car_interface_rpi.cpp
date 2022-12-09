#include <ros/ros.h>
#include <boost/algorithm/clamp.hpp>
#include "car_interface/car_interface.hpp"

#include <algorithm>
#include <cmath>

static const double maxAngle = M_PI / 6;


/**
 * Brace initialising the different variables and publishers and subcribers
 * */
//changed command callback queue to more than 1. might have to change the corresponding publisher queue as well
//remove  dynamic reconfigure stuff (no reconfiguring trim and throttle on the fly) -> might be better to keep it. It just won't work in the gui
//use launch args to append subscribe commands with the right arguments
CarInterface::CarInterface(ros::NodeHandle &node, ros::NodeHandle &nodePriv)
    : node(node),
      nodePriv(nodePriv),
      commandSub(node.subscribe("command", 10, &CarInterface::commandCallback, this)),
      resetSub(node.subscribe(
          "reset",
          1,
          &CarInterface::resetCallback,
          this)),
      safetySub{node.subscribe("/is_safe", 1, &CarInterface::safetyCallback, this)},
      commandValuePub(node.advertise<car_interface::CarCommandValue>("command_value", 1)),
      safe{false},
      steerNeutral(adas_common::integerParam<uint16_t>(node, "interface/steer_neutral", 0)),
      // Maximum left/right value offset from the neutral value
      steerMaxOffset(adas_common::integerParam<uint16_t>(node, "interface/steer_offset", 0)),
      // Flip from positive -> above neutral to positive -> below neutral
      steerReverse(adas_common::boolParam(node, "interface/steer_reverse")),
      throttleNeutral(adas_common::integerParam<uint16_t>(node, "interface/throttle_neutral", 0)),
      throttleMax(adas_common::integerParam<uint16_t>(node, "interface/throttle_max", 0)),
      steerTrim{adas_common::integerParamWithDefault<int16_t>(
          node, "interface/steer_trim", 0, -1000, 1000)},
      throttleTrim{adas_common::integerParamWithDefault<int16_t>(
          node, "interface/throttle_trim", 0, -1000, 1000)},
      configMutex{},
      server{configMutex},
      f{boost::bind(&CarInterface::trimCallback, this, _1, _2)},
      firstConfig{true}
{
    server.setCallback(f);

    car_interface::TrimConfig cfg;
    cfg.steer_trim = steerTrim;
    cfg.throttle_trim = throttleTrim;
    server.updateConfig(cfg);

    ROS_ASSERT(steerMaxOffset <= steerNeutral);

    publishStop();
}

/**
 * in this callback the bit magic happens! enqueue your stuff here. Start a ros timer that dequeues every time period and sends over serial/CAN
 * (compare with your multithreaded can example) -> look at multithreading in ros 1
 * 
 * microsteps: 
 * - read about multithreading. look at ros threads 
 * - setup multithreaded queue
 * - browse arduino and C++ serial
 * - integrate with Serial
 * 
 * - write corresponding firmware (begin by just serial echoing what you recieve)
 * 
 * read about spinners and timer callbacks ros1 -> roscpp
 * what're you trying to do ->
 * 1. everytime there is a callback, queue the data. 
 * 2. periodically, dequeue it and send over serial and CAN
 * 3. you need thread safety as you don't want both threads picking from the queue at the same time (use a lock_guard mutex for this)
 * 4. get the timer and command callbacks from different threads
 * 
 * queue up structs with command values translated (3 uint16 data fields with steering, throttle and delimiter)
 * 
 * current todo:
 * - make a ros multithreaded application. Let it decide which threads to call what callbacks on 
 * - it doesn't make sense to use async threading as you're not waiting on anything. The multithreaded way will block until it gets the mutex and then dequeue
 * 
 * backtrack and find the frequency at which the system is running and accordingly set the timer publisher at that rate
 */

void CarInterface::commandCallback(const car_interface::CarCommandConstPtr &msg)
{
    /**
     * TODO:
     * change data manipulation to your mapping (pull values from the param file -> seems like you just need to divide by 10)
     * enqueue here 
     * look into synchronising OS clocks between machines (ROS has ideas for that in multimachine setup)
     * 
     * FINAL architecture:
     * - use a timer with a callback that manages your CAN/Serial communication (make sure to exit that callback if queue is empty)
     * - use a threadsafe queue that contains all the data you must transmit
     * - use multithreaded spinner
     * - encode/decode messages (get that from xbee)
     * - STEPS: check if !que.empty -> then open serialport -> deque, encode and write in a loop until queue empty -> close serial port
     * OR
     * '' -> deque evertyhing -> encode and make new queue -> open serial port -> write -> close serial port (this way queue is mutex unlocked to be written to as well)
     * have the right function to dequeue everything! -> just compile it
     */
    if (!safe)
    {
        publishStop();
        return;
    }

    car_interface::CarCommandValue commandValue;
    /**
     * NOTE: this is where you change how the data is encoded. This simply callsback the controller value
     */ 
    // msg->steer is in radians,msg->throttle is in percent
    double steerOffset = std::max(std::min(msg->steer, maxAngle), -maxAngle) / maxAngle * steerMaxOffset;

    const int16_t steerMultiplier = [this] () {
        if (steerReverse) { return -1; }
        return 1; } ();

    // Ensure within range with trim
    commandValue.steer = static_cast<uint16_t>(boost::algorithm::clamp(
        steerNeutral + steerMultiplier * (boost::math::iround(steerOffset) + steerTrim),
        steerNeutral - steerMaxOffset,
        steerNeutral + steerMaxOffset));

    double throttleOffset = std::max(std::min(msg->throttle, 100.0), 0.0) / 100.0 *
                            (throttleMax - throttleNeutral);

    // Ensure within range with trim
    commandValue.throttle = static_cast<uint16_t>(boost::algorithm::clamp(
        throttleNeutral + throttleTrim + throttleOffset,
        throttleNeutral,
        throttleMax));

    commandValuePub.publish(commandValue);
}

void CarInterface::resetCallback(const std_msgs::TimeConstPtr &msg)
{
    ROS_INFO(
        "Stopping car %u...",
        adas_common::integerParam<unsigned int>(node, "car_id"));
    car_interface::CarCommandValue commandValue;

    publishStop();
}

void CarInterface::trimCallback(car_interface::TrimConfig &cfg, uint32_t)
{
    // First update is meaningless, as it is the defaults from dynamic_reconfigure.
    if (firstConfig)
    {
        firstConfig = false;
        return;
    }
    steerTrim = static_cast<int16_t>(cfg.steer_trim);
    throttleTrim = static_cast<int16_t>(cfg.throttle_trim);
}

/**
 * from watchdog timer - maybe? 
 */
void CarInterface::safetyCallback(const std_msgs::BoolConstPtr &msg)
{
    safe = msg->data;
    if (!safe)
    { publishStop(); }
}

/**
 * publishes neutral values  
 * */
void CarInterface::publishStop()
{
    car_interface::CarCommandValue commandValue;
    commandValue.steer = steerNeutral;
    commandValue.throttle = throttleNeutral;
    commandValuePub.publish(commandValue);
}


