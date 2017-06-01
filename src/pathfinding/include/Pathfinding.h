/*
 * Created By: Gareth Ellis
 * Created On: June 1, 2017
 * Description: TODO
 */

#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sb_utils.h>

class MyClass {
public:
    MyClass(int argc, char **argv, std::string node_name);
    /**
     * Adds an exclamation point to a given string
     *
     * Some Longer explanation should go here
     *
     * @param input_string the string to add an exclamation point to
     *
     * @return input_string with an exclamation point added to it
     */
     static std::string addCharacterToString(std::string input_string, std::string suffix);
     std::string suffix;

private:
    /**
     * Callback function for when a new string is received
     *
     * @param msg the string received in the callback
     */
    void subscriberCallBack(const std_msgs::String::ConstPtr& msg);
    /**
     * Publishes a given string
     *
     * @param msg_to_publish the string to publish
     */
    void republishMsg(std::string msg_to_publish);

    ros::Subscriber my_subscriber;
    ros::Publisher my_publisher;
};
#endif //PATHFINDING_H
