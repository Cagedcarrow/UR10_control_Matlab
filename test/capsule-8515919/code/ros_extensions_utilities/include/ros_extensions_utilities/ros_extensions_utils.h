/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   ros_extensions_utils.h
 * Author:  Enrico Ferrentino, Vincenzo Petrone
 * Org.:    UNISA
 * Date:    Jun 13, 2019
 *
 * Library collecting utilities used in the modules extending ROS.
 *
 * -------------------------------------------------------------------
 */

#pragma once

// ROS
#include <rclcpp/rclcpp.hpp>

// MoveIt!
#include <moveit/robot_model/joint_model_group.h>

// Exceptions
#include <ros_extensions_utilities/exceptions.h>

// ****** Declarations

namespace utils {

/**
 * @brief Allow retrieving parameters without allocating on the heap
 *
 * @param[in] nh node handle needed to access the parameter
 * @param[in] key parameter name to be searched
 * @return the templated value of the parameter to be retrieved
 *
 * @throw ros::UnavailableParameterException if the parameter is not present
 */
template <typename T>
T getParameterFromNode(const rclcpp::Node::ConstSharedPtr& node, const std::string& key);

/**
 * @brief Allow retrieving parameters without allocating on the heap, returning a default
 * value if the parameter does not exist
 *
 * @param[in] nh node handle needed to access the parameter
 * @param[in] key parameter name to be searched
 * @param[in] default_value the value to return if \e key does not exist
 * @return the templated value of the parameter to be retrieved
 *
 * @throw ros::UnavailableParameterException if the parameter is not present*
 */
template <typename T>
T getParameterFromNode(const rclcpp::Node::ConstSharedPtr& node, const std::string& key, const T& default_value);

/**
 * @brief Returns the current time stamp in the form of a string
 *
 * @return the string with the current time stamp in the format %Y-%m-%d-%H-%M-%S
 */
std::string nowStr();

/**
 * @brief Extract all link names in a planning group
 *
 * This function is necessary because sometimes the first link of a chain
 * is not included in the planning group. So, we need to retrieve the first
 * link name and add it to the list of link names if it is not there yet.
 *
 * @param[in] jmg the joint model group corresponding to the planning group to extract link names from
 * @return the link names
 */
std::vector<std::string> extractLinkNamesFromJointModelGroup(const moveit::core::JointModelGroup* jmg);

}  // namespace utils

// ****** Definitions

template <typename T>
T utils::getParameterFromNode(const rclcpp::Node::ConstSharedPtr& node, const std::string& key) {
    T parameter;
    if (!node->get_parameter<T>(key, parameter))
        throw UnavailableParameterException("Could not retrieve parameter '" + key + "'");
    return parameter;
}

template <typename T>
T utils::getParameterFromNode(const rclcpp::Node::ConstSharedPtr& node, const std::string& key, const T& alternative_value) {
    T parameter;
    node->get_parameter_or(key, parameter, alternative_value);
    return parameter;
}

std::string utils::nowStr() {
    std::stringstream str;
    std::time_t now = std::time(0);
    tm* local_time = std::localtime(&now);
    const char* format = "%Y-%m-%d-%H-%M-%S";
    const std::time_put<char>& date_writer = std::use_facet<std::time_put<char>>(str.getloc());
    date_writer.put(str, str, ' ', local_time, format, format + std::strlen(format));
    return str.str();
}

std::vector<std::string> utils::extractLinkNamesFromJointModelGroup(const moveit::core::JointModelGroup* jmg) {
    std::vector<std::string> dynamic_chain_link_names = jmg->getLinkModelNames();
    std::string first_link = jmg->getJointModel(jmg->getVariableNames().front())->getParentLinkModel()->getName();

    if (std::find(dynamic_chain_link_names.begin(), dynamic_chain_link_names.end(), first_link) == dynamic_chain_link_names.end())
        dynamic_chain_link_names.insert(dynamic_chain_link_names.begin(), first_link);

    return dynamic_chain_link_names;
}
