/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   exceptions.h
 * Author:  Enrico Ferrentino, Vincenzo Petrone
 * Org.:    UNISA
 * Date:    Mar 23, 2021
 *
 * Domain specific exceptions for ros_extensions_utilities.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <rclcpp/exceptions/exceptions.hpp>

namespace utils {

/**
 * @brief Thrown when a parameter is unavailable, could not be found
 */
class UnavailableParameterException : public rclcpp::exceptions::InvalidParametersException {
  public:
    UnavailableParameterException(const std::string& what) : rclcpp::exceptions::InvalidParametersException("Parameter unavailable: " + what) {}
};

}  // namespace utils
