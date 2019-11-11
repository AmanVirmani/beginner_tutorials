/******************************************************************************
 * MIT License

 * Copyright (c) 2019 Aman Virmani

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 ******************************************************************************/

/**
 * @file      talker.cpp
 * @author    Aman Virmani
 * @copyright MIT License
 * @brief     ROS publisher node implementation 
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/modifyTalkerString.h"
#include "std_msgs/String.h"

/**
 * @brief      Tests whether the service exists
 * @param      TestSuite        gtest framework
 * @param      ifServiceExists  Name of the test
 */

TEST(TestSuite,ifServiceExists) {
        // Creating ROS node handle
        ros::NodeHandle n;
        // Creating Client to use the service
        auto client = n.serviceClient<beginner_tutorials::modifyTalkerString>("");
        // check if the service exists
        EXPECT_FALSE(client.waitForExistence(ros::Duration(10)));
}
