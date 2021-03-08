/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */
/* Modified by: Christian Lanegger */

#include <ros/ros.h>
#include <boost/python.hpp>  // Needs to be above stl_iterator!
#include <boost/python/stl_iterator.hpp>

#include <boost/thread.hpp>
#include <memory>

#include "roscpp_initializer/roscpp_initializer.h"

static std::vector<std::string>& ROScppArgs() {
  static std::vector<std::string> args;
  return args;
}

static std::string& ROScppNodeName() {
  static std::string node_name;
  return node_name;
}

void rcI::string_from_list(const boost::python::list& values,
                           std::vector<std::string>* list_str) {
  boost::python::stl_input_iterator<std::string> begin(values), end;
  list_str->assign(begin, end);
}

void rcI::roscpp_set_arguments(const std::string& node_name,
                               boost::python::list& argv) {
  ROScppNodeName() = node_name;
  std::vector<std::string> argv_str;
  string_from_list(argv, &argv_str);
  ROScppArgs() = argv_str;
}

namespace {
struct InitProxy {
  InitProxy() {
    const std::vector<std::string>& args = ROScppArgs();
    int fake_argc = args.size();
    char** fake_argv = new char*[args.size()];
    for (std::size_t i = 0; i < args.size(); ++i)
      fake_argv[i] = strdup(args[i].c_str());

    ros::init(
        fake_argc, fake_argv, ROScppNodeName(),
        ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    for (int i = 0; i < fake_argc; ++i) delete[] fake_argv[i];
    delete[] fake_argv;
  }

  ~InitProxy() {
    if (ros::isInitialized() && !ros::isShuttingDown()) ros::shutdown();
  }
};
}  // namespace

static void roscpp_init_or_stop(bool init) {
  // ensure we do not accidentally initialize ROS multiple times per process
  static boost::mutex lock;
  boost::mutex::scoped_lock slock(lock);

  // once per process, we start a spinner
  static bool once = true;
  static std::unique_ptr<InitProxy> proxy;
  static std::unique_ptr<ros::AsyncSpinner> spinner;

  // initialize only once
  if (once && init) {
    once = false;

    // if ROS (cpp) is not initialized, we initialize it
    if (!ros::isInitialized()) {
      proxy.reset(new InitProxy());
      spinner.reset(new ros::AsyncSpinner(1));
      spinner->start();
    }
  }

  // shutdown if needed
  if (!init) {
    once = false;
    proxy.reset();
    spinner.reset();
  }
}

void rcI::roscpp_init(const std::string& node_name, boost::python::list& argv) {
  roscpp_set_arguments(node_name, argv);
  roscpp_init_or_stop(true);
}

void rcI::roscpp_shutdown() { roscpp_init_or_stop(false); }
