/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <iostream>

#ifndef Q_MOC_RUN
#include <ignition/gui/Iface.hh>
#endif

static const char version_str[] = "Visualizer 0.1.0";

int main(int argc, char *argv[])
{
  std::cout << version_str << std::endl;

  Q_INIT_RESOURCE(resources);

  // Initialize app
  ignition::gui::initApp();

  // Look for all plugins in the same place
  ignition::gui::setPluginPathEnv("VISUALIZER_PLUGIN_PATH");

  // Then look for plugins on compile-time defined path.
  // Plugins installed by gazebo end up here
  //ignition::gui::addPluginPath(GAZEBO_PLUGIN_INSTALL_PATH);

  // Create main window
  ignition::gui::createMainWindow();

  auto win = ignition::gui::mainWindow();
  win->setWindowTitle(version_str);

  ignition::gui::runMainWindow();

  ignition::gui::stop();

  return 0;
}
