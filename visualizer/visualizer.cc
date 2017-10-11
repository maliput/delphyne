// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>

#ifndef Q_MOC_RUN
#include <ignition/gui/Iface.hh>
#endif

static const char versionStr[] = "Visualizer 0.1.0";
static const std::string initialConfigFile = "visualizer/initialLayout.config";

/// \brief Get the path of the default configuration file for Delphyne.
/// \return The default configuration path.
std::string defaultConfigPath() {
  std::string homePath;
  ignition::common::env("HOME", homePath);
  std::string defaultConfigPath =
      ignition::common::joinPaths(homePath, ".delphyne", "delphyne.config");

  return defaultConfigPath;
}

int main(int argc, char* argv[]) {
  ignition::common::Console::SetVerbosity(3);
  ignmsg << versionStr << std::endl;

  // Parse custom config file from args
  std::string configFile = initialConfigFile;
  if (argc > 1) {
    configFile = argv[1];
  }

  Q_INIT_RESOURCE(resources);

  // Initialize app
  ignition::gui::initApp();

  // Set the default location for saving user settings.
  ignition::gui::setDefaultConfigPath(defaultConfigPath());

  // Look for all plugins in the same place
  ignition::gui::setPluginPathEnv("VISUALIZER_PLUGIN_PATH");

  // Then look for plugins on compile-time defined path.
  // Plugins installed by gazebo end up here
  ignition::gui::addPluginPath(PLUGIN_INSTALL_PATH);

  // Attempt to load window layout from default config file.
  // If it's not availeble, load it from configFile.
  if (!ignition::gui::loadDefaultConfig()) {
    // If config file parsed from args is not
    // a valid path, exit the application.
    if(!ignition::gui::loadConfig(configFile)) {
      return 1;
    }
  }

  // Create main window
  ignition::gui::createMainWindow();

  auto win = ignition::gui::mainWindow();
  win->setWindowTitle(versionStr);

  ignition::gui::runMainWindow();

  // Save configuration after the window has
  // been closed (but not destroyed)
  win->SaveConfig(ignition::gui::defaultConfigPath());

  ignition::gui::stop();

  return 0;
}
