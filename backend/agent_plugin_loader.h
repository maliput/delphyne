// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>
#include <string>

#include "../include/delphyne/agent_plugin_base.h"

namespace delphyne {

  /// The function that can be used to load in a loadable Agent from a shared
  /// object.  Given a `plugin_name`, this call will look for a file called
  /// `libfile_name.so` in 1) paths referred to by the DELPHYNE_AGENT_PLUGIN_PATH
  /// environment variable, 2) ~/.delphyne/plugin path and finally 3) delphyne
  /// paths relative to the location of this library.
  ///
  /// If found, the shared object will be loaded into the simulation, configured, and
  /// initialized.  See `agent_plugin_base.h` for more information about the
  /// methods that a loadable agent need to implement to get loaded in.
  ///
  /// If no class_name is provided, then it will simply load the first agent
  /// the plugin enumerates (convenience mechanism for plugins that only hold one
  /// agent).
  template <typename T>
  std::unique_ptr<AgentPluginBase<T>> LoadPlugin(
    const std::string& plugin_library_name,
    const std::string& plugin_name="default");

}  // namespace delphyne
