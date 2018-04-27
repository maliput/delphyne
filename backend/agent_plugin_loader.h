// Copyright 2017 Toyota Research Institute

#pragma once

#include <memory>
#include <string>

#include "backend/agent_plugin_base.h"
#include "backend/system.h"

namespace delphyne {

/// The function that can be used to load in a loadable Agent from a shared
/// object.  Given a `file_name`, this call will look for a file called
/// `libfile_name.so` in the hard-coded ~/.delphyne/plugin path and the path
/// referred to by the DELPHYNE_AGENT_PLUGIN_PATH environment variable.  If
/// found, the shared object will be loaded into the simulation, configured, and
/// initialized.  See `agent_plugin_base.h` for more information about the
/// methods that a loadable agent need to implement to get loaded in.
template <typename T>
std::unique_ptr<delphyne::AgentPluginBase<T>> LoadPlugin(
    const std::string& file_name);
}  // namespace delphyne
