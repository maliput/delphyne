// Copyright 2018 Open Source Robotics Foundation
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

#include "backend/agent_plugin_loader.h"

#include <memory>
#include <string>
#include <unordered_set>

#include "backend/linb-any"

#include <ignition/common/Console.hh>
#include <ignition/common/Plugin.hh>
#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>

#include "backend/agent_plugin_base.h"

// default implementation
template <typename T>
struct TypeName {
  static const char* Get() { return typeid(T).name(); }
};

// a specialization for each type of those you want to support
// and don't like the string returned by typeid
template <>
struct TypeName<double> {
  static const char* Get() { return "Double"; }
};

template <>
struct TypeName<::drake::AutoDiffXd> {
  static const char* Get() { return "AutoDiffXd"; }
};

template <>
struct TypeName<::drake::symbolic::Expression> {
  static const char* Get() { return "Expression"; }
};

/// The function that loads a plugin given the name of the plugin.  Given a
/// plugin name like "Foo", this function will look for a file named 'libFoo.so'
/// in the colon-separated paths in AGENT_PLUGIN_PATH, followed by
/// ~/.delphyne/plugins.  If the file is found, it is loaded up, initialized,
/// and a std::unique_ptr to the concrete class is returned.  If it cannot be
/// found, or the load or initialization fails for some reason, a nullptr is
/// returned.
///
/// @tparam T must generally be a "double-like" type.  Currently, the supported
///           template types are 'double', 'drake::AutoDiffXd', and
///           'drake::symbolic::Expression'.
/// @tparam U must be one of the AgentPluginFactory types from
///           agent_plugin_base.h.  Due to some current limitations, it cannot
///           be a templated type like 'AgentPluginFactoryBase<double>';
///           instead, it must be something like 'AgentPluginFactoryDoubleBase'.
template <typename T, typename U>
std::unique_ptr<delphyne::backend::AgentPluginBase<T>> loadPluginInternal(
    const std::string& _filename) {
  igndbg << "Loading plugin [" << _filename << "]" << std::endl;

  // Setup the paths that we'll use to find the shared library.  Note that we
  // do not use the SetPluginPathEnv() method, since that puts the environment
  // variable at the back of the list, and we want to use it first.
  ignition::common::SystemPaths systemPaths;
  std::string env;
  if (ignition::common::env("AGENT_PLUGIN_PATH", env)) {
    systemPaths.AddPluginPaths(env);
  }
  if (ignition::common::env("HOME", env)) {
    systemPaths.AddPluginPaths(env + "/.delphyne/plugins");
  }

  // Now find the shared library.
  std::string pathToLib = systemPaths.FindSharedLibrary(_filename);
  if (pathToLib.empty()) {
    ignerr << "Failed to load plugin [" << _filename
           << "] : couldn't find shared library." << std::endl;
    return nullptr;
  }

  // Load plugin
  ignition::common::PluginLoader pluginLoader;

  std::unordered_set<std::string> pluginNames =
      pluginLoader.LoadLibrary(pathToLib);
  if (pluginNames.empty()) {
    ignerr << "Failed to load plugin [" << _filename
           << "] : couldn't load library on path [" << pathToLib << "]."
           << std::endl;
    return nullptr;
  }

  for (auto& pluginName : pluginNames) {
    if (pluginName.empty()) {
      continue;
    }
    ignition::common::PluginPtr commonPlugin =
        pluginLoader.Instantiate(pluginName);
    if (!commonPlugin) {
      continue;
    }

    std::ostringstream type;
    type << "::delphyne::backend::AgentPluginFactory" << TypeName<T>::Get()
         << "Base";

    // The reason for the factory style here is a bit opaque.  The problem is
    // that something has to hold onto the shared_ptr reference that is
    // commonPlugin.  One way to do this is to use
    // commonPlugin->QueryInterfaceSharedPtr() and return the shared_ptr, but
    // the higher layers of the drake DiagramBuilder expect to get a unique_ptr
    // of which they control the lifecycle.  Just returning a raw pointer (such
    // as what commonPlugin->QueryInterface would return) doesn't properly hold
    // the reference, and hence the plugin would get destructed during the
    // return from this method.  Instead, the loadable agents actually expose
    // a factory method as their interface, and the code below calls the
    // ->Create() method on the factory to actually create the real object
    // inside of the loadable agent.  Once that is done, we use the
    // ->setFactoryPlugin() method to store a reference to the commonPlugin
    // shared_ptr, which makes sure it stays around for the lifetime of the
    // loaded agent.
    auto factory = commonPlugin->QueryInterface<U>(type.str());
    if (factory == nullptr) {
      ignerr << "Failed to load plugin [" << _filename
             << "] : couldn't load library factory." << std::endl;
      return nullptr;
    }
    std::unique_ptr<delphyne::backend::AgentPluginBase<T>> plugin =
        factory->Create();
    plugin->setFactoryPlugin(commonPlugin);
    return plugin;
  }

  ignerr << "Failed to load plugin [" << _filename
         << "] : couldn't load library on path [" << pathToLib << "]."
         << std::endl;
  return nullptr;
}

namespace delphyne {
namespace backend {

// This needs to be in the delphyne::backend namespace explicitly due to a
// gcc bug.

template <>
std::unique_ptr<delphyne::backend::AgentPluginBase<double>> loadPlugin<double>(
    const std::string& _filename) {
  return loadPluginInternal<double,
                            delphyne::backend::AgentPluginFactoryDoubleBase>(
      _filename);
}

template <>
std::unique_ptr<delphyne::backend::AgentPluginBase<::drake::AutoDiffXd>>
loadPlugin<::drake::AutoDiffXd>(const std::string& _filename) {
  return loadPluginInternal<
      ::drake::AutoDiffXd, delphyne::backend::AgentPluginFactoryAutoDiffXdBase>(
      _filename);
}

template <>
std::unique_ptr<
    delphyne::backend::AgentPluginBase<::drake::symbolic::Expression>>
loadPlugin<::drake::symbolic::Expression>(const std::string& _filename) {
  return loadPluginInternal<
      ::drake::symbolic::Expression,
      delphyne::backend::AgentPluginFactoryExpressionBase>(_filename);
}
}  // namespace backend
}  // namespace delphyne
