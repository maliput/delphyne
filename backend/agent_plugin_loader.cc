// Copyright 2017 Toyota Research Institute

#include "backend/agent_plugin_loader.h"

#include <memory>
#include <string>
#include <unordered_set>

#include "backend/agent_plugin_base.h"

#include <ignition/common/Console.hh>
#include <ignition/common/Plugin.hh>
#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>

namespace {

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

// The function that loads a plugin given the name of the plugin.  Given a
// plugin name like "Foo", this function will look for a file named 'libFoo.so'
// in the colon-separated paths in AGENT_PLUGIN_PATH, followed by
// ~/.delphyne/plugins.  If the file is found, it is loaded up, initialized,
// and a std::unique_ptr to the concrete class is returned.  If it cannot be
// found, or the load or initialization fails for some reason, a nullptr is
// returned.
//
// @tparam T must generally be a "double-like" type.  Currently, the supported
//           template types are 'double', 'drake::AutoDiffXd', and
//           'drake::symbolic::Expression'.
// @tparam U must be one of the AgentPluginFactory types from
//           agent_plugin_base.h.  Due to some current limitations, it cannot
//           be a templated type like 'AgentPluginFactoryBase<double>';
//           instead, it must be something like 'AgentPluginFactoryDoubleBase'.
template <typename T, typename U>
std::unique_ptr<delphyne::AgentPluginBase<T>> LoadPluginInternal(
    const std::string& file_name) {
  igndbg << "Loading plugin [" << file_name << "]" << std::endl;

  // Setup the paths that we'll use to find the shared library.  Note that we
  // do not use the SetPluginPathEnv() method, since that puts the environment
  // variable at the back of the list, and we want to use it first.
  ignition::common::SystemPaths system_paths;
  std::string env;
  if (ignition::common::env("DELPHYNE_AGENT_PLUGIN_PATH", env)) {
    system_paths.AddPluginPaths(env);
  }
  if (ignition::common::env("HOME", env)) {
    system_paths.AddPluginPaths(env + "/.delphyne/plugins");
  }

  // Now find the shared library.
  const std::string path_to_lib = system_paths.FindSharedLibrary(file_name);
  if (path_to_lib.empty()) {
    ignerr << "Failed to load plugin [" << file_name
           << "] : couldn't find shared library." << std::endl;
    return nullptr;
  }

  // Load plugin
  ignition::common::PluginLoader plugin_loader;

  const std::unordered_set<std::string> plugin_names =
      plugin_loader.LoadLibrary(path_to_lib);
  if (plugin_names.empty()) {
    ignerr << "Failed to load plugin [" << file_name
           << "] : couldn't load library on path [" << path_to_lib << "]."
           << std::endl;
    return nullptr;
  }

  for (const std::string& plugin_name : plugin_names) {
    if (plugin_name.empty()) {
      continue;
    }
    ignition::common::PluginPtr common_plugin =
        plugin_loader.Instantiate(plugin_name);
    if (!common_plugin) {
      continue;
    }

    std::ostringstream type;
    type << "::delphyne::AgentPluginFactory" << TypeName<T>::Get() << "Base";

    // The reason for the factory style here is a bit opaque.  The problem is
    // that something has to hold onto the shared_ptr reference that is
    // common_plugin.  One way to do this is to use
    // common_plugin->QueryInterfaceSharedPtr() and return the shared_ptr, but
    // the higher layers of the drake DiagramBuilder expect to get a unique_ptr
    // of which they control the lifecycle.  Just returning a raw pointer (such
    // as what common_plugin->QueryInterface would return) doesn't properly hold
    // the reference, and hence the plugin would get destructed during the
    // return from this method.  Instead, the loadable agents actually expose
    // a factory method as their interface, and the code below calls the
    // ->Create() method on the factory to actually create the real object
    // inside of the loadable agent.  Once that is done, we use the
    // ->SetPlugin() method to store a reference to the common_plugin
    // shared_ptr, which makes sure it stays around for the lifetime of the
    // loaded agent.
    U* factory = common_plugin->QueryInterface<U>(type.str());
    if (factory == nullptr) {
      ignerr << "Failed to load plugin [" << file_name
             << "] : couldn't load library factory." << std::endl;
      return nullptr;
    }
    std::unique_ptr<delphyne::AgentPluginBase<T>> plugin = factory->Create();
    plugin->SetPlugin(common_plugin);
    return plugin;
  }

  ignerr << "Failed to load plugin [" << file_name
         << "] : couldn't load library on path [" << path_to_lib << "]."
         << std::endl;
  return nullptr;
}

}  // namespace

namespace delphyne {

// This needs to be in the delphyne::backend namespace explicitly due to a
// gcc bug.

template <>
std::unique_ptr<delphyne::AgentPluginBase<double>> LoadPlugin<double>(
    const std::string& file_name) {
  return LoadPluginInternal<double, delphyne::AgentPluginFactoryDoubleBase>(
      file_name);
}

template <>
std::unique_ptr<delphyne::AgentPluginBase<::drake::AutoDiffXd>>
LoadPlugin<::drake::AutoDiffXd>(const std::string& file_name) {
  return LoadPluginInternal<::drake::AutoDiffXd,
                            delphyne::AgentPluginFactoryAutoDiffXdBase>(
      file_name);
}

template <>
std::unique_ptr<delphyne::AgentPluginBase<::drake::symbolic::Expression>>
LoadPlugin<::drake::symbolic::Expression>(const std::string& file_name) {
  return LoadPluginInternal<::drake::symbolic::Expression,
                            delphyne::AgentPluginFactoryExpressionBase>(
      file_name);
}
}  // namespace delphyne
