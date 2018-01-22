/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#pragma once

#if defined(__GNUC__)
#define DELPHYNE_BACKEND_DEPRECATED(version) __attribute__((deprecated))
#define DELPHYNE_BACKEND_FORCEINLINE __attribute__((always_inline))
#elif defined(_WIN32)
#define DELPHYNE_BACKEND_DEPRECATED(version) ()
#define DELPHYNE_BACKEND_FORCEINLINE __forceinline
#else
#define DELPHYNE_BACKEND_DEPRECATED(version) ()
#define DELPHYNE_BACKEND_FORCEINLINE
#endif

/// \def DELPHYNE_BACKEND_VISIBLE
/// Use to represent "symbol visible" if supported

/// \def DELPHYNE_BACKEND_HIDDEN
/// Use to represent "symbol hidden" if supported
#if defined _WIN32 || defined __CYGWIN__
#ifdef BUILDING_DLL
#ifdef __GNUC__
#define DELPHYNE_BACKEND_VISIBLE __attribute__((dllexport))
#else
#define DELPHYNE_BACKEND_VISIBLE __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define DELPHYNE_BACKEND_VISIBLE __attribute__((dllimport))
#else
#define DELPHYNE_BACKEND_VISIBLE __declspec(dllimport)
#endif
#endif
#define DELPHYNE_BACKEND_HIDDEN
#else
#if __GNUC__ >= 4
#define DELPHYNE_BACKEND_VISIBLE __attribute__((visibility("default")))
#define DELPHYNE_BACKEND_HIDDEN __attribute__((visibility("hidden")))
#else
#define DELPHYNE_BACKEND_VISIBLE
#define DELPHYNE_BACKEND_HIDDEN
#endif
#endif
