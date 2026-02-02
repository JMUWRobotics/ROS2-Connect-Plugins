// Copyright (c) 2026 Chair of Robotics (Computer Science XVII) @ Julius–Maximilians–University
// 
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef CONNECT_PLUGINS__VISIBILITY_CONTROL_H_
#define CONNECT_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONNECT_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define CONNECT_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define CONNECT_PLUGINS_EXPORT __declspec(dllexport)
    #define CONNECT_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONNECT_PLUGINS_BUILDING_LIBRARY
    #define CONNECT_PLUGINS_PUBLIC CONNECT_PLUGINS_EXPORT
  #else
    #define CONNECT_PLUGINS_PUBLIC CONNECT_PLUGINS_IMPORT
  #endif
  #define CONNECT_PLUGINS_PUBLIC_TYPE CONNECT_PLUGINS_PUBLIC
  #define CONNECT_PLUGINS_LOCAL
#else
  #define CONNECT_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define CONNECT_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define CONNECT_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define CONNECT_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONNECT_PLUGINS_PUBLIC
    #define CONNECT_PLUGINS_LOCAL
  #endif
  #define CONNECT_PLUGINS_PUBLIC_TYPE
#endif

#endif  // CONNECT_PLUGINS__VISIBILITY_CONTROL_H_
