// Copyright 2027 Lunabot
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// Standard ROS 2 symbol visibility boilerplate. Only Windows actually needs
// the dllimport/dllexport dance; on Linux this reduces to the visibility
// attribute. Kept because pluginlib libraries are expected to have it and
// omitting it produces confusing link errors if anyone ever cross-builds.

#ifndef LUNABOT_HARDWARE__VISIBILITY_CONTROL_H_
#define LUNABOT_HARDWARE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LUNABOT_HARDWARE_EXPORT __attribute__((dllexport))
#define LUNABOT_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define LUNABOT_HARDWARE_EXPORT __declspec(dllexport)
#define LUNABOT_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef LUNABOT_HARDWARE_BUILDING_DLL
#define LUNABOT_HARDWARE_PUBLIC LUNABOT_HARDWARE_EXPORT
#else
#define LUNABOT_HARDWARE_PUBLIC LUNABOT_HARDWARE_IMPORT
#endif
#define LUNABOT_HARDWARE_PUBLIC_TYPE LUNABOT_HARDWARE_PUBLIC
#define LUNABOT_HARDWARE_LOCAL
#else
#define LUNABOT_HARDWARE_EXPORT __attribute__((visibility("default")))
#define LUNABOT_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define LUNABOT_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define LUNABOT_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define LUNABOT_HARDWARE_PUBLIC
#define LUNABOT_HARDWARE_LOCAL
#endif
#define LUNABOT_HARDWARE_PUBLIC_TYPE
#endif

#endif  // LUNABOT_HARDWARE__VISIBILITY_CONTROL_H_
