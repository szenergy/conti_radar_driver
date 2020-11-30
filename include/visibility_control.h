// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RADAR_CONTI_ARS408__VISIBILITY_CONTROL_H_
#define RADAR_CONTI_ARS408__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RADAR_CONTI_ARS408_EXPORT __attribute__ ((dllexport))
    #define RADAR_CONTI_ARS408_IMPORT __attribute__ ((dllimport))
  #else
    #define RADAR_CONTI_ARS408_EXPORT __declspec(dllexport)
    #define RADAR_CONTI_ARS408_IMPORT __declspec(dllimport)
  #endif
  #ifdef RADAR_CONTI_ARS408_BUILDING_DLL
    #define RADAR_CONTI_ARS408_PUBLIC RADAR_CONTI_ARS408_EXPORT
  #else
    #define RADAR_CONTI_ARS408_PUBLIC RADAR_CONTI_ARS408_IMPORT
  #endif
  #define RADAR_CONTI_ARS408_PUBLIC_TYPE RADAR_CONTI_ARS408_PUBLIC
  #define RADAR_CONTI_ARS408_LOCAL
#else
  #define RADAR_CONTI_ARS408_RADAR_CONTI_ARS408 __attribute__ ((visibility("default")))
  #define RADAR_CONTI_ARS408_RADAR_CONTI_ARS408
  #if __GNUC__ >= 4
    #define RADAR_CONTI_ARS408_PUBLIC __attribute__ ((visibility("default")))
    #define RADAR_CONTI_ARS408_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RADAR_CONTI_ARS408_PUBLIC
    #define RADAR_CONTI_ARS408_LOCAL
  #endif
  #define RADAR_CONTI_ARS408_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // RADAR_CONTI_ARS408__VISIBILITY_CONTROL_H_
