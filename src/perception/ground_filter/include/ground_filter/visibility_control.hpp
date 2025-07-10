#ifndef GROUND_FILTER__VISIBILITY_CONTROL_HPP_
#define GROUND_FILTER__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GROUND_FILTER_EXPORT __attribute__ ((dllexport))
    #define GROUND_FILTER_IMPORT __attribute__ ((dllimport))
  #else
    #define GROUND_FILTER_EXPORT __declspec(dllexport)
    #define GROUND_FILTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef GROUND_FILTER_BUILDING_DLL
    #define GROUND_FILTER_PUBLIC GROUND_FILTER_EXPORT
  #else
    #define GROUND_FILTER_PUBLIC GROUND_FILTER_IMPORT
  #endif
  #define GROUND_FILTER_PUBLIC_TYPE GROUND_FILTER_PUBLIC
  #define GROUND_FILTER_LOCAL
#else
  #define GROUND_FILTER_EXPORT __attribute__ ((visibility("default")))
  #define GROUND_FILTER_IMPORT
  #if __GNUC__ >= 4
    #define GROUND_FILTER_PUBLIC __attribute__ ((visibility("default")))
    #define GROUND_FILTER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GROUND_FILTER_PUBLIC
    #define GROUND_FILTER_LOCAL
  #endif
  #define GROUND_FILTER_PUBLIC_TYPE
#endif

#endif  // GROUND_FILTER__VISIBILITY_CONTROL_HPP_ 