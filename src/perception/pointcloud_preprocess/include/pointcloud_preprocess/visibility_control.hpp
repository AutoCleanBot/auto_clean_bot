#ifndef POINTCLOUD_PREPROCESS__VISIBILITY_CONTROL_HPP_
#define POINTCLOUD_PREPROCESS__VISIBILITY_CONTROL_HPP_

// 定义导出宏，用于库的导出控制
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POINTCLOUD_PREPROCESS_EXPORT __attribute__ ((dllexport))
    #define POINTCLOUD_PREPROCESS_IMPORT __attribute__ ((dllimport))
  #else
    #define POINTCLOUD_PREPROCESS_EXPORT __declspec(dllexport)
    #define POINTCLOUD_PREPROCESS_IMPORT __declspec(dllimport)
  #endif
  #ifdef POINTCLOUD_PREPROCESS_BUILDING_DLL
    #define POINTCLOUD_PREPROCESS_PUBLIC POINTCLOUD_PREPROCESS_EXPORT
  #else
    #define POINTCLOUD_PREPROCESS_PUBLIC POINTCLOUD_PREPROCESS_IMPORT
  #endif
  #define POINTCLOUD_PREPROCESS_PUBLIC_TYPE POINTCLOUD_PREPROCESS_PUBLIC
  #define POINTCLOUD_PREPROCESS_LOCAL
#else
  #define POINTCLOUD_PREPROCESS_EXPORT __attribute__ ((visibility("default")))
  #define POINTCLOUD_PREPROCESS_IMPORT
  #if __GNUC__ >= 4
    #define POINTCLOUD_PREPROCESS_PUBLIC __attribute__ ((visibility("default")))
    #define POINTCLOUD_PREPROCESS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POINTCLOUD_PREPROCESS_PUBLIC
    #define POINTCLOUD_PREPROCESS_LOCAL
  #endif
  #define POINTCLOUD_PREPROCESS_PUBLIC_TYPE
#endif

#endif  // POINTCLOUD_PREPROCESS__VISIBILITY_CONTROL_HPP_ 