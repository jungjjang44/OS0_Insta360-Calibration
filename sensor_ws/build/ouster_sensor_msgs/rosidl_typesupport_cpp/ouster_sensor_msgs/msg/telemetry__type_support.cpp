// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from ouster_sensor_msgs:msg/Telemetry.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ouster_sensor_msgs/msg/detail/telemetry__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ouster_sensor_msgs
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _Telemetry_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Telemetry_type_support_ids_t;

static const _Telemetry_type_support_ids_t _Telemetry_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Telemetry_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Telemetry_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Telemetry_type_support_symbol_names_t _Telemetry_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ouster_sensor_msgs, msg, Telemetry)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ouster_sensor_msgs, msg, Telemetry)),
  }
};

typedef struct _Telemetry_type_support_data_t
{
  void * data[2];
} _Telemetry_type_support_data_t;

static _Telemetry_type_support_data_t _Telemetry_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Telemetry_message_typesupport_map = {
  2,
  "ouster_sensor_msgs",
  &_Telemetry_message_typesupport_ids.typesupport_identifier[0],
  &_Telemetry_message_typesupport_symbol_names.symbol_name[0],
  &_Telemetry_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Telemetry_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Telemetry_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace ouster_sensor_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ouster_sensor_msgs::msg::Telemetry>()
{
  return &::ouster_sensor_msgs::msg::rosidl_typesupport_cpp::Telemetry_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, ouster_sensor_msgs, msg, Telemetry)() {
  return get_message_type_support_handle<ouster_sensor_msgs::msg::Telemetry>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
