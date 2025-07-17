// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from yolo_data:msg/MonitorData.idl
// generated code does not contain a copyright notice

#ifndef YOLO_DATA__MSG__DETAIL__MONITOR_DATA__FUNCTIONS_H_
#define YOLO_DATA__MSG__DETAIL__MONITOR_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "yolo_data/msg/rosidl_generator_c__visibility_control.h"

#include "yolo_data/msg/detail/monitor_data__struct.h"

/// Initialize msg/MonitorData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * yolo_data__msg__MonitorData
 * )) before or use
 * yolo_data__msg__MonitorData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
bool
yolo_data__msg__MonitorData__init(yolo_data__msg__MonitorData * msg);

/// Finalize msg/MonitorData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
void
yolo_data__msg__MonitorData__fini(yolo_data__msg__MonitorData * msg);

/// Create msg/MonitorData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * yolo_data__msg__MonitorData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
yolo_data__msg__MonitorData *
yolo_data__msg__MonitorData__create();

/// Destroy msg/MonitorData message.
/**
 * It calls
 * yolo_data__msg__MonitorData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
void
yolo_data__msg__MonitorData__destroy(yolo_data__msg__MonitorData * msg);

/// Check for msg/MonitorData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
bool
yolo_data__msg__MonitorData__are_equal(const yolo_data__msg__MonitorData * lhs, const yolo_data__msg__MonitorData * rhs);

/// Copy a msg/MonitorData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
bool
yolo_data__msg__MonitorData__copy(
  const yolo_data__msg__MonitorData * input,
  yolo_data__msg__MonitorData * output);

/// Initialize array of msg/MonitorData messages.
/**
 * It allocates the memory for the number of elements and calls
 * yolo_data__msg__MonitorData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
bool
yolo_data__msg__MonitorData__Sequence__init(yolo_data__msg__MonitorData__Sequence * array, size_t size);

/// Finalize array of msg/MonitorData messages.
/**
 * It calls
 * yolo_data__msg__MonitorData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
void
yolo_data__msg__MonitorData__Sequence__fini(yolo_data__msg__MonitorData__Sequence * array);

/// Create array of msg/MonitorData messages.
/**
 * It allocates the memory for the array and calls
 * yolo_data__msg__MonitorData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
yolo_data__msg__MonitorData__Sequence *
yolo_data__msg__MonitorData__Sequence__create(size_t size);

/// Destroy array of msg/MonitorData messages.
/**
 * It calls
 * yolo_data__msg__MonitorData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
void
yolo_data__msg__MonitorData__Sequence__destroy(yolo_data__msg__MonitorData__Sequence * array);

/// Check for msg/MonitorData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
bool
yolo_data__msg__MonitorData__Sequence__are_equal(const yolo_data__msg__MonitorData__Sequence * lhs, const yolo_data__msg__MonitorData__Sequence * rhs);

/// Copy an array of msg/MonitorData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_yolo_data
bool
yolo_data__msg__MonitorData__Sequence__copy(
  const yolo_data__msg__MonitorData__Sequence * input,
  yolo_data__msg__MonitorData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // YOLO_DATA__MSG__DETAIL__MONITOR_DATA__FUNCTIONS_H_
