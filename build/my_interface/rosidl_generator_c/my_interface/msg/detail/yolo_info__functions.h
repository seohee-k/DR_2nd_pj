// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from my_interface:msg/YoloInfo.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACE__MSG__DETAIL__YOLO_INFO__FUNCTIONS_H_
#define MY_INTERFACE__MSG__DETAIL__YOLO_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "my_interface/msg/rosidl_generator_c__visibility_control.h"

#include "my_interface/msg/detail/yolo_info__struct.h"

/// Initialize msg/YoloInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * my_interface__msg__YoloInfo
 * )) before or use
 * my_interface__msg__YoloInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
bool
my_interface__msg__YoloInfo__init(my_interface__msg__YoloInfo * msg);

/// Finalize msg/YoloInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
void
my_interface__msg__YoloInfo__fini(my_interface__msg__YoloInfo * msg);

/// Create msg/YoloInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * my_interface__msg__YoloInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
my_interface__msg__YoloInfo *
my_interface__msg__YoloInfo__create();

/// Destroy msg/YoloInfo message.
/**
 * It calls
 * my_interface__msg__YoloInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
void
my_interface__msg__YoloInfo__destroy(my_interface__msg__YoloInfo * msg);

/// Check for msg/YoloInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
bool
my_interface__msg__YoloInfo__are_equal(const my_interface__msg__YoloInfo * lhs, const my_interface__msg__YoloInfo * rhs);

/// Copy a msg/YoloInfo message.
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
ROSIDL_GENERATOR_C_PUBLIC_my_interface
bool
my_interface__msg__YoloInfo__copy(
  const my_interface__msg__YoloInfo * input,
  my_interface__msg__YoloInfo * output);

/// Initialize array of msg/YoloInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * my_interface__msg__YoloInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
bool
my_interface__msg__YoloInfo__Sequence__init(my_interface__msg__YoloInfo__Sequence * array, size_t size);

/// Finalize array of msg/YoloInfo messages.
/**
 * It calls
 * my_interface__msg__YoloInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
void
my_interface__msg__YoloInfo__Sequence__fini(my_interface__msg__YoloInfo__Sequence * array);

/// Create array of msg/YoloInfo messages.
/**
 * It allocates the memory for the array and calls
 * my_interface__msg__YoloInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
my_interface__msg__YoloInfo__Sequence *
my_interface__msg__YoloInfo__Sequence__create(size_t size);

/// Destroy array of msg/YoloInfo messages.
/**
 * It calls
 * my_interface__msg__YoloInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
void
my_interface__msg__YoloInfo__Sequence__destroy(my_interface__msg__YoloInfo__Sequence * array);

/// Check for msg/YoloInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_my_interface
bool
my_interface__msg__YoloInfo__Sequence__are_equal(const my_interface__msg__YoloInfo__Sequence * lhs, const my_interface__msg__YoloInfo__Sequence * rhs);

/// Copy an array of msg/YoloInfo messages.
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
ROSIDL_GENERATOR_C_PUBLIC_my_interface
bool
my_interface__msg__YoloInfo__Sequence__copy(
  const my_interface__msg__YoloInfo__Sequence * input,
  my_interface__msg__YoloInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MY_INTERFACE__MSG__DETAIL__YOLO_INFO__FUNCTIONS_H_
