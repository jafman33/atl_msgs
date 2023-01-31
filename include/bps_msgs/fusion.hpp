#ifndef ATL_MSGS__FUSION_HPP_
#define ATL_MSGS__FUSION_HPP_

#include <boost/fusion/adapted/struct.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>


#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/list_parameters_result.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_event_descriptors.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>


#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>


#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>


#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>


#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <sensor_msgs/msg/laser_echo.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>


#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_init.hpp>
#include <visualization_msgs/msg/interactive_marker_pose.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>


#include <atl_msgs/msg/servo_input.hpp>
#include <atl_msgs/msg/servos_input.hpp>


// builtin_interfaces

BOOST_FUSION_ADAPT_STRUCT(
  builtin_interfaces::msg::Duration,
  sec, nanosec)

BOOST_FUSION_ADAPT_STRUCT(
  builtin_interfaces::msg::Time,
  sec, nanosec)


// rcl_interfaces

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::FloatingPointRange,
  from_value, to_value, step)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::IntegerRange,
  from_value, to_value, step)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::ListParametersResult,
  names, prefixes)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::Log,
  stamp, level, name, msg, file, function, line)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::Parameter,
  name, value)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::ParameterDescriptor,
  name, type, description, additional_constraints, read_only, dynamic_typing)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::ParameterEvent,
  stamp, node, new_parameters, changed_parameters, deleted_parameters)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::ParameterEventDescriptors,
  new_parameters, changed_parameters, deleted_parameters)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::ParameterType)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::ParameterValue,
  type, bool_value, integer_value, double_value, string_value, byte_array_value, bool_array_value,
  integer_array_value, double_array_value, string_array_value)

BOOST_FUSION_ADAPT_STRUCT(
  rcl_interfaces::msg::SetParametersResult,
  successful, reason)


// std_msgs

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Bool,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Byte,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::ByteMultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Char,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::ColorRGBA,
  r, g, b, a)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Empty)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Float32,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Float32MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Float64,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Float64MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Header,
  stamp, frame_id)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Int16,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Int16MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Int32,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Int32MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Int64,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Int64MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Int8,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::Int8MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::MultiArrayDimension,
  label, size, stride)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::MultiArrayLayout,
  dim, data_offset)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::String,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::UInt16,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::UInt16MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::UInt32,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::UInt32MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::UInt64,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::UInt64MultiArray,
  layout, data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::UInt8,
  data)

BOOST_FUSION_ADAPT_STRUCT(
  std_msgs::msg::UInt8MultiArray,
  layout, data)


// geometry_msgs

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Accel,
  linear, angular)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::AccelStamped,
  header, accel)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::AccelWithCovariance,
  accel, covariance)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::AccelWithCovarianceStamped,
  header, accel)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Inertia,
  m, com, ixx, ixy, ixz, iyy, iyz, izz)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::InertiaStamped,
  header, inertia)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Point,
  x, y, z)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Point32,
  x, y, z)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::PointStamped,
  header, point)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Polygon,
  points)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::PolygonStamped,
  header, polygon)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Pose,
  position, orientation)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Pose2D,
  x, y, theta)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::PoseArray,
  header, poses)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::PoseStamped,
  header, pose)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::PoseWithCovariance,
  pose, covariance)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::PoseWithCovarianceStamped,
  header, pose)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Quaternion,
  x, y, z, w)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::QuaternionStamped,
  header, quaternion)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Transform,
  translation, rotation)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::TransformStamped,
  header, child_frame_id, transform)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Twist,
  linear, angular)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::TwistStamped,
  header, twist)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::TwistWithCovariance,
  twist, covariance)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::TwistWithCovarianceStamped,
  header, twist)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Vector3,
  x, y, z)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Vector3Stamped,
  header, vector)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::Wrench,
  force, torque)

BOOST_FUSION_ADAPT_STRUCT(
  geometry_msgs::msg::WrenchStamped,
  header, wrench)


// nav_msgs

BOOST_FUSION_ADAPT_STRUCT(
  nav_msgs::msg::GridCells,
  header, cell_width, cell_height, cells)

BOOST_FUSION_ADAPT_STRUCT(
  nav_msgs::msg::MapMetaData,
  map_load_time, resolution, width, height, origin)

BOOST_FUSION_ADAPT_STRUCT(
  nav_msgs::msg::OccupancyGrid,
  header, info, data)

BOOST_FUSION_ADAPT_STRUCT(
  nav_msgs::msg::Odometry,
  header, child_frame_id, pose, twist)

BOOST_FUSION_ADAPT_STRUCT(
  nav_msgs::msg::Path,
  header, poses)


// sensor_msgs

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::BatteryState,
  header, voltage, temperature, current, charge, capacity, design_capacity, percentage,
  power_supply_status, power_supply_health, power_supply_technology, present, cell_voltage,
  cell_temperature, location, serial_number)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::CameraInfo,
  header, height, width, distortion_model, d, k, r, p, binning_x, binning_y, roi)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::ChannelFloat32,
  name, values)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::CompressedImage,
  header, format, data)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::FluidPressure,
  header, fluid_pressure, variance)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::Illuminance,
  header, illuminance, variance)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::Image,
  header, height, width, encoding, is_bigendian, step, data)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::Imu,
  header, orientation, orientation_covariance, angular_velocity, angular_velocity_covariance,
  linear_acceleration, linear_acceleration_covariance)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::JointState,
  header, name, position, velocity, effort)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::Joy,
  header, axes, buttons)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::JoyFeedback,
  type, id, intensity)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::JoyFeedbackArray,
  array)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::LaserEcho,
  echoes)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::LaserScan,
  header, angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max,
  ranges, intensities)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::MagneticField,
  header, magnetic_field, magnetic_field_covariance)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::MultiDOFJointState,
  header, joint_names, transforms, twist, wrench)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::MultiEchoLaserScan,
  header, angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max,
  ranges, intensities)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::NavSatFix,
  header, status, latitude, longitude, altitude, position_covariance, position_covariance_type)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::NavSatStatus,
  status, service)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::PointCloud,
  header, points, channels)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::PointCloud2,
  header, height, width, fields, is_bigendian, point_step, row_step, data, is_dense)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::PointField,
  name, offset, datatype, count)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::Range,
  header, radiation_type, field_of_view, min_range, max_range, range)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::RegionOfInterest,
  x_offset, y_offset, height, width, do_rectify)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::RelativeHumidity,
  header, relative_humidity, variance)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::Temperature,
  header, temperature, variance)

BOOST_FUSION_ADAPT_STRUCT(
  sensor_msgs::msg::TimeReference,
  header, time_ref, source)


// visualization_msgs

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::ImageMarker,
  header, ns, id, type, action, position, scale, outline_color, filled, fill_color, lifetime,
  points, outline_colors)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::InteractiveMarker,
  header, pose, name, description, scale, menu_entries, controls)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::InteractiveMarkerControl,
  name, orientation, orientation_mode, interaction_mode, always_visible, markers,
  independent_marker_orientation, description)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::InteractiveMarkerFeedback,
  header, client_id, marker_name, control_name, event_type, pose, menu_entry_id, mouse_point,
  mouse_point_valid)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::InteractiveMarkerInit,
  server_id, seq_num, markers)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::InteractiveMarkerPose,
  header, pose, name)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::InteractiveMarkerUpdate,
  server_id, seq_num, type, markers, poses, erases)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::Marker,
  header, ns, id, type, action, pose, scale, color, lifetime, frame_locked, points, colors, text,
  mesh_resource, mesh_use_embedded_materials)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::MarkerArray,
  markers)

BOOST_FUSION_ADAPT_STRUCT(
  visualization_msgs::msg::MenuEntry,
  id, parent_id, title, command, command_type)


// atl_msgs


BOOST_FUSION_ADAPT_STRUCT(
  atl_msgs::msg::EngineInput,
  delta)

BOOST_FUSION_ADAPT_STRUCT(
  atl_msgs::msg::EnginesInput,
  header, inputs, active)

#endif  // ATL_LIBRARY__ATL_MSG_FUSION_HPP_
