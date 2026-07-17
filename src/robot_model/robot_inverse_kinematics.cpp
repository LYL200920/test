#include "robot_inverse_kinematics.h"

#include "robot_joint_state_builder.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

namespace robot_model
{
namespace
{

constexpr std::size_t JOINT_COUNT = 6;
constexpr double PI = 3.14159265358979323846;
constexpr double RADIANS_TO_DEGREES = 180.0 / PI;

using Joint_Vector = std::array<double, JOINT_COUNT>;

template<std::size_t Task_Size>
using Task_Vector = std::array<double, Task_Size>;

template<std::size_t Task_Size>
using Task_Matrix = std::array<std::array<double, Task_Size>, Task_Size>;

template<std::size_t Task_Size>
using Task_Jacobian =
  std::array<std::array<double, JOINT_COUNT>, Task_Size>;

class Solve_Clock
{
public:
  explicit Solve_Clock (double time_budget_ms)
    : m_time_budget_ms (time_budget_ms),
      m_started_at (std::chrono::steady_clock::now ( )) { }

  bool Expired ( ) const
  {
    return m_time_budget_ms > 0.0 && Elapsed_Milliseconds ( ) >= m_time_budget_ms;
  }

  double Elapsed_Milliseconds ( ) const
  {
    return std::chrono::duration<double, std::milli> (
      std::chrono::steady_clock::now ( ) - m_started_at).count ( );
  }

private:
  double m_time_budget_ms = 0.0;
  std::chrono::steady_clock::time_point m_started_at;
};

double vector_length (const Point3& value)
{
  return std::sqrt (
    value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

Point3 subtract (const Point3& lhs, const Point3& rhs)
{
  return { lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2] };
}

Point3 cross_product (const Point3& lhs, const Point3& rhs)
{
  return {
    lhs[1] * rhs[2] - lhs[2] * rhs[1],
    lhs[2] * rhs[0] - lhs[0] * rhs[2],
    lhs[0] * rhs[1] - lhs[1] * rhs[0]
  };
}

template<std::size_t Size>
bool solve_linear_system (Task_Matrix<Size> matrix,
                          Task_Vector<Size> rhs,
                          Task_Vector<Size>* solution)
{
  if( !solution ) return false;

  for( std::size_t column = 0; column < Size; ++column )
  {
    std::size_t pivot = column;
    for( std::size_t row = column + 1; row < Size; ++row )
    {
      if( std::abs (matrix[row][column]) >
          std::abs (matrix[pivot][column]) )
      {
        pivot = row;
      }
    }
    if( std::abs (matrix[pivot][column]) < 1.0e-12 ) return false;
    if( pivot != column )
    {
      std::swap (matrix[pivot], matrix[column]);
      std::swap (rhs[pivot], rhs[column]);
    }

    const double divisor = matrix[column][column];
    for( std::size_t item = column; item < Size; ++item )
    {
      matrix[column][item] /= divisor;
    }
    rhs[column] /= divisor;

    for( std::size_t row = 0; row < Size; ++row )
    {
      if( row == column ) continue;
      const double factor = matrix[row][column];
      for( std::size_t item = column; item < Size; ++item )
      {
        matrix[row][item] -= factor * matrix[column][item];
      }
      rhs[row] -= factor * rhs[column];
    }
  }

  *solution = rhs;
  return true;
}

template<std::size_t Task_Size>
bool solve_damped_least_squares (
  const Task_Jacobian<Task_Size>& jacobian,
  const Task_Vector<Task_Size>& error,
  double damping,
  Joint_Vector* joint_delta_radians)
{
  if( !joint_delta_radians ) return false;

  Task_Matrix<Task_Size> normal = { };
  for( std::size_t row = 0; row < Task_Size; ++row )
  {
    for( std::size_t column = 0; column < Task_Size; ++column )
    {
      for( std::size_t joint = 0; joint < JOINT_COUNT; ++joint )
      {
        normal[row][column] +=
          jacobian[row][joint] * jacobian[column][joint];
      }
    }
    normal[row][row] += damping * damping;
  }

  Task_Vector<Task_Size> weighted_error = { };
  if( !solve_linear_system (normal, error, &weighted_error) ) return false;

  joint_delta_radians->fill (0.0);
  for( std::size_t joint = 0; joint < JOINT_COUNT; ++joint )
  {
    for( std::size_t row = 0; row < Task_Size; ++row )
    {
      ( *joint_delta_radians )[joint] +=
        jacobian[row][joint] * weighted_error[row];
    }
  }
  return true;
}

double lower_limit_at (const Robot_Kinematic_Params& params,
                       std::size_t index)
{
  return index < params.joint_mins.size ( )
    ? params.joint_mins[index]
    : -std::numeric_limits<double>::infinity ( );
}

double upper_limit_at (const Robot_Kinematic_Params& params,
                       std::size_t index)
{
  return index < params.joint_maxs.size ( )
    ? params.joint_maxs[index]
    : std::numeric_limits<double>::infinity ( );
}

double clamp_joint_input (double value,
                          const Robot_Kinematic_Params& params,
                          std::size_t index)
{
  double lower = lower_limit_at (params, index);
  double upper = upper_limit_at (params, index);
  if( lower > upper ) std::swap (lower, upper);
  return std::clamp (value, lower, upper);
}

bool joint_is_locked (const Robot_Kinematic_Params& params,
                      std::size_t joint)
{
  return std::abs (
    upper_limit_at (params, joint) - lower_limit_at (params, joint)) < 1.0e-12;
}

Joint_Vector clamped_input_angles (const Robot_Joint_State& initial_state,
                                   const Robot_Kinematic_Params& params)
{
  auto input_angles = initial_state.input_angles_deg;
  for( std::size_t joint = 0; joint < JOINT_COUNT; ++joint )
  {
    input_angles[joint] = clamp_joint_input (
      input_angles[joint], params, joint);
  }
  return input_angles;
}

void apply_joint_delta (const Joint_Vector& delta_radians,
                        const Robot_Kinematic_Params& params,
                        double max_joint_step_deg,
                        Joint_Vector* input_angles)
{
  if( !input_angles ) return;
  const double max_step = std::max (max_joint_step_deg, 1.0e-6);
  for( std::size_t joint = 0; joint < JOINT_COUNT; ++joint )
  {
    const double delta_degrees = std::clamp (
      delta_radians[joint] * RADIANS_TO_DEGREES,
      -max_step,
      max_step);
    ( *input_angles )[joint] = clamp_joint_input (
      ( *input_angles )[joint] + delta_degrees, params, joint);
  }
}

Task_Jacobian<3> build_position_jacobian (
  const Robot_Forward_Kinematics_Result& forward,
  const Robot_Kinematic_Params& params,
  const Point3& flange_position)
{
  Task_Jacobian<3> jacobian = { };
  for( std::size_t joint = 0; joint < JOINT_COUNT; ++joint )
  {
    if( joint_is_locked (params, joint) ) continue;
    const auto radius = subtract (
      flange_position, forward.joint_positions_world[joint]);
    const auto derivative = cross_product (
      forward.joint_axes_world[joint], radius);
    const double direction = Joint_Direction_At (params, joint);
    for( std::size_t row = 0; row < 3; ++row )
    {
      jacobian[row][joint] = derivative[row] * direction;
    }
  }
  return jacobian;
}

Task_Jacobian<6> build_pose_jacobian (
  const Robot_Forward_Kinematics_Result& forward,
  const Robot_Kinematic_Params& params,
  const Point3& flange_position,
  double orientation_weight)
{
  Task_Jacobian<6> jacobian = { };
  const auto position_jacobian = build_position_jacobian (
    forward, params, flange_position);
  for( std::size_t joint = 0; joint < JOINT_COUNT; ++joint )
  {
    for( std::size_t row = 0; row < 3; ++row )
    {
      jacobian[row][joint] = position_jacobian[row][joint];
      if( !joint_is_locked (params, joint) )
      {
        jacobian[row + 3][joint] =
          forward.joint_axes_world[joint][row] *
          Joint_Direction_At (params, joint) * orientation_weight;
      }
    }
  }
  return jacobian;
}

Point3 orientation_error_vector (const Matrix4& target,
                                 const Matrix4& current)
{
  double relative[3][3] = { };
  for( std::size_t row = 0; row < 3; ++row )
  {
    for( std::size_t column = 0; column < 3; ++column )
    {
      for( std::size_t item = 0; item < 3; ++item )
      {
        relative[row][column] +=
          target[row][item] * current[column][item];
      }
    }
  }

  const double cosine = std::clamp (
    ( relative[0][0] + relative[1][1] + relative[2][2] - 1.0 ) * 0.5,
    -1.0,
    1.0);
  const double angle = std::acos (cosine);
  if( angle < 1.0e-9 )
  {
    return {
      ( relative[2][1] - relative[1][2] ) * 0.5,
      ( relative[0][2] - relative[2][0] ) * 0.5,
      ( relative[1][0] - relative[0][1] ) * 0.5
    };
  }

  const double sine = std::sin (angle);
  if( std::abs (sine) < 1.0e-8 )
  {
    Point3 axis = {
      std::sqrt (std::max (0.0, ( relative[0][0] + 1.0 ) * 0.5)),
      std::sqrt (std::max (0.0, ( relative[1][1] + 1.0 ) * 0.5)),
      std::sqrt (std::max (0.0, ( relative[2][2] + 1.0 ) * 0.5))
    };
    if( relative[2][1] - relative[1][2] < 0.0 ) axis[0] = -axis[0];
    if( relative[0][2] - relative[2][0] < 0.0 ) axis[1] = -axis[1];
    if( relative[1][0] - relative[0][1] < 0.0 ) axis[2] = -axis[2];
    const double length = vector_length (axis);
    if( length > 1.0e-12 )
    {
      for( double& value : axis ) value = value * angle / length;
    }
    return axis;
  }

  const double scale = angle / ( 2.0 * sine );
  return {
    ( relative[2][1] - relative[1][2] ) * scale,
    ( relative[0][2] - relative[2][0] ) * scale,
    ( relative[1][0] - relative[0][1] ) * scale
  };
}

Point3 flange_position (const Matrix4& world_from_flange)
{
  return {
    world_from_flange[0][3],
    world_from_flange[1][3],
    world_from_flange[2][3]
  };
}

} // namespace

Robot_Position_IK_Result Solve_Flange_Position_IK (
  const Robot_Forward_Kinematics_Model& model,
  const Robot_Kinematic_Params& params,
  const Robot_Joint_State& initial_state,
  const Point3& target_position_world,
  const Robot_Position_IK_Options& options)
{
  Robot_Position_IK_Result result;
  Solve_Clock clock (options.time_budget_ms);
  if( !model.has_flange || model.neutral_world_from_parts.empty ( ) )
  {
    result.solve_time_ms = clock.Elapsed_Milliseconds ( );
    return result;
  }

  auto input_angles = clamped_input_angles (initial_state, params);
  const double tolerance = std::max (options.position_tolerance_mm, 0.0);
  const double damping = std::max (options.damping_mm, 1.0e-6);

  result.status = Robot_IK_Status::Maximum_Iterations;
  double best_error = std::numeric_limits<double>::infinity ( );
  Robot_Joint_State best_state;
  Point3 best_position = { };

  for( std::size_t iteration = 0;
       iteration <= options.max_iterations;
       ++iteration )
  {
    const auto state = Build_Joint_State_From_Input_Angles (
      params, input_angles);
    const auto forward = Compute_Forward_Kinematics (model, state);
    if( !forward.has_flange )
    {
      result.status = Robot_IK_Status::Invalid_Model;
      result.solve_time_ms = clock.Elapsed_Milliseconds ( );
      return result;
    }

    const auto current_position = flange_position (forward.world_from_flange);
    const auto error = subtract (target_position_world, current_position);
    const double error_length = vector_length (error);
    if( error_length < best_error )
    {
      best_error = error_length;
      best_state = state;
      best_position = current_position;
      result.iterations = iteration;
    }
    if( error_length <= tolerance )
    {
      result.status = Robot_IK_Status::Converged;
      result.joint_state = state;
      result.achieved_position_world = current_position;
      result.position_error_mm = error_length;
      result.iterations = iteration;
      result.solve_time_ms = clock.Elapsed_Milliseconds ( );
      return result;
    }
    if( iteration == options.max_iterations ) break;
    if( clock.Expired ( ) )
    {
      result.status = Robot_IK_Status::Time_Budget_Exceeded;
      break;
    }

    const auto jacobian = build_position_jacobian (
      forward, params, current_position);
    Joint_Vector joint_delta = { };
    if( !solve_damped_least_squares (
          jacobian, error, damping, &joint_delta) ) break;
    apply_joint_delta (
      joint_delta, params, options.max_joint_step_deg, &input_angles);
  }

  result.joint_state = best_state;
  result.achieved_position_world = best_position;
  result.position_error_mm = best_error;
  result.solve_time_ms = clock.Elapsed_Milliseconds ( );
  return result;
}

Robot_Pose_IK_Result Solve_Flange_Pose_IK (
  const Robot_Forward_Kinematics_Model& model,
  const Robot_Kinematic_Params& params,
  const Robot_Joint_State& initial_state,
  const Matrix4& target_world_from_flange,
  const Robot_Pose_IK_Options& options)
{
  Robot_Pose_IK_Result result;
  Solve_Clock clock (options.time_budget_ms);
  if( !model.has_flange || model.neutral_world_from_parts.empty ( ) )
  {
    result.solve_time_ms = clock.Elapsed_Milliseconds ( );
    return result;
  }

  auto input_angles = clamped_input_angles (initial_state, params);
  const double position_tolerance =
    std::max (options.position_tolerance_mm, 0.0);
  const double orientation_tolerance_radians =
    std::max (options.orientation_tolerance_deg, 0.0) * PI / 180.0;
  const double orientation_weight =
    std::max (options.orientation_weight_mm, 1.0e-6);
  const double damping = std::max (options.damping_mm, 1.0e-6);
  const auto target_position = flange_position (target_world_from_flange);

  result.status = Robot_IK_Status::Maximum_Iterations;
  double best_score = std::numeric_limits<double>::infinity ( );
  for( std::size_t iteration = 0;
       iteration <= options.max_iterations;
       ++iteration )
  {
    const auto state = Build_Joint_State_From_Input_Angles (
      params, input_angles);
    const auto forward = Compute_Forward_Kinematics (model, state);
    if( !forward.has_flange )
    {
      result.status = Robot_IK_Status::Invalid_Model;
      result.solve_time_ms = clock.Elapsed_Milliseconds ( );
      return result;
    }

    const auto& current = forward.world_from_flange;
    const auto current_position = flange_position (current);
    const auto position_error = subtract (target_position, current_position);
    const auto orientation_error = orientation_error_vector (
      target_world_from_flange, current);
    const double position_length = vector_length (position_error);
    const double orientation_length = vector_length (orientation_error);
    const double score =
      position_length + orientation_weight * orientation_length;
    if( score < best_score )
    {
      best_score = score;
      result.joint_state = state;
      result.achieved_world_from_flange = current;
      result.position_error_mm = position_length;
      result.orientation_error_deg =
        orientation_length * RADIANS_TO_DEGREES;
      result.iterations = iteration;
    }
    if( position_length <= position_tolerance &&
        orientation_length <= orientation_tolerance_radians )
    {
      result.status = Robot_IK_Status::Converged;
      result.joint_state = state;
      result.achieved_world_from_flange = current;
      result.position_error_mm = position_length;
      result.orientation_error_deg =
        orientation_length * RADIANS_TO_DEGREES;
      result.iterations = iteration;
      result.solve_time_ms = clock.Elapsed_Milliseconds ( );
      return result;
    }
    if( iteration == options.max_iterations ) break;
    if( clock.Expired ( ) )
    {
      result.status = Robot_IK_Status::Time_Budget_Exceeded;
      break;
    }

    const auto jacobian = build_pose_jacobian (
      forward, params, current_position, orientation_weight);
    const Task_Vector<6> error = {
      position_error[0], position_error[1], position_error[2],
      orientation_error[0] * orientation_weight,
      orientation_error[1] * orientation_weight,
      orientation_error[2] * orientation_weight
    };
    Joint_Vector joint_delta = { };
    if( !solve_damped_least_squares (
          jacobian, error, damping, &joint_delta) ) break;
    apply_joint_delta (
      joint_delta, params, options.max_joint_step_deg, &input_angles);
  }

  result.solve_time_ms = clock.Elapsed_Milliseconds ( );
  return result;
}

} // namespace robot_model
