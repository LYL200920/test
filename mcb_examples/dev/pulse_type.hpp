
#ifndef includeguard_dev_pulse_type_hpp_includeguard
#define includeguard_dev_pulse_type_hpp_includeguard

namespace dev
{

enum struct pulse_type
{
  // no pulse, used to disable something
  disable,

  // single pulse, no direction (1 wire)
  single_pulse,

  // single pulse + direction (2 wire)
  single_pulse_direction,

  // double pulse (one for each direction, 2 wire)
  double_pulse,

  // AB phase single edge evaluation (2 wire)
  ab_phase_single_edge,

  // AB phase double edge evaluation (2 wire)
  ab_phase_double_edge,

  // AB phase quad edge evaluation (2 wire)
  ab_phase_quad_edge
};


} // namespace dev
#endif // includeguard_dev_pulse_type_hpp_includeguard
