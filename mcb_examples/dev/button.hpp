/*

a generic hardware button that is connected to some input port.

*/

#ifndef includeguard_dev_button_includeguard
#define includeguard_dev_button_includeguard

namespace dev
{

template <typename InputPortFn> class button : private InputPortFn
{
public:
  button (InputPortFn d = { }) : InputPortFn (d)
  {
    m_cur_value = m_prev_value = read_port ();
  }

  void exec (void)
  {
    m_prev_value = m_cur_value;
    m_cur_value = read_port ();
  }

  bool is_on (void) const { return m_cur_value; }
  bool is_off (void) const { return !m_cur_value; }
  bool is_rising_edge (void) const { return m_cur_value == true && m_prev_value == false; }
  bool is_falling_edge (void) const { return m_cur_value == false && m_prev_value == true; }
  bool is_any_edge (void) const { return m_cur_value != m_prev_value; }

private:
  bool read_port (void) const { return (*(InputPortFn*)this)().read (); }

  bool m_cur_value;
  bool m_prev_value;
};

template <typename PortFn> inline button<PortFn> make_button (PortFn f)
{
  return button<PortFn> (f);
}

} // namespace dev
#endif // includeguard_dev_button_includeguard
