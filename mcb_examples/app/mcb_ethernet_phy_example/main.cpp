/*
  MCB example application to access and control the ethernet
  PHY LSI (LAN8710).

*/

#include <cstdio>
#include <chrono>

#include <board/board.hpp>
#include <dev/eth.hpp>

int main (void)
{
  auto& debug_io = this_board::inst ().debug_usart;

  this_board::type::lan8710_t& phy = this_board::inst ().eth0_phy;
  phy.reset ();

  phy.set_auto_neg_caps_advertisement (this_board::type::lan8710_t::auto_neg_caps_t ()
	.set_selector_field (1)
	.set_cap_10_base_t ()
	.set_cap_10_base_t_full ()
	.set_cap_100_base_x ()
	.set_cap_100_base_x_full ()
	.set_cap_100_base_t4 ()
  );


  phy.set_control (this_board::type::lan8710_t::ctrl_t ()
	.set_restart_auto_neg ()
	.set_auto_neg_enable ());

  this_board::type::lan8710_t::status_t prev_phy_status;

  auto prev_time = std::chrono::system_clock::now ();

  auto event_time = prev_time;
  unsigned int event_count = 0;

  for (unsigned int main_loop_count = 0; ; ++main_loop_count)
  {
    auto cur_time = std::chrono::system_clock::now ();

    if (cur_time - event_time >= std::chrono::milliseconds (100))
    {
      // FIXME: hide this with a background timer/thread or something.
      this_board::inst ().exec ();

      event_time = cur_time;
      ++event_count;

#if 1
      auto phy_status = phy.status ();
      if (phy_status != prev_phy_status)
      {
	auto link = phy.auto_neg_caps_link_partner ();
	auto ct = phy.control ();

	std::printf ("phy status change 0x%04x -> 0x%04x: \n"
		     "   link status: %d\n"
		     "   auto neg compl: %d\n"
		     "   lp caps: 0x%04x\n"
		     "   lp caps 10baseT half:  %d\n"
		     "   lp caps 10baseT full:  %d\n"
		     "   lp caps 100baseX half: %d\n"
		     "   lp caps 100baseX full: %d\n"
		     "   lp caps 100baseT4:     %d\n"
		     "   lp caps pause:         %d\n"
		     "   lp caps asym pause:    %d\n"
		     "   lp remote fault:       %d\n"
		     "   ctrl = 0x%04x  duplex: %d  speed: %d\n"
		     "   speed indication = %d\n",
			prev_phy_status.value (), phy_status.value (),
			phy_status.link_status (),
			phy_status.auto_neg_completed (),
			link.value (),
			link.cap_10_base_t (),
			link.cap_10_base_t_full (),
			link.cap_100_base_x (),
			link.cap_100_base_x_full (),
			link.cap_100_base_t4 (),
			link.cap_pause (),
			link.cap_asymmetric_pause (),
			link.remote_fault (),
			ct.value (), ct.duplex (), ct.speed (),
			(int)phy.phy_special_ctrl_status ().speed_indication ());

	bool link_down = prev_phy_status.link_status () & !phy_status.link_status ();
	bool link_up = !prev_phy_status.link_status () && phy_status.link_status ();

	if (link_down)
	  std::printf ("link down\n");

	if (link_up)
	  std::printf ("link up\n");

	prev_phy_status = phy_status;
      }
#endif
    }

    prev_time = cur_time;

    if (!debug_io.rx_fifo_empty ())
    {
      char c;
      if (debug_io.read (&c, 1))
      {
	if (c == 'i')
	{
	  auto ctrl = phy.control ();

	  if (ctrl.isolate ())
	  {
	    printf ("un-isolating link...\n");
	    ctrl.set_isolate (false);
	  }
	  else
	  {
	    printf ("isolating link...\n");
	    ctrl.set_isolate (true);
	  }
	  phy.set_control (ctrl);
	}
	else if (c == 'n')
	{
	  printf ("restarting auto negotiation...\n");

	  phy.set_control (this_board::type::lan8710_t::ctrl_t ()
		.set_restart_auto_neg ()
		.set_auto_neg_enable ());
	}
      }
    }
  }

  return 0;
}
