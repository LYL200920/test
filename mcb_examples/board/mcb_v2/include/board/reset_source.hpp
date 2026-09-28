#ifndef includeguard_board_reset_source_includeguard
#define includeguard_board_reset_source_includeguard

enum struct reset_source
{
  // reset has been triggered by software
  soft_reset,

  soft_standby,
  vdet2,
  vdet1,
  watchdog_timer,
  independent_watchdog_timer,
  vdet0,

  // reset after power-on
  poweron,

  // reset has been triggered by hardware signal (reset button)
  hard_reset
};


#endif // includeguard_board_reset_source_includeguard
