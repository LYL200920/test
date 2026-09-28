/*

  DLPC3479 I2C Controller

*/

#ifndef includeguard_dev_dlpc3479_includeguard
#define includeguard_dev_dlpc3479_includeguard

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <array>

#include <dev/i2c.hpp>
#include <dev/interrupt.hpp>
#include <utils/bits.hpp>
#include <utils/vec_mat.hpp>

extern void dlpc_printf (const char* s);

template<typename T, typename... Args>
extern void dlpc_printf (const char *s, const T& value, Args... args);

namespace dev
{


template < unsigned int SlaveAddr, unsigned int Rlim_m_Ohm = 25>
class dlpc3479
{
public:

  dlpc3479 (void) = delete;

  dlpc3479 (dev::i2c_master& d) : m_i2c (&d)
  {
  }

  ~dlpc3479 (void) { }

  bool is_power_on (void) const { return m_is_power_on; };
  void set_power_on (bool val) { m_is_power_on = val; };

  void init (void)
  {
    // we set to display - test pattern generator Mode
    if (operating_mode () != op_mode::display_test_pat_gen)
      set_operating_mode (op_mode::display_test_pat_gen);

    while (operating_mode () != op_mode::display_test_pat_gen) { }

    // set display image orientation disable .
    if (image_orientation () != img_flip::disable)
      set_image_orientation (img_flip::disable);

    // set led out put control to manual mode, disable caic algo.
    if (led_control_mode () != led_ctrl_mode::manual)
      set_led_control_mode (led_ctrl_mode::manual);

    // set rgb led enable.
    enable_led (led::all);

    // set current max.
    set_rgb_led_current_max ({6000, 6000, 6000});

    // set led init current to 3A
    set_rgb_led_current ({3000, 3000, 3000});

    // we set to light control – internal pattern streaming Mode
    if (operating_mode () != op_mode::light_ctrl_inter_pat_str)
      set_operating_mode (op_mode::light_ctrl_inter_pat_str);

    while (operating_mode () != op_mode::light_ctrl_inter_pat_str) { }

    // set trigger in config
    //  - disable
    // we don't use trigger in mode, only use free running mode
    // if set "trigger_in_mcu", we can into it's interrupt func,
    // and then send I2C command to DLPC to display pre-stored
    // patterns
    if (trig_in_config () != trig_in_cfg::disable)
      set_trig_in_config (trig_in_cfg::disable);

    // set pattern ready config
    //  - enable
    //  - no inverted
    if (pattern_ready_config () != patt_rdy_cfg::enable_no_invert)
      set_pattern_ready_config (patt_rdy_cfg::enable_invert);

    // set trigger out 1 config
    // set trigger out 2 config
    //  - enable
    //  - no inverted
    set_trig_out_config (0, trig_out_cfg::enable_no_invert);
    set_trig_out_config (1, trig_out_cfg::enable_no_invert);

    // set internal pattern to stop
    set_internal_pattern_ctrl (inter_patt_ctrl::stop);

    m_status = status_t::ready;
  }

  // -------------------------------------------------------------------
  // Common Useful Api.
  enum status_t
  {
    off,
    ready,
    on,
  };

  status_t status (void) const
  {
    return m_status;
  }

  bool set_off (void)
  {
    if (set_ready ())
    {
      m_status = status_t::off;
      return true;
    }

    return false;
  }

  bool set_ready (void)
  {
    if (m_status == status_t::ready)
      return true;

    // we set to light control – internal pattern streaming Mode
    bool succeed = set_operating_mode (op_mode::light_ctrl_inter_pat_str);

    if (succeed)
      m_status = status_t::ready;

    return succeed;
  }

  bool set_on (void)
  {
    if (m_status == status_t::on)
      return true;

    bool succeed = true;

    // we must set test pattern to solid_field before
    // display splash. otherwize, splash will only
    // flash by, not stay for a long time
    succeed = display_test_pattern ();

    if (succeed)
      succeed = display_splash_screen ();

    if (succeed)
      m_status = status_t::on;

    return succeed;
  }

  bool trigger_once (void)
  {
    if (!set_ready ())
      return false;

    return set_internal_pattern_ctrl (inter_patt_ctrl::start);

    return false;
  }

  bool trigger_inf (void)
  {
    if (!internal_pattern_ready ())
      return set_internal_pattern_ctrl (inter_patt_ctrl::start, 0xFF);

    return false;
  }

  bool trigger_stop (void)
  {
    if (!internal_pattern_ready ())
      return set_internal_pattern_ctrl (inter_patt_ctrl::stop);

    return false;
  }

  bool display_test_pattern (void)
  {
    set_image_freeze (false);

    // we set to display - test pattern generator mode
    if (operating_mode () != op_mode::display_test_pat_gen)
      set_operating_mode (op_mode::display_test_pat_gen);

    set_input_image_size (1920, 1080);

    enable_led (led::all);

    return set_test_pattern_select (false,
				    left_patt::solid_field,
				    color::white);

  }

  bool display_splash_screen (void)
  {
    set_image_freeze (false);

    // we set to display - splash screen mode
    if (operating_mode () != op_mode::display_splash_screen_mode)
      set_operating_mode (op_mode::display_splash_screen_mode);

    set_input_image_size (1920, 1080);
    set_display_size (1920, 1080);

    enable_led (led::all);

    set_splash_screen_select (1);

    // we should send this command some times,
    unsigned int send_count = 5;
    while (!splash_screen_execute () && send_count--) { }

    // execute error
    if (send_count == 0)
      return false;

    return true;
  }

  // -------------------------------------------------------------------
  // Administrative Commands

  uint8_t device_id (void) { return read_reg (0xD4).value_or (0); }

  // -------------------------------------------------------------------
  // General Operation Commands

  // operating mode
  enum op_mode : uint8_t
  {
    display_ext_video          = 0x00,
    display_test_pat_gen       = 0x01,
    display_splash_screen_mode = 0x02,
    light_ctrl_ext_pat_str     = 0x03,
    light_ctrl_inter_pat_str   = 0x04,
    light_ctrl_splash_pat      = 0x05,
    // reserve
    stand_by = 0xFF,
  };

  op_mode operating_mode (void)           { return (op_mode)read_reg (0x06).value_or (0xFF); }
  bool set_operating_mode (op_mode value) { return write_reg (0x05, value); }

  // test pattern select
  enum left_patt : uint8_t
  {
    solid_field                  = 0x00,
    fixed_step_horizontal_ramp   = 0x01,
    fixed_step_vertical_ramp     = 0x02,
    horizontal_lines             = 0x03,
    diagonal_lines               = 0x04,
    vertical_lines               = 0x05,
    horizontal_and_vertical_grid = 0x06,
    checkerboard                 = 0x07,
    color_bars                   = 0x08,
    // reserve
  };

  enum color : uint8_t
  {
    black   = 0x00,
    red     = 0x01,
    green   = 0x02,
    blue    = 0x03,
    cyan    = 0x04,
    magenta = 0x05,
    yellow  = 0x06,
    white   = 0x07,
    // reserve
  };

  bool test_pattern_select (bool& border_enable,
			    left_patt& l_patt,
			    color& foreground,
			    color& background = color::black)
  {
    uint8_t rd_data[6];

    if (!read_reg (0x0C, rd_data))
      return false;

    border_enable = rd_data[0] & (1 << 7);
    l_patt        = left_patt (rd_data[0] & 0x0F);
    foreground    = color ((rd_data[1] & 0xF0) >> 4);

    if (l_patt >= horizontal_lines && l_patt < color_bars)
      background = color ((rd_data[1] & 0x0F));
    else
      background = color::black;

    return true;
  }

  bool set_test_pattern_select (bool border_enable,
				left_patt l_patt,
				color foreground,
				color background = color::black)
  {
    uint8_t TPG_pattern_select = 0
			| (border_enable << 7)
			| l_patt;

    if (l_patt < horizontal_lines || l_patt == color_bars)
      background = color::black;

    uint8_t fore_back_color = 0
			| (foreground << 4)
			| background;

    // FIXME: we only use Solid field, so we
    //        don't need byte 3-6.
    //        if we use other command, please
    //        read programe guide.
    return write_reg (0x0B, { TPG_pattern_select,
			      fore_back_color,
			      uint8_t (0x00), uint8_t(0x00),
			      uint8_t (0x00), uint8_t(0x00)});
  }

  uint8_t splash_screen_select (void)           { return read_reg (0x0E).value_or (0xFF); }
  bool set_splash_screen_select (uint8_t value) { return write_reg (0x0D, value); }

  bool splash_screen_execute (void) { return write_reg (0x35); }

  // Set Display Size
  bool display_size (uint16_t& pixels_per_line, uint16_t& lines_per_frame)
  {
    uint8_t rd_data[4];

    if (!read_reg (0x13, rd_data))
      return false;

    pixels_per_line = uint16_t (rd_data[1] << 8) | rd_data[0];
    lines_per_frame = uint16_t (rd_data[3] << 8) | rd_data[2];

    return true;
  }
  bool set_display_size (uint16_t pixels_per_line, uint16_t lines_per_frame)
  {
    return write_reg (0x12,
			{ uint8_t (pixels_per_line & 0x00ff), uint8_t (pixels_per_line >> 8),
			  uint8_t (lines_per_frame & 0x00ff), uint8_t (lines_per_frame >> 8)
			}
		     );
  }

  // Display Image Orientation
  enum struct img_flip : uint8_t
  {
    disable         = 0b000,
    long_axis       = 0b010,
    short_axis      = 0b100,
    short_long_axis = 0b110,
    unknown         = 0xff
  };

  img_flip image_orientation (void)           { return (img_flip)read_reg (0x15).value_or (0xff); }
  bool set_image_orientation (img_flip value) { return write_reg (0x14, (uint8_t)value); }

  // Set Image Freeze
  bool image_freeze (void)           { return read_reg (0x1B).value_or (0); }
  bool set_image_freeze (bool value) { return write_reg (0x1A, value); }

  // Set Input Image Size
  bool input_image_size (uint16_t& pixels_per_line, uint16_t& lines_per_frame)
  {
    uint8_t rd_data[4];

    if (!read_reg (0x2F, rd_data))
      return false;

    pixels_per_line = uint16_t (rd_data[1] << 8) | rd_data[0];
    lines_per_frame = uint16_t (rd_data[3] << 8) | rd_data[2];

    return true;
  }
  bool set_input_image_size (uint16_t pixels_per_line, uint16_t lines_per_frame)
  {
    return write_reg (0x2E,
			{ uint8_t (pixels_per_line & 0x00ff), uint8_t (pixels_per_line >> 8),
			  uint8_t (lines_per_frame & 0x00ff), uint8_t (lines_per_frame >> 8)
			}
		     );
  }

  // -------------------------------------------------------------------
  // Illumination Control Commands

  // led output control method
  enum led_ctrl_mode : uint8_t
  {
    manual = 0x00, // disable CAIC algo
    caic   = 0x01, // enable  CAIC algo
    // reserve
    unknow_led_ctrl_mode = 0xff
  };

  led_ctrl_mode led_control_mode (void) { return (led_ctrl_mode)read_reg (0x51).value_or (0xff); }
  bool set_led_control_mode (led_ctrl_mode value) { return write_reg (0x50, value); }

  // led rgb select
  enum struct led : uint8_t
  {
    red   = 1 << 0,
    green = 1 << 1,
    blue  = 1 << 2,

    red_green  = red | green,
    red_blue   = red | blue,
    green_blue = red | blue,

    all = red | green | blue,

    unknown = 0xff
  };

  // led rgb enable status
  led led_enable_st (void)    { return (led)read_reg (0x53).value_or (0xff); }
  bool enable_led (led value) { return write_reg (0x52, (uint8_t)value); }


  utils::vec3<uint16_t> led_current_from_IDAC (const utils::vec3<uint16_t>& idac_ma)
  {
    utils::vec3<float> val_f (idac_ma.r, idac_ma.g, idac_ma. b);

    //                     IDAC_10bits + 1        150 mV
    // led current (mA) = —————————————————— * ————————————— * 1000
    //                         1024              Rlim (mΩ)
    val_f = ((val_f + 1) / 1024 * 150 / Rlim_m_Ohm) * 1000;

    return {val_f.r, val_f.g, val_f.b};
  }

  utils::vec3<uint16_t> led_current_to_IDAC (const utils::vec3<uint16_t>& curr_ma)
  {
    utils::vec3<float> val_f (curr_ma.r, curr_ma.g, curr_ma. b);

    //                 led current * 1024 * Rlim (mΩ)
    // IDAC_10bits = ————————————————————————————————— - 1
    //                       150 mV * 1000
    val_f = val_f * 1024 * Rlim_m_Ohm / (150 * 1000) - 1;

    return {val_f.r, val_f.g, val_f.b};
  }

  bool set_rgb_led_current (const utils::vec3<uint16_t>& rgb_cur_ma)
  {
    auto idac_val = led_current_to_IDAC (rgb_cur_ma);

    return write_reg (0x54,
			{ uint8_t (idac_val.r & 0x00ff), uint8_t (idac_val.r >> 8),
			  uint8_t (idac_val.g & 0x00ff), uint8_t (idac_val.g >> 8),
			  uint8_t (idac_val.b & 0x00ff), uint8_t (idac_val.b >> 8) }
		     );
  }

  utils::vec3<uint16_t> rgb_led_current_ma ()
  {
    uint8_t rd_data[6];

    if (!read_reg (0x55, rd_data))
      return false;

    uint16_t r_cur = uint16_t (rd_data[1] << 8) | rd_data[0];
    uint16_t g_cur = uint16_t (rd_data[3] << 8) | rd_data[2];
    uint16_t b_cur = uint16_t (rd_data[5] << 8) | rd_data[4];

    return led_current_from_IDAC ({r_cur, g_cur, b_cur});
  }

  bool set_rgb_led_current_max (const utils::vec3<uint16_t>& rgb_cur_m)
  {
    auto idac_val = led_current_to_IDAC (rgb_cur_m);

    return write_reg (0x5C,
			{ uint8_t (idac_val.r & 0x00ff), uint8_t (idac_val.r >> 8),
			  uint8_t (idac_val.g & 0x00ff), uint8_t (idac_val.g >> 8),
			  uint8_t (idac_val.b & 0x00ff), uint8_t (idac_val.b >> 8) }
		     );
  }

  utils::vec3<uint16_t> rgb_led_current_max_ma ()
  {
    uint8_t rd_data[6];

    if (!read_reg (0x5D, rd_data))
      return false;

    uint16_t r_cur = uint16_t (rd_data[1] << 8) | rd_data[0];
    uint16_t g_cur = uint16_t (rd_data[3] << 8) | rd_data[2];
    uint16_t b_cur = uint16_t (rd_data[5] << 8) | rd_data[4];

    return led_current_from_IDAC ({r_cur, g_cur, b_cur});
  }

  // if we use caic ctrl mode
  utils::vec3<uint16_t> rgb_led_caic_current_ma ()
  {
    uint8_t rd_data[6];

    if (!read_reg (0x5F, rd_data))
      return false;

    uint16_t r_cur = uint16_t (rd_data[1] << 8) | rd_data[0];
    uint16_t g_cur = uint16_t (rd_data[3] << 8) | rd_data[2];
    uint16_t b_cur = uint16_t (rd_data[5] << 8) | rd_data[4];

    return led_current_from_IDAC ({r_cur, g_cur, b_cur});
  }

  // -------------------------------------------------------------------
  // Light Control Commands

  enum struct trig_in_cfg : uint8_t
  {
    disable     = 0b00,
    enable_low  = 0b01,
    enable_high = 0b11,
    unknown = 0xff
  };

  trig_in_cfg trig_in_config (void)         { return (trig_in_cfg)read_reg (0x91).value_or (0xff); }
  bool set_trig_in_config (trig_in_cfg cfg) { return write_reg (0x90, (uint8_t)cfg); }


  enum struct trig_out_cfg : uint8_t
  {
    disable          = 0b00,
    enable_no_invert = 0b01,
    enable_invert    = 0b11,
  };

  bool set_trig_out_config (uint8_t trig_select, trig_out_cfg cfg, int16_t trig_delay_us = 0)
  {
    // FIXME: trig_select only have 0 and 1.
    //         - 0 : trigger out 1
    //         - 1 : trigger out 2

    uint8_t trig_config =   trig_select | ((uint8_t)cfg << 1);

    // delay range:
    //   - trigger out 1: [0, Pattern Period]
    //   - trigger out 2: [ -Pre-Illumination Dark Time, Pattern Period]

    // FIXME: trigger out 1 can't set negative delay.
    // FIXME: trigger out 2 can   set negative delay.
    //        but we don't deal it. beacuse of uint8_t.

    return write_reg (0x92, { trig_config,
			      uint8_t (trig_delay_us & 0x00ff),
			      uint8_t (0x00), uint8_t(0x00),
			      uint8_t (trig_delay_us >> 8) });
  }

  bool trigger_out_config (uint8_t trig_select, trig_out_cfg& cfg, int16_t& trig_delay_us)
  {
    // FIXME: trig_select only have 0 and 1.
    //         - 0 : trigger out 1
    //         - 1 : trigger out 2

    uint8_t rd_data[5];

    if (!read_reg (0x93, trig_select, rd_data))
      return false;

    cfg = trig_out_cfg(rd_data[0] >> 1);

    // trigger out 2 supports negative values,
    // meaning the trigger can be sent in advance.
    trig_delay_us = int16_t(rd_data[5] << 8) | rd_data[2];

    return true;
  }

  enum patt_rdy_cfg : uint8_t
  {
    disable          = 0b00,
    enable_no_invert = 0b01,
    enable_invert    = 0b11,
    unknow_patt_rdy_cfg = 0xff
  };

  patt_rdy_cfg pattern_ready_config (void) { return (patt_rdy_cfg)read_reg (0x95).value_or (0xff); }
  bool set_pattern_ready_config (patt_rdy_cfg cfg) { return write_reg (0x94, (uint8_t)cfg); }


  struct pattern_order_table_entry_t
  {
    // this only used for write pattern_order_table_entry.
    // 0h: Continue
    // 1h: Start
    // 2h: Reload from flash
    // 3h–FFh: Reserved
    enum control_t : uint8_t
    {
      continue_mode     = 0x00,
      start             = 0x01,
      reload_form_flash = 0x02,
      reserved,
    } write_control;

    // the following used for write and read
    uint8_t pattern_set_entry_index;
    uint8_t pattern_set_index;
    uint8_t number_of_patterns_to_display;
    led     illumination_select;
    uint32_t pattern_invert_lword;
    uint32_t pattern_invert_mword;
    uint32_t illumination_time;
    uint32_t pre_illumination_dark_time;
    uint32_t post_illumination_dark_time;

    pattern_order_table_entry_t (void) { }

    pattern_order_table_entry_t (const uint8_t data[24])
    {
      pattern_set_index             = data[0];
      number_of_patterns_to_display = data[1];
      illumination_select           = (led)data[2];
      pattern_invert_lword          = *(uint32_t*)(&data[3]);
      pattern_invert_mword          = *(uint32_t*)(&data[7]);
      illumination_time             = *(uint32_t*)(&data[11]);
      pre_illumination_dark_time    = *(uint32_t*)(&data[15]);
      post_illumination_dark_time   = *(uint32_t*)(&data[19]);
      pattern_set_entry_index       = data[23];
    }

    void serialize_set_array (uint8_t data[25]) const
    {
      data[0] = write_control;
      data[1] = pattern_set_index;
      data[2] = number_of_patterns_to_display;
      data[3] = (uint8_t)illumination_select;
      std::memcpy (&data[4], &pattern_invert_lword, sizeof (pattern_invert_lword));
      std::memcpy (&data[8], &pattern_invert_mword, sizeof (pattern_invert_mword));
      std::memcpy (&data[12], &illumination_time, sizeof (illumination_time));
      std::memcpy (&data[16], &pre_illumination_dark_time, sizeof (pre_illumination_dark_time));
      std::memcpy (&data[20], &post_illumination_dark_time, sizeof (post_illumination_dark_time));
      data[24] = pattern_set_entry_index;
    }

  };

  pattern_order_table_entry_t pattern_order_table_entry (uint8_t pattern_order_table_entry_index)
  {
    uint8_t rd_data[24];

    if (!read_reg (0x99, pattern_order_table_entry_index, rd_data))
      return { };

    return pattern_order_table_entry_t (rd_data);
  }
  bool set_pattern_order_table_entry (const pattern_order_table_entry_t& entry)
  {
    uint8_t wd_data[25];
    entry.serialize_set_array (wd_data);

    return write_reg (0x98, wd_data);
  }

  bool validate_exposure_time ( uint32_t& exposure_time,
				uint32_t& pre_exposure_time,
				uint32_t& post_exposure_time )
  {
    uint8_t wr_data[6];
    // pattern_mode
    //   external = 0x00,
    //   internal = 0x01,
    //   splash   = 0x02,
    //   reserved,
    wr_data[0] = 0x01;
    // bit_depth
    //   mono_1_bit = 0x00,
    //   rgb_1_bit  = 0x01,
    //   mono_8_bit = 0x02,
    //   rgb_8_bit  = 0x03,
    //   mono_4_bit = 0x04, // supported only in Internal Pattern Mode
    //   mono_5_bit = 0x05, // supported only in Internal Pattern Mode
    //   mono_6_bit = 0x06, // supported only in Internal Pattern Mode
    //   reserved,
    wr_data[1] = 0x02;
    // requested exposure time
    std::memcpy (&wr_data[2], &exposure_time, sizeof (exposure_time));

    uint8_t rd_data[13];

    if (!read_reg (0x9D, wr_data, rd_data))
    {
      exposure_time = 0;
      pre_exposure_time = 0;
      post_exposure_time = 0;
      return false;
    }

    bool exposure_time_support = rd_data[0] & 1;
    //bool zero_dark_time_support = rd_data[0] & (1 << 1);

    exposure_time      = *(uint32_t*)(&rd_data[1]);
    pre_exposure_time  = *(uint32_t*)(&rd_data[5]);
    post_exposure_time = *(uint32_t*)(&rd_data[9]);

    return exposure_time_support;
  }

  enum inter_patt_ctrl : uint8_t
  {
    start  = 0x00,
    stop   = 0x01,
    pause  = 0x02,
    step   = 0x03,
    resume = 0x04,
    reset  = 0x05,
  };

  // repeat_count is used only when start is selected.
  // if repea_count = 0, trigger once!
  // if repea_count == 0xFF, it will repeat indefinitely
  bool set_internal_pattern_ctrl (inter_patt_ctrl ctrl, uint8_t repeat_count = 0)
  {
    return write_reg (0x9E, { (uint8_t)ctrl, (uint8_t)repeat_count } );
  }

  // FIXME: if the other byte is useful
  //          - rd_data[0] Pattern Ready Status:
  //                       Status is true  while internal pattern start.
  //                       Status is false while internal pattern stop.
  //          - rd_data[1] Number of Pattern Order Table Entries
  //          - rd_data[2] Current Pattern Order Table Entry Index
  //          - rd_data[3] Current Pattern Set Index
  //          - rd_data[4] Number of Patterns in the current Pattern Set
  //          - rd_data[5] Number of Patterns displayed from current Pattern Set
  //          - rd_data[6] Next Pattern Set Index
  bool internal_pattern_ready ()
  {
    uint8_t rd_data[7];

    if (!read_reg (0x9F, rd_data))
      return false;

    uint8_t patt_ready_st = rd_data[0];

    return patt_ready_st;
  }

private:
  dev::i2c_master* m_i2c;
  status_t m_status;
  bool m_is_power_on;

  bool write_reg (uint8_t reg_addr)
  {
    if (!is_power_on ())
      return false;

    uint8_t data[] = { reg_addr };
    return m_i2c->send (SlaveAddr, data, 1);
  }

  bool write_reg (uint8_t reg_addr, uint8_t value)
  {
    if (!is_power_on ())
      return false;

    uint8_t data[] = { reg_addr, value };
    return m_i2c->send (SlaveAddr, data, 2);
  }

  template < size_t N >
  bool write_reg (uint8_t reg_addr, const uint8_t (&value)[N])
  {
    if (!is_power_on ())
      return false;

    std::array<uint8_t, N + 1> data;
    data[0] = reg_addr;
    std::memcpy (&data[1], &value[0], N);

    return m_i2c->send (SlaveAddr, data.data (), data.size ());
  }

  std::optional<uint8_t> read_reg (uint8_t reg_addr)
  {
    if (!is_power_on ())
      return 0;

    uint8_t wr_data[] = { reg_addr };
    uint8_t rd_data[] = { 0 };

    if (m_i2c->send_recv (SlaveAddr, wr_data, 1,
				     rd_data, 1))
      return rd_data[0];
    else
      return { };
  }
  std::optional<uint8_t> read_reg (uint8_t reg_addr, uint8_t wr_byte)
  {
    if (!is_power_on ())
      return 0;

    uint8_t wr_data[] = { reg_addr, wr_byte};
    uint8_t rd_data[] = { 0 };

    if (m_i2c->send_recv (SlaveAddr, wr_data, 2,
				     rd_data, 1))
      return rd_data[0];
    else
      return { };
  }

  template < size_t N >
  bool read_reg (uint8_t reg_addr, uint8_t (&rd_data)[N])
  {
    if (!is_power_on ())
      return false;

    uint8_t wr_data[] = { reg_addr };

    if (m_i2c->send_recv (SlaveAddr, wr_data, 1,
				     rd_data, N))
    {
      return true;
    }
    else
      return false;
  }

  template < size_t N >
  bool read_reg (uint8_t reg_addr, uint8_t wr_byte, uint8_t (&rd_data)[N])
  {
    if (!is_power_on ())
      return false;

    uint8_t wr_data[] = { reg_addr, wr_byte};

    if (m_i2c->send_recv (SlaveAddr, wr_data, 2,
				     rd_data, N))
    {
      return true;
    }
    else
      return false;
  }

  template < size_t WR_N, size_t RD_N>
  bool read_reg (uint8_t reg_addr, const uint8_t(&wr_byte)[WR_N], uint8_t (&rd_data)[RD_N])
  {
    if (!is_power_on ())
      return false;

    uint8_t wr_data[WR_N + 1];

    wr_data[0] = reg_addr;

    std::copy (wr_byte, wr_byte + WR_N, &wr_data[1]);

    if (m_i2c->send_recv (SlaveAddr, wr_data, WR_N + 1,
				     rd_data, RD_N))
    {
      return true;
    }
    else
      return false;
  }

}; // class dlpc3479

} // namespace dev
#endif // includeguard_dev_dlpc3479_includeguard