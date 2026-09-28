#ifndef includeguard_mcbv2_board_hpp_includeguard
#define includeguard_mcbv2_board_hpp_includeguard
#ifdef __cplusplus

#include <board/board_clk.hpp>
#include <board/reset_source.hpp>

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

#include <dev/renesas/rx63_interrupt.hpp>
#include <dev/renesas/rx63_fcu.hpp>

#include <dev/renesas/rx_scic.hpp>

#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
      || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

// the RX64 and RX71 peripherals are compatible.

#include "dev/renesas/rx64_interrupt.hpp" // use local include
#include <dev/renesas/rx64_fcu.hpp>

#include <dev/renesas/rx_scig.hpp>
#include <dev/renesas/rx_scifa.hpp>
#include <dev/renesas/rx_edmaca.hpp>
#include <dev/renesas/rx_cac.hpp>

#else
  #error board variant not selected
#endif

#include <dev/renesas/rx_lvda.hpp>
#include <dev/renesas/rx_sci_usart.hpp>
#include <dev/renesas/rx_scif_usart.hpp>
#include <dev/renesas/rx_sci_i2c.hpp>
#include <dev/renesas/rx_sci_spi.hpp>

// the TPUa units are the same on RX63, RX64, RX71
#include <dev/renesas/rx_tpua.hpp>

// the GPTa units are the same on RX64, RX71
#include <dev/renesas/rx_gpta.hpp>

// the etherc units are the same on RX63, RX64, RX71
#include <dev/renesas/rx_edmac.hpp>
#include <dev/renesas/rx_etherc.hpp>
#include <dev/renesas/rx_etherc_mdio_sta.hpp>
#include <dev/renesas/rx_etherc_eth.hpp>

// the CRC unit is the same on RX63, RX64, RX71
#include <dev/renesas/rx63_crc.hpp>

// the CMT that we use as a system timer is register compatible on
// RX63, RX64 and RX71
#include <dev/renesas/rx63_cmt_timer.hpp>
#include <dev/renesas/rx_cmt.hpp>

// module stop registers
#include <dev/renesas/rx_module_stop_regs.hpp>

// IO ports
#include <dev/renesas/rx_gpio.hpp>

// DTCa is same on RX63, RX64, RX71
#include <dev/renesas/rx_dtca.hpp>

// DMACA is basically the same on RX63, RX64, RX71.
// on RX64 and RX71 it as 8 channels instead of 4.
#include <dev/renesas/rx_dmaca.hpp>

#include <dev/pcd4641.hpp>
#include <dev/mcx51x.hpp>

// the LAN8720 is register compatible with LAN8710, it's just the RMII-only
// version of the same device.
#include <dev/lan8710.hpp>

#include <dev/lan9250.hpp>
#include <dev/lan9250_eth.hpp>

#include <dev/dac124s085.hpp>

#ifdef MCB_USE_LEDS
  #include "dev/led_outputs.hpp"
#endif

#include "dev/digital_io.hpp"
#include "dev/trigger_io.hpp"
#include "dev/dipswitch_inputs.hpp"
#include "dev/mcx_axis_io.hpp"
#include "dev/pcd_axis_io.hpp"
#include "dev/emg_stop_inputs.hpp"

#include <algorithm>

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
  namespace rx_interrupt = dev::rx63_interrupt;
#else
  namespace rx_interrupt = dev::rx64_interrupt;
#endif

class mcb_v2_board : public mcb_v2_board_clk
{
private:
  mcb_v2_board (void);
  ~mcb_v2_board (void);

  static mcb_v2_board g_inst;

  struct reset_hw_init_t { reset_hw_init_t (mcb_v2_board& brd); };
  struct release_peripheral_reset_t { release_peripheral_reset_t (mcb_v2_board& brd); };
  struct set_system_timer_initialized_t { set_system_timer_initialized_t (void); };
  struct init_dac_default_values_t { init_dac_default_values_t (mcb_v2_board& brd); };

  struct devices_begin_t { };
  struct devices_end_t { };

  struct board_info_eth0_addr { const std::array<uint8_t, 6> operator () (void); };
  struct board_info_eth1_addr { const std::array<uint8_t, 6> operator () (void); };
  struct board_info_eth2_addr { const std::array<uint8_t, 6> operator () (void); };

  static void set_standby_supply_enable (bool val);

  void transition_to_standby_mode_0 (void);
  void transition_to_standby_mode_1 (void);
  static void enter_deep_standby_mode (void);

  std::chrono::high_resolution_clock::time_point m_last_exec_time;

  #if defined (MCB_USE_TPU)
  std::bitset<6> m_tpu_en_bits;
  #endif

  #if defined (MCB_USE_GPT)
  std::bitset<4> m_gpt_en_bits;
  #endif

  #if defined (MCB_USE_DMACA)
  std::bitset<8> m_dmaca_en_bits;
  #endif

  #if defined (MCB_USE_MCX514) || defined (MCB_USE_MCX512)
  uint32_t m_mcx_axis_outputs_cache;
  #endif

public:
  static void set_battery_vtest_enable (bool val);
  static void set_battery_ltest_enable (bool val);

  // a read of the MDMONR register takes 3 iclock cycles
  // see also specializations for Count = 1 and Count = 0 below outside
  // the class.
  template <unsigned int Count> static void cpu_wait_3_iclocks (void)
  {
    cpu_wait_3_iclocks<1> ();
    cpu_wait_3_iclocks<Count - 1> ();
  }

  static constexpr mcb_v2_board& inst (void) { return g_inst; }

  using board_id_t = std::array<uint8_t, 32>;
  board_id_t board_id (void) const;

  // soft-reset and transfer CPU excution control to the specified address.
  void reset_to_func (void (*func)(void));

  // soft-reset to bootmode.
  // this simply jumps to the user boot rom and might not always work.
  void reset_to_bootmode (void)
  {
    // user bootloader ROM end address is the same for RX63, RX64 and RX71.
    reset_to_func ([] { auto f = (void(*)(void)) (*(volatile uint32_t*)0xFF7FFFFC); f (); });
  }

  // hard reset.
  // this triggers a hardware reset.  the mode after reset is the mode that
  // is selected by the DIP switch (boot mode or normal mode).
  void reset (void);

  // run periodic tasks of the board
  void exec (void);

  // does some board-specific things like checking for a magic user input
  // sequence and reset the configuration partition to clear all configuration
  // data.
  void maybe_reset_config_to_factory_default (void);

  // reset peripherals such as MCX, PCD, etc.
  static void set_peripheral_reset (bool val);

  static enum reset_source reset_source (void);
  static int reset_counter (void);
  static void set_reset_counter (int val);

  // CMT timers are used for system internal purposes.
  // CMT0 - free running system timer (pclkb / 8)
  // CMT1 -
  // CMT2 - LED DTC PWM (pclkb / 32)
  // CMT3 - PCD DTC axis IO sampler (pclkb / 32, for encoder)

  typedef dev::rx_cmt::hw_chn_inst <0x0008'8010, 2, pclkb_hz,
	rx_interrupt::line<rx_interrupt::cmt2_cmi>,
	dev::rx_mstpcra_bit<14>,
	dev::timer::no_user_trigger_callback
	> cmt2_t;
  struct cmt2_inst { constexpr cmt2_t& operator () (void) const { return inst ().cmt2; } };


  #if defined (MCB_USE_TPU)
  auto& tpu_en_bits (void) { return m_tpu_en_bits; }

  // on RX63 there are two TPUs but the interrupt numbers of the second
  // TPU unit overlap with the MTU interrupt numbers.
  // so for now just use the first unit only.

  template <unsigned int TpuChannelNumber>
  struct tpu_module_enable_func
  {
    // remember the individual enable bits for each tpu channel in the board
    // instance and enable or disable the tpu module based on that.
    void operator () (bool val)
    {
      auto& en_bits = mcb_v2_board::inst ().tpu_en_bits ();
      en_bits[TpuChannelNumber] = val;

      dev::rx_mstpcra_bit<13> () (en_bits.any ());
    }
  };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088110, 0, 0x00088100, 0x00088101, 0x00088108, pclkb_hz,
	rx_interrupt::line<rx_interrupt::tpu0_tgia>,
	rx_interrupt::line<rx_interrupt::tpu0_tgib>,
	rx_interrupt::line<rx_interrupt::tpu0_tgic>,
	rx_interrupt::line<rx_interrupt::tpu0_tgid>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<0> > tpu0_t;
  struct tpu0_inst { constexpr tpu0_t& operator () (void) const { return inst ().tpu0; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088120, 1, 0x00088100, 0x00088101, 0x00088109, pclkb_hz,
	rx_interrupt::line<rx_interrupt::tpu1_tgia>,
	rx_interrupt::line<rx_interrupt::tpu1_tgib>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<1> > tpu1_t;
  struct tpu1_inst { constexpr tpu1_t& operator () (void) const { return inst ().tpu1; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088130, 2, 0x00088100, 0x00088101, 0x0008810A, pclkb_hz,
	rx_interrupt::line<rx_interrupt::tpu2_tgia>,
	rx_interrupt::line<rx_interrupt::tpu2_tgib>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<2> > tpu2_t;
  struct tpu2_inst { constexpr tpu2_t& operator () (void) const { return inst ().tpu2; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088140, 3, 0x00088100, 0x00088101, 0x0008810B, pclkb_hz,
	rx_interrupt::line<rx_interrupt::tpu3_tgia>,
	rx_interrupt::line<rx_interrupt::tpu3_tgib>,
	rx_interrupt::line<rx_interrupt::tpu3_tgic>,
	rx_interrupt::line<rx_interrupt::tpu3_tgid>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<3> > tpu3_t;
  struct tpu3_inst { constexpr tpu3_t& operator () (void) const { return inst ().tpu3; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088150, 4, 0x00088100, 0x00088101, 0x0008810C, pclkb_hz,
	rx_interrupt::line<rx_interrupt::tpu4_tgia>,
	rx_interrupt::line<rx_interrupt::tpu4_tgib>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<4> > tpu4_t;
  struct tpu4_inst { constexpr tpu4_t& operator () (void) const { return inst ().tpu4; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088160, 5, 0x00088100, 0x00088101, 0x0008810D, pclkb_hz,
	  rx_interrupt::line<rx_interrupt::tpu5_tgia>,
	  rx_interrupt::line<rx_interrupt::tpu5_tgib>,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	tpu_module_enable_func<5> > tpu5_t;
  struct tpu5_inst { constexpr tpu5_t& operator () (void) const { return inst ().tpu5; } };

  #endif // MCB_USE_TPU


  #if MCB_USE_GPT
  auto& gpt_en_bits (void) { return m_gpt_en_bits; }

  template <unsigned int GptChannelNumber>
  struct gpt_module_enable_func
  {
    // remember the individual enable bits for each tpu channel in the board
    // instance and enable or disable the tpu module based on that.
    void operator () (bool val)
    {
      auto& en_bits = mcb_v2_board::inst ().gpt_en_bits ();
      en_bits[GptChannelNumber] = val;

      dev::rx_mstpcra_bit<7> () (en_bits.any ());
    }
  };

  typedef dev::rx_gpta::hw_chn_inst <
	0x000C2100, 0, 0x000C2000, pclka_hz,
	rx_interrupt::line<rx_interrupt::gpt0_gtcia>,
	rx_interrupt::line<rx_interrupt::gpt0_gtcib>,
	rx_interrupt::line<rx_interrupt::gpt0_gtciv>,
	rx_interrupt::line<rx_interrupt::gpt0_gtciu>,
	gpt_module_enable_func<0> > gpt0_t;
  struct gpt0_inst { constexpr gpt0_t& operator () (void) const { return inst ().gpt0; } };

  typedef dev::rx_gpta::hw_chn_inst <
	0x000C2180, 1, 0x000C2000, pclka_hz,
	rx_interrupt::line<rx_interrupt::gpt1_gtcia>,
	rx_interrupt::line<rx_interrupt::gpt1_gtcib>,
	rx_interrupt::line<rx_interrupt::gpt1_gtciv>,
	rx_interrupt::line<rx_interrupt::gpt1_gtciu>,
	gpt_module_enable_func<1> > gpt1_t;
  struct gpt1_inst { constexpr gpt1_t& operator () (void) const { return inst ().gpt1; } };

  typedef dev::rx_gpta::hw_chn_inst <
	0x000C2200, 2, 0x000C2000, pclka_hz,
	rx_interrupt::line<rx_interrupt::gpt2_gtcia>,
	rx_interrupt::line<rx_interrupt::gpt2_gtcib>,
	rx_interrupt::line<rx_interrupt::gpt2_gtciv>,
	rx_interrupt::line<rx_interrupt::gpt2_gtciu>,
	gpt_module_enable_func<2> > gpt2_t;
  struct gpt2_inst { constexpr gpt2_t& operator () (void) const { return inst ().gpt2; } };

  typedef dev::rx_gpta::hw_chn_inst <
	0x000C2280, 3, 0x000C2000, pclka_hz,
	rx_interrupt::line<rx_interrupt::gpt3_gtcia>,
	rx_interrupt::line<rx_interrupt::gpt3_gtcib>,
	rx_interrupt::line<rx_interrupt::gpt3_gtciv>,
	rx_interrupt::line<rx_interrupt::gpt3_gtciu>,
	gpt_module_enable_func<3> > gpt3_t;
  struct gpt3_inst { constexpr gpt3_t& operator () (void) const { return inst ().gpt3; } };

  #endif // MCB_USE_GPT



  // on this board DTC to external address space can be used to stream data
  // to the digital outputs (trigger, normal, MCX axes) in the CS7 area.
  // thus we need to use full address mode for DTC.
  #if defined (MCB_USE_DTC)
  using dtc_insn_type = dev::dtca::insn_full;

  static dev::dtca::vector_table_t<dtc_insn_type>& dtc_vector_table_inst (void);

  using dtc_t = dev::dtca::hw_inst<dtc_insn_type, &dtc_vector_table_inst>;
  struct dtc_inst { constexpr dtc_t& operator () (void) const { return inst ().dtc; } };

  #endif

  #if defined (MCB_USE_DMACA)
    auto& dmaca_en_bits (void) { return m_dmaca_en_bits; }

    template <unsigned int ChannelNumber>
    struct dmaca_module_enable_func
    {
      // remember the individual enable bits for each tpu channel in the board
      // instance and enable or disable the tpu module based on that.
      void operator () (bool val)
      {
	auto& en_bits = mcb_v2_board::inst ().dmaca_en_bits ();
	en_bits[ChannelNumber] = val;
	dev::rx_dmaca::dmast = en_bits.any () ? 1 : 0;
      }
    };

    typedef dev::rx_dmaca::hw_chn_inst < 0, 0x00082000,
	rx_interrupt::line<rx_interrupt::dmac0i>,
	dmaca_module_enable_func, dtc_inst> dmaca0_t;

    typedef dev::rx_dmaca::hw_chn_inst < 1, 0x00082040,
	rx_interrupt::line<rx_interrupt::dmac1i>,
	dmaca_module_enable_func, dtc_inst> dmaca1_t;

    typedef dev::rx_dmaca::hw_chn_inst < 2, 0x00082080,
	rx_interrupt::line<rx_interrupt::dmac2i>,
	dmaca_module_enable_func, dtc_inst> dmaca2_t;

    typedef dev::rx_dmaca::hw_chn_inst < 3, 0x000820C0,
	rx_interrupt::line<rx_interrupt::dmac3i>,
	dmaca_module_enable_func, dtc_inst> dmaca3_t;

    #if !defined (MCB_USE_RX63)
      typedef dev::rx_dmaca::hw_chn_inst < 4, 0x00082100,
	rx_interrupt::line<rx_interrupt::dmac4i>,
	dmaca_module_enable_func, dtc_inst> dmaca4_t;

      typedef dev::rx_dmaca::hw_chn_inst < 5, 0x00082140,
	rx_interrupt::line<rx_interrupt::dmac5i>,
	dmaca_module_enable_func, dtc_inst> dmaca5_t;

      typedef dev::rx_dmaca::hw_chn_inst < 6, 0x00082180,
	rx_interrupt::line<rx_interrupt::dmac6i>,
	dmaca_module_enable_func, dtc_inst> dmaca6_t;

      typedef dev::rx_dmaca::hw_chn_inst < 7, 0x000821C0,
	rx_interrupt::line<rx_interrupt::dmac7i>,
	dmaca_module_enable_func, dtc_inst> dmaca7_t;
    #endif
  #endif


  #if defined (MCB_USE_LEDS)
    #if !defined (MCB_USE_DTC)
      #error LEDs need DTC
    #endif

  using led_outputs_t = dev::led_outputs<dtc_inst, cmt2_inst>;
  struct led_outputs_inst { constexpr led_outputs_t& operator () (void) const { return inst ().led_outputs; } };

  #endif



  devices_begin_t devices_begin;

  reset_hw_init_t reset_hw_init;

  // device constructors might already try to use DTC stuff for their setup,
  // so it must come first.
#if defined (MCB_USE_DTC)
  dtc_t dtc;
#endif

  typedef dev::rx63_cmt_timer <pclkb_hz, system_clock_frequency_hz > system_timer_t;
  system_timer_t system_timer;

  set_system_timer_initialized_t set_system_timer_initialized;


#if defined (MCB_USE_DMACA)
  dmaca0_t dmaca0;
  dmaca1_t dmaca1;
  dmaca2_t dmaca2;
  dmaca3_t dmaca3;

  #if !defined (MCB_USE_RX63)
    dmaca4_t dmaca4;
    dmaca5_t dmaca5;
    dmaca6_t dmaca6;
    dmaca7_t dmaca7;
  #endif

   dev::dma::dispatcher dmaca_dispatcher;
#endif

  // the rx63_cmt_timer references RX63 interrupt numbers.
  // luckily CMT0_CMI and CMT1_CMI numbers are the same for RX63 and RX64/RX71.
  #if defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) \
      || defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)
  static_assert ((int)dev::rx63_interrupt::cmt0_cmi == (int)dev::rx64_interrupt::cmt0_cmi, "");
  static_assert ((int)dev::rx63_interrupt::cmt1_cmi == (int)dev::rx64_interrupt::cmt1_cmi, "");
  #endif

  cmt2_t cmt2;


#if defined (MCB_USE_TPU)
  tpu0_t tpu0;
  tpu1_t tpu1;
  tpu2_t tpu2;
  tpu3_t tpu3;
  tpu4_t tpu4;
  tpu5_t tpu5;
#endif

#if defined (MCB_USE_GPT)
  gpt0_t gpt0;
  gpt1_t gpt1;
  gpt2_t gpt2;
  gpt3_t gpt3;
#endif

#if defined (MCB_USE_LEDS)
  led_outputs_t led_outputs;
#endif



// debugging SCI1 in usart mode
// use max 1.5 Mbps as that works with 60 mhz and 48 mhz pbclk
// on RX71 SCIg will use double speed clocking to support bitrate divisor 60/1.5 = 40
#if defined (MCB_USE_SCI_DEBUG)
  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A020, 1, pclkb_hz, 1'500'000,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_tei>,
			//dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<30>
		     >, 32, 32 > debug_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A020, 1, pclkb_hz, 1'500'000,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci1_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci1_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci1_tei>,
			//dev::rx64_interrupt::line<dev::rx64_interrupt::sci1_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<30>
		     >, 32, 32 > debug_usart_t;
  #endif

  debug_usart_t debug_usart;
  struct debug_usart_inst { constexpr debug_usart_t& operator () (void) const { return inst ().debug_usart; } };
#endif


// the general purpose serial interfaces (GPSI) should probably be instantiated dynamically.
// for now put them in usart mode.
// for board tester, need to put them into different mdoes?  or maybe just
// toggling pins as GPIOs is enough to verify functionality ...

#if defined (MCB_USE_SCI0_USART)

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A000, 0, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci0_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci0_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci0_tei>,
//			dev::rx63_interrupt::line<dev::rx63_interrupt::sci0_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<31>
		     >, 32, 32 > gpsi0_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A000, 0, pclkb_hz, pclkb_hz / 16,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci0_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci0_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci0_tei>,
			//dev::rx64_interrupt::line<dev::rx64_interrupt::sci0_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<31>
		     >, 32, 32 > gpsi0_usart_t;
  #endif

  gpsi0_usart_t gpsi0_usart;
  struct gpsi0_usart_inst { constexpr gpsi0_usart_t& operator () (void) const { return inst ().gpsi0_usart; } };

#endif


#if defined (MCB_USE_SCI3_USART)

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A060, 3, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci3_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci3_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci3_tei>,
//			dev::rx63_interrupt::line<dev::rx63_interrupt::sci3_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<28>
		     >, 32, 32 > gpsi1_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A060, 3, pclkb_hz, pclkb_hz / 16,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci3_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci3_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci3_tei>,
			//dev::rx64_interrupt::line<dev::rx64_interrupt::sci3_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<28>
		     >, 32, 32 > gpsi1_usart_t;
  #endif

  gpsi1_usart_t gpsi1_usart;
  struct gpsi1_usart_inst { constexpr gpsi1_usart_t& operator () (void) const { return inst ().gpsi1_usart; } };

#endif


#if defined (MCB_USE_SCI5_USART)

// FIXME: SCI5 can also use RTS and CTS, have to configure that somehow.

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A0A0, 5, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci5_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci5_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci5_tei>,
//			dev::rx63_interrupt::line<dev::rx63_interrupt::sci5_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<26>
		     >, 32, 32 > gpsi2_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A0A0, 5, pclkb_hz, pclkb_hz / 16,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci5_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci5_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci5_tei>,
			//dev::rx64_interrupt::line<dev::rx64_interrupt::sci5_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<26>
		     >, 32, 32 > gpsi2_usart_t;
  #endif

  gpsi2_usart_t gpsi2_usart;
  struct gpsi2_usart_inst { constexpr gpsi2_usart_t& operator () (void) const { return inst ().gpsi2_usart; } };

#endif


#if defined (MCB_USE_SCI2_USART)

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A040, 2, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_tei>,
//			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<29>
		     >, 32, 32 > gpsi3_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A040, 2, pclkb_hz, pclkb_hz / 16,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci2_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci2_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci2_tei>,
			//dev::rx64_interrupt::line<dev::rx64_interrupt::sci2_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<29>
		     >, 32, 32 > gpsi3_usart_t;
  #endif

  gpsi3_usart_t gpsi3_usart;
  struct gpsi3_usart_inst { constexpr gpsi3_usart_t& operator () (void) const { return inst ().gpsi3_usart; } };

#endif



#if defined (MCB_USE_RS485)

   typedef dev::gpio_transceiver<dev::rx_gpio::exclusive_output_port< decltype (dev::rx_gpio::podr::pj), 0xFF, 0x00 >> rs485_ch1_tx_en_t; // PJ
   typedef dev::gpio_transceiver<dev::rx_gpio::exclusive_output_port< decltype (dev::rx_gpio::podr::pb), 0xFF, 0x00 >> rs485_ch2_tx_en_t; // PB

   // CH1: SCI6

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A0C0, 6, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_tei>,
//			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<25>
		     >, 32, 32, rs485_ch1_tx_en_t > rs485_ch1_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A0C0, 6, pclkb_hz, pclkb_hz / 16,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_tei>,
			//dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<25>
		     >, 32, 32, rs485_ch1_tx_en_t > rs485_ch1_t;
  #endif

   // CH2: SCI9 / SCIFA9

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A120, 9, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci9_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci9_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci9_tei>,
//			dev::rx63_interrupt::line<dev::rx63_interrupt::sci9_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrc_bit<26>
		     >, 32, 32, rs485_ch2_tx_en_t > rs485_ch2_t;


  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_scif_usart <
	dev::rx_scifa < 0xD0020, 9, pclka_hz, pclka_hz / 16,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa9_bri>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa9_eri>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa9_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa9_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa9_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa9_dri>,
			dev::rx_mstpcrc_bit<25>
		      >, 32, 32, rs485_ch2_tx_en_t > rs485_ch2_t;
  #endif

  rs485_ch1_t rs485_ch1;
  struct rs485_ch1_inst { constexpr rs485_ch1_t& operator () (void) const { return inst ().rs485_ch1; } };

  rs485_ch2_t rs485_ch2;
  struct rs485_ch2_inst { constexpr rs485_ch2_t& operator () (void) const { return inst ().rs485_ch2; } };

#endif

#if defined (MCB_USE_SPI)
  // slave-select / chip-select line outputs for the SCI4 SPI port.
  // SS0 - DAC_CS1N
  // SS1 - DAC_CS2N
  // SS2 - DAC_CS3
  // SS3 - external SPI port CN27

  using sci4_spi_ss0 = dev::rx_gpio::exclusive_output_port <
	decltype (dev::rx_gpio::podr::p9), (uint8_t)~(1u << 3), 0xFF >;

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144) \
      || defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144)

  using sci4_spi_ss1 = dev::rx_gpio::exclusive_output_port <
	decltype (dev::rx_gpio::podr::p9), (uint8_t)~(1u << 2), 0xFF >;

  using sci4_spi_ss2 = dev::rx_gpio::exclusive_output_port <
	decltype (dev::rx_gpio::podr::p6), (uint8_t)~(1u << 0), 0xFF >;

  #elif defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

  using sci4_spi_ss1 = dev::rx_gpio::exclusive_output_port <
	decltype (dev::rx_gpio::podr::p9), (uint8_t)~(1u << 6), 0xFF >;

  using sci4_spi_ss2 = dev::rx_gpio::exclusive_output_port <
	decltype (dev::rx_gpio::podr::p9), (uint8_t)~(1u << 7), 0xFF >;

  #endif

  using sci4_spi_ss3 = dev::rx_gpio::exclusive_output_port <
	decltype (dev::rx_gpio::podr::p9), (uint8_t)~(1u << 0), 0xFF >;


  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    // 12 MHz
    typedef dev::rx_sci_spi_master <
	dtc_inst,
	dev::rx_scic < 0x8A080, 4, pclkb_hz, pclkb_hz / 4,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_tei>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_eri>,
			dev::rx_mstpcrb_bit<27>
		     >,
	sci4_spi_ss0, sci4_spi_ss1, sci4_spi_ss2, sci4_spi_ss3
	> sci4_spi_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    // 18 MHz
    typedef dev::rx_sci_spi_master <
	dtc_inst,
	dev::rx_scig < 0x8A080, 4, pclkb_hz, pclkb_hz / 4,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci4_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci4_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci4_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci4_eri>,
			dev::rx_mstpcrb_bit<27>
		     >,
	sci4_spi_ss0, sci4_spi_ss1, sci4_spi_ss2, sci4_spi_ss3
	> sci4_spi_t;
  #endif

  sci4_spi_t sci4_spi;
  struct sci4_spi_inst { constexpr auto& operator () (void) const { return inst ().sci4_spi; } };

#endif

#if defined (MCB_USE_DAC124S085)
  #if !defined (MCB_USE_SPI)
    #error DAC124S085 needs internal SPI
  #endif

  // there are 3 DAC124S085
  // max speed for each device is about 30 MHz

  typedef dev::dac124s085 < 0, 30'000'000 > dac0_t;
  typedef dev::dac124s085 < 1, 30'000'000 > dac1_t;
  typedef dev::dac124s085 < 2, 30'000'000 > dac2_t;

  dac0_t dac0;
  struct dac0_inst { constexpr dac0_t& operator () (void) const { return inst ().dac0; } };

  dac1_t dac1;
  struct dac1_inst { constexpr dac1_t& operator () (void) const { return inst ().dac1; } };

  dac2_t dac2;
  struct dac2_inst { constexpr dac2_t& operator () (void) const { return inst ().dac2; } };

  init_dac_default_values_t init_dac_default_values;

#endif


#if defined (MCB_USE_MCX514) || defined (MCB_USE_MCX512)

  // MCX51x is connected to CS3 and CS4 (16 bit) and clocked variably by
  // MTIOC3C output.
  // additional delay cycles for command writes are done by separate CS area
  // and timings.

  // the MCX51x trigger outputs require some configuration immediately after
  // reset relese.  so it must be instantiated before peripheral release.

  struct mcx_cmd_write_delay_func { void operator () (void) { } };

  // when accessing MCX51x, set address bits PA5,PA6,PA7 to avoid conflicts with
  // dipswitch readings.
  using mcx_bus_interface = dev::mcx51x::if_parallel16 <
	0x040000E0, 0x050000E0,	mcx_cmd_write_delay_func, utils::little_endian >;

  // the mcx_axis_inputs and mcx_axis_outputs need the mcx axis instances
  // for initialization.  thus, we put them in the board class after the
  // mcx instance and the mcx hw_inst gets only functors to access the
  // actual input/output devices.
  struct mcx_axis_outputs_func
  {
    void* outputs;

    template <typename Mcx, typename McxAxis> auto&& operator () (Mcx& m, McxAxis& a);
  };

  struct mcx_axis_inputs_func
  {
    void* inputs;

    template <typename Mcx, typename McxAxis> auto&& operator () (Mcx& m, McxAxis& a);
  };


  // try to run the MCX at 16 MHz, but that might not always be possible.
  // on RX63N MTU clock = PCLKB = 48 MHz, while on RX64/RX71 it is 96 MHz
  // or 120 MHz. so we get something like that
  //   48 MHz MTU -> 12 MHz MCX (fastest possible clock speed)
  //   96 MHz MTU -> 16 MHz MCX (exact clock speed)
  //  120 MHz MTU -> 20 MHz MCX (round up to next faster clock speed)
  //
  // some clock values can't be represented as an integer.
  // to allow compiler optimizations of MCX speed calculations, store the
  // clock as a rational number.

  // MCX514 tends to run a bit hot.  if we wanted to coserve some power, could also
  // run it at 8 MHz.  maybe make a board build config option for that.

  using mcx_clock_hz = mtu_tick_count_to_hz_ratio <
    std::max (1u, hz_to_mtu_tick_count_floor <std::ratio<16'000'000, 1>> () ) >;


  #if defined (MCB_USE_MCX514)
    typedef dev::mcx51x::mcx514_hw_inst <
  #elif defined (MCB_USE_MCX512)
    typedef dev::mcx51x::mcx512_hw_inst <
  #endif
	mcx_bus_interface, mcx_clock_hz,

	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144) \
	    || defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144)
	  rx_interrupt::line<rx_interrupt::icu_irq3>,
	  rx_interrupt::line<rx_interrupt::icu_irq5>,
	#elif defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
	  rx_interrupt::line<rx_interrupt::icu_irq3>,
	  rx_interrupt::line<rx_interrupt::icu_irq0>,
	#endif

	mcx_axis_inputs_func, mcx_axis_outputs_func

   > mcx51x_t;

  mcx51x_t mcx51x;
  struct mcx51x_inst { constexpr mcx51x_t& operator () (void) const { return inst ().mcx51x; } };

  using mcx_axis_inputs_dev_t = dev::mcx_axis_inputs<mcx51x_t::axis_t>;
  using mcx_axis_outputs_dev_t = dev::mcx_axis_outputs<mcx51x_t::axis_t>;

  std::array<mcx_axis_inputs_dev_t, mcx51x_t::axis_count> mcx51x_axis_inputs;
  std::array<mcx_axis_outputs_dev_t, mcx51x_t::axis_count> mcx51x_axis_outputs;

  template <unsigned int AxisNum>
  struct mcx51x_axis_inputs_inst { constexpr mcx_axis_inputs_dev_t& operator () (void) const { return inst ().mcx51x_axis_inputs[AxisNum]; } };

  template <unsigned int AxisNum>
  struct mcx51x_axis_outputs_inst { constexpr mcx_axis_outputs_dev_t& operator () (void) const { return inst ().mcx51x_axis_outputs[AxisNum]; } };

#endif


  release_peripheral_reset_t release_peripheral_reset;


#if defined (MCB_USE_PCD4641)
  struct cmt3_cmi_pcd_axis_inputs_sampler_isr_bridge
  {
    void operator () (void) const
    {
      mcb_v2_board::inst ().pcd_axis_inputs_sampler.invoke_isr ();
    }
  };

  // CMT3 is used as a DTC timer for IO sampling for soft-encoders.
  typedef dev::rx_cmt::hw_chn_inst <0x0008'8010, 3, pclkb_hz,
	rx_interrupt::line<rx_interrupt::cmt3_cmi>,
	dev::rx_mstpcra_bit<14>,
	dev::timer::fixed_trigger_callback < cmt3_cmi_pcd_axis_inputs_sampler_isr_bridge >
	> cmt3_t;

  cmt3_t cmt3;
  struct cmt3_inst { constexpr cmt3_t& operator () (void) const { return inst ().cmt3; } };

  // PCD4641 is connected to CS1 and CS2 (8 bit), with a twiddled address
  // interface and clocked variably by the MTIOC2A output.

  // the PCD clock has an impact on the number of bus wait cycles
  // that need to be inserted after a command write, and there is a limit to
  // that.  so the PCD clock and bus clock should not be too far away from each
  // other.
  // although overclocking the PCD to e.g. 12 MHz seems to work OK,
  // there are subtle issues regarding its IO readings and other things.
  // it does run and output pulse signals, but some other things seem to
  // go out the window when clock speed is higher than the specified 10 MHz.
  // and actually it seems it already starts happening at 10 MHz so run it
  // at a lower frequency.

  // MTU 120 MHz = PCD 7.5 MHz
  // MTU  96 MHz = PCD 8 MHz
  // MTU  48 MHz = PCD 8 MHz
  static constexpr unsigned int pcd4641_clock_hz =
    mtu_tick_count_to_hz (std::max (1u, hz_to_mtu_tick_count_ceil<std::ratio<8'000'000, 1>> ()));

  struct pcd4641_cmd_write_delay_func { void operator () (void) { }; };

  struct dtc_cmt3_mutex
  {
    void lock (void)
    {
      // could actually implement a full lock by turning off the timer
      // that drives the DTC.  but it seems we don't need to go that far.
      std::invoke (mcb_v2_board::dtc_inst ()).wait_for < cmt3_t::cmi_interrupt_line > ();
    }

    void unlock (void) { }
  };

  // when accessing PCD, set address bits PA5,PA6,PA7 to avoid conflicts with
  // dipswitch readings.
  using pcd4641_bus_interface =
	dev::pcd4641::if_a0a1_twiddled_parallel8 <
		0x060000E0, 0x070000E0, pcd4641_cmd_write_delay_func,
		utils::big_endian, dtc_cmt3_mutex >;

  // the pcd_axis_inputs and pcd_axis_outputs need the pcd axis instances
  // for initialization.  thus, we put them in the board class after the
  // pcd instance and the pcd hw_inst gets only functors to access the
  // actual input/output devices.

  struct pcd_axis_inputs_axis_func
  {
    auto&& operator () (const void* thizz);
  };

  struct pcd_axis_outputs_axis_func
  {
    auto&& operator () (const void* thizz);
  };

  struct pcd_axis_num_func
  {
    unsigned int operator () (const void* axis_ptr);
  };

  typedef dev::pcd_axis_outputs<pcd_axis_outputs_axis_func> pcd_axis_outputs_dev_t;
  typedef dev::pcd_axis_inputs<pcd_axis_inputs_axis_func> pcd_axis_inputs_dev_t;

  typedef dev::pcd4641::hw_inst <
	pcd4641_bus_interface, pcd4641_clock_hz,
	rx_interrupt::line<rx_interrupt::icu_irq7>,
	pcd_axis_inputs_dev_t, pcd_axis_outputs_dev_t, pcd_axis_num_func
  > pcd4641_t;

  pcd4641_t pcd4641;
  struct pcd4641_inst { constexpr pcd4641_t& operator () (void) const { return inst ().pcd4641; } };

  template < unsigned int AxisNum >
  struct pcd4641_axis_inputs_inst { constexpr pcd_axis_inputs_dev_t& operator () (void) const { return inst ().pcd4641.axis (AxisNum).inputs (); } };

  template < unsigned int AxisNum >
  struct pcd4641_axis_outputs_inst { constexpr pcd_axis_outputs_dev_t& operator () (void) const { return inst ().pcd4641.axis (AxisNum).outputs (); } };

  #if !defined (MCB_USE_DTC)
    #error PCD IO sampler needs DTC
  #endif

  #define MCB_HAVE_PCD_AXIS_INPUTS_SAMPLER
  using pcd_axis_inputs_sampler_t = dev::pcd_axis_inputs_sampler<
	pcd4641_bus_interface, dtc_inst, cmt3_inst >;

  pcd_axis_inputs_sampler_t pcd_axis_inputs_sampler;
  struct pcd4641_inputs_sampler_inst { constexpr pcd_axis_inputs_sampler_t& operator () (void) const { return inst ().pcd_axis_inputs_sampler; } };
#endif

#if defined (MCB_USE_FCU)

  #if defined (MCB_USE_RX63)
    typedef dev::rx63_fcu fcu_t;

  #elif defined (MCB_USE_RX64_RX71)
    typedef dev::rx64_fcu::hw_inst<
	dev::rx64_interrupt::line<dev::rx64_interrupt::fcu_fiferr>,
	dev::rx64_interrupt::line<dev::rx64_interrupt::fcu_frdyi>> fcu_t;

  #endif

  fcu_t fcu;
  struct fcu_inst { constexpr fcu_t& operator () (void) const { return inst ().fcu; } };

  auto& data_flash (void) { return fcu.data_flash_dev (); }

#endif

#if !defined (MCB_NO_CRC)

  // RX63, RX64 and RX71 all have the same CRC unit.
  using crc_t = dev::rx63_crc;
  crc_t crc;

  struct crc_inst { constexpr crc_t& operator () (void) const { return inst ().crc; } };
#endif


#if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC1)

  #if defined (MCB_USE_RX63N_144)
    typedef dev::rx_etherc_mdio_sta < 0x000C0120, iclk_hz, 5 > mdio_sta_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) \
        || defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_etherc_mdio_sta < 0x000C0120, pclka_hz, 9 > mdio_sta_t;
  #endif

  mdio_sta_t mdio_sta;
  struct mdio_sta_inst { constexpr mdio_sta_t& operator () (void) const { return inst ().mdio_sta; } };

#endif

#if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC0_RAW)

  #if defined (MCB_USE_RX63N_144)

    typedef dev::rx_etherc::hw_inst < 0x000C0100, pclka_hz > rx_etherc0_t;
    typedef dev::rx_edmac::hw_inst <
		0x000C0000, pclka_hz, rx_etherc0_t,
		dev::rx63_interrupt::line<dev::rx63_interrupt::ether_eint>,
		dev::interrupt::virtual_line<0>,
		dev::rx_mstpcrb_bit<15>
	 > rx_edmac0_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) \
        || defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)
    typedef dev::rx_etherc::hw_inst < 0x000C0100, pclka_hz > rx_etherc0_t;

    typedef dev::rx_edmaca::hw_inst <
		0x000C0000, pclka_hz, rx_etherc0_t,
		dev::rx64_interrupt::line<dev::rx64_interrupt::edmac0_eint>,
		dev::interrupt::virtual_line<0>,
		dev::rx_mstpcrb_bit<15>
	> rx_edmac0_t;

  #endif

  typedef dev::lan8710 < 0, dev::interrupt::unconnected > lan8720_0_t;

  using eth0_mac_t = rx_edmac0_t;
  using eth0_phy_t = lan8720_0_t;

  rx_etherc0_t eth0_etherc;
  struct eth0_etherc_inst { constexpr rx_etherc0_t& operator () (void) const { return inst ().eth0_etherc; } };

  eth0_mac_t eth0_mac;
  struct eth0_mac_inst { constexpr eth0_mac_t& operator () (void) const { return inst ().eth0_mac; } };

  eth0_phy_t eth0_phy;
  struct eth0_phy_isnt { constexpr eth0_phy_t& operator () (void) const { return inst ().eth0_phy; } };

  #if !defined (MCB_USE_RX_ETHERC0_RAW)
    using eth0_t = dev::rx_etherc_eth < rx_edmac0_t, lan8720_0_t, board_info_eth0_addr >;
    eth0_t eth0;

    struct eth0_inst { constexpr eth0_t& operator () (void) const { return inst ().eth0; } };
  #endif

#endif



#if defined (MCB_USE_RX_ETHERC1) || defined (MCB_USE_RX_ETHERC1_RAW)

  #if defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_etherc::hw_inst < 0x000C0300, pclka_hz > rx_etherc1_t;
    typedef dev::rx_edmaca::hw_inst <
		0x000C0200, pclka_hz, rx_etherc1_t,
		dev::rx64_interrupt::line<dev::rx64_interrupt::edmac1_eint>,
		dev::interrupt::virtual_line<1>,
		dev::rx_mstpcrb_bit<14>
	> rx_edmac1_t;
  #endif

  typedef dev::lan8710 < 1, dev::interrupt::unconnected > lan8720_1_t;

  using eth1_mac_t = rx_edmac1_t;
  using eth1_phy_t = lan8720_1_t;

  rx_etherc1_t eth1_etherc;
  struct eth1_etherc_inst { constexpr rx_etherc1_t& operator () (void) const { return inst ().eth1_etherc; } };

  eth1_mac_t eth1_mac;
  struct eth1_mac_inst { constexpr eth1_mac_t& operator () (void) const { return inst ().eth1_mac; } };

  eth1_phy_t eth1_phy;
  struct eth1_phy_inst { constexpr eth1_phy_t& operator () (void) const { return inst ().eth1_phy; } };

  #if !defined (MCB_USE_RX_ETHERC1_RAW)

    using eth1_t = dev::rx_etherc_eth < rx_edmac1_t, lan8720_1_t, board_info_eth1_addr >;
    eth1_t eth1;
    struct eth1_inst { constexpr eth1_t& operator () (void) const { return inst ().eth1; } };
  #endif

#endif


#if defined (MCB_USE_LAN9250) || defined (MCB_USE_LAN9250_RAW)

  // the LAN9250 is connected as 16 bit parallel interface to CS5 area.
  // the FIFOSEL is connected to A17, which gives us the following
  // memory mapping:
  //   0x03000000 - 0x0300001F: indexed register access
  //   0x03020000 - 0x0302FFFF: FIFO access

  using lan9250_bus_interface =
	dev::lan9250::if_parallel_indexed_16_fifo_area <
		0x03000000, 0x03020000, 0x0302FFFF, utils::little_endian>;

  typedef dev::lan9250::hw_inst <
	lan9250_bus_interface,
	dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq4> > lan9250_t;

  using eth2_mac_t = lan9250_t;
  using eth2_phy_t = lan9250_t::phy_t;

  eth2_mac_t eth2_mac;
  struct eth2_mac_inst { constexpr eth2_mac_t& operator () (void) const { return inst ().eth2_mac; } };

  eth2_phy_t eth2_phy;
  struct eth2_phy_inst { constexpr eth2_phy_t& operator () (void) const { return inst ().eth2_phy; } };

  #if !defined (MCB_USE_LAN9250_RAW)

    using eth2_t = dev::lan9250_eth < lan9250_t, lan9250_t::phy_t, board_info_eth2_addr >;
    eth2_t eth2;
    struct eth2_inst { constexpr eth2_t& operator () (void) const { return inst ().eth2; } };
  #endif

#endif


#if defined (MCB_USE_STANDBY_POWER_SUPPLY)

  // the standby power supply allows running the RX in deep standby modes
  // which use an external 32 khz crystal as oscillator.  the frequency of
  // that crystal can be measured against another frequency.  this allows
  // keeping track of the time in standby mode.

  #if defined (MCB_USE_RX63)

    // have to use the MCK peripheral, which uses MTU0/TPU0 and MTU1/TPU1..
    // looks a bit annoying.

    // also LVDA is more restricted and can do only NMI.
    // thus, for now ...

    #error standby power supply not supported on RX63


  #elif defined (MCB_USE_RX64_RX71)

    typedef dev::rx_cac::hw_inst <
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::rx_mstpcrc_bit<19>
    > cac_t;

  cac_t cac;
  struct cac_inst { constexpr cac_t& operator () (void) const { return inst ().cac; } };

    typedef dev::rx_lvda::hw_inst_rx64 <
	dev::rx64_interrupt::line<dev::rx64_interrupt::lvd1>,
	dev::rx64_interrupt::line<dev::rx64_interrupt::lvd2>> lvda_t;

  lvda_t lvda;
  struct lvda_inst { constexpr lvda_t& operator () (void) const { return inst ().lvda; } };

  #endif

#endif


  // MCB specific devices
#if defined (MCB_USE_EMG_STOP)
  using emg_stop_inputs_t = dev::emg_stop_inputs;
  emg_stop_inputs_t emg_stop_inputs;
  struct emg_stop_inputs_isnt { constexpr emg_stop_inputs_t& operator () (void) const { return inst ().emg_stop_inputs; } };
#endif

#if defined (MCB_USE_TRIGGER_IO)

  using trigger_inputs_t = dev::trigger_inputs;
  trigger_inputs_t trigger_inputs;
  struct trigger_inputs_inst { constexpr trigger_inputs_t& operator () (void) const { return inst ().trigger_inputs; } };

  using trigger_outputs_t = dev::trigger_outputs;
  trigger_outputs_t trigger_outputs;
  struct trigger_outputs_inst { constexpr trigger_outputs_t& operator () (void) const { return inst ().trigger_outputs; } };

#endif


#if defined (MCB_USE_DIGITAL_IO)
  using digital_inputs_t = dev::digital_inputs;
  digital_inputs_t digital_inputs;
  struct digital_inputs_inst { constexpr digital_inputs_t& operator () (void) const { return inst ().digital_inputs; } };

  using digital_outputs_t = dev::digital_outputs;
  digital_outputs_t digital_outputs;
  struct digital_outputs_inst { constexpr digital_outputs_t& operator () (void) const { return inst ().digital_outputs; } };
#endif


#if defined (MCB_USE_DIPSWITCH)
  using dipswitch_inputs_t = dev::dipswitch_inputs;
  dipswitch_inputs_t dipswitch_inputs;
  struct dipswitch_inputs_inst { constexpr dipswitch_inputs_t& operator () (void) const { return inst ().dipswitch_inputs; } };
#endif

  devices_end_t devices_end;
};

template <> inline void mcb_v2_board::cpu_wait_3_iclocks<1> (void)
{
  // accessing the SYSTEM register is 3 ICLK on RX63, RX64 and RX71.
  [[gnu::unused]] uint16_t val = *((volatile uint16_t*)0x00080000);
}

template <> inline void mcb_v2_board::cpu_wait_3_iclocks<0> (void) { }

#if defined (MCB_USE_PCD4641)

inline auto&& mcb_v2_board::pcd_axis_inputs_axis_func::operator () (const void* thizz)
{
  auto& a = mcb_v2_board::inst ().pcd4641.axis (0);
  auto& ai = a.inputs ();

  unsigned int off = (uintptr_t)&ai - (uintptr_t)&a;
  return *(pcd4641_t::axis_t*)((uintptr_t)thizz - off);
}

inline auto&& mcb_v2_board::pcd_axis_outputs_axis_func::operator () (const void* thizz)
{
  auto& a = mcb_v2_board::inst ().pcd4641.axis (0);
  auto& ao = a.outputs ();

  unsigned int off = (uintptr_t)&ao - (uintptr_t)&a;
  return *(pcd4641_t::axis_t*)((uintptr_t)thizz - off);
}

inline unsigned int mcb_v2_board::pcd_axis_num_func::operator () (const void* axis_ptr)
{
  auto& a0 = mcb_v2_board::inst ().pcd4641.axis (0);

  return ((uintptr_t)axis_ptr - (uintptr_t)&a0) / a0.this_size ();
};

#endif


#if defined (MCB_USE_MCX514) || defined (MCB_USE_MCX512)
template <typename Mcx, typename McxAxis>
inline auto&& mcb_v2_board::mcx_axis_inputs_func::operator () ([[gnu::unused]] Mcx& m, McxAxis& a)
{
  return (*((std::array<mcx_axis_inputs_dev_t, mcx51x_t::axis_count>*)inputs))[a.num ()];
}

template <typename Mcx, typename McxAxis>
inline auto&& mcb_v2_board::mcx_axis_outputs_func::operator () ([[gnu::unused]] Mcx& m, McxAxis& a)
{
  return (*((std::array<mcx_axis_outputs_dev_t, mcx51x_t::axis_count>*)outputs))[a.num ()];
}
#endif

namespace this_board
{
using type = mcb_v2_board;
inline constexpr mcb_v2_board& inst (void) { return mcb_v2_board::inst (); }

}

#endif // __cplusplus
#endif // includeguard_mcbv2_board_hpp_includeguard
