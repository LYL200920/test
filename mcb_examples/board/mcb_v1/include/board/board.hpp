#ifndef includeguard_board_hpp_includeguard
#define includeguard_board_hpp_includeguard
#ifdef __cplusplus

#include "board_clk.hpp"
#include "reset_source.hpp"

#include <dev/renesas/rx63_interrupt.hpp>
#include <dev/renesas/rx63_cmt_timer.hpp>
#include <dev/pcd4641.hpp>
#include <dev/mcx51x.hpp>
#include <dev/renesas/rx63_fcu.hpp>
#include <dev/renesas/rx63_crc.hpp>
#include <dev/lan8710.hpp>
#include <dev/pca9698.hpp>
#include <dev/dac124s085.hpp>

#include <dev/renesas/rx_module_stop_regs.hpp>
#include <dev/renesas/rx_gpio.hpp>
#include <dev/renesas/rx_scic.hpp>
#include <dev/renesas/rx_sci_usart.hpp>
#include <dev/renesas/rx_sci_i2c.hpp>
#include <dev/renesas/rx_sci_spi.hpp>
#include <dev/renesas/rx_etherc_mdio_sta.hpp>
#include <dev/renesas/rx_etherc.hpp>
#include <dev/renesas/rx_edmac.hpp>
#include <dev/renesas/rx_etherc_eth.hpp>
#include <dev/renesas/rx_tpua.hpp>
#include <dev/renesas/rx_dtca.hpp>

#include "dev/trigger_io.hpp"
#include "dev/dipswitch_inputs.hpp"
#include "dev/led_outputs.hpp"
#include "dev/digital_io.hpp"
#include "dev/mcx_axis_io.hpp"
#include "dev/pcd_axis_io.hpp"

class mcb_v1_board : public mcb_v1_board_clk
{
private:
  // initialze board.  invoked by startup code before main.
  // the board initialization order is controlled by the members in the board
  // class below.
  mcb_v1_board (void);

  static mcb_v1_board g_inst;

  struct reset_hw_init_t { reset_hw_init_t (mcb_v1_board& brd); };
  struct release_peripheral_reset_t { release_peripheral_reset_t (mcb_v1_board& brd); };

  struct devices_begin_t { };
  struct devices_end_t { };

  struct board_info_eth0_addr { const std::array<uint8_t, 6> operator () (void); };


  // a read of the MDMONR register takes 3 iclock cycles
  // see also specializations for Count = 1 and Count = 0 below outside
  // the class.
  template <unsigned int Count> static void cpu_wait_3_iclocks (void)
  {
    cpu_wait_3_iclocks<1> ();
    cpu_wait_3_iclocks<Count - 1> ();
  }

  std::chrono::high_resolution_clock::time_point m_last_exec_time;

  #if defined (MCB_USE_TPU)
  std::bitset<6> m_tpu_en_bits;

public:

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
      auto& en_bits = mcb_v1_board::inst ().tpu_en_bits ();
      en_bits[TpuChannelNumber] = val;

      dev::rx_mstpcra_bit<13> () (en_bits.any ());
    }
  };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088110, 0, 0x00088100, 0x00088101, 0x00088108, pclock_hz,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu0_tgia>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu0_tgib>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu0_tgic>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu0_tgid>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<0> > tpu0_t;
  struct tpu0_inst { constexpr tpu0_t& operator () (void) const { return inst ().tpu0; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088120, 1, 0x00088100, 0x00088101, 0x00088109, pclock_hz,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu1_tgia>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu1_tgib>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<1> > tpu1_t;
  struct tpu1_inst { constexpr tpu1_t& operator () (void) const { return inst ().tpu1; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088130, 2, 0x00088100, 0x00088101, 0x0008810A, pclock_hz,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu2_tgia>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu2_tgib>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<2> > tpu2_t;
  struct tpu2_inst { constexpr tpu2_t& operator () (void) const { return inst ().tpu2; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088140, 3, 0x00088100, 0x00088101, 0x0008810B, pclock_hz,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu3_tgia>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu3_tgib>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu3_tgic>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu3_tgid>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<3> > tpu3_t;
  struct tpu3_inst { constexpr tpu3_t& operator () (void) const { return inst ().tpu3; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088150, 4, 0x00088100, 0x00088101, 0x0008810C, pclock_hz,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu4_tgia>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu4_tgib>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<4> > tpu4_t;
  struct tpu4_inst { constexpr tpu4_t& operator () (void) const { return inst ().tpu4; } };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088160, 5, 0x00088100, 0x00088101, 0x0008810D, pclock_hz,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu5_tgia>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::tpu5_tgib>,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	dev::interrupt::unconnected,
	tpu_module_enable_func<5> > tpu5_t;
  struct tpu5_inst { constexpr tpu5_t& operator () (void) const { return inst ().tpu5; } };

  #endif // MCB_USE_TPU

public:

  // on this board DTC to external address space makes only little sense.
  // could also use short address mode.
  #if defined (MCB_USE_DTC)
  using dtc_insn_type = dev::dtca::insn_short;

  static dev::dtca::vector_table_t<dtc_insn_type>& dtc_vector_table_inst (void);

  using dtc_t = dev::dtca::hw_inst<dtc_insn_type, &dtc_vector_table_inst>;
  struct dtc_inst { constexpr dtc_t& operator () (void) const { return inst ().dtc; } };

  #endif // MCB_USE_DTC

  // convert nanoseconds to a number of bus clock cycles
  static constexpr inline unsigned int ns_to_bclk (unsigned int ns)
  {
    // 1 bus clock in ns = 1000000000 / 96000000 = 10.4166

    // 20 ns = 20 / (1000000000 / 96000000) = 1.92

    // 200 ns = 200 / (1000000000 / 96000000) = 19.2
    //          (200 * 96000000) / 1000000000 = 19.2
    //          (200 * 96000000 + 1000000000 - 1) / 1000000000 = 20.1999

    constexpr uint64_t hz_to_ns = 1000000000LL;

    uint64_t r = ((uint64_t)ns * (uint64_t)bclk_hz + hz_to_ns - 1) / hz_to_ns;

    return (unsigned int)r;
  }

  static constexpr mcb_v1_board& inst (void) { return g_inst; }

  using board_id_t = std::array<uint8_t, 32>;
  board_id_t board_id (void) const;

  // soft-reset and transfer CPU excution control to the specified address.
  void reset_to_func (void (*func)(void));

  // soft-reset to bootmode.
  // this simply jumps to the user boot rom and might not always work.
  void reset_to_bootmode (void)
  {
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

  // reset peripherals such as MCX, PCA, PCD, etc.
  static void set_peripheral_reset (bool val);

  static enum reset_source reset_source (void);
  static int reset_counter (void);
  static void set_reset_counter (int val);

  #if defined (MCB_USE_TPU)
  auto& tpu_en_bits (void) { return m_tpu_en_bits; }
  #endif


  devices_begin_t devices_begin;

  reset_hw_init_t reset_hw_init;

#if defined (MCB_USE_DTC)
  dtc_t dtc;
#endif

  typedef dev::rx63_cmt_timer <pclock_hz, system_clock_frequency_hz > system_timer_t;
  system_timer_t system_timer;

  release_peripheral_reset_t release_peripheral_reset;

#if defined (MCB_USE_LEDS)
  using led_outputs_t = dev::led_outputs;
  led_outputs_t led_outputs;
  struct led_outputs_inst { constexpr led_outputs_t& operator () (void) const { return inst ().led_outputs; } };
#endif

  // debugging SCI uart
#if defined (MCB_USE_SCI_DEBUG)
  typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A020, 1, pclock_hz, 1'500'000,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_tei>,
			//dev::rx63_interrupt::group<dev::rx63_interrupt::icu_group12, 1>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<30>
		     >, 32, 32 > debug_usart_t;

  debug_usart_t debug_usart;
  struct debug_usart_inst { constexpr debug_usart_t& operator () (void) const { return inst ().debug_usart; } };
#endif


  // the ICL3232CVZ level shifter on SCI0 has a bitrate limit of 250 kbit/sec
#if defined (MCB_USE_SCI0_RS232C)
  typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A000, 0, pclock_hz, 250000,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci0_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci0_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci0_tei>,
			//dev::rx63_interrupt::group<dev::rx63_interrupt::icu_group12, 0>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<31>
		     >, 32, 32 > rs232c_t;

  rs232c_t rs232c;
  struct rs232c_inst { constexpr rs232c_t& operator () (void) const { return inst ().rs232c; } };
#endif


  // use SCI6 and SCI7 as RS485.  from the software point of view, it's
  // a USART, except that we have to enable/disable the rs485 driver (isl3172e)
  // via an GPIO and when sending we have to turn off reception.
  // maybe for rs485 it would be helpful to have some sort of collision
  // detection (with ISR)...

  // note: the isl3172e transceiver's minimum bitrate is 250 kbps and
  //       the maximum 800 kbps, according to the datasheet.
  //       the SCI can do either do 750000 or 375000 or 187500 bps in that range.
  //       while 750000 does not work reliably, 375000 seems to be fine.
#if defined (MCB_USE_RS485)
  typedef dev::gpio_transceiver<
	dev::rx_gpio::shared_output_port < decltype (dev::rx_gpio::podr::p0), 3 >> rs485_ch1_tx_en_t;

  typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A0C0, 6, pclock_hz, 375000/1,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_tei>,
			//dev::rx63_interrupt::group<dev::rx63_interrupt::icu_group12, 6>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<25>
		     >, 256, 256, rs485_ch1_tx_en_t > rs485_ch1_t;

  rs485_ch1_t rs485_ch1;
  struct rs485_ch1_inst { constexpr rs485_ch1_t& operator () (void) const { return inst ().rs485_ch1; } };
#endif

#if defined (MCB_USE_RS485)
  typedef dev::gpio_transceiver<
	dev::rx_gpio::shared_output_port < decltype (dev::rx_gpio::podr::p9), 1 >> rs485_ch2_tx_en_t;

  typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A0E0, 7, pclock_hz, 375000/1,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci7_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci7_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci7_tei>,
			//dev::rx63_interrupt::group<dev::rx63_interrupt::icu_group12, 7>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<24>
		     >, 256, 256, rs485_ch2_tx_en_t > rs485_ch2_t;

  rs485_ch2_t rs485_ch2;
  struct rs485_ch2_inst { constexpr rs485_ch2_t& operator () (void) const { return inst ().rs485_ch2; } };
#endif


#if defined (MCB_USE_I2C)
  // i2c clock cycle = 750 ns = 1.3 mhz
  typedef dev::rx_sci_i2c_master <
	dev::rx_scic < 0x8A040, 2, pclock_hz, pclock_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_tei>,
			//dev::rx63_interrupt::group<dev::rx63_interrupt::icu_group12, 2>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<29>
		     >> internal_i2c_t;

  internal_i2c_t internal_i2c;
  struct internal_i2c_inst { constexpr internal_i2c_t& operator () (void) const { return inst ().internal_i2c; } };
#endif


#if defined (MCB_USE_SPI)
  // the SPI port is a write-only port.  hence receive related interrupts are
  // not connected. 12 MHz
  typedef dev::rx_sci_spi_master <
	dev::rx_scic < 0x8A080, 4, pclock_hz, pclock_hz / 4,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_txi>,
			dev::interrupt::unconnected,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_tei>,
			//dev::rx63_interrupt::group<dev::rx63_interrupt::icu_group12, 4>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<27>
		     >> internal_spi_t;

  internal_spi_t internal_spi;
  struct internal_spi_inst { constexpr internal_spi_t& operator () (void) const { return inst ().internal_spi; } };
#endif



#if defined (MCB_USE_MCX514_I2C_EXT) || defined (MCB_USE_PCA9698_I2C_EXT)
  typedef dev::rx63_sci_i2c_master <pclock_hz, pclock_hz / 32, dev::rx63_sci5> external_i2c_t;
  external_i2c_t external_i2c;
#endif

#if defined (MCB_USE_PCA9698)
  #if !defined (MCB_USE_I2C)
    #error PCA9698 needs internal I2C
  #endif

  // FIXME: the inputs-PCA has an interrupt line but it conflicts with the trigger
  // inputs.  we'd need a shared interrupt line for this.  see also MCB-15.
//  typedef dev::pca9698 < 0x40, 1'500'000,
//			 dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq12 > pca9698_i0_t;

  typedef dev::pca9698 < 0x40, 1'500'000,
			 dev::interrupt::unconnected > pca9698_i0_t;


  typedef dev::pca9698 < 0x42, 1'500'000,
			 dev::interrupt::unconnected > pca9698_i1_t;

  pca9698_i0_t pca9698_i0;
  pca9698_i1_t pca9698_i1;
#endif

  // PCD4641 is connected to CS1 (8 bit) and clocked at 4.915200 MHz
  // the interrupt line is connected to P17/IRQ7.
#if defined (MCB_USE_PCD4641)
  static constexpr unsigned int pcd4641_clock_hz = 4'915'200;

  struct pcd4641_cmd_write_delay_func { void operator () (void) const
  {
    // after writing a command, wait for 2 PCD clock cycles
    // 2 clocks @ 4.915200 mhz = 407 ns
    // it's also OK to wait one cycle before the command write and
    // one cycle after.
    constexpr unsigned int wait_hz = pcd4641_clock_hz/2;
    static_assert (iclock_hz >= wait_hz, "");

    constexpr unsigned int iclock_count = (iclock_hz + wait_hz - 1) / wait_hz;

    // even if 4 reads that take 3 iclock cycles are executed, it seems because
    // of a write buffer it turns out one too short and a subsequent next
    // access to the mcx514 will happen too early.
    cpu_wait_3_iclocks < ((iclock_count + 3 - 1) / 3) + 1 > ();
  }};

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

  using pcd_axis_outputs_dev_t = dev::pcd_axis_outputs<pcd_axis_outputs_axis_func>;
  using pcd_axis_inputs_dev_t = dev::pcd_axis_inputs<pcd_axis_inputs_axis_func>;

  typedef dev::pcd4641::hw_inst <
	dev::pcd4641::if_parallel8_le<
		0x07000000, 0x07000000,
		pcd4641_cmd_write_delay_func, utils::little_endian >,
	pcd4641_clock_hz,
	dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq7>,
	pcd_axis_inputs_dev_t, pcd_axis_outputs_dev_t, pcd_axis_num_func
  > pcd4641_t;

  pcd4641_t pcd4641;
  struct pcd4641_inst { constexpr pcd4641_t& operator () (void) const { return inst ().pcd4641; } };

  template < unsigned int AxisNum >
  struct pcd4641_axis_inputs_inst { constexpr pcd_axis_inputs_dev_t& operator () (void) const { return inst ().pcd4641.axis (AxisNum).inputs (); } };

  template < unsigned int AxisNum >
  struct pcd4641_axis_outputs_inst { constexpr pcd_axis_outputs_dev_t& operator () (void) const { return inst ().pcd4641.axis (AxisNum).outputs (); } };

#endif


#if defined (MCB_USE_MCX514)
  // MCX514 is connected to CS2 (16 bit) and clocked at 16 MHz
  // the CS area is fixed to little-endian, independent of CPU endian setting.
  // the bus controller will perform byte swapping if needed.
  static constexpr unsigned int mcx514_clock_hz = 16'000'000;

  struct mcx514_cmd_write_delay_func { void operator () (void) const
  {
    // after writing a command, it needs to wait for 2 MCX clock cycles.
    // 2 mcx clocks @ 16 mhz = 125 ns
    // example iclock = 50 mhz = 20 ns
    // (50000000+(16000000÷2)−1)÷(16000000÷2) = 7.249999875 => 7 iclocks
    // 7 * 20 ns = 140 ns.

    constexpr unsigned int wait_hz = mcx514_clock_hz/2;
    static_assert (iclock_hz >= wait_hz, "");

    constexpr unsigned int iclock_count = (iclock_hz + wait_hz - 1) / wait_hz;

    // even if 4 reads that take 3 iclock cycles are executed, it seems because
    // of a write buffer it turns out one too short and a subsequent next
    // access to the mcx514 will happen too early.
    // so if we take subtract the 1x3 iclk extra cycles, we get exactly 125 ns
    // waiting time.  if RX and MCX clocks were synchronized (like on MCB2)
    // that would be OK, since required waiting time at 16 mhz MCX clock is
    // 125 ns.  however, on this board the MCX uses its own oscillator which
    // can has been observed to have a big variance in clock range and drift.
    // so add one more 3-cycle wait.
    cpu_wait_3_iclocks < ((iclock_count + 3 - 1) / 3) + 2 > ();
  }};

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

  // although this board can only use the MCX514, name it as "mcx_51x" for
  // compatibility with future boards that also support the MCX512.
  typedef dev::mcx51x::mcx514_hw_inst <
	dev::mcx51x::if_parallel16 <
		0x06000000, 0x06000000,
		mcx514_cmd_write_delay_func, utils::little_endian >,
	std::ratio<mcx514_clock_hz,1>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq3>,
	dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq5>,
	mcx_axis_inputs_func, mcx_axis_outputs_func > mcx51x_t;

  mcx51x_t mcx51x;
  struct mcx51x_inst { constexpr mcx51x_t& operator () (void) const { return inst ().mcx51x; } };

  typedef dev::mcx_axis_inputs<mcx51x_t::axis_t, pca9698_i0_t> mcx_axis_inputs_dev_t;
  typedef dev::mcx_axis_outputs<mcx51x_t::axis_t, pca9698_i1_t> mcx_axis_outputs_dev_t;

  std::array<mcx_axis_inputs_dev_t, mcx51x_t::axis_count> mcx51x_axis_inputs;
  std::array<mcx_axis_outputs_dev_t, mcx51x_t::axis_count> mcx51x_axis_outputs;

  template <unsigned int AxisNum>
  struct mcx51x_axis_inputs_inst { constexpr mcx_axis_inputs_dev_t& operator () (void) const { return inst ().mcx51x_axis_inputs[AxisNum]; } };

  template <unsigned int AxisNum>
  struct mcx51x_axis_outputs_inst { constexpr mcx_axis_outputs_dev_t& operator () (void) const { return inst ().mcx51x_axis_outputs[AxisNum]; } };

#endif

#if defined (MCB_USE_MCX514_I2C_EXT)
  // some more MCX514 connected via SCI5 I2C expansion bus.
  // the reference to 'external_i2c' is specified in the constructor.
  // the devices will be probed during initialization.  if there's a timeout,
  // the corresponding device will be marked as offline.
  // official max I2C speed for MCX is 1 Mbps.  but it seems to run OK
  // at 1.5 Mbps.

  // officially, I2C addresses 0..7 are reserved.  thus we can't use MCX
  // address 0.  if the expansion board also has the PCAs, the addresses 0x40
  // and 0x42 are reserved for those, which is the MCX address 0x02.  thus
  // can't use MCX address 2 either.

  typedef dev::mcx51x::mcx514_hw_inst < dev::mcx51x::if_i2c < 0x01, 1'500'000 >, 16'000'000,
		dev::interrupt::unconnected, dev::interrupt::unconnected > mcx514_e0_t;

  typedef dev::mcx51x::mcx514_hw_inst < dev::mcx51x::if_i2c < 0x03, 1'500'000 >, 16'000'000,
		dev::interrupt::unconnected, dev::interrupt::unconnected > mcx514_e1_t;

  typedef dev::mcx51x::mcx514_hw_inst < dev::mcx51x::if_i2c < 0x04, 1'500'000 >, 16'000'000,
		dev::interrupt::unconnected, dev::interrupt::unconnected > mcx514_e2_t;

  mcx514_e0_t mcx514_e0;

#endif

#if defined (MCB_USE_PCA9698_I2C_EXT)
  // on the extension board, can't use the interrupt line.
  typedef dev::pca9698 < 0x40, 1'500'000,
			 dev::interrupt::unconnected > pca9698_e0_t;

  typedef dev::pca9698 < 0x42, 1'500'000,
			 dev::interrupt::unconnected > pca9698_e1_t;


  pca9698_e0_t pca9698_e0;
  pca9698_e1_t pca9698_e1;
#endif


#if defined (MCB_USE_FCU)
  using fcu_t =dev::rx63_fcu;
  fcu_t fcu;
  struct fcu_inst { constexpr fcu_t& operator () (void) const { return inst ().fcu; } };

  auto& data_flash (void) { return fcu.data_flash_dev (); }

#endif


#if defined (MCB_USE_TPU)
  tpu0_t tpu0;
  tpu1_t tpu1;
  tpu2_t tpu2;
  tpu3_t tpu3;
  tpu4_t tpu4;
  tpu5_t tpu5;
#endif


#if !defined (MCB_NO_CRC)
  using crc_t = dev::rx63_crc;
  crc_t crc;
  struct crc_inst { constexpr crc_t& operator () (void) const { return inst ().crc; } };
#endif

#if defined (MCB_USE_ETHERC) || defined (MCB_USE_ETHERC_RAW)

  typedef dev::rx_etherc_mdio_sta < 0x000C0120, pclock_hz * 2, 5 > rx_etherc_mdio_sta_t;
  rx_etherc_mdio_sta_t mdio_sta;

  typedef dev::rx_etherc::hw_inst < 0x000C0100, bclk_hz > rx_etherc_t;

  typedef dev::rx_edmac::hw_inst <
		0x000C0000, bclk_hz, rx_etherc_t,
		dev::rx63_interrupt::line<dev::rx63_interrupt::ether_eint>,
		dev::interrupt::virtual_line<0>,
		dev::rx_mstpcrb_bit<15>
	> rx_edmac_t;

  typedef dev::lan8710 < 0, rx_edmac_t::link_status_interrupt_line > lan8710_t;

  using eth0_phy_t = lan8710_t;
  using eth0_mac_t = rx_edmac_t;

  rx_etherc_t eth0_etherc;
  struct eth0_etherc_inst { constexpr rx_etherc_t& operator () (void) const { return inst ().eth0_etherc; } };

  eth0_phy_t eth0_phy;
  struct eth0_phy_isnt { constexpr eth0_phy_t& operator () (void) const { return inst ().eth0_phy; } };

  eth0_mac_t eth0_mac;
  struct eth0_mac_inst { constexpr eth0_mac_t& operator () (void) const { return inst ().eth0_mac; } };

#endif

#if defined (MCB_USE_ETHERC) && !defined (MCB_USE_ETHERC_RAW)
  typedef dev::rx_etherc_eth < rx_edmac_t, lan8710_t, board_info_eth0_addr > eth0_t;
  eth0_t eth0;

  struct eth0_inst { constexpr eth0_t& operator () (void) const { return inst ().eth0; } };
#endif


#if defined (MCB_USE_DAC124S085)
  #if !defined (MCB_USE_SPI)
    #error DAC124S085 needs internal SPI
  #endif

  // there are 3 DAC124S085 and their CS signals are connected as:
  //   DAC_CS1N: PB6
  //   DAC_CS2N: PB5
  //   DAC_CS3N: PB4
  //
  // max speed for each device is about 30 MHz

  // if DTC is used for SCI SPI data transfers, it can be setup in such a
  // way that it automatically sets the CS signal on as a prefix transfer and
  // sets it off as a suffix transfer.

  // the SCI (SPI) device should provide an interface for doing prefix and
  // suffix data transfers in a transparent way.  if it uses DTC, then it can
  // do that transparently.

  // the CS signal enable/disable is a device which can be asked whether it can
  // work as part of a DTC transfer chain.  for some GPIO pin allocations it
  // might be possible, for some it might not and the device's CPI function
  // has to be used.

  // in this particular case, using DTC will not work because of the
  // shared output ports.

  typedef dev::dac124s085 <
	dev::rx_shared_output_port < dev::rx_podr::pb, 6 >, 30'000'000> dac0_t;

  typedef dev::dac124s085 <
	dev::rx_shared_output_port < dev::rx_podr::pb, 5 >, 30'000'000> dac1_t;

  typedef dev::dac124s085 <
	dev::rx_shared_output_port < dev::rx_podr::pb, 4 >, 30'000'000> dac2_t;

  dac0_t dac0;
  struct dac0_inst { constexpr dac0_t& operator () (void) const { return inst ().dac0; } };

  dac1_t dac1;
  struct dac1_inst { constexpr dac1_t& operator () (void) const { return inst ().dac1; } };

  dac2_t dac2;
  struct dac2_inst { constexpr dac2_t& operator () (void) const { return inst ().dac2; } };

#endif


  // MCB specific devices

#if defined (MCB_USE_TRIGGER_IO)

  using trigger_inputs_t = dev::trigger_inputs;
  trigger_inputs_t trigger_inputs;
  struct trigger_inputs_inst { constexpr trigger_inputs_t& operator () (void) const { return inst ().trigger_inputs; } };

  using trigger_outputs_t = dev::trigger_outputs;
  trigger_outputs_t trigger_outputs;
  struct trigger_outputs_inst { constexpr trigger_outputs_t& operator () (void) const { return inst ().trigger_outputs; } };

#endif


#if defined (MCB_USE_DIPSWITCH)
  using dipswitch_inputs_t = dev::dipswitch_inputs < pca9698_i1_t >;
  dipswitch_inputs_t dipswitch_inputs;
  struct dipswitch_inputs_inst { constexpr dipswitch_inputs_t& operator () (void) const { return inst ().dipswitch_inputs; } };
#endif


#if defined (MCB_USE_DIGITAL_IO)
  using digital_inputs_t = dev::digital_inputs< pca9698_i0_t >;
  digital_inputs_t digital_inputs;
  struct digital_inputs_inst { constexpr digital_inputs_t& operator () (void) const { return inst ().digital_inputs; } };

  using digital_outputs_t = dev::digital_outputs < pca9698_i1_t >;
  digital_outputs_t digital_outputs;
  struct digital_outputs_inst { constexpr digital_outputs_t& operator () (void) const { return inst ().digital_outputs; } };
#endif



  devices_end_t devices_end;
};

template <> inline void mcb_v1_board::cpu_wait_3_iclocks<1> (void)
{
  [[gnu::unused]] uint16_t val = *((volatile uint16_t*)0x00080000);
}

template <> inline void mcb_v1_board::cpu_wait_3_iclocks<0> (void) { }

#if defined (MCB_USE_PCD4641)
inline auto&& mcb_v1_board::pcd_axis_inputs_axis_func::operator () (const void* thizz)
{
  auto& a = mcb_v1_board::inst ().pcd4641.axis (0);
  auto& ai = a.inputs ();

  unsigned int off = (uintptr_t)&ai - (uintptr_t)&a;
  return *(pcd4641_t::axis_t*)((uintptr_t)thizz - off);
}

inline auto&& mcb_v1_board::pcd_axis_outputs_axis_func::operator () (const void* thizz)
{
  auto& a = mcb_v1_board::inst ().pcd4641.axis (0);
  auto& ao = a.outputs ();

  unsigned int off = (uintptr_t)&ao - (uintptr_t)&a;
  return *(pcd4641_t::axis_t*)((uintptr_t)thizz - off);
}

inline unsigned int mcb_v1_board::pcd_axis_num_func::operator () (const void* axis_ptr)
{
  auto& a0 = mcb_v1_board::inst ().pcd4641.axis (0);

  return ((uintptr_t)axis_ptr - (uintptr_t)&a0) / a0.this_size ();
};
#endif


#if defined (MCB_USE_MCX514)
template <typename Mcx, typename McxAxis>
inline auto&& mcb_v1_board::mcx_axis_inputs_func::operator () ([[gnu::unused]] Mcx& m, McxAxis& a)
{
  return (*((std::array<mcx_axis_inputs_dev_t, mcx51x_t::axis_count>*)inputs))[a.num ()];
}

template <typename Mcx, typename McxAxis>
inline auto&& mcb_v1_board::mcx_axis_outputs_func::operator () ([[gnu::unused]] Mcx& m, McxAxis& a)
{
  return (*((std::array<mcx_axis_outputs_dev_t, mcx51x_t::axis_count>*)outputs))[a.num ()];
}
#endif


namespace this_board
{
using type = mcb_v1_board;
inline constexpr mcb_v1_board& inst (void) { return mcb_v1_board::inst (); }

}

#endif // __cplusplus
#endif // includeguard_board_hpp_includeguard
