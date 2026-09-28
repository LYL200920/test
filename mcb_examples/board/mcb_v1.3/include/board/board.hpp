#ifndef includeguard_board_hpp_includeguard
#define includeguard_board_hpp_includeguard
#ifdef __cplusplus

#include <board/board_clk.hpp>
#include <board/reset_source.hpp>

#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

#include <dev/rx63_interrupt.hpp>
#include <dev/rx63_fcu.hpp>

#include <dev/rx_scic.hpp>
#include <dev/rx_edmac.hpp>

#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
      || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

// the RX64 and RX71 peripherals are compatible.

#include "dev/rx64_interrupt.hpp" // use local include
#include <dev/rx64_fcu.hpp>

#include <dev/rx_scig.hpp>
#include <dev/rx_scifa.hpp>
#include <dev/rx_edmaca.hpp>
#include <dev/rx_edmac.hpp>
#include <dev/rx_tpua.hpp>

#else
  #error board variant not selected
#endif

#include <dev/rx_sci_usart.hpp>
#include <dev/rx_scif_usart.hpp>
#include <dev/rx_sci_i2c.hpp>
#include <dev/rx_sci_spi.hpp>

// the etherc units are the same on RX63, RX64, RX71
#include <dev/rx_etherc.hpp>
#include <dev/rx_etherc_mdio_sta.hpp>
#include <dev/rx_etherc_eth.hpp>

// the CRC unit is the same on RX63, RX64, RX71
#include <dev/rx63_crc.hpp>

// the CMT that we use as a system timer is register compatible on
// RX63, RX64 and RX71
#include <dev/rx63_cmt_timer.hpp>

// module stop registers
#include <dev/rx_module_stop_regs.hpp>

// IO ports
#include <dev/rx_gpio.hpp>

#include <dev/pcd4641.hpp>
#include <dev/mcx51x.hpp>

// the LAN8720 is register compatible with LAN8710, it's just the RMII-only
// version of the same device.
#include <dev/lan8710.hpp>

#include <dev/lan9250.hpp>
#include <dev/lan9250_eth.hpp>

#include <dev/pca9698.hpp>
#include <dev/dac124s085.hpp>

#include "dev/led_outputs.hpp"
#include "dev/digital_io.hpp"
#include "dev/trigger_io.hpp"
#include "dev/dipswitch_inputs.hpp"
#include "dev/mcx_axis_io.hpp"
#include "dev/pcd_axis_io.hpp"


class mcb_v13_board : public mcb_v13_board_clk
{
private:
  // initialze board.  invoked by startup code before main.
  // the board initialization order is controlled by the members in the board
  // class below.
  mcb_v13_board (void);

  static mcb_v13_board g_inst;

  struct reset_hw_init_t { reset_hw_init_t (mcb_v13_board& brd); };
  struct release_peripheral_reset_t { release_peripheral_reset_t (mcb_v13_board& brd); };

  struct devices_begin_t { };
  struct devices_end_t { };

  struct board_info_eth0_addr { const std::array<uint8_t, 6> operator () (void); };
  struct board_info_eth1_addr { const std::array<uint8_t, 6> operator () (void); };
  struct board_info_eth2_addr { const std::array<uint8_t, 6> operator () (void); };


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
  #endif

public:

  static constexpr mcb_v13_board& inst (void) { return g_inst; }

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

  typedef dev::rx63_cmt_timer <pclkb_hz, system_clock_frequency_hz > system_timer_t;
  system_timer_t system_timer;

  // the rx63_cmt_timer references RX63 interrupt numbers.
  // luckily CMT0_CMI and CMT1_CMI numbers are the same for RX63 and RX64/RX71.
  #if defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) \
      || defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)
  static_assert ((int)dev::rx63_interrupt::cmt0_cmi == (int)dev::rx64_interrupt::cmt0_cmi, "");
  static_assert ((int)dev::rx63_interrupt::cmt1_cmi == (int)dev::rx64_interrupt::cmt1_cmi, "");
  #endif

  release_peripheral_reset_t release_peripheral_reset;

#if defined (MCB_USE_LEDS)
  using led_outputs_t = dev::led_outputs;
  led_outputs_t led_outputs;
#endif


// debugging SCI1 in usart mode
#if defined (MCB_USE_SCI_DEBUG)
  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)

    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A020, 1, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_tei>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci1_eri>,
			dev::interrupt::unconnected,
			dev::rx_mstpcrb_bit<30>
		     >, 32, 32 > debug_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_sci_usart <
// 7 Mbps might be a bit too fast for this
//	dev::rx_scig < 0x8A020, 1, pclkb_hz, pclkb_hz / 8,
//	dev::rx_scig < 0x8A020, 1, pclkb_hz, pclkb_hz / 16,
	dev::rx_scig < 0x8A020, 1, pclkb_hz, 3'000'000,
//	dev::rx_scig < 0x8A020, 1, pclkb_hz, 115200,	// 115200 bps works with 120 and 96 mhz bus clock
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci1_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci1_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci1_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci1_eri>,
			dev::rx_mstpcrb_bit<30>
		     >, 32, 32 > debug_usart_t;
  #endif

  debug_usart_t debug_usart;

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
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci0_eri>,
			dev::rx_mstpcrb_bit<31>
		     >, 32, 32 > gpsi0_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A000, 0, pclkb_hz, pclkb_hz / 8,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci0_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci0_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci0_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci0_eri>,
			dev::rx_mstpcrb_bit<31>
		     >, 32, 32 > gpsi0_usart_t;
  #endif

  gpsi0_usart_t gpsi0_usart;

#endif


#if defined (MCB_USE_SCI3_USART)

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A060, 3, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci3_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci3_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci3_tei>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci3_eri>,
			dev::rx_mstpcrb_bit<28>
		     >, 32, 32 > gpsi1_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A060, 3, pclkb_hz, pclkb_hz / 8,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci3_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci3_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci3_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci3_eri>,
			dev::rx_mstpcrb_bit<28>
		     >, 32, 32 > gpsi1_usart_t;
  #endif

  gpsi1_usart_t gpsi1_usart;

#endif


#if defined (MCB_USE_SCI5_USART)

// FIXME: SCI5 can also use RTS and CTS, have to configure that somehow....

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A0A0, 5, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci5_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci5_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci5_tei>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci5_eri>,
			dev::rx_mstpcrb_bit<26>
		     >, 32, 32 > gpsi2_usart_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A0A0, 5, pclkb_hz, pclkb_hz / 8,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci5_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci5_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci5_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci5_eri>,
			dev::rx_mstpcrb_bit<26>
		     >, 32, 32 > gpsi2_usart_t;
  #endif

  gpsi2_usart_t gpsi2_usart;

#endif


#if defined (MCB_USE_RS485)

   typedef dev::gpio_transceiver<dev::rx_gpio::exclusive_output_port< decltype (dev::rx_gpio::podr::pa), 0xFF, 0x00 >> rs485_ch1_tx_en_t; // PA
   typedef dev::gpio_transceiver<dev::rx_gpio::exclusive_output_port< decltype (dev::rx_gpio::podr::pj), 0xFF, 0x00 >> rs485_ch2_tx_en_t; // PJ

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A0C0, 6, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_tei>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci6_eri>,
			dev::rx_mstpcrb_bit<25>
		     >, 32, 32, rs485_ch1_tx_en_t > rs485_ch1_t;

    typedef dev::rx_sci_usart <
	dev::rx_scic < 0x8A0E0, 7, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci7_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci7_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci7_tei>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci7_eri>,
			dev::rx_mstpcrb_bit<24>
		     >, 32, 32, rs485_ch2_tx_en_t > rs485_ch2_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144)
    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A0C0, 6, pclkb_hz, pclkb_hz / 8,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_eri>,
			dev::rx_mstpcrb_bit<25>
		     >, 32, 32, rs485_ch1_tx_en_t > rs485_ch1_t;

    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A0E0, 7, pclkb_hz, pclkb_hz / 8,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci7_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci7_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci7_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci7_eri>,
			dev::rx_mstpcrb_bit<24>
		     >, 32, 32, rs485_ch2_tx_en_t > rs485_ch2_t;

  #elif defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    typedef dev::rx_sci_usart <
	dev::rx_scig < 0x8A0C0, 6, pclkb_hz, pclkb_hz / 8,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci6_eri>,
			dev::rx_mstpcrb_bit<25>
		     >, 32, 32, rs485_ch1_tx_en_t > rs485_ch1_t;

    // that's 120 / 8 = 16 mbps for SCIFA10.  not sure if this will actually work.
    typedef dev::rx_scif_usart <
	dev::rx_scifa < 0xD0040, 10, pclka_hz, pclka_hz / 8,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa10_bri>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa10_eri>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa10_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa10_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa10_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::scifa10_dri>,
			dev::rx_mstpcrc_bit<25>
		      >, 32, 32, rs485_ch2_tx_en_t > rs485_ch2_t;
  #endif

  rs485_ch1_t rs485_ch1;
  rs485_ch2_t rs485_ch2;

#endif


#if defined (MCB_USE_I2C)

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    // 1.3 MHz
    typedef dev::rx_sci_i2c_master <
	dev::rx_scic < 0x8A040, 2, pclkb_hz, pclkb_hz / 16,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_txi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_rxi>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_tei>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci2_eri>,
			dev::rx_mstpcrb_bit<29>
		     >> internal_i2c_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    // (pclkb = 48 mhz, div = 16) OK (1.3 MHz)
    // (pclkb = 48 mhz, div = 8) can be set but will not go any faster than div = 16.
    // (pclkb = 60 mhz, div = 16) OK (1.6 MHz)
    typedef dev::rx_sci_i2c_master <
	dev::rx_scig < 0x8A040, 2, pclkb_hz, pclkb_hz / 16,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci2_txi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci2_rxi>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci2_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci2_eri>,
			dev::rx_mstpcrb_bit<29>
		     >> internal_i2c_t;
  #endif

  internal_i2c_t internal_i2c;
#endif


#if defined (MCB_USE_SPI)

  // the SPI port is a write-only port.  hence receive related interrupts are
  // not connected.

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    // 12 MHz
    typedef dev::rx_sci_spi_master <
	dev::rx_scic < 0x8A080, 4, pclkb_hz, pclkb_hz / 4,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_txi>,
			dev::interrupt::unconnected,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_tei>,
			dev::rx63_interrupt::line<dev::rx63_interrupt::sci4_eri>,
			dev::rx_mstpcrb_bit<27>
		     >> internal_spi_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144) \
        || defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
    // 18 MHz
    typedef dev::rx_sci_spi_master <
	dev::rx_scig < 0x8A080, 4, pclkb_hz, pclkb_hz / 4,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci4_txi>,
			dev::interrupt::unconnected,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci4_tei>,
			dev::rx64_interrupt::line<dev::rx64_interrupt::sci4_eri>,
			dev::rx_mstpcrb_bit<27>
		     >> internal_spi_t;
  #endif

  internal_spi_t internal_spi;
#endif


#if defined (MCB_USE_PCA9698)
  #if !defined (MCB_USE_I2C)
    #error PCA9698 needs internal I2C
  #endif

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq2 > pca9698_i0_irq_line_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) \
		|| defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)
    typedef dev::rx64_interrupt::line<dev::rx64_interrupt::icu_irq2 > pca9698_i0_irq_line_t;

  #endif

  typedef dev::pca9698 < 0x40, internal_i2c_t::max_bitrate, pca9698_i0_irq_line_t > pca9698_i0_t;
  typedef dev::pca9698 < 0x42, internal_i2c_t::max_bitrate, dev::interrupt::unconnected > pca9698_i1_t;

  pca9698_i0_t pca9698_i0;
  pca9698_i1_t pca9698_i1;
#endif

#if defined (MCB_USE_DAC124S085)
  #if !defined (MCB_USE_SPI)
    #error DAC124S085 needs internal SPI
  #endif

  // there are 3 DAC124S085 and their CS signals are connected as:
  //   DAC_CS1N: PB7
  //   DAC_CS2N: PB6
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
  // might be possible, for some it might not and the device's CPU function
  // has to be used.

  typedef dev::dac124s085 <
	dev::rx_exclusive_output_port < dev::rx_podr::pb, ~(1 << 7), 0xFF >,
	30'000'000> > dac0_t;

  typedef dev::dac124s085 <
	dev::rx_exclusive_output_port < dev::rx_podr::pb, ~(1 << 6), 0xFF >,
	30'000'000> > dac1_t;

  typedef dev::dac124s085 <
	dev::rx_exclusive_output_port < dev::rx_podr::pb, ~(1 << 4), 0xFF >,
	30'000'000> > dac2_t;

  dac0_t dac0;
  dac1_t dac1;
  dac2_t dac2;

#endif


#if defined (MCB_USE_PCD4641)

  // PCD4641 is connected to CS3 and CS4 (8 bit), with a twiddled address
  // interface and clocked variably by the MTIOC2A output.

  struct pcd4641_cmd_write_delay_func { void operator () (void) { }; };

  using pcd4641_bus_interface =
	dev::pcd4641::if_a0a1_twiddled_parallel8 <
		0x04000000, 0x05000000, pcd4641_cmd_write_delay_func, utils::big_endian >;

  // the pcd_axis_inputs and pcd_axis_outputs need the pcd axis instances
  // for initialization.  thus, we put them in the board class after the
  // pcd instance and the pcd hw_inst gets only functors to access the
  // actual input/output devices.
  struct pcd_axis_inputs_func
  {
    // because of circular dependencies, have to use void*.
    void* inputs;

    template <typename Pcd, typename PcdAxis> auto&& operator () (Pcd& m, PcdAxis& a);
  };

  struct pcd_axis_outputs_func
  {
    void* outputs;

    template <typename Pcd, typename PcdAxis> auto&& operator () (Pcd& m, PcdAxis& a);
  };

  typedef dev::pcd4641::hw_inst <
	pcd4641_bus_interface, pcd4641_clock_hz,

	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
	  dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq7>,
	#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) \
		|| defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)
	  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_irq7>,
	#endif

	pcd_axis_inputs_func, pcd_axis_outputs_func
  > pcd4641_t;

  pcd4641_t pcd4641;

  typedef dev::pcd_axis_inputs<pcd4641_t::axis_t> pcd_axis_inputs_dev_t;
  typedef dev::pcd_axis_outputs<pcd4641_t::axis_t> pcd_axis_outputs_dev_t;

  std::array<pcd_axis_inputs_dev_t, dev::pcd4641::axis_count> pcd4641_axis_inputs;
  std::array<pcd_axis_outputs_dev_t, dev::pcd4641::axis_count> pcd4641_axis_outputs;
#endif


#if defined (MCB_USE_MCX514)

  // MCX514 is connected to CS1 and CS2 (16 bit) and clocked variably by
  // (MTIOC3C XOR MTIOC3D) output.
  // additional delay cycles for command writes are done by separate CS area
  // and timings.

  struct mcx514_cmd_write_delay_func { void operator () (void) { }; };

  using mcx514_bus_interface = dev::mcx51x::if_parallel16 <
	0x06000000, 0x07000000,	mcx514_cmd_write_delay_func, utils::little_endian >;

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

  typedef dev::mcx51x::mcx514_hw_inst <
	mcx514_bus_interface,	mcx514_clock_hz,

	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
	  dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq3>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::icu_irq5>,
	#elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX71M_144)
	  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_irq3>,
	  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_irq5>,
	#elif defined (MCB_USE_RX64M_176) || defined (MCB_USE_RX71M_176)
	  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_irq3>,
	  dev::rx64_interrupt::line<dev::rx64_interrupt::icu_irq0>,
	#endif

	mcx_axis_inputs_func, mcx_axis_outputs_func

   > mcx51x_t;

  mcx51x_t mcx51x;

  typedef dev::mcx_axis_inputs<mcx51x_t::axis_t> mcx_axis_inputs_dev_t;
  typedef dev::mcx_axis_outputs<mcx51x_t::axis_t, pca9698_i0_t, pca9698_i1_t> mcx_axis_outputs_dev_t;

  std::array<mcx_axis_inputs_dev_t, mcx51x_t::axis_count> mcx51x_axis_inputs;
  std::array<mcx_axis_outputs_dev_t, mcx51x_t::axis_count> mcx51x_axis_outputs;

#endif


#if defined (MCB_USE_FCU)

  #if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
    typedef dev::rx63_fcu fcu_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) \
        || defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)
    typedef dev::rx64_fcu fcu_t;

  #endif

  fcu_t fcu;

  auto& data_flash (void) { return fcu.data_flash_dev (); }

#endif

#if defined (MCB_USE_TPU)
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
      auto& en_bits = mcb_v13_board::inst ().tpu_en_bits ();
      en_bits[TpuChannelNumber] = val;

      dev::rx_mstpcra_bit<13> () (en_bits.any ());
    }
  };

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088110, 0, 0x00088100, 0x00088101, 0x00088108, pclkb_hz,
	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu0_tgia>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu0_tgib>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu0_tgic>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu0_tgid>,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#else
	  // FIXME: need to setup RX64 / RX71 interrupt tables first ...
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#endif

	tpu_module_enable_func<0> > tpu0_t;
  tpu0_t tpu0;

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088120, 1, 0x00088100, 0x00088101, 0x00088109, pclkb_hz,
	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu1_tgia>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu1_tgib>,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#else
	  // FIXME: need to setup RX64 / RX71 interrupt tables first ...
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#endif

	tpu_module_enable_func<1> > tpu1_t;
  tpu1_t tpu1;

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088130, 2, 0x00088100, 0x00088101, 0x0008810A, pclkb_hz,
	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu2_tgia>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu2_tgib>,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#else
	  // FIXME: need to setup RX64 / RX71 interrupt tables first ...
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#endif

	tpu_module_enable_func<2> > tpu2_t;
  tpu2_t tpu2;

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088140, 3, 0x00088100, 0x00088101, 0x0008810B, pclkb_hz,
	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu3_tgia>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu3_tgib>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu3_tgic>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu3_tgid>,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#else
	  // FIXME: need to setup RX64 / RX71 interrupt tables first ...
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#endif

	tpu_module_enable_func<3> > tpu3_t;
  tpu3_t tpu3;

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088150, 4, 0x00088100, 0x00088101, 0x0008810C, pclkb_hz,
	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu4_tgia>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu4_tgib>,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#else
	  // FIXME: need to setup RX64 / RX71 interrupt tables first ...
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#endif

	tpu_module_enable_func<4> > tpu4_t;
  tpu4_t tpu4;

  typedef dev::rx_tpua::hw_chn_inst <
	0x00088160, 5, 0x00088100, 0x00088101, 0x0008810D, pclkb_hz,
	#if defined (MCB_USE_RX631_144) || defined (MCB_USE_RX63N_144)
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu5_tgia>,
	  dev::rx63_interrupt::line<dev::rx63_interrupt::tpu5_tgib>,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#else
	  // FIXME: need to setup RX64 / RX71 interrupt tables first ...
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	  dev::interrupt::unconnected,
	#endif

	tpu_module_enable_func<5> > tpu5_t;
  tpu5_t tpu5;
#endif




#if !defined (MCB_NO_CRC)

  // RX63, RX64 and RX71 all have the same CRC unit.
  typedef dev::rx63_crc crc_t;
  crc_t crc;
#endif


#if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC1)

  #if defined (MCB_USE_RX63N_144)
    typedef dev::rx_etherc_mdio_sta < 0x000C0120, iclk_hz, 5 > mdio_sta_t;

  #elif defined (MCB_USE_RX64M_144) || defined (MCB_USE_RX64M_176) \
        || defined (MCB_USE_RX71M_144) || defined (MCB_USE_RX71M_176)

    typedef dev::rx_etherc_mdio_sta < 0x000C0120, pclka_hz, 9 > mdio_sta_t;
  #endif

  mdio_sta_t mdio_sta;

#endif

#if defined (MCB_USE_RX_ETHERC0) || defined (MCB_USE_RX_ETHERC0_RAW)

  #if defined (MCB_USE_RX63N_144)

    typedef dev::rx_etherc::hw_inst < 0x000C0100, pclka_hz > rx_etherc0_t;
    typedef dev::rx_edmac < 0x000C0000, plcka, rx_etherc0_t,
			    dev::rx63_interrupt::line<dev::rx63_interrupt::ether_eint>
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
  eth0_mac_t eth0_mac;
  eth0_phy_t eth0_phy;

  #if !defined (MCB_USE_RX_ETHERC0_RAW)
    typedef dev::rx_etherc_eth < rx_edmac0_t, lan8720_0_t, board_info_eth0_addr > eth0_t;
    eth0_t eth0;
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
  eth1_mac_t eth1_mac;
  eth1_phy_t eth1_phy;

  #if !defined (MCB_USE_RX_ETHERC1_RAW)

    typedef dev::rx_etherc_eth < rx_edmac1_t, lan8720_1_t, board_info_eth1_addr > eth1_t;
    eth1_t eth1;
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
  eth2_phy_t eth2_phy;

  #if !defined (MCB_USE_LAN9250_RAW)

    typedef dev::lan9250_eth < lan9250_t, lan9250_t::phy_t, board_info_eth2_addr > eth2_t;
    eth2_t eth2;
  #endif

#endif


  // MCB specific devices

#if defined (MCB_USE_TRIGGER_IO)

  using trigger_inputs_t = dev::trigger_inputs;
  trigger_inputs_t trigger_inputs;

  using trigger_outputs_t = dev::trigger_outputs;
  trigger_outputs_t trigger_outputs;

#endif


#if defined (MCB_USE_DIGITAL_IO)
  using digital_inputs_t = dev::digital_inputs< pca9698_i0_t >;
  digital_inputs_t digital_inputs;

  using digital_outputs_t = dev::digital_outputs < pca9698_i1_t >;
  digital_outputs_t digital_outputs;
#endif


#if defined (MCB_USE_DIPSWITCH)
  using dipswitch_inputs_t = dev::dipswitch_inputs;
  dipswitch_inputs_t dipswitch_inputs;
#endif

  devices_end_t devices_end;
};

template <> inline void mcb_v13_board::cpu_wait_3_iclocks<1> (void)
{
  // accessing the SYSTEM register is 3 ICLK on RX63, RX64 and RX71.
  [[gnu::unused]] uint16_t val = *((volatile uint16_t*)0x00080000);
}

template <> inline void mcb_v13_board::cpu_wait_3_iclocks<0> (void) { }

#if defined (MCB_USE_PCD4641)
template <typename Pcd, typename PcdAxis>
inline auto&& mcb_v13_board::pcd_axis_inputs_func::operator () ([[gnu::unused]] Pcd& m, PcdAxis& a)
{
  return (*((std::array<pcd_axis_inputs_dev_t, dev::pcd4641::axis_count>*)inputs))[a.num ()];
}

template <typename Pcd, typename PcdAxis>
inline auto&& mcb_v13_board::pcd_axis_outputs_func::operator () ([[gnu::unused]] Pcd& m, PcdAxis& a)
{
  return (*((std::array<pcd_axis_outputs_dev_t, dev::pcd4641::axis_count>*)outputs))[a.num ()];
}
#endif


#if defined (MCB_USE_MCX514)
template <typename Mcx, typename McxAxis>
inline auto&& mcb_v13_board::mcx_axis_inputs_func::operator () ([[gnu::unused]] Mcx& m, McxAxis& a)
{
  return (*((std::array<mcx_axis_inputs_dev_t, mcx51x_t::axis_count>*)inputs))[a.num ()];
}

template <typename Mcx, typename McxAxis>
inline auto&& mcb_v13_board::mcx_axis_outputs_func::operator () ([[gnu::unused]] Mcx& m, McxAxis& a)
{
  return (*((std::array<mcx_axis_outputs_dev_t, mcx51x_t::axis_count>*)outputs))[a.num ()];
}
#endif

namespace this_board
{
using type = mcb_v13_board;
inline constexpr mcb_v13_board& inst (void) { return mcb_v13_board::inst (); }

#if defined (MCB_USE_LEDS)
struct led_outputs_fn { auto& operator () (void) const { return inst ().led_outputs; } };
#endif

#if defined (MCB_USE_TRIGGER_IO)
struct trigger_inputs_fn { auto& operator () (void) const { return inst ().trigger_inputs; } };
struct trigger_outputs_fn { auto& operator () (void) const { return inst ().trigger_outputs; } };
#endif

#if defined (MCB_USE_DIPSWITCH)
struct dipswitch_inputs_fn { auto& operator () (void) const { return inst ().dipswitch_inputs; } };
#endif

#if defined (MCB_USE_DIGITAL_IO)
struct digital_inputs_fn { auto& operator () (void) const { return inst ().digital_inputs; } };
struct digital_outputs_fn { auto& operator () (void) const { return inst ().digital_outputs; } };
#endif

}

#endif // __cplusplus
#endif // includeguard_board_hpp_includeguard
