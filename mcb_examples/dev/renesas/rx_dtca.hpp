/*

RX MCU DTCa device

the DTC device executes memory copy scripts which can be triggered by
other hardware peripherals via interrupts.  the interrupt controller has a
setting, which allows redirecting an interrupt request to the DTC as an
activation trigger instead of sending it to the CPU.  it will trigger the
DTC which will perform one or more transfer instructions.  when the transfer

the DTC can be used to implement complex tasks to offload the CPU and improve
latency.  some applications are:

- sending data packets to SCI devices with automatic prefix and suffix
  data transfers to enable/disable external transceivers

- playing back signal sequences (precomputed data stream in RAM)
  at fixed or variable frequencies.  for that, a timer is used as a driving
  pulse for the data transfers.  to achieve variable frequencies, the DTC
  can re-write timer registers (trigger points, period).

  one use case is playback of signal sequences for external IOs

  another use case is clocked bitbang serial communication, like MDIO on
  Renesas EtherC devices.

- self modifying transfer lists
  if the DTC interrupt table is located in RAM, a DTC script can re-write the
  entry points of DTC scripts, inlcuding itself.  this allows for complex
  sequences like looping through an array of pointers, "do once" patterns
  and so on.


although the DTC can only copy memory, that can be used to write constants
as well.  this is done by building a constant pool as a side data of the
instruction list.

the DTCa normally runs off the ICLK, i.e. it's as fast as the CPU.
however, every DTC transfer instruction has an overhead of instruction read
and optional write-back.
for example, a chain transfer of 2 transfers = 26 DTC cycles.


the DTCa allows short-address and full-address descriptors, but that setting
is global.  short-address mode allows access to on-chip resources like
RAM, peripherals and ROM.  short-address instructions are 12 bytes long , while
full-address instructions are 16 bytes long.  the shorter instructions are
faster as they save 1 setup cycle (1 memory read less).  whether full-address
mode is required or not depends on the system setup.  if there are no devices
in the external address space, which should be accessed by the DTC, then
short-address mode can be used.

one disadvantage of short addresses is that the DTC scripts can't be
constructed at compile time, because the addresses are not stored as-is, i.e.
sequences can't be put into ROM and let the linker fill in the address blanks.

to allow system wide selection of the DTC address mode, it is made an
template parameter of the hardware instance.

DTC instruction class synopsis:

class insn  // name is implementation defined.
            // it is typedef'ed by the board class.  refer to it by
            //   this_board::type::dtc_t::insn
{
public:
  // automatically deduces element size from T
  template <typename T>
  static insn fixed_addr_copy (const T* src, T* dst);

  insn& set_src_addr (const void* p);
  insn& set_src_addr (uintptr_t p);

  constexpr addressing_mode src_addr_mode (void) const;
  insn& set_src_addr_mode (addressing_mode val);

  constexpr enum element_size element_size (void) const;
  insn& set_element_size (enum element_size val);

  constexpr enum transfer_mode transfer_mode (void) const;
  insn& set_transfer_mode (enum transfer_mode val);

  insn& set_dst_addr (const void* p);
  insn& set_dst_addr (uintptr_t p);

  constexpr addressing_mode dst_addr_mode (void) const;
  insn& set_dst_addr_mode (addressing_mode val);

  constexpr bool src_area_repeat (void) const;
  insn& set_src_area_repeat (bool val);

  constexpr bool dst_area_repeat (void) const;
  insn& set_dst_area_repeat (bool val);

  constexpr bool cpu_irq_at_every_transfer (void) const;
  insn& set_cpu_irq_at_every_transfer (bool val);

  constexpr enum chain_mode chain_mode (void) const;
  insn& set_chain_mode (enum chain_mode val);


  // count setting for normal transfer types
  constexpr uint32_t normal_transfer_count (void) const;
  insn& set_normal_transfer_count (uint32_t val);


  // count setting for repeat transfer type
  constexpr unsigned int repeat_transfer_count (void) const;
  insn& set_repeat_transfer_count (unsigned int val);

  constexpr unsigned int repeat_transfer_current_count (void) const;
  insn& set_repeat_transfer_current_count (unsigned int val);


  // count setting for block transfer type
  constexpr unsigned int block_transfer_count (void) const;
  insn& set_block_transfer_count (unsigned int val);

  constexpr unsigned int block_transfer_current_count (void) const;
  insn& set_block_transfer_current_count (unsigned int val);

  constexpr uint32_t block_count (void) const;
  insn& set_block_count (uint32_t val);
};


---------------------
the following are some ideas how to make DTC scripts easier to work with
by adding a script compiling library on top of the plain insn array.


---------------------
// an example script to copy one variable to another.
// the variables could also be peripheral registers.

uint32_t value_a;
uint32_t value_b;

void test (void)
{
  auto&& dtc = board.dtc;

  // the board's dtc type will instantiate the appropriate insn formats
  // depending on the system's dtc address size setting.

  auto script = dtc.make_script (
    dtc.copy (&value_a, &value_b) // default parameters automatically derived
  );
}

---------------------
// example to set one variable to a constant value.

uint32_t value_a;

void test (void)
{
  auto&& dtc = this_board::inst ().dtc;

  auto script = dtc.make_script (
    dtc.copy (1234, &value_a)
  );
}

---------------------
// example to fill an array with a constant value.
// all array parameters for the copy instructions will result
// in block transfers.
// the destination address will be incremented automatically.

uint32_t values[40];

void test (void)
{
  auto&& dtc = this_board::inst ().dtc;

  auto script = dtc.make_script (
    dtc.copy (1234, values)	// array type and size automatically derived.
  );

  auto script = dtc.make_script (
    dtc.copy (1234, values, 40) // array size manually specified.
  );

  auto script = dtc.make_script (
    dtc.copy (1234, values, dtc.element_32bit, dtc.post_inc); // specify all parameters
}

---------------------
// multiple instructions per script
// this will automatically create chained transfers

uint32_t values[40];
uint8_t something_a;
uint16_t something_b;

void test (void)
{
  auto&& dtc = this_board::inst ().dtc;

  auto script = dtc.make_script (
    dtc.copy (0x60, &something_a),
    dtc.copy (0x10, &something_b),
    dtc.copy (1, values)
  );
}

---------------------
// interrupt requests
// the last transfer (count = 0) will always generate an interrupt to the CPU.
// it is possible to request an interrupt for each transfer trigger.

uint32_t values[40];
uint8_t something_a;
uint16_t something_b;

void test (void)
{
  auto&& dtc = this_board::inst ().dtc;

  auto script = dtc.make_script (
    dtc.copy (0x60, &something_a),
    dtc.copy (0x10, &something_b),
    dtc.copy (1, values),
    dtc.irq ()		// irq will be generated always after the last transfer
  );
}


---------------------
// do_once, do_last

std::vector<char> buffer;

void test (void)
{
  auto&& dtc = this_board::inst ().dtc;

  auto script = dtc.make_script (

    // executed only for the first trigger
    dtc.do_once (dtc.make_script (
	dct.copy (1, &some_port_register)
    )),

    // executed for every trigger
    // copy specified array, 1 element at a time.
    dtc.copy_1 ({ buffer.data (), buffer.size () } , &sci_data_register),

    // executed only for the last trigger
    dtc.do_last (dtc.make_script (
	dtc.copy (0, &some_port_register)
    ))
  );

  // alternatively (unified initialization constructor calls):

  auto script = dtc.script
  {
    // executed only for the first trigger
    dtc.do_once
    {
      dct.copy (1, &some_port_register)
    },

    // executed for every trigger
    // copy specified array, 1 element at a time.
    dtc.copy_1 ({ buffer.data (), buffer.size () } , &sci_data_register),

    // executed only for the last trigger
    dtc.do_last
    {
      dtc.copy (0, &some_port_register)
    }
  };
}


the do_once will inject a dtc copy transfer to modify the script's start
address to point to the address following the do_once contents.
the script's start address is stored in the DTC vector table and when building
the script instruction, the address is not known.  it is known only when
assigning the script to one entry in the vector table.

also, the final address of the instruction following the do_once block is only
known once the script is at the final RAM location.

to implement that, relocations and symbolic addresses are used in the
intermediate script representation.  when binding the script to the DTC vector
table, it is compiled and the symbolic addresses are resolved and replaced
with constant values.

a symbolic address can be in the constant pool of the script or it can be
in an instruction itself (absolute addressing).

because script compilation (and address resolution) takes processing time,
it is not feasible to re-compile the script every time it is used.  instead,
the script is precompiled with placeholder variables, which allows replacing
the values in those variables after the script has been compiled.

for example:

  std::vector<char> buffer;

  auto script = dtc.script
  {
    dtc.do_once
    {
      dct.copy (1, &some_port_register) // address resolved during script compile
    },

    dtc.copy_1 ({ dtc.var<0>, dtc.var<1>, dtc.element_8bit, dtc.post_inc },
		&sci_data_register),

    dtc.do_last
    {
      dtc.copy (0, &some_port_register)
    }
  };

  // bind and compile the script.
  // however, it does not actually activate the script
  script.bind (some_interrupt_line);

  // change variables in the copmiled script dynamically:
  script.var<0> = buffer.data ();
  script.var<1> = buffer.size ();

  // activate the script
  // this just sets the address of the script in the DTC vector table.
  // because the script has been bound, it already knows the vector table
  // entry number (and its location in RAM) and dtc instance.
  script.activate ();


this could also be improved to derive things automatically, like array type.

this is implemented by synthesizing setter functions which are called when
a variable in a compiled script is being assigned.  the setter function
then knows which instruction to modify and how to modify it.

because scripts can contain only absolute addresses, copying a bound script
in RAM is possible, but needs fix-up of addresses that refer to the script
data itself (mainly references to the contant pool).  this is done by
keeping a list of self-referencing addresses and re-writing them when the
script is copied.  a simpler way is to prohibit copying of a bound script but
that seems unpractical.

the bound script is a different type than the intermediate script type.
because the bound script does not contain a lot of meta-data that is needed
for script compilation, it consumes less memory.  usually applications should
keep a copy of a compiled bound script, not the intermediate script.


this is useful when implementing SCI SPI.
before every SPI transaction,
the SS signal needs to be set.  after the transaction it has to be cleared.
if the MCU pin and port allocation is done accordingly, this can be accomplished
by writing 0xFF or 0x00 to a data port.  for this, do_once and do_last can
be used.
there can be many devices connected to the same SCI SPI device.  every device
has its own way how to assert and deassert the SS line.  this part of the script
is always fixed.  the transfer destination address and all the other paramters
are also fixed.  the only thing that needs to be updated in the script is
the source address and the transfer counter.

to reduce the DTC script setup time to a minimum, the SPI device asks the
SPI master for a script during device initialization:

  // the SPI device will create a dtc script from the individual
  // script pieces and might make other specific changes to it.
  // because the SPI device does not know which interrupt line the SPI master
  // device uses, the SPI master will return a bound script.

  spi_device::spi_device ( ... )
  {
    auto tx_script = m_spi->make_dtc_script (
	dtc::do_once { m_csline.set_dtc_script () },
	m_spi->dtc_tx_script (),
	dtc::do_last { m_csline.clear_dtc_script () }
    );

    // initialize command queue.  the script is copied and
    // automatically re-bound.
    m_cmd_queue.reserve (queue_size);
    for (int i = 0; i < queue_size; ++i)
      m_cmd_queue.push_back ({tx_script});

    // set the command completion functions.
    // it will put the command object back into the command queue.
    for (auto& c : m_cmd_queue)
      c.on_completion = [this, c] () { this->free_cmd (c); };
  }

  // when the SPI device wants to send data, it uses the bound dtc script
  // and sets the data buffer variables in the script.
  std::future<send_result> spi_device::send (const std::shared_ptr<buffer>& buf)
  {
    // there are 2 options here ... either block until there is one element
    // in the cmd queue to be removed, or return nullptr and fail.
    auto* cmd = alloc_cmd ();

    cmd->buf = buf;
    cmd->dtc_script.set_var<0> (buf->data);
    cmd->dtc_script.set_var<1> (buf->size);

    // the user can explicitly wait for the send_result future if it's needed.
    // the additional callback function will be called when the send command
    // has completed but before setting the promise in the future.
    return m_spi->send (cmd);
  }


  // to start the send command, the SPI device only has to activate the
  // dtc script (which just sets its address in the vector table)
  // and enable SCI transmission.


---------------------

---------------------

---------------------

---------------------

*/

#ifndef includeguard_dev_rx_dtca_includeguard
#define includeguard_dev_rx_dtca_includeguard

#include <cstdint>
#include <dev/hwreg.hpp>
#include <type_traits>
#include <array>

namespace dev
{
namespace dtca
{

enum addressing_mode
{
  fixed = 0b00,
  post_inc = 0b10,
  post_dec = 0b11
};

enum element_size
{
  element_8bit = 0b00,
  element_16bit = 0b01,
  element_32bit = 0b10
};

template <typename T, typename En = void> struct get_element_size;

template <typename T> struct get_element_size<T, typename std::enable_if< sizeof (T) == 1 >::type>
{
  static constexpr auto value = element_8bit;
};

template <typename T> struct get_element_size<T, typename std::enable_if< sizeof (T) == 2 >::type>
{
  static constexpr auto value = element_16bit;
};

template <typename T> struct get_element_size<T, typename std::enable_if< sizeof (T) == 4 >::type>
{
  static constexpr auto value = element_32bit;
};

enum transfer_mode
{
  normal = 0b00,
  repeat = 0b01,
  block = 0b10
};

enum chain_mode
{
  no_chain = 0b00,
  always_chain = 0b10,
  conditional_chain = 0b11
};

/*
struct address_desc
{
  addressing_mode mode;
  element_size size;
  uintptr_t addr;
};
*/

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// a single transfer instruction (short address mode)
// the 24 bit address is sign extended to get the full 32 bit address.
// very clever memory map of the RX.
class insn_short
{
public:
  template <typename T, typename U>
  static constexpr typename std::enable_if_t<
	std::is_same<std::remove_cv_t<T>, std::remove_cv_t<U>>::value, insn_short>
  fixed_addr_copy (const T* src, U* dst)
  {
    return insn_short ().set_src_addr (src).set_src_addr_mode (fixed)
			.set_dst_addr (dst).set_dst_addr_mode (fixed)
			.set_element_size (get_element_size<T>::value)
			.set_transfer_mode (normal)
			.set_normal_transfer_count (1)
			.set_chain_mode (no_chain);
  }

  template <typename T, typename U>
  static constexpr typename std::enable_if_t<
	std::is_same<std::remove_cv_t<T>, std::remove_cv_t<U>>::value, insn_short>
  fixed_addr_copy (insn_short tmpl, const T* src, U* dst)
  {
    return tmpl
	.set_src_addr (src).set_src_addr_mode (fixed)
	.set_dst_addr (dst).set_dst_addr_mode (fixed)
	.set_element_size (get_element_size<T>::value);
  }

  constexpr insn_short& set_src_addr (const void* p) { return set_src_addr ((uintptr_t)p); }
  constexpr insn_short& set_src_addr (const volatile void* p) { return set_src_addr ((uintptr_t)p); }

  constexpr addressing_mode src_addr_mode (void) const
  {
    return (addressing_mode)((m_mra_sar >> 26) & 0b11);
  }

  constexpr insn_short& set_src_addr_mode (addressing_mode val)
  {
    m_mra_sar = (m_mra_sar & ~(0b11 << 26)) | ((uint32_t)val << 26);
    return *this;
  }

  constexpr enum element_size element_size (void) const
  {
    return (enum element_size)((m_mra_sar >> 28) & 0b11);
  }

  constexpr insn_short& set_element_size (enum element_size val)
  {
    m_mra_sar = (m_mra_sar & ~(0b11 << 28)) | ((uint32_t)val << 28);
    return *this;
  }

  template <typename T> constexpr insn_short& set_element_size (void)
  {
    return set_element_size (get_element_size<T>::value);
  }

  constexpr enum transfer_mode transfer_mode (void) const
  {
    return (enum transfer_mode)(m_mra_sar >> 30);
  }

  constexpr insn_short& set_transfer_mode (enum transfer_mode val)
  {
    m_mra_sar = (m_mra_sar & ~(0b11 << 30)) | ((uint32_t)val << 30);
    return *this;
  }

  constexpr insn_short& set_dst_addr (void* p) { return set_dst_addr ((uintptr_t)p); }
  constexpr insn_short& set_dst_addr (volatile void* p) { return set_dst_addr ((uintptr_t)p); }

  constexpr addressing_mode dst_addr_mode (void) const
  {
    return (addressing_mode)((m_mrb_dar >> 26) & 0b11);
  }

  constexpr insn_short& set_dst_addr_mode (addressing_mode val)
  {
    m_mrb_dar = (m_mrb_dar & ~(0b11 << 26)) | ((uint32_t)val << 26);
    return *this;
  }

  // if dst is repeat/block area the dst addr will be rewound
  // if src is repeat/block area the src addr will be rewound
  constexpr bool src_area_repeat (void) const { return (m_mrb_dar & (1 << 28)) != 0; }
  constexpr insn_short& set_src_area_repeat (bool val = true)
  {
    m_mrb_dar = (m_mrb_dar & ~(1 << 28)) | ((uint32_t)val << 28);
    return *this;
  }

  constexpr bool dst_area_repeat (void) const { return !src_area_repeat (); }
  constexpr insn_short& set_dst_area_repeat (bool val = true) { return set_src_area_repeat (!val); }


  constexpr bool cpu_irq_at_every_transfer (void) const { return (m_mrb_dar & (1 << 29)) != 0; }
  constexpr insn_short& set_cpu_irq_at_every_transfer (bool val = true)
  {
    m_mrb_dar = (m_mrb_dar & ~(1 << 29)) | ((uint32_t)val << 29);
    return *this;
  }

  // conditional chain transfer.  the following transfer is executed only
  // if the transfer counter changes from 1 to 0 or from 1 to CRAH (when it
  // the transfer counter is reloaded in repeat mode).
  constexpr enum chain_mode chain_mode (void) const
  {
    return (enum chain_mode)(m_mrb_dar >> 30);
  }

  constexpr insn_short& set_chain_mode (enum chain_mode val)
  {
    m_mrb_dar = (m_mrb_dar & ~(0b11 << 30)) | ((uint32_t)val << 30);
    return *this;
  }

  // depending on the selected transfer type, use the according function
  // to set the transfer counters.

  // .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  // normal transfer mode
  // transfer counter max value = 65536
  constexpr uint32_t normal_transfer_count (void) const
  {
    return (m_cra_crb >> 16) == 0 ? 65536 : (m_cra_crb >> 16);
  }
  constexpr insn_short& set_normal_transfer_count (uint32_t val)
  {
    m_cra_crb = (m_cra_crb & 0x0000FFFF) | ((val) << 16);
    return *this;
  }

  // .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  // repeat transfer mode
  // transfer counter max value = 256
  constexpr unsigned int repeat_transfer_count (void) const
  {
    return (m_cra_crb >> 24) == 0 ? 256 : (m_cra_crb >> 24);
  }
  constexpr insn_short& set_repeat_transfer_count (unsigned int val)
  {
    m_cra_crb = (m_cra_crb & 0x00FFFFFF) | (((uint32_t)val) << 24);
    return *this;
  }

  // the current transfer counter variable that is decremented at each
  // transfer and reloaded from the above transfer count.  it's possible
  // to set the initial repeat count to a lower/higher value...
  constexpr unsigned int repeat_transfer_current_count (void) const
  {
    return ((m_cra_crb >> 16) & 0xFF) == 0 ? 256 : ((m_cra_crb >> 16) & 0xFF);
  }
  constexpr insn_short& set_repeat_transfer_current_count (unsigned int val)
  {
    m_cra_crb = (m_cra_crb & 0xFF00FFFF) | ((((uint32_t)val) & 0xFF) << 16);
    return *this;
  }

  // .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  // block transfer mode

  constexpr unsigned int block_transfer_count (void) const
  {
    return (m_cra_crb >> 24) == 0 ? 256 : (m_cra_crb >> 24);
  }
  constexpr insn_short& set_block_transfer_count (unsigned int val)
  {
    m_cra_crb = (m_cra_crb & 0x00FFFFFF) | (((uint32_t)val) << 24);
    return *this;
  }

  constexpr unsigned int block_transfer_current_count (void) const
  {
    return ((m_cra_crb >> 16) & 0xFF) == 0 ? 256 : ((m_cra_crb >> 16) & 0xFF);
  }
  constexpr insn_short& set_block_transfer_current_count (unsigned int val)
  {
    m_cra_crb = (m_cra_crb & 0xFF00FFFF) | ((((uint32_t)val) & 0xFF) << 16);
    return *this;
  }

  constexpr uint32_t block_count (void) const
  {
    return (m_cra_crb & 0xFFFF) == 0 ? 65536 : (m_cra_crb & 0xFFFF);
  }
  constexpr insn_short& set_block_count (uint32_t val)
  {
    m_cra_crb = (m_cra_crb & 0xFFFF0000) | ((val) & 0xFFFF);
    return *this;
  }

private:
  constexpr insn_short& set_src_addr (uintptr_t a)
  {
    m_mra_sar = (m_mra_sar & 0xFF000000) | (a & 0x00FFFFFF);
    return *this;
  }

  constexpr insn_short& set_dst_addr (uintptr_t a)
  {
    m_mrb_dar = (m_mrb_dar & 0xFF000000) | (a & 0x00FFFFFF);
    return *this;
  }


  uint32_t m_mra_sar = 0;	// 8 bit MRA, 24 bit SAR
  uint32_t m_mrb_dar = 0;	// 8 bit MRB, 24 bit DAR
  uint32_t m_cra_crb = 0;	// 16 bit CRA, 16 bit CRB
};

static_assert (sizeof (insn_short) == 12, "");



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// a single transfer instruction (full address mode)

class insn_full
{
public:
  template <typename T, typename U>
  static constexpr typename std::enable_if_t<
	std::is_same<std::remove_cv_t<T>, std::remove_cv_t<U>>::value, insn_full>
  fixed_addr_copy (const T* src, U* dst)
  {
    return insn_full ().set_src_addr (src).set_src_addr_mode (fixed)
			.set_dst_addr (dst).set_dst_addr_mode (fixed)
			.set_element_size (get_element_size<T>::value)
			.set_transfer_mode (normal)
			.set_normal_transfer_count (1)
			.set_chain_mode (no_chain);
  }

  template <typename T, typename U>
  static constexpr typename std::enable_if_t<
	std::is_same<std::remove_cv_t<T>, std::remove_cv_t<U>>::value, insn_full>
  fixed_addr_copy (insn_full tmpl, const T* src, U* dst)
  {
    return tmpl
	.set_src_addr (src).set_src_addr_mode (fixed)
	.set_dst_addr (dst).set_dst_addr_mode (fixed)
	.set_element_size (get_element_size<T>::value);
  }


  constexpr insn_full& set_src_addr (const void* p) { return set_src_addr ((uintptr_t)p); }
  constexpr insn_full& set_src_addr (const volatile void* p) { return set_src_addr ((uintptr_t)p); }

  constexpr addressing_mode src_addr_mode (void) const
  {
    return (addressing_mode)((m_mra_mrb >> 26) & 0b11);
  }
  constexpr insn_full& set_src_addr_mode (addressing_mode val)
  {
    m_mra_mrb = (m_mra_mrb & ~(0b11 << 26)) | ((uint32_t)val << 26);
    return *this;
  }

  constexpr enum element_size element_size (void) const
  {
    return (enum element_size)((m_mra_mrb >> 28) & 0b11);
  }
  constexpr insn_full& set_element_size (enum element_size val)
  {
    m_mra_mrb = (m_mra_mrb & ~(0b11 << 28)) | ((uint32_t)val << 28);
    return *this;
  }

  template <typename T> constexpr insn_full& set_element_size (void)
  {
    return set_element_size (get_element_size<T>::value);
  }

  constexpr enum transfer_mode transfer_mode (void) const
  {
    return (enum transfer_mode)(m_mra_mrb >> 30);
  }
  constexpr insn_full& set_transfer_mode (enum transfer_mode val)
  {
    m_mra_mrb = (m_mra_mrb & ~(0b11 << 30)) | ((uint32_t)val << 30);
    return *this;
  }

  constexpr insn_full& set_dst_addr (void* p) { return set_dst_addr ((uintptr_t)p); }
  constexpr insn_full& set_dst_addr (volatile void* p) { return set_dst_addr ((uintptr_t)p); }

  constexpr addressing_mode dst_addr_mode (void) const
  {
    return (addressing_mode)((m_mra_mrb >> 18) & 0b11);
  }
  constexpr insn_full& set_dst_addr_mode (addressing_mode val)
  {
    m_mra_mrb = (m_mra_mrb & ~(0b11 << 18)) | ((uint32_t)val << 18);
    return *this;
  }

  constexpr bool src_area_repeat (void) const
  {
    return (m_mra_mrb & (1 << 20)) != 0;
  }
  constexpr insn_full& set_src_area_repeat (bool val = true)
  {
    m_mra_mrb = (m_mra_mrb & ~(1 << 20)) | ((uint32_t)val << 20);
    return *this;
  }

  constexpr bool dst_area_repeat (void) const { return !src_area_repeat (); }
  constexpr insn_full& set_dst_area_repeat (bool val = true) { return set_src_area_repeat (!val); }

  constexpr bool cpu_irq_at_every_transfer (void) const
  {
    return m_mra_mrb & (1 << 21);
  }
  constexpr insn_full& set_cpu_irq_at_every_transfer (bool val = true)
  {
    m_mra_mrb = (m_mra_mrb & ~(1 << 21)) | ((uint32_t)val << 21);
    return *this;
  }

  constexpr enum chain_mode chain_mode (void) const
  {
    return (enum chain_mode)((m_mra_mrb >> 22) & 0b11);
  }
  constexpr insn_full& set_chain_mode (enum chain_mode val)
  {
    m_mra_mrb = (m_mra_mrb & ~(0b11 << 22)) | ((uint32_t)val << 22);
    return *this;
  }

  // .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  // normal transfer mode
  // transfer counter max value = 65536
  constexpr uint32_t normal_transfer_count (void) const
  {
    return (m_cra_crb >> 16) == 0 ? 65536 : (m_cra_crb >> 16);
  }
  constexpr insn_full& set_normal_transfer_count (uint32_t val)
  {
    m_cra_crb = (m_cra_crb & 0x0000FFFF) | ((val) << 16);
    return *this;
  }

  // .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  // repeat transfer mode
  // transfer counter max value = 256
  constexpr unsigned int repeat_transfer_count (void) const
  {
    return (m_cra_crb >> 24) == 0 ? 256 : (m_cra_crb >> 24);
  }
  constexpr insn_full& set_repeat_transfer_count (unsigned int val)
  {
    m_cra_crb = (m_cra_crb & 0x00FFFFFF) | (((uint32_t)val) << 24);
    return *this;
  }

  // the current transfer counter variable that is decremented at each
  // transfer and reloaded from the above transfer count.  it's possible
  // to set the initial repeat count to a lower/higher value...
  constexpr unsigned int repeat_transfer_current_count (void) const
  {
    return ((m_cra_crb >> 16) & 0xFF) == 0 ? 256 : ((m_cra_crb >> 16) & 0xFF);
  }
  constexpr insn_full& set_repeat_transfer_current_count (unsigned int val)
  {
    m_cra_crb = (m_cra_crb & 0xFF00FFFF) | ((((uint32_t)val) & 0xFF) << 16);
    return *this;
  }

  // .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
  // block transfer mode

  constexpr unsigned int block_transfer_count (void) const
  {
    return (m_cra_crb >> 24) == 0 ? 256 : (m_cra_crb >> 24);
  }
  constexpr insn_full& set_block_transfer_count (unsigned int val)
  {
    m_cra_crb = (m_cra_crb & 0x00FFFFFF) | (((uint32_t)val) << 24);
    return *this;
  }

  constexpr unsigned int block_transfer_current_count (void) const
  {
    return ((m_cra_crb >> 16) & 0xFF) == 0 ? 256 : ((m_cra_crb >> 16) & 0xFF);
  }
  constexpr insn_full& set_block_transfer_current_count (unsigned int val)
  {
    m_cra_crb = (m_cra_crb & 0xFF00FFFF) | ((((uint32_t)val) & 0xFF) << 16);
    return *this;
  }

  constexpr uint32_t block_count (void) const
  {
    return (m_cra_crb & 0xFFFF) == 0 ? 65536 : (m_cra_crb & 0xFFFF);
  }
  constexpr insn_full& set_block_count (uint32_t val)
  {
    m_cra_crb = (m_cra_crb & 0xFFFF0000) | ((val) & 0xFFFF);
    return *this;
  }

private:
  static_assert (sizeof (uintptr_t) == sizeof (uint32_t), "");

  constexpr insn_full& set_src_addr (uintptr_t p)
  {
    m_sar = p;
    return *this;
  }

  constexpr insn_full& set_dst_addr (uintptr_t p)
  {
    m_dar = p;
    return *this;
  }


  uint32_t m_mra_mrb = 0;	// 8 bit MRA, 8 bit MRB, 16 bit reserved
  uint32_t m_sar = 0;		// 32 bit SAR
  uint32_t m_dar = 0;		// 32 bit DAR
  uint32_t m_cra_crb = 0;	// 16 bit CRA, 16 bit CRB
};

static_assert (sizeof (insn_full) == 16, "");


/*
// a script which can consist of one or more instructions and an array
// for storing constants.
template <typename InsnType, unsigned int InsnCount, unsigned int ConstArraySize>
class script
{
public:

private:
};

*/

// even if there are more than 256 interrupts (cascaded, shared, grouped, ...)
// the DTC vector table is only 256 entries long, since only those interrupts
// can trigger the DTC.
// vector table elements are volatile because they are read by the DTC
// and can also be modified by DTC instructions.

template<typename InsnType>
using vector_table_t = std::array<InsnType* volatile, 256>;


// the DTC vector table could be included as a member of the hw_inst class
// but its 1024 byte alignment constraint results in bloaty code and data layout.
// its more efficient to allocate it elsewhere, giving the compiler/linker more
// freedom where to put it, especially when -fdata-sections is used.
template <typename InsnType, vector_table_t<InsnType>& (*vector_table_inst)(void)>
class hw_inst
{
public:
  using insn = InsnType;
  using vector_table_t = dev::dtca::vector_table_t<InsnType>;

  [[gnu::cold]] hw_inst (void)
  {
    if (std::is_same<InsnType, insn_short>::value)
      DTCADMOD = 1;

    auto table_addr = (uint32_t)&vector_table_inst ().front ();
    assert ((table_addr & 1023) == 0);

    DTCVBR = table_addr;

    // skip/don't skip vector table fetch (default after reset)
    // skipping the vector table fetch can improve performance
    // but is not safe for self-modifying DTC scripts that
    // rewrite the start address.  although, if the last DTC transfer was a
    // chain transfer, the vector table fetch is always done by the next
    // DTC trigger.  usually self-modifying DTC scripts will be some kind of
    // chain-transfer, so it might be OK to permanently enable this...

    // always set bit 3 of DTCCR.  if not set, then conditional chain transfers
    // in repeat mode will not work (chain transfer will never be executed).
    DTCCR = 0 | (1 << 3);
    // DTCCR = 1 | (1 << 3);

    DTCST = 1;
  }

  ~hw_inst (void)
  {
    DTCST = 0;
  }

  // tells whether the DTC is currently active and which vector number it
  // is currently processing.
  // this can be useful when the CPU program needs to modify an entry in
  // a safe way, i.e. with some kind of locking mechanism.
  // the sequence would be
  //   - DTCST = 0     // stop the DTC after the current operation (if any)
  //   - check DTCSTS
  //        if it is equal to the entry the CPU wants to modify, wait
  //   - modify entry
  //   - DTCST = 1     // resume DTC operation
  std::pair<unsigned int, bool> active_vector_number (void)
  {
    const unsigned int r = DTCSTS;
    return { r & 0xFF, (r & (1 << 15)) != 0 };
  }

  // if the current processed vector number is the specified interrupt
  // number, spin wait.
  template <typename InterruptLine>
  void wait_for (void)
  {
    constexpr unsigned int dtcrts_wait_val =
	(1 << 15) | (unsigned int)InterruptLine::isr_num;

    while (true)
    {
      if (DTCSTS != dtcrts_wait_val)
	break;
    }
  }

  template <typename InterruptLine> typename vector_table_t::value_type&
  insns (void)
  {
    auto& vector_table = vector_table_inst ();

    return (unsigned int)InterruptLine::isr_num < vector_table.size ()
	   ? vector_table[InterruptLine::isr_num]
	   : vector_table[0];  // use as dummy entry.  never used by DTC.
  }

  template <typename InterruptLine>
  void enable (bool val = true)
  {
    // invoke enable/disable dtc functions on the interrupt line if they
    // exist.  if they don't exist (e.g. unconnected interrupt), do nothing.
    if (val)
      invoke_enable_dtc<InterruptLine> () ();
    else
      invoke_disable_dtc<InterruptLine> () ();
  }

private:
  static constexpr hw_reg_rw<uint8_t, const_addr<0x00082400>> DTCCR = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00082404>> DTCVBR = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x00082408>> DTCADMOD = { };
  static constexpr hw_reg_rw<uint8_t, const_addr<0x0008240C>> DTCST = { };
  static constexpr hw_reg_r<uint16_t, const_addr<0x0008240E>> DTCSTS = { };

  template <typename InterruptLine> struct invoke_enable_dtc
  {
    template <typename C> static bool test (decltype (&C::enable_dtc));
    template <typename C> static int test (...);

    template <typename IL = InterruptLine>
    typename std::enable_if<sizeof (test<IL> (0)) == sizeof (bool)>::type
    operator () () { IL::enable_dtc (); }

    template <typename IL = InterruptLine>
    typename std::enable_if<sizeof (test<IL> (0)) != sizeof (bool)>::type
    operator () () { }
  };

  template <typename InterruptLine> struct invoke_disable_dtc
  {
    template <typename C> static bool test (decltype (&C::disable_dtc));
    template <typename C> static int test (...);

    template <typename IL = InterruptLine>
    typename std::enable_if<sizeof (test<IL> (0)) == sizeof (bool)>::type
    operator () () { IL::disable_dtc (); }

    template <typename IL = InterruptLine>
    typename std::enable_if<sizeof (test<IL> (0)) != sizeof (bool)>::type
    operator () () { }
  };
};


} // namespace dtca
} // namespace dev
#endif // includeguard_dev_rx_dtca_includeguard

