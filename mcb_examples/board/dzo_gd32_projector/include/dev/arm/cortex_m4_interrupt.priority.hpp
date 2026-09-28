
/*

GD32F30 cortex_m4 priority mapping

it uses 4 bits for the priority levels
(#define __NVIC_PRIO_BITS 4)

priority level of 0-15 for each interrupt
priority 0 is the highest level priority.

map this to 16 priority levels where priority 15 has the highest priority.

*/

constexpr inline unsigned int remap_priority (unsigned int val)
{
  // remap priority number [0 ... 15] -> [ 255 ... 0 ]

  // we use the default value(0x000) of PRIGROUP: Interrupt priority grouping field
  // 4 bits for pre-emption priority 0 bits for subpriority:
  //   - 4-bit Group priorities : 16
  //   - 0-bit Sub priorities   : 0

  // it will use only bits [7:4] and ignore the writes of any other bits,
  // so don't bother to apply a bitmask on the result.
  return ~(val * 16);
}

