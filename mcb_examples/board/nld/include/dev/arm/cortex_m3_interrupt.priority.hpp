
/*

STM32F051 priority mapping

it uses 2 bits for the priority levels
(#define __NVIC_PRIO_BITS 2)

priority levels are 0-192 in 64 steps: { 0, 64, 128, 192 }.
priority 0 is the highest level priority.

map this to 16 priority levels where priority 15 has the highest priority.

*/

constexpr inline unsigned int remap_priority (unsigned int val)
{
  // remap priority number [0 ... 15] -> [ 192 ... 0 ]

  // it will use only bits [7:6] and ignore the writes of any other bits,
  // so don't bother to apply a bitmask on the result.

  return ~(val * 16);
}

