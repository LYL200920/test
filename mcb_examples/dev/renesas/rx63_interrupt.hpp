
#ifndef includeguard_dev_rx63_interrupt_hpp_includeguard
#define includeguard_dev_rx63_interrupt_hpp_includeguard

#include <cstdint>
#include <dev/interrupt.hpp>
#include <dev/hwreg.hpp>

namespace dev
{
namespace rx63_interrupt
{

enum isr_num_t
{
  // artificial numbers which are in the fixed vector table.
  privileged_instruction_exception = -4,
  access_exception = -4,
  undefined_instruction_exception = -3,
  fpu_exception = -2,
  nmi = -1,

  // relocatable vector table numbers
  reserved_0 = 0,
  reserved_1,
  reserved_2,
  reserved_3,
  reserved_4,
  reserved_5,
  reserved_6,
  reserved_7,
  reserved_8,
  reserved_9,
  reserved_10,
  reserved_11,
  reserved_12,
  reserved_13,
  reserved_14,
  reserved_15,
  bsc_buserr = 16,	// level triggered, IER02.IEN0, IPR000
  reserved_17,
  reserved_18,
  reserved_19,
  reserved_20,

  fcu_fiferr = 21,	// level triggered, IER02.IEN5, IPR001
  reserved_22,
  fcu_frdyi = 23,	// edge triggered, IER02.IEN7, IPR002

  reserved_24,
  reserved_25,
  reserved_26,

  icu_swint = 27,	// edge triggered, IER03.IEN3, IPR003, DCTER027

  cmt0_cmi = 28,	// edge triggered, IER03.IEN4, IPR004, DCTER028
  cmt1_cmi = 29,	// edge triggered, IER03.IEN5, IPR005, DCTER029
  cmt2_cmi = 30,	// edge triggered, IER03.IEN6, IPR006, DCTER030
  cmt3_cmi = 31,	// edge triggered, IER03.IEN7, IPR007, DCTER031

  ether_eint = 32,	// level triggered, IER04.IEN0, IPR032

  usb0_d0fifo = 33,	// edge triggered, IER04.IEN1, IPR033, DTCER033
  usb0_d1fifo = 34,	// edge triggered, IER04.IEN2, IPR034, DTCER034
  usb0_usbi = 35,	// edge triggered, IER04.IEN3

  usb1_d0fifo = 36,	// edge triggered, IER04.IEN4, IPR036, DTCER036
  usb1_d1fifo = 37,	// edge triggered, IER04.IEN5, IPR037, DTCER037
  usb1_usbi = 38,	// edge triggered, IER04.IEN6

  rspi0_spri = 39,	// edge triggered, IER04.IEN7, IPR039, DTCER039
  rspi0_spti = 40,	// edge triggered, IER05.IEN0, IPR039, DTCER040
  rspi0_spii = 41,	// level triggered, IER05.IEN1, IPR039

  rspi1_spri = 42,	// edge triggered, IER05.IEN2, IPR042, DTCER042
  rspi1_spti = 43,	// edge triggered, IER05.IEN3, IPR042, DTCER043
  rspi1_spii = 44,	// level triggered, IER05.IEN4, IPR042

  rspi2_spri = 45,	// edge triggered, IER05.IEN5, IPR045, DTCER045
  rspi2_spti = 46,	// edge triggered, IER05.IEN6, IPR045, DTCER046
  rspi2_spii = 47,	// level triggered, IER05.IEN7, IPR045

  can0_rxf = 48,	// edge triggered, IER06.IEN0, IPR048
  can0_txf = 49,	// edge triggered, IER06.IEN1, IPR048
  can0_rxm = 50,	// edge triggered, IER06.IEN2, IPR048
  can0_txm = 51,	// edge triggered, IER06.IEN3, IPR048

  can1_rxf = 52,	// edge triggered, IER06.IEN4, IPR052
  can1_txf = 53,	// edge triggered, IER06.IEN5, IPR052
  can1_rxm = 54,	// edge triggered, IER06.IEN6, IPR052
  can1_txm = 55,	// edge triggered, IER06.IEN7, IPR052

  can2_rxf = 56,	// edge triggered, IER07.IEN0, IPR056
  can2_txf = 57,	// edge triggered, IER07.IEN1, IPR056
  can2_rxm = 58,	// edge triggered, IER07.IEN2, IPR056
  can2_txm = 59,	// edge triggered, IER07.IEN2, IPR056

  reserved_60,
  reserved_61,

  rtc_cup = 62,		// edge triggered, IER07.IEN6, IPR062

  reserved_63,

  icu_irq0 = 64,	// edge/level triggered, IER08.IEN0, IPR064
  icu_irq1 = 65,	// edge/level triggered, IER08.IEN1, IPR065
  icu_irq2 = 66,	// edge/level triggered, IER08.IEN2, IPR066
  icu_irq3 = 67,	// edge/level triggered, IER08.IEN3, IPR067
  icu_irq4 = 68,	// edge/level triggered, IER08.IEN4, IPR068
  icu_irq5 = 69,	// edge/level triggered, IER08.IEN5, IPR069
  icu_irq6 = 70,	// edge/level triggered, IER08.IEN6, IPR070
  icu_irq7 = 71,	// edge/level triggered, IER08.IEN7, IPR071
  icu_irq8 = 72,	// edge/level triggered, IER09.IEN0, IPR072
  icu_irq9 = 73,	// edge/level triggered, IER09.IEN1, IPR073
  icu_irq10 = 74,	// edge/level triggered, IER09.IEN2, IPR074
  icu_irq11 = 75,	// edge/level triggered, IER09.IEN3, IPR075
  icu_irq12 = 76,	// edge/level triggered, IER09.IEN4, IPR076
  icu_irq13 = 77,	// edge/level triggered, IER09.IEN5, IPR077
  icu_irq14 = 78,	// edge/level triggered, IER09.IEN6, IPR078
  icu_irq15 = 79,	// edge/level triggered, IER09.IEN7, IPR079

  reserved_80,
  reserved_81,
  reserved_82,
  reserved_83,
  reserved_84,
  reserved_85,
  reserved_86,
  reserved_87,
  reserved_88,
  reserved_89,

  usb_usbr0 = 90,	// level triggered, IER0B.IEN2, IPR090
  usb_usbr1 = 91,	// level triggered, IER0B.IEN3, IPR091

  rtc_alm = 92,		// edge triggered, IER0B.IEN4, IPR092
  rtc_prd = 93,		// edge triggered, IER0B.IEN5, IPR093

  reserved_94,
  reserved_95,
  reserved_96,
  reserved_97,

  ad0_ad = 98,		// edge triggered, IER0C.IEN2, IPR098, DTCER098
  reserved_99,
  reserved_100,
  reserved_101,

  s12ad_s12adi0 = 102,	// edge triggered, IER0C.IEN6, IPR102, DTCER102
  reserved_103,
  reserved_104,
  reserved_105,

  icu_group0 = 106,	// level, IER0D.IEN2, IPR106
  icu_group1 = 107,	// level, IER0D.IEN3, IPR107
  icu_group2 = 108,	// level, IER0D.IEN4, IPR108
  icu_group3 = 109,	// level, IER0D.IEN5, IPR109
  icu_group4 = 110,	// level, IER0D.IEN6, IPR110
  icu_group5 = 111,	// level, IER0D.IEN7, IPR111
  icu_group6 = 112,	// level, IER0E.IEN0, IPR112

  reserved_113,

  icu_group12 = 114,	// level, IER0E.IEN2, IPR114

  reserved_115,
  reserved_116,
  reserved_117,
  reserved_118,
  reserved_119,
  reserved_120,
  reserved_121,

  sci12_scix0 = 122,	// level, IER0F.IEN2, IPR122
  sci12_scix1 = 123,	// level, IER0F.IEN3, IPR122
  sci12_scix2 = 124,	// level, IER0F.IEN4, IPR122
  sci12_scix3 = 125,	// level, IER0F.IEN5, IPR122

  tpu0_tgia = 126,	// edge, IER0F.IEN6, IPR126
  tpu0_tgib = 127,	// edge, IER0F.IEN7, IPR126
  tpu0_tgic = 128,	// edge, IER10.IEN0, IPR126
  tpu0_tgid = 129,	// edge, IER10.IEN1, IPR126

  tpu1_tgia = 130,	// edge, IER10.IEN2, IPR130
  tpu1_tgib = 131,	// edge, IER10.IEN3, IPR130

  tpu2_tgia = 132,	// edge, IER10.IEN4, IPR132
  tpu2_tgib = 133,	// edge, IER10.IEN5, IPR132

  tpu3_tgia = 134,	// edge, IER10.IEN6, IPR134
  tpu3_tgib = 135,	// edge, IER10.IEN7, IPR134
  tpu3_tgic = 136,	// edge, IER11.IEN0, IPR134
  tpu3_tgid = 137,	// edge, IER11.IEN1, IPR134

  tpu4_tgia = 138,	// edge, IER11.IEN2, IPR138
  tpu4_tgib = 139,	// edge, IER11.IEN3, IPR138

  tpu5_tgia = 140,	// edge, IER11.IEN4, IPR140
  tpu5_tgib = 141,	// edge, IER11.IEN5, IPR140

  tpu6_mtu0_tgia = 142,	// edge, IER11.IEN6, IPR142
  tpu6_mtu0_tgib = 143,	// edge, IER11.IEN7, IPR142
  tpu6_mtu0_tgic = 144,	// edge, IER12.IEN0, IPR142
  tpu6_mtu0_tgid = 145,	// edge, IER12.IEN1, IPR142
  mtu0_tgie = 146,	// edge, IER12.IEN2, IPR146
  mtu0_tgif = 147,	// edge, IER12.IEN3, IPR146

  tpu7_mtu1_tgia = 148,	// edge, IER12.IEN4, IPR148
  tpu7_mtu1_tgib = 149,	// edge, IER12.IEN5, IPR148

  tpu8_mtu2_tgia = 150,	// edge, IER12.IEN6, IPR150
  tpu8_mtu2_tgib = 151,	// edge, IER12.IEN7, IPR150

  tpu9_mtu3_tgia = 152,	// edge, IER13.IEN0, IPR152
  tpu9_mtu3_tgib = 153,	// edge, IER13.IEN1, IPR152
  tpu9_mtu3_tgic = 154,	// edge, IER13.IEN2, IPR152
  tpu9_mtu3_tgid = 155,	// edge, IER13.IEN3, IPR152

  tpu10_mtu4_tgia = 156,	// edge, IER13.IEN4, IPR156
  tpu10_mtu4_tgib = 157,	// edge, IER13.IEN5, IPR156

  mtu4_tgic = 158,	// edge, IER13.IEN6, IPR156
  mtu4_tgid = 159,	// edge, IER13.IEN7, IPR156
  mtu4_tciv = 160,	// edge, IER14.IEN0, IPR160

  mtu5_tgiu = 161,
  mtu5_tgiv = 162,
  mtu5_tgiw = 163,

  tpu11_tgia = 164,
  tpu11_tgib = 165,

  poe_oei1 = 166,
  poe_oei2 = 167,

  reserved_168,
  reserved_169,

  tmr0_cmia = 170,
  tmr0_cmib = 171,
  tmr0_ovi = 172,

  tmr1_cmia = 173,
  tmr1_cmib = 174,
  tmr1_ovi = 175,

  tmr2_cmia = 176,
  tmr2_cmib = 177,
  tmr2_ovi = 178,

  tmr3_cmia = 179,
  tmr3_cmib = 180,
  tmr3_ovi = 181,

  riic0_eei = 182,
  riic0_rxi = 183,
  riic0_txi = 184,
  riic0_tei = 185,

  riic1_eei = 186,
  riic1_rxi = 187,
  riic1_txi = 188,
  riic1_tei = 189,

  riic2_eei = 190,
  riic2_rxi = 191,
  riic2_txi = 192,
  riic2_tei = 193,

  riic3_eei = 194,
  riic3_rxi = 195,
  riic3_txi = 196,
  riic3_tei = 197,

  dmac0i = 198,
  dmac1i = 199,
  dmac2i = 200,
  dmac3i = 201,

  exdmac0i = 202,
  exdmac1i = 203,

  reserved_204,
  reserved_205,

  deu0 = 206,
  deu1 = 207,

  pdc_pcdfi = 208,
  pdc_pcfei = 209,
  pdc_pceri = 210,

  reserved_211,
  reserved_212,
  reserved_213,

  sci0_rxi = 214,
  sci0_txi = 215,
  sci0_tei = 216,

  sci1_rxi = 217,
  sci1_txi = 218,
  sci1_tei = 219,

  sci2_rxi = 220,
  sci2_txi = 221,
  sci2_tei = 222,

  sci3_rxi = 223,
  sci3_txi = 224,
  sci3_tei = 225,

  sci4_rxi = 226,
  sci4_txi = 227,
  sci4_tei = 228,

  sci5_rxi = 229,
  sci5_txi = 230,
  sci5_tei = 231,

  sci6_rxi = 232,
  sci6_txi = 233,
  sci6_tei = 234,

  sci7_rxi = 235,
  sci7_txi = 236,
  sci7_tei = 237,

  sci8_rxi = 238,
  sci8_txi = 239,
  sci8_tei = 240,

  sci9_rxi = 241,
  sci9_txi = 242,
  sci9_tei = 243,

  sci10_rxi = 244,
  sci10_txi = 245,
  sci10_tei = 246,

  sci11_rxi = 247,
  sci11_txi = 248,
  sci11_tei = 249,

  sci12_rxi = 250,
  sci12_txi = 251,
  sci12_tei = 252,

  ieb_iebint = 253,

  reserved_254,
  reserved_255
};

static constexpr unsigned int count = 256;

template <isr_num_t IsrNum> struct line
{
  static constexpr unsigned int isr_num = IsrNum;

  // the first interrupt that can trigger the DTC is SWINT (27) and hence in
  // the hardware manual the base register for the DTCER array starts at 0x0008711B.
  static constexpr hw_reg_rw<uint8_t, const_addr<0x00087100 + isr_num>> DTCER = { };

  static void connect_func (void(*)(void))
  {
    // RX interrupt functions are collected and put into the ISR table during
    // compile time.  thus this function does nothing.
  }

  static void enable (interrupt::trigger_type tt, unsigned int priority)
  {
    // N.B. the first 16 vectors are reserved and hence in the hardware
    // manual the base register for the IR register array starts at 0x00087010.
    *(volatile int8_t*)(0x00087000 + isr_num) = 0;

    if (isr_num >= icu_irq0 && isr_num <= icu_irq15)
    {
      int8_t val;
      switch (tt)
      {
      default: val = 0; break;
      case interrupt::low_level: val = 0; break;
      case interrupt::falling_edge: val = 1 << 2; break;
      case interrupt::rising_edge: val = 2 << 2; break;
      case interrupt::any_edge: val = 3 << 2; break;
      }

      // ICU.IRQCRi
      *(volatile int8_t*)(0x00087500 + (isr_num - icu_irq0)) = val;
    }

    // ICU.IPRn
    constexpr unsigned int ipr_n = ipr_isr_num ();
    *(volatile int8_t*)(0x00087300 + ipr_n) = priority & 0x0F;

    // ICU.IERm.IENj
    {
      auto m = isr_num / 8u;
      auto n = isr_num % 8u;

      // N.B. the first 16 vectors are reserved and hence in the hardware
      // manual the base register for the imask bit array starts at 0x00087202.
      *(volatile int8_t*)(0x00087200 + m) |= 1 << n;
    }
  }

  static void disable (void)
  {
    // ICU.IERm.IENj
    {
      auto m = isr_num / 8u;
      auto n = isr_num % 8u;

      // N.B. the first 16 vectors are reserved and hence in the hardware
      // manual the base register of the imask bit array starts at 0x00087202.
      *(volatile int8_t*)(0x00087200 + m) &= 0xFF ^ (1 << n);
    }

    *(volatile int8_t*)(0x00087000 + isr_num) = 0;
  }

  static void enable_dtc (void)
  {
    if (isr_num < 256)
      DTCER = 1;
  }

  static void disable_dtc (void)
  {
    if (isr_num < 256)
      DTCER = 0;
  }

  static bool status (void)
  {
    return *(volatile int8_t*)(0x00087000 + isr_num) & 1;
  }

  static void clear (void)
  {
    *(volatile int8_t*)(0x00087000 + isr_num) = 0;
  }

  static constexpr unsigned int ipr_isr_num (void)
  {
    // see "Table 15.3  Interrupt Vector Table" in the hardware manual.
    switch (isr_num)
    {
      default: return isr_num;

      case 16: return 0;
      case 21: return 1;
      case 23: return 2;
      case 27: return 3;
      case 28: return 4;
      case 29: return 5;
      case 30: return 6;
      case 31: return 7;
      case 39: case 40: case 41: return 39;
      case 42: case 43: case 44: return 42;
      case 45: case 46: case 47: return 45;
      case 48: case 49: case 50: case 51: return 48;
      case 52: case 53: case 54: case 55: return 52;
      case 56: case 57: case 58: case 59: return 56;
      case 122: case 123: case 124: case 125: return 122;
      case 126: case 127: case 128: case 129: return 126;
      case 130: case 131: return 130;
      case 132: case 133: return 132;
      case 134: case 135: case 136: case 137: return 134;
      case 138: case 139: return 138;
      case 140: case 141: return 140;
      case 142: case 143: case 144: case 145: return 142;
      case 146: case 147: return 146;
      case 148: case 149: return 148;
      case 150: case 151: return 150;
      case 152: case 153: case 154: case 155: return 152;
      case 156: case 157: case 158: case 159: return 156;
      case 161: case 162: case 163: return 161;
      case 164: case 165: return 164;
      case 166: case 167: return 166;
      case 170: case 171: case 172: return 170;
      case 173: case 174: case 175: return 173;
      case 176: case 177: case 178: return 176;
      case 179: case 180: case 181: return 179;
      case 214: case 215: case 216: return 214;
      case 217: case 218: case 219: return 217;
      case 220: case 221: case 222: return 220;
      case 223: case 224: case 225: return 223;
      case 226: case 227: case 228: return 226;
      case 229: case 230: case 231: return 229;
      case 232: case 233: case 234: return 232;
      case 235: case 236: case 237: return 235;
      case 238: case 239: case 240: return 238;
      case 241: case 242: case 243: return 241;
      case 244: case 245: case 246: return 244;
      case 247: case 248: case 249: return 247;
      case 250: case 251: case 252: return 250;
    }
  }
};

} // namespace rx63_interrupt
} // namespace dev

#endif // includeguard_dev_rx63_interrupt_hpp_includeguard
