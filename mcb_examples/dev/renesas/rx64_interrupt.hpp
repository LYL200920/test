
/*
the RX64 and RX71 interrupt controllers (ICUA) are compatible.

interrupts 128..207 are software configurable and for use of PCLKB peripherals.
interrupts 208..255 are software configurable and for use of PCLKA peripherals.

the assignment of those configurable interrupts is done by the BSP.
for that purpose the BSP must provide an x-macro include file
"rx64_interrupt.x.hpp", which will be included by this file to populate the
isr_num_t enum.

*/

#ifndef includeguard_rx64_interrupt_hpp_includeguard
#define includeguard_rx64_interrupt_hpp_includeguard

#include <cstdint>
#include <dev/hwreg.hpp>
#include <dev/interrupt.hpp>
#include <utils/bits.hpp>

namespace dev
{

namespace rx64_interrupt
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
  ram_ramerr = 18,	// level triggered, IER02.IEN2, IPR000
  reserved_19,
  reserved_20,

  fcu_fiferr = 21,	// level triggered, IER02.IEN5, IPR001
  reserved_22,
  fcu_frdyi = 23,	// edge triggered, IER02.IEN7, IPR002

  reserved_24,
  reserved_25,

  icu_swint2 = 26,	// edge triggered, IER03.IEN2, IPR003, DCTER026
  icu_swint = 27,	// edge triggered, IER03.IEN3, IPR003, DCTER027

  cmt0_cmi = 28,	// edge triggered, IER03.IEN4, IPR004, DCTER028
  cmt1_cmi = 29,	// edge triggered, IER03.IEN5, IPR005, DCTER029
  cmtw0_cmi = 30,	// edge triggered, IER03.IEN6, IPR006, DCTER030
  cmtw1_cmi = 31,	// edge triggered, IER03.IEN7, IPR007, DCTER031

  usba_d0fifo = 32,	// edge triggered, IER04.IEN0, IPR032, DTCER032
  usba_d1fifo = 32,	// edge triggered, IER04.IEN1, IPR033, DTCER033

  usb0_d0fifo = 34,	// edge triggered, IER04.IEN2, IPR034, DTCER034
  usb0_d1fifo = 35,	// edge triggered, IER04.IEN3, IPR035, DTCER035

  reserved_36,
  reserved_37,

  rspi0_spri = 38,	// edge triggered, IER04.IEN6, IPR038, DTCER038
  rspi0_spti = 39,	// edge triggered, IER04.IEN7, IPR039, DTCER039

  reserved_40,
  reserved_41,

  qspi_spri = 42,	// edge triggered, IER05.IEN2, IPR042, DTCER042
  qspi_spti = 43,	// edge triggered, IER05.IEN3, IPR043, DTCER043

  sdhi_sbfai = 44,	// edge triggered, IER05.IEN4, IPR044, DTCER044

  mmcif_mbfai = 45,	// edge triggered, IER05.IEN5, IPR045, DTCER045

  ssi0_ssitxi = 46,	// edge triggered, IER05.IEN6, IPR046, DTCER046
  ssi0_ssirxi = 47,	// edge triggered, IER05.IEN7, IPR047, DTCER047

  ssi1_ssirti = 48,	// edge triggered, IER06.IEN0, IPR048, DTCER048

  reserved_49,

  src_idei = 50,	// edge triggered, IER06.IEN2, IPR050, DTCER050
  src_odfi = 51,	// edge triggered, IER06.IEN3, IPR051, DTCER051

  riic0_rxi = 52,	// edge triggered, IER06.IEN4, IPR052, DTCER052
  riic0_txi = 53,	// edge triggered, IER06.IEN5, IPR053, DTCER053

  riic2_rxi = 54,	// edge triggered, IER06.IEN6, IPR054, DTCER054
  riic2_txi = 55,	// edge triggered, IER06.IEN7, IPR055, DTCER055

  reserved_56,
  reserved_57,

  sci0_rxi = 58,	// edge triggered, IER07.IEN2, IPR058, DTCER058
  sci0_txi = 59,	// edge triggered, IER07.IEN3, IPR059, DTCER059

  sci1_rxi = 60,	// edge triggered, IER07.IEN4, IPR060, DTCER060
  sci1_txi = 61,	// edge triggered, IER07.IEN5, IPR061, DTCER061

  sci2_rxi = 62,	// edge triggered, IER07.IEN6, IPR062, DTCER062
  sci2_txi = 63,	// edge triggered, IER07.IEN7, IPR063, DTCER063

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

  sci3_rxi = 80,	// edge triggered, IER0A.IEN0, IPR080, DTCER080
  sci3_txi = 81,	// edge triggered, IER0A.IEN1, IPR081, DTCER081

  sci4_rxi = 82,	// edge triggered, IER0A.IEN2, IPR082, DTCER082
  sci4_txi = 83,	// edge triggered, IER0A.IEN3, IPR083, DTCER083

  sci5_rxi = 84,	// edge triggered, IER0A.IEN4, IPR084, DTCER084
  sci5_txi = 85,	// edge triggered, IER0A.IEN5, IPR085, DTCER085

  sci6_rxi = 86,	// edge triggered, IER0A.IEN6, IPR086, DTCER086
  sci6_txi = 87,	// edge triggered, IER0A.IEN7, IPR087, DTCER087

  lvd1 = 88,		// edge triggered, IER0B.IEN0, IPR088
  lvd2 = 89,		// edge triggered, IER0B.IEN1, IPR089

  usb0_usbr = 90,	// level triggered, IER0B.IEN2, IPR090

  reserved_91,

  rtc_alm = 92,		// edge triggered, IER0B.IEN4, IPR092
  rtc_prd = 93,		// edge triggered, IER0B.IEN5, IPR093

  usba_usbar = 94,	// level triggered, IER0B.IEN6, IPR094

  iwdt_uni = 95,	// edge triggered, IER0B.IEN7, IPR095

  wdt_uni = 96,		// edge triggered, IER0C.IEN0, IPR096

  pdc_fi = 97,		// edge triggered, IER0C.IEN1, IPR097, DTCER097

  sci7_rxi = 98,	// edge triggered, IER0C.IEN2, IPR098, DTCER098
  sci7_txi = 99,	// edge triggered, IER0C.IEN3, IPR099, DTCER099

  scifa8_rxi = 100,	// edge triggered, IER0C.IEN4, IPR100, DTCER100
  scifa8_txi = 101,	// edge triggered, IER0C.IEN5, IPR101, DTCER101

  scifa9_rxi = 102,	// edge triggered, IER0C.IEN6, IPR102, DTCER102
  scifa9_txi = 103,	// edge triggered, IER0C.IEN7, IPR103, DTCER103

  scifa10_rxi = 104,	// edge triggered, IER0D.IEN0, IPR104, DTCER104
  scifa10_txi = 105,	// edge triggered, IER0D.IEN1, IPR105, DTCER105

  // PCLKB group interrupts
  icu_group_be0 = 106,	// edge triggered, IER0D.IEN2, IPR106
  reserved_107,
  reserved_108,
  reserved_109,
  icu_group_bl0 = 110,	// level triggered, IER0D.IEN6, IPR110
  icu_group_bl1 = 111,	// level triggered, IER0D.IEN7, IPR111


  // PCLKA shared interrupts
  icu_group_al0 = 112,	// level triggered, IER0E.IEN0, IPR112
  icu_group_al1 = 113,	// level triggered, IER0E.IEN1, IPR113

  scifa11_rxi = 114,	// edge triggered, IER0E.IEN2, IPR114, DTCER114
  scifa11_txi = 115,	// edge triggered, IER0E.IEN3, IPR115, DTCER115

  sci12_rxi = 116,	// edge triggered, IER0E.IEN4, IPR116, DTCER116
  sci12_txi = 117,	// edge triggered, IER0E.IEN5, IPR117, DTCER117

  reserved_118,
  reserved_119,

  dmac0i = 120,		// edge triggered, IER0F.IEN0, IPR120, DTCER120
  dmac1i = 121,		// edge triggered, IER0F.IEN1, IPR121, DTCER121
  dmac2i = 122,		// edge triggered, IER0F.IEN2, IPR122, DTCER122
  dmac3i = 123,		// edge triggered, IER0F.IEN3, IPR123, DTCER123
  dmac74i = 124,	// edge triggered, IER0F.IEN4, IPR124

  ostdi = 125,		// edge triggered, IER0F.IEN5, IPR125

  exdmac0i = 126,	// edge triggered, IER0F.IEN6, IPR126, DTCER126
  exdmac1i = 127,	// edge triggered, IER0F.IEN7, IPR127, DTCER127

  // configurable slots 128..207 for PCLKB devices (all edge triggered)
  perib_128 = 128,
  perib_129 = 129,
  perib_130 = 130,
  perib_131 = 131,
  perib_132 = 132,
  perib_133 = 133,
  perib_134 = 134,
  perib_135 = 135,
  perib_136 = 135,
  perib_137 = 137,
  perib_138 = 138,
  perib_139 = 139,
  perib_140 = 140,
  perib_141 = 141,
  perib_142 = 142,
  perib_143 = 143,
  perib_144 = 144,
  perib_145 = 145,
  perib_146 = 145,
  perib_147 = 147,
  perib_148 = 148,
  perib_149 = 149,
  perib_150 = 150,
  perib_151 = 151,
  perib_152 = 152,
  perib_153 = 153,
  perib_154 = 154,
  perib_155 = 155,
  perib_156 = 155,
  perib_157 = 157,
  perib_158 = 158,
  perib_159 = 159,
  perib_160 = 160,
  perib_161 = 161,
  perib_162 = 162,
  perib_163 = 163,
  perib_164 = 164,
  perib_165 = 165,
  perib_166 = 165,
  perib_167 = 167,
  perib_168 = 168,
  perib_169 = 169,
  perib_170 = 170,
  perib_171 = 171,
  perib_172 = 172,
  perib_173 = 173,
  perib_174 = 174,
  perib_175 = 175,
  perib_176 = 175,
  perib_177 = 177,
  perib_178 = 178,
  perib_179 = 179,
  perib_180 = 180,
  perib_181 = 181,
  perib_182 = 182,
  perib_183 = 183,
  perib_184 = 184,
  perib_185 = 185,
  perib_186 = 185,
  perib_187 = 187,
  perib_188 = 188,
  perib_189 = 189,
  perib_190 = 190,
  perib_191 = 191,
  perib_192 = 192,
  perib_193 = 193,
  perib_194 = 194,
  perib_195 = 195,
  perib_196 = 195,
  perib_197 = 197,
  perib_198 = 198,
  perib_199 = 199,
  perib_200 = 200,
  perib_201 = 201,
  perib_202 = 202,
  perib_203 = 203,
  perib_204 = 204,
  perib_205 = 205,
  perib_206 = 205,
  perib_207 = 207,

  isr_num_t_swcfg_intb_begin = perib_128 - 1,

  #define swcfg_intb(i) i,
  #define swcfg_inta(i)
    #if __has_include (<dev/rx64_interrupt.x.hpp>)
      #include <dev/rx64_interrupt.x.hpp>
    #endif
  #undef swcfg_intb
  #undef swcfg_inta

  isr_num_t_swcfg_intb_end,

  // configurable slots 208..255 for PCLKA devices (all edge triggered)
  peria_208 = 208,
  peria_209 = 209,
  peria_210 = 210,
  peria_211 = 211,
  peria_212 = 212,
  peria_213 = 213,
  peria_214 = 214,
  peria_215 = 215,
  peria_216 = 216,
  peria_217 = 217,
  peria_218 = 218,
  peria_219 = 219,
  peria_220 = 220,
  peria_221 = 221,
  peria_222 = 222,
  peria_223 = 223,
  peria_224 = 224,
  peria_225 = 225,
  peria_226 = 226,
  peria_227 = 227,
  peria_228 = 228,
  peria_229 = 229,
  peria_230 = 230,
  peria_231 = 231,
  peria_232 = 232,
  peria_233 = 233,
  peria_234 = 234,
  peria_235 = 235,
  peria_236 = 236,
  peria_237 = 237,
  peria_238 = 238,
  peria_239 = 239,
  peria_240 = 240,
  peria_241 = 241,
  peria_242 = 242,
  peria_243 = 243,
  peria_244 = 244,
  peria_245 = 245,
  peria_246 = 246,
  peria_247 = 247,
  peria_248 = 248,
  peria_249 = 249,
  peria_250 = 250,
  peria_251 = 251,
  peria_252 = 252,
  peria_253 = 253,
  peria_254 = 254,
  peria_255 = 255,

  isr_num_t_swcfg_inta_begin = peria_208 - 1,

  #define swcfg_intb(i)
  #define swcfg_inta(i) i,
    #if __has_include (<dev/rx64_interrupt.x.hpp>)
      #include <dev/rx64_interrupt.x.hpp>
    #endif
  #undef swcfg_intb
  #undef swcfg_inta

  isr_num_t_swcfg_inta_end,


  // shared interrupts are appended to the table.  the shared ISR handler
  // will use the slots > 255

  // BE0 group, PLCKB edge triggered
  be0_0 = 256,

  can0_ers = be0_0,	// GENBE0.EN0 GRPBE0.IS0 GCRBE0.CLR0
  can1_ers,		// GENBE0.EN1 GRPBE0.IS1 GCRBE0.CLR1
  can2_ers,		// GENBE0.EN2 GRPBE0.IS2 GCRBE0.CLR2


  // re-purpose unused slots for multiplexed DMACA4,5,6,7
  // it is demultiplexed by the board/driver code.  not invoked directly
  // in hardware.
  dmac4i,
  dmac5i,
  dmac6i,
  dmac7i,


  // BL0 group, PCLKB level triggered
  bl0_0 = 288,

  sci0_tei = bl0_0 + 0,
  sci0_eri = bl0_0 + 1,

  sci1_tei = bl0_0 + 2,
  sci1_eri = bl0_0 + 3,

  sci2_tei = bl0_0 + 4,
  sci2_eri = bl0_0 + 5,

  sci3_tei = bl0_0 + 6,
  sci3_eri = bl0_0 + 7,

  sci4_tei = bl0_0 + 8,
  sci4_eri = bl0_0 + 9,

  sci5_tei = bl0_0 + 10,
  sci5_eri = bl0_0 + 11,

  sci6_tei = bl0_0 + 12,
  sci6_eri = bl0_0 + 13,

  sci7_tei = bl0_0 + 14,
  sci7_eri = bl0_0 + 15,

  sci12_tei = bl0_0 + 16,
  sci12_eri = bl0_0 + 17,
  sci12_x0 = bl0_0 + 18,
  sci12_x1 = bl0_0 + 19,
  sci12_x2 = bl0_0 + 20,
  sci12_x3 = bl0_0 + 21,

  reserved_bl0_22 = bl0_0 + 22,
  reserved_bl0_23 = bl0_0 + 23,

  qspi_ssli = bl0_0 + 24,

  reserved_bl0_25 = bl0_0 + 25,

  cac_ferri = bl0_0 + 26,
  cac_mendi = bl0_0 + 27,
  cac_ovfi = bl0_0 + 28,

  doc = bl0_0 + 29,

  pdc_fei = bl0_0 + 30,
  pdc_eri = bl0_0 + 31,


  // BL1 group, PCLKB level triggered
  bl1_0 = 320,

  src_ovfi = bl1_0 + 0,
  src_udfi = bl1_0 + 1,
  src_cefi = bl1_0 + 2,

  sdhi_cdeti = bl1_0 + 3,
  sdhi_caci = bl1_0 + 4,
  sdhi_sdaci = bl1_0 + 5,

  mmcif_cdetio = bl1_0 + 6,
  mmcif_errio = bl1_0 + 7,
  mmcif_accio = bl1_0 + 8,

  poe3_oei1 = bl1_0 + 9,
  poe3_oei2 = bl1_0 + 10,
  poe3_oei3 = bl1_0 + 11,
  poe3_oei4 = bl1_0 + 12,

  riic0_tei = bl1_0 + 13,
  riic0_eei = bl1_0 + 14,

  riic2_tei = bl1_0 + 15,
  riic2_eei = bl1_0 + 16,

  ssi0 = bl1_0 + 17,
  ssi1 = bl1_0 + 18,

  reserved_bl1_19 = bl1_0 + 19,

  s12ad_cmpi = bl1_0 + 20,

  reserved_bl1_21 = bl1_0 + 21,

  s12ad1_cmpi = bl1_0 + 22,

  // 23 .. 31: reserved

  // AL0 group, PCLKA level triggered
  al0_0 = 352,

  scifa8_tei = al0_0 + 0,
  scifa8_eri = al0_0 + 1,
  scifa8_bri = al0_0 + 2,
  scifa8_dri = al0_0 + 3,

  scifa9_tei = al0_0 + 4,
  scifa9_eri = al0_0 + 5,
  scifa9_bri = al0_0 + 6,
  scifa9_dri = al0_0 + 7,

  scifa10_tei = al0_0 + 8,
  scifa10_eri = al0_0 + 9,
  scifa10_bri = al0_0 + 10,
  scifa10_dri = al0_0 + 11,

  scifa11_tei = al0_0 + 12,
  scifa11_eri = al0_0 + 13,
  scifa11_bri = al0_0 + 14,
  scifa11_dri = al0_0 + 15,

  rspi_spii = al0_0 + 16,
  rspi_spei = al0_0 + 17,

  // 18 .. 31: reserved



  // AL1 group, PCLKA level triggered
  al1_0 = 384,

  eptpc_mint = al1_0 + 0,
  ptpedmac_pint = al1_0 + 1,

  reserved_al0_2 = al1_0 + 2,
  reserved_al0_3 = al1_0 + 3,

  edmac0_eint = al1_0 + 4,
  edmac1_eint = al1_0 + 5

  // 6 .. 31: reserved
};

static_assert (isr_num_t_swcfg_intb_end <= 208, "");
static_assert (isr_num_t_swcfg_inta_end <= 256, "");

static constexpr unsigned int count = 416;


// software configurable interrupts that can be assigned to perib_128..perib_207
// and peria_208..peria_255 slots.

enum struct cfg_isr_b
{
			// interrupt status flag
  none = 0,		// PIBR0.PIR0

  cmt2_cmi = 1,		// PIBR0.PIR1
  cmt3_cmi = 2,		// PIBR0.PIR2

  tmr0_cmia = 3,	// PIBR0.PIR3
  tmr0_cmib = 4,	// PIBR0.PIR4
  tmr0_ovi = 5,		// PIBR0.PIR5

  tmr1_cmia = 6,	// PIBR0.PIR6
  tmr1_cmib = 7,	// PIBR0.PIR7
  tmr1_ovi = 8,		// PIBR1.PIR0

  tmr2_cmia = 9,	// PIBR1.PIR1
  tmr2_cmib = 10,	// PIBR1.PIR2
  tmr2_ovi = 11,	// PIBR1.PIR3

  tmr3_cmia = 12,	// PIBR1.PIR4
  tmr3_cmib = 13,	// PIBR1.PIR5
  tmr3_ovi = 14,	// PIBR1.PIR6

  tpu0_tgia = 15,	// PIBR1.PIR7
  tpu0_tgib = 16,	// PIBR2.PIR0
  tpu0_tgic = 17,	// PIBR2.PIR1
  tpu0_tgid = 18,	// PIBR2.PIR2
  tpu0_tciv = 19,	// PIBR2.PIR3

  tpu1_tgia = 20,	// PIBR1.PIR4
  tpu1_tgib = 21,	// PIBR2.PIR5
  tpu1_tciv = 22,	// PIBR2.PIR6
  tpu1_tciu = 23,	// PIBR2.PIR7

  tpu2_tgia = 24,	// PIBR3.PIR0
  tpu2_tgib = 25,	// PIBR3.PIR1
  tpu2_tciv = 26,	// PIBR3.PIR2
  tpu2_tciu = 27,	// PIBR3.PIR3

  tpu3_tgia = 28,	// PIBR3.PIR4
  tpu3_tgib = 29,	// PIBR3.PIR5
  tpu3_tgic = 30,	// PIBR3.PIR6
  tpu3_tgid = 31,	// PIBR3.PIR7
  tpu3_tciv = 32,	// PIBR4.PIR0

  tpu4_tgia = 33,	// PIBR4.PIR1
  tpu4_tgib = 34,	// PIBR4.PIR2
  tpu4_tciv = 35,	// PIBR4.PIR3
  tpu4_tciu = 36,	// PIBR4.PIR4

  tpu5_tgia = 37,	// PIBR4.PIR5
  tpu5_tgib = 38,	// PIBR4.PIR6
  tpu5_tciv = 39,	// PIBR4.PIR7
  tpu5_tciu = 40,	// PIBR5.PIR0

  cmtw0_ic0i = 41,	// PIBR5.PIR1
  cmtw0_ic1i = 42,	// PIBR5.PIR2
  cmtw0_oc0i = 43,	// PIBR5.PIR3
  cmtw0_oc1i = 44,	// PIBR5.PIR4

  cmtw1_ic0i = 45,	// PIBR5.PIR5
  cmtw1_ic1i = 46,	// PIBR5.PIR6
  cmtw1_oc0i = 47,	// PIBR5.PIR7
  cmtw1_oc1i = 48,	// PIBR6.PIR0

  rtc_cup = 49,		// PIBR6.PIR1

  can0_rxf = 50,	// PIBR6.PIR2
  can0_txf = 51,	// PIBR6.PIR3
  can0_rxm = 52,	// PIBR6.PIR4
  can0_txm = 53,	// PIBR6.PIR5

  can1_rxf = 54,	// PIBR6.PIR6
  can1_txf = 55,	// PIBR6.PIR7
  can1_rxm = 56,	// PIBR7.PIR0
  can1_txm = 57,	// PIBR7.PIR1

  can2_rxf = 58,	// PIBR7.PIR2
  can2_txf = 59,	// PIBR7.PIR3
  can2_rxm = 60,	// PIBR7.PIR4
  can2_txm = 61,	// PIBR7.PIR5

  usb0_usbi = 62,	// PIBR7.PIR6

  s12ad_s12adi = 64,	// PIBR8.PIR0
  s12ad_s12gbadi = 65,	// PIBR8.PIR1

  s12ad1_s12adi = 68,	// PIBR8.PIR4
  s12ad1_s12gbadi = 69,	// PIBR8.PIR5

  des_desend = 73,	// PIBR9.PIR1

  sha_shadend = 74,	// PIBR9.PIR2
  sha_shaend = 75,	// PIBR9.PIR3

  rng_rngend = 76,	// PIBR9.PIR4

  elc_elsr18i = 79,	// PIBR9.PIR7
  elc_elsr19i = 80	// PIBRA.PIR0
};


enum struct cfg_isr_a
{
			// interrupt status flag
  none = 0,		// PIAR0.PIR0

  mtu0_tgia = 1,	// PIAR0.PIR1
  mtu0_tgib = 2,	// PIAR0.PIR2
  mtu0_tgic = 3,	// PIAR0.PIR3
  mtu0_tgid = 4,	// PIAR0.PIR4
  mtu0_tciv = 5,	// PIAR0.PIR5
  mtu0_tgie = 6,	// PIAR0.PIR6
  mtu0_tgif = 7,	// PIAR0.PIR7

  mtu1_tgia = 8,	// PIAR1.PIR0
  mtu1_tgib = 9,	// PIAR1.PIR1
  mtu1_tciv = 10,	// PIAR1.PIR2
  mtu1_tciu = 11,	// PIAR1.PIR3

  mtu2_tgia = 12,	// PIAR1.PIR4
  mtu2_tgib = 13,	// PIAR1.PIR5
  mtu2_tciv = 14,	// PIAR1.PIR6
  mtu2_tciu = 15,	// PIAR1.PIR7

  mtu3_tgia = 16,	// PIAR2.PIR0
  mtu3_tgib = 17,	// PIAR2.PIR1
  mtu3_tgic = 18,	// PIAR2.PIR2
  mtu3_tgid = 19,	// PIAR2.PIR3
  mtu3_tciv = 20,	// PIAR2.PIR4

  mtu4_tgia = 21,	// PIAR2.PIR5
  mtu4_tgib = 22,	// PIAR2.PIR6
  mtu4_tgic = 23,	// PIAR2.PIR7
  mtu4_tgid = 24,	// PIAR3.PIR0
  mtu4_tciv = 25,	// PIAR3.PIR1

  mtu5_tgiu = 27,	// PIAR3.PIR3
  mtu5_tgiv = 28,	// PIAR3.PIR4
  mtu5_tgiw = 29,	// PIAR3.PIR5

  mtu6_tgia = 30,	// PIAR3.PIR6
  mtu6_tgib = 31,	// PIAR3.PIR7
  mtu6_tgic = 32,	// PIAR4.PIR0
  mtu6_tgid = 33,	// PIAR4.PIR1
  mtu6_tciv = 34,	// PIAR4.PIR2

  mtu7_tgia = 35,	// PIAR3.PIR3
  mtu7_tgib = 36,	// PIAR3.PIR4
  mtu7_tgic = 37,	// PIAR4.PIR5
  mtu7_tgid = 38,	// PIAR4.PIR6
  mtu7_tciv = 39,	// PIAR4.PIR7

  mtu8_tgia = 40,	// PIAR5.PIR1
  mtu8_tgib = 41,	// PIAR5.PIR2
  mtu8_tgic = 42,	// PIAR5.PIR3
  mtu8_tgid = 43,	// PIAR5.PIR4
  mtu8_tciv = 44,	// PIAR5.PIR5

  gpt0_gtcia = 47,	// PIAR5.PIR7
  gpt0_gtcib = 48,	// PIAR6.PIR0
  gpt0_gtcic = 49,	// PIAR6.PIR1
  gpt0_gtcid = 50,	// PIAR6.PIR2
  gpt0_gdte = 51,	// PIAR6.PIR3
  gpt0_gtcie = 52,	// PIAR6.PIR4
  gpt0_gtcif = 53,	// PIAR6.PIR5
  gpt0_gtciv = 54,	// PIAR6.PIR6
  gpt0_gtciu = 55,	// PIAR6.PIR7

  gpt_etgin = 56,	// PIAR7.PIR0
  gpt_etgip = 56,	// PIAR7.PIR1

  gpt1_gtcia = 57,	// PIAR7.PIR2
  gpt1_gtcib = 58,	// PIAR7.PIR3
  gpt1_gtcic = 59,	// PIAR7.PIR4
  gpt1_gtcid = 60,	// PIAR7.PIR5
  gpt1_gdte = 61,	// PIAR7.PIR6
  gpt1_gtcie = 62,	// PIAR7.PIR7
  gpt1_gtcif = 63,	// PIAR8.PIR0
  gpt1_gtciv = 64,	// PIAR8.PIR1
  gpt1_gtciu = 65,	// PIAR8.PIR2

  gpt2_gtcia = 66,	// PIAR8.PIR3
  gpt2_gtcib = 67,	// PIAR8.PIR4
  gpt2_gtcic = 68,	// PIAR8.PIR5
  gpt2_gtcid = 69,	// PIAR8.PIR6
  gpt2_gdte = 70,	// PIAR8.PIR7
  gpt2_gtcie = 71,	// PIAR9.PIR0
  gpt2_gtcif = 72,	// PIAR9.PIR1
  gpt2_gtciv = 73,	// PIAR9.PIR2
  gpt2_gtciu = 74,	// PIAR9.PIR3

  gpt3_gtcia = 75,	// PIAR9.PIR4
  gpt3_gtcib = 76,	// PIAR9.PIR5
  gpt3_gtcic = 77,	// PIAR9.PIR6
  gpt3_gtcid = 78,	// PIAR9.PIR7
  gpt3_gdte = 79,	// PIARA.PIR0
  gpt3_gtcie = 80,	// PIARA.PIR1
  gpt3_gtcif = 81,	// PIARA.PIR2
  gpt3_gtciv = 82,	// PIARA.PIR3
  gpt3_gtciu = 83,	// PIARA.PIR4

  eptpc_ipls = 86,	// PIARA.PIR6

  aes_rdy = 88,		// PIARB.PIR0
  aes_end = 89		// PIARB.PIR1
};



template <isr_num_t IsrNum> struct line
{
  static constexpr unsigned int isr_num = IsrNum;

  static constexpr unsigned int ipr_isr_num (void)
  {
    // see "Table 15.5  Interrupt Vector Table" in the RX64 hardware manual.
    switch (isr_num)
    {
      default: return isr_num;

      case 16: case 18: return 0;
      case 21: return 1;
      case 23: return 2;
      case 26: case 27: return 3;
      case 28: return 4;
      case 29: return 5;
      case 30: return 6;
      case 31: return 7;
    }
  }


  // N.B. the first 16 vectors are reserved and hence in the hardware
  // manual the base register for the IR register array starts at 0x00087010.
  static constexpr hw_reg_rw<int8_t, const_addr<0x00087000 + isr_num>> IRn = { };

  // N.B. the first 16 vectors are reserved and hence in the hardware
  // manual the base register for the imask bit array starts at 0x00087202.
  static constexpr hw_reg_rw<uint8_t, const_addr<0x00087200 + isr_num/8u>> IERm = { };
  static constexpr unsigned int IENj = isr_num % 8u;

  static constexpr hw_reg_rw<uint8_t, const_addr<0x00087300 + ipr_isr_num ()>> IPRn = { };
  static constexpr hw_reg_rw<int8_t, const_addr<0x00087500 + (isr_num - icu_irq0)>> IRQCRi = { };

  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087640>> GENBE0 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087670>> GENBL0 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087674>> GENBL1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087870>> GENAL0 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087874>> GENAL1 = { };

  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087600>> GRPBE0 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087630>> GRPBL0 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087634>> GRPBL1 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087830>> GRPAL0 = { };
  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087834>> GRPAL1 = { };

  static constexpr hw_reg_rw<uint32_t, const_addr<0x00087680>> GCRBE0 = { };


  // the first interrupt that can trigger the DTC is SWINT2 (26) and hence in
  // the hardware manual the base register for the DTCER array starts at 0x0008711A.
  static constexpr hw_reg_rw<uint8_t, const_addr<0x00087100 + isr_num>> DTCER = { };


  static void connect_func (void(*)(void))
  {
    // RX interrupt functions are collected and put into the ISR table during
    // compile time.  thus this function does nothing.
  }

  static void enable (interrupt::trigger_type tt, unsigned int priority)
  {
    if (isr_num < be0_0)
    {
      // normal interrupts
      IRn = 0;

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

	IRQCRi = val;
      }

      IPRn = priority & 0x0F;
      IERm = utils::set_bit (IERm.read (), IENj);
    }

    // to enable the group interrupts, the group interrupt itself needs to
    // be enabled.  this is done only once during board hw init.
    else if (isr_num < bl0_0)
    {
      if (isr_num >= dmac4i && isr_num <= dmac7i)
	line<dmac74i>::enable (tt, priority);
      else
	GENBE0 = utils::set_bit (GENBE0.read (), isr_num - be0_0, true);
    }
    else if (isr_num < bl1_0)
      GENBL0 = utils::set_bit (GENBL0.read (), isr_num - bl0_0, true);
    else if (isr_num < al0_0)
      GENBL1 = utils::set_bit (GENBL1.read (), isr_num - bl1_0, true);
    else if (isr_num < al1_0)
      GENAL0 = utils::set_bit (GENAL0.read (), isr_num - al0_0, true);
    else
      GENAL1 = utils::set_bit (GENAL1.read (), isr_num - al1_0, true);
  }

  static void disable (void)
  {
    if (isr_num < be0_0)
    {
      IERm = utils::clear_bit (IERm.read (), IENj);
      IRn = 0;
    }
    else if (isr_num < bl0_0)
    {
      if (isr_num >= dmac4i && isr_num <= dmac7i)
	line<dmac74i>::disable ();
      else
	GENBE0 = utils::set_bit (GENBE0.read (), isr_num - be0_0, false);
    }
    else if (isr_num < bl1_0)
      GENBL0 = utils::set_bit (GENBL0.read (), isr_num - bl0_0, false);
    else if (isr_num < al0_0)
      GENBL1 = utils::set_bit (GENBL1.read (), isr_num - bl1_0, false);
    else if (isr_num < al1_0)
      GENAL0 = utils::set_bit (GENAL0.read (), isr_num - al0_0, false);
    else
      GENAL1 = utils::set_bit (GENAL1.read (), isr_num - al1_0, false);
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
    if (isr_num < be0_0)
      return IRn != 0;
    else if (isr_num < bl0_0)
    {
      if (isr_num >= dmac4i && isr_num <= dmac7i)
      {
	static constexpr hw_reg_r<uint8_t, const_addr<0x00082204>> dmac74_dmist = { };
	return utils::get_bit (dmac74_dmist.read (), isr_num - dmac4i + 4);
      }
      else
	return utils::get_bit (GRPBE0.read (), isr_num - be0_0);
    }
    else if (isr_num < bl1_0)
      return utils::get_bit (GRPBL0.read (), isr_num - bl0_0);
    else if (isr_num < al0_0)
      return utils::get_bit (GRPBL1.read (), isr_num - bl1_0);
    else if (isr_num < al1_0)
      return utils::get_bit (GRPAL0.read (), isr_num - al0_0);
    else
      return utils::get_bit (GRPAL1.read (), isr_num - al1_0);
  }

  static void clear (void)
  {
    if (isr_num < be0_0)
      IRn = 0;
    else if (isr_num < bl1_0)
      GCRBE0 = utils::set_bit (GCRBE0.read (), isr_num - be0_0, true);

    // other grouped interrupts can only be cleared by clearing the
    // interrupt in the corresponding peripheral module.
  }


};


inline void init_sw_cfg_intb_inta (void)
{
  #if __has_include (<dev/rx64_interrupt.x.hpp>)

  // software configurable interrupts B
  // SLIBXRn = 0x00087780 ... 0x0008778F   (128..143)
  // SLIBRn =  0x00087790 ... 0x000877CF   (144..207)
  constexpr uint8_t sw_cfg_int_b[] =
  {
    #define swcfg_intb(i) (uint8_t)cfg_isr_b::i,
    #define swcfg_inta(i)
      #include <dev/rx64_interrupt.x.hpp>
    #undef swcfg_intb
    #undef swcfg_inta
  };

  for (unsigned int i = 0; i < std::extent<decltype (sw_cfg_int_b)>::value; ++i)
    *(volatile uint8_t*)(0x00087780 + i) = sw_cfg_int_b[i];


  // software configurable interrupts A
  // SLIARn 0x000879D0 ... 0x000879FF  (208...255)
  constexpr uint8_t sw_cfg_int_a[] =
  {
    #define swcfg_intb(i)
    #define swcfg_inta(i) (uint8_t)cfg_isr_a::i,
      #include <dev/rx64_interrupt.x.hpp>
    #undef swcfg_intb
    #undef swcfg_inta
  };

  for (unsigned int i = 0; i < std::extent<decltype (sw_cfg_int_a)>::value; ++i)
    *(volatile uint8_t*)(0x000879D0 + i) = sw_cfg_int_a[i];

  static constexpr dev::hw_reg_rw<uint8_t, dev::const_addr<0x00087A00>> SLIPRCR;
  while (SLIPRCR != 1) { SLIPRCR = 1; }

  #endif // __has_include
}


} // namespace rx64_interrupt

// group interrupts are handled by a special ISR function which invokes the
// actual ISR functions of the individual ISRs.  all of those are put in
// the vector table.  however, the individual ISRs that are invoked by a
// top level group ISR must not do ISR specific function call handling, such
// as returning from interrupt etc.  to get that we specialize the
// connected_isr for those particualr isr numbers.

namespace interrupt
{
template <rx64_interrupt::isr_num_t IsrNum, typename Func>
class connected_isr<rx64_interrupt::line<IsrNum>, Func,
		    typename std::enable_if< (IsrNum >= rx64_interrupt::be0_0) >::type >
{
public:
  typedef rx64_interrupt::line<IsrNum> interrupt_line;

  static constexpr unsigned int isr_num = interrupt_line::isr_num;

  connected_isr (void* param)
  {
    isr_param = param;
    interrupt_line::connect_func (isr_func);
  }

  void enable (trigger_type tt, unsigned int priority)
  {
    interrupt_line::enable (tt, priority);
  }

  void disable (void)
  {
    interrupt_line::disable ();
  }

  bool status (void) const
  {
    return interrupt_line::status ();
  }

  void clear (void)
  {
    interrupt_line::clear ();
  }

  //static void __isr_func_attr__ isr_func (void)
  static void isr_func (void)
  {
    Func::invoke (isr_param);
  }

private:
  static void* isr_param;
};

template <rx64_interrupt::isr_num_t IsrNum, typename Func>
void* connected_isr< rx64_interrupt::line<IsrNum>, Func,
		     typename std::enable_if< (IsrNum >= rx64_interrupt::be0_0) >::type >::isr_param;

} // namespace interrupt

} // namespace dev

#endif // includeguard_rx64_interrupt_hpp_includeguard
