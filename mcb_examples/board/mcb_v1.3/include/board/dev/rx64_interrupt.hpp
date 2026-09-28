
/*
board specific configurable interrupt allocation.

*/

#ifndef includeguard_mcb13_dev_rx64_interrupt_hpp_includeguard
#define includeguard_mcb13_dev_rx64_interrupt_hpp_includeguard

#include "../../../dev/rx64_interrupt.hpp"

namespace dev
{

namespace rx64_interrupt
{

#if 0
enum isr_num_ext_t
{
  // configurable slots 128..207 for PCLKB devices (all edge triggered)
  cmt2_cmi = perib_128,
  cmt3_cmi = perib_129,

  tmr0_cmia = perib_130,
  tmr0_cmib = perib_131,
  tmr0_ovi  = perib_132,

  tmr1_cmia = perib_133,
  tmr1_cmib = perib_134,
  tmr1_ovi  = perib_135,

  tmr2_cmia = perib_136,
  tmr2_cmib = perib_137,
  tmr2_ovi  = perib_138,

  tmr3_cmia = perib_139,
  tmr3_cmib = perib_140,
  tmr3_ovi  = perib_141,

  tpu0_tgia = perib_142,
  tpu0_tgib = perib_143,
  tpu0_tgic = perib_144,
  tpu0_tgid = perib_145,
  tpu0_tciv = perib_146,

  tpu1_tgia = perib_147,
  tpu1_tgib = perib_148,
  tpu1_tciv = perib_149,
  tpu1_tciu = perib_150,

  tpu2_tgia = perib_151,
  tpu2_tgib = perib_152,
  tpu2_tciv = perib_153,
  tpu2_tciu = perib_154,

  tpu3_tgia = perib_155,
  tpu3_tgib = perib_156,
  tpu3_tgic = perib_157,
  tpu3_tgid = perib_158,
  tpu3_tciv = perib_159,

  tpu4_tgia = perib_160,
  tpu4_tgib = perib_161,
  tpu4_tciv = perib_162,
  tpu4_tciu = perib_163,

  tpu5_tgia = perib_164,
  tpu5_tgib = perib_165,
  tpu5_tciv = perib_166,
  tpu5_tciu = perib_167,

  cmtw0_ic0i = perib_170,
  cmtw0_ic1i = perib_171,
  cmtw0_oc0i = perib_172,
  cmtw0_oc1i = perib_173,

  cmtw1_ic0i = perib_174,
  cmtw1_ic1i = perib_175,
  cmtw1_oc0i = perib_176,
  cmtw1_oc1i = perib_177,

  rtc_cup = perib_178,

  can1_rxf = perib_179,
  can1_txf = perib_180,
  can1_rxm = perib_181,
  can1_txm = perib_182,

  usb0_usbi = perib_183,

  s12ad_s12adi = perib_184,
  s12ad_s12gbadi = perib_185,

  s12ad1_s12adi = perib_186,
  s12ad1_s12gbadi = perib_187,

  des_desend = perib_188,

  sha_shadend = perib_189,
  sha_shaend = perib_190,

  rng_rngend = perib_191,

  elc_elsr18i = perib_192,
  elc_elsr19i = perib_193,

  // 194 ... 207: free

  // configurable slots 208..255 for PCLKA devices (all edge triggered)

  mtu0_tgia = peria_208,
  mtu0_tgib = peria_209,
  mtu0_tciv = peria_210,

  mtu1_tgia = peria_211,
  mtu1_tgib = peria_212,
  mtu1_tciv = peria_213,

  mtu2_tgia = peria_214,
  mtu2_tgib = peria_215,
  mtu2_tciv = peria_216,

  mtu3_tgia = peria_217,
  mtu3_tgib = peria_218,
  mtu3_tciv = peria_219,

  mtu4_tgia = peria_220,
  mtu4_tgib = peria_221,
  mtu4_tciv = peria_222,

  mtu5_tgiu = peria_223,
  mtu5_tgiv = peria_224,
  mtu5_tciw = peria_225,

  mtu6_tgia = peria_226,
  mtu6_tgib = peria_227,
  mtu6_tciv = peria_228,

  mtu7_tgia = peria_229,
  mtu7_tgib = peria_230,
  mtu7_tciv = peria_231,

  gpt0_gtcia = peria_232,
  gpt0_gtcib = peria_233,
  gpt0_gtciv = peria_234,

  gpt1_gtcia = peria_235,
  gpt1_gtcib = peria_236,
  gpt1_gtciv = peria_237,

  gpt2_gtcia = peria_238,
  gpt2_gtcib = peria_239,
  gpt2_gtciv = peria_240,

  gpt3_gtcia = peria_241,
  gpt3_gtcib = peria_242,
  gpt3_gtciv = peria_243,

  eptpc_ipls = peria_244,

  aes_rdy = peria_245,
  aes_end = peria_246

  // peria_247...peria_255: free
};

#endif


} // namespace rx64_interrupt
} // namespace dev

#endif // includeguard_mcb13_dev_rx64_interrupt_hpp_includeguard
