
// configurable slots 128..207 for PCLKB devices (all edge triggered)
// always starts with perib_128

swcfg_intb (cmt2_cmi)
swcfg_intb (cmt3_cmi)

swcfg_intb (tmr0_cmia)
swcfg_intb (tmr0_cmib)
swcfg_intb (tmr0_ovi)

swcfg_intb (tmr1_cmia)
swcfg_intb (tmr1_cmib)
swcfg_intb (tmr1_ovi)

swcfg_intb (tmr2_cmia)
swcfg_intb (tmr2_cmib)
swcfg_intb (tmr2_ovi)

swcfg_intb (tmr3_cmia)
swcfg_intb (tmr3_cmib)
swcfg_intb (tmr3_ovi)

swcfg_intb (tpu0_tgia)
swcfg_intb (tpu0_tgib)
swcfg_intb (tpu0_tgic)
swcfg_intb (tpu0_tgid)
swcfg_intb (tpu0_tciv)

swcfg_intb (tpu1_tgia)
swcfg_intb (tpu1_tgib)
swcfg_intb (tpu1_tciv)
swcfg_intb (tpu1_tciu)

swcfg_intb (tpu2_tgia)
swcfg_intb (tpu2_tgib)
swcfg_intb (tpu2_tciv)
swcfg_intb (tpu2_tciu)

swcfg_intb (tpu3_tgia)
swcfg_intb (tpu3_tgib)
swcfg_intb (tpu3_tgic)
swcfg_intb (tpu3_tgid)
swcfg_intb (tpu3_tciv)

swcfg_intb (tpu4_tgia)
swcfg_intb (tpu4_tgib)
swcfg_intb (tpu4_tciv)
swcfg_intb (tpu4_tciu)

swcfg_intb (tpu5_tgia)
swcfg_intb (tpu5_tgib)
swcfg_intb (tpu5_tciv)
swcfg_intb (tpu5_tciu)

swcfg_intb (cmtw0_ic0i)
swcfg_intb (cmtw0_ic1i)
swcfg_intb (cmtw0_oc0i)
swcfg_intb (cmtw0_oc1i)

swcfg_intb (cmtw1_ic0i)
swcfg_intb (cmtw1_ic1i)
swcfg_intb (cmtw1_oc0i)
swcfg_intb (cmtw1_oc1i)

swcfg_intb (rtc_cup)

swcfg_intb (can1_rxf)
swcfg_intb (can1_txf)
swcfg_intb (can1_rxm)
swcfg_intb (can1_txm)

swcfg_intb (usb0_usbi)

swcfg_intb (s12ad_s12adi)
swcfg_intb (s12ad_s12gbadi)

swcfg_intb (s12ad1_s12adi)
swcfg_intb (s12ad1_s12gbadi)

swcfg_intb (des_desend)

swcfg_intb (sha_shadend)
swcfg_intb (sha_shaend)

swcfg_intb (rng_rngend)

swcfg_intb (elc_elsr18i)
swcfg_intb (elc_elsr19i)


// configurable slots 208..255 for PCLKA devices (all edge triggered)
// always starts with peria_208
swcfg_inta (mtu0_tgia)
swcfg_inta (mtu0_tgib)
swcfg_inta (mtu0_tgic)
swcfg_inta (mtu0_tgid)
swcfg_inta (mtu0_tciv)

swcfg_inta (mtu1_tgia)
swcfg_inta (mtu1_tgib)
swcfg_inta (mtu1_tciv)
swcfg_inta (mtu1_tciu)

// MTIOC2A is PCD clock output, not usable
// MTIOC3C is MCX clock output, not usable
// MTIOC4C is USB HUB clock output, not usable

swcfg_inta (mtu5_tgiu)
swcfg_inta (mtu5_tgiv)
swcfg_inta (mtu5_tgiw)

swcfg_inta (mtu6_tgia)
swcfg_inta (mtu6_tgib)
swcfg_inta (mtu6_tgic)
swcfg_inta (mtu6_tgid)
swcfg_inta (mtu6_tciv)

swcfg_inta (mtu7_tgia)
swcfg_inta (mtu7_tgib)
swcfg_inta (mtu7_tgic)
swcfg_inta (mtu7_tgid)
swcfg_inta (mtu7_tciv)

swcfg_inta (mtu8_tgia)
swcfg_inta (mtu8_tgib)
swcfg_inta (mtu8_tgic)
swcfg_inta (mtu8_tgid)
swcfg_inta (mtu8_tciv)

// there are not enough slots to map all GPT interrupts.
// limit to 2 trigger points and under/overflow.
swcfg_inta (gpt0_gtcia)
swcfg_inta (gpt0_gtcib)
swcfg_inta (gpt0_gtciv)
swcfg_inta (gpt0_gtciu)

swcfg_inta (gpt1_gtcia)
swcfg_inta (gpt1_gtcib)
swcfg_inta (gpt1_gtciv)
swcfg_inta (gpt1_gtciu)

swcfg_inta (gpt2_gtcia)
swcfg_inta (gpt2_gtcib)
swcfg_inta (gpt2_gtciv)
swcfg_inta (gpt2_gtciu)

swcfg_inta (gpt3_gtcia)
swcfg_inta (gpt3_gtcib)
swcfg_inta (gpt3_gtciv)
swcfg_inta (gpt3_gtciu)

swcfg_inta (eptpc_ipls)

swcfg_inta (aes_rdy)
swcfg_inta (aes_end)

