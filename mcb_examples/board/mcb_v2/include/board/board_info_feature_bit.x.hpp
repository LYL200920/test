
#ifdef expand_feature_bit

expand_feature_bit (0, rx631_144, "RX631_144", "RX631 144 pin MCU")
expand_feature_bit (1, rx63n_144, "RX63N_144", "RX63N 144 pin MCU")
expand_feature_bit (2, rx64m_144, "RX64M_144", "RX64M 144 pin MCU")
expand_feature_bit (3, rx64m_176, "RX64M_176", "RX64M 176 pin MCU")
expand_feature_bit (4, rx71m_144, "RX71M_144", "RX71M 144 pin MCU")
expand_feature_bit (5, rx71m_176, "RX71M_176", "RX71M 176 pin MCU")

expand_feature_bit (6, digital_io_0_15, "DIO_0_15", "Digital IOs 0-15 (CN9)")
expand_feature_bit (7, digital_io_16_31, "DIO_16_31", "Digital IOs 16-31 (CN10)")

expand_feature_bit (8, eth0, "ETH0", "ETH0 on RX EtherC 0 and LAN8720")
expand_feature_bit (9, eth1, "ETH1", "ETH1 on RX EtherC 1 and LAN8720")
expand_feature_bit (10, eth2, "ETH2", "ETH2 on LAN9250")

expand_feature_bit (11, usb0_b, "USB0 B", "USB0 (USB function)")
expand_feature_bit (12, usba_a, "USB0 A", "USBA (USB host)")

expand_feature_bit (13, dac124s085, "DAC124S085",
		    "3 analog output channels for each MCX axis.")

expand_feature_bit (14, pcd4641, "PCD4641", "4 Axis PTO Controller")
expand_feature_bit (15, mcx514, "MCX514", "4 Axis PTO Controller")
expand_feature_bit (16, mcx512, "MCX512", "2 Axis PTO Controller")

expand_feature_bit (17, rs485_sci6, "RS485 SCI6", "RS485 on SCI6")
expand_feature_bit (18, rs485_sci9_scifa9, "RS485 SCI9/SCIFA9", "RS485 on SCI9/SCIFA9")

expand_feature_bit (19, can, "CAN", "CAN bus interface")

expand_feature_bit (20, rx_trigger_inputs, "RX_TRG_IN",
		    "8 RX trigger inputs")

expand_feature_bit (21, rx_trigger_outputs, "RX_TRG_OUT",
		    "8 RX trigger outputs")

expand_feature_bit (22, mcx_trigger_inputs_x, "MCX_TRG_IN_X",
		    "MCX X trigger inputs")

expand_feature_bit (23, mcx_trigger_inputs_y, "MCX_TRG_IN_Y",
		    "MCX Y trigger inputs")

expand_feature_bit (24, mcx_trigger_inputs_z, "MCX_TRG_IN_Z",
		    "MCX Z trigger inputs")

expand_feature_bit (25, mcx_trigger_inputs_u, "MCX_TRG_IN_U",
		    "MCX U trigger inputs")

expand_feature_bit (26, mcx_trigger_outputs_x, "MCX_TRG_OUT_X",
		    "MCX X trigger outputs")

expand_feature_bit (27, mcx_trigger_outputs_y, "MCX_TRG_OUT_Y",
		    "MCX Y trigger outputs")

expand_feature_bit (28, mcx_trigger_outputs_z, "MCX_TRG_OUT_Z",
		    "MCX Z trigger outputs")

expand_feature_bit (29, mcx_trigger_outputs_u, "MCX_TRG_OUT_U",
		    "MCX U trigger outputs")

expand_feature_bit (30, rx_crypto, "RX_CRYPTO", "RX MCU Crypto Accelerator Functions")

expand_feature_bit (31, nand_flash_128m, "NAND_FL_128M", "128 MByte NAND flash")
expand_feature_bit (32, nand_flash_256m, "NAND_FL_256M", "256 MByte NAND flash")
expand_feature_bit (33, nand_flash_512m, "NAND_FL_512M", "512 MByte NAND flash")
expand_feature_bit (34, nand_flash_1g, "NAND_FL_1G", "1 GByte NAND flash")
expand_feature_bit (35, nand_flash_2g, "NAND_FL_2G", "2 GByte NAND flash")
expand_feature_bit (36, nand_flash_4g, "NAND_FL_4G", "4 GByte NAND flash")
expand_feature_bit (37, nand_flash_8g, "NAND_FL_8G", "8 GByte NAND flash")

expand_feature_bit (38, standby_battery_power, "STANDBY_POWER", "Standby Battery Power Supply")
expand_feature_bit (39, ice40, "ICE40_FPGA", "iCE40 FPGA")

#undef expand_feature_bit
#endif
