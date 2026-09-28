
#ifdef expand_feature_bit

expand_feature_bit (0, dac124s085, "DAC124S085",
		    "3 analog output channels for each servo axis.")

expand_feature_bit (1, pca9698_0x40, "PCA9698 0x40",
		    "Digital IO")

expand_feature_bit (2, pca9698_0x42, "PCA9698 0x42",
		    "Digital IO")

expand_feature_bit (3, pcd4641, "PCD4641",
		    "4 Axis Pulse Motor Controller")

expand_feature_bit (4, mcx514, "MCX514",
		    "4 Axis Pulse Motor Controller")

expand_feature_bit (5, eth0, "ETH0",
		    "Eth0 on RX63N/RX64M/RX71M EtherC and LAN8720")

expand_feature_bit (6, eth1, "ETH1",
		    "Eth1 on RX64M/RX71M EtherC and LAN8720")

expand_feature_bit (7, eth2, "ETH2",
		    "Eth2 on LAN9250")

expand_feature_bit (8, usb0_b, "USB0 B",
		    "USB0 type B (USB function)")

expand_feature_bit (7, rs485_sci6, "RS485 SCI6",
		    "RS485 on SCI6 with ISL3176E (max 25 MBps)")

expand_feature_bit (8, rs485_sci7_scif10, "RS485 SCI7/SCIF10",
		    "RS485 on SCI7/SCIF10 with ISL3176EIUZ (max 25 Mbps)")

expand_feature_bit (9, can, "CAN",
		    "CAN bus interface (max 1 Mbps)")

expand_feature_bit (10, trigger_inputs, "TRG_IN",
		    "8 trigger inputs")

expand_feature_bit (11, trigger_outputs, "TRG_OUT",
		    "8 trigger outputs")

expand_feature_bit (12, digital_inputs_cn9, "DI_CN9",
		    "16 digital inputs on CN9")

expand_feature_bit (13, digital_outputs_cn9, "DO_CN9",
		    "16 digital outputs on CN9")

expand_feature_bit (14, digital_inputs_cn10, "DI_CN10",
		    "16 digital inputs on CN10")

expand_feature_bit (15, digital_outputs_cn10, "DO_CN10",
		    "16 digital outputs on CN10")

expand_feature_bit (16, dcdc1_6w, "DCDC1_6W",
		    "6 Watts DCDC1 supply")

expand_feature_bit (17, dcdc1_15w, "DCDC1_15W",
		    "15 Watts DCDC1 supply")

#undef expand_feature_bit
#endif
