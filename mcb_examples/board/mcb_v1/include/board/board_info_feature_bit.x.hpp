
#ifdef expand_feature_bit

expand_feature_bit (0, dac124s085, "DAC124S085",
		    "3 analog output channels for each servo axis.")

expand_feature_bit (1, pca9698_u23, "PCA9698 U23",
		    "Digital IO")

expand_feature_bit (2, pca9698_u24, "PCA9698 U24",
		    "Digital IO")

expand_feature_bit (3, pcd4641, "PCD4641",
		    "4 Axis Pulse Motor Controller")

expand_feature_bit (4, mcx514, "MCX514",
		    "4 Axis Pulse Motor Controller")

expand_feature_bit (5, eth0, "ETH0",
		    "Eth0 on RX63N EtherC and LAN8710AI")

expand_feature_bit (6, usb0_b, "USB0 B",
		    "RX63N USB0 type B (USB function)")

expand_feature_bit (7, rs485_sci6, "RS485 SCI6",
		    "RS485 on SCI6 with ISL3172E (max 250 Kbps)")


expand_feature_bit (8, rs485_sci7, "RS485 SCI7",
		    "RS485 on SCI7 with ISL3172E (max 250 Kbps)")

expand_feature_bit (9, rs232c_sci0, "RS232C SCI0",
		    "RS232-C on SCI0 with ICL3232CVZ (max 250 Kbps)")

expand_feature_bit (10, sci5_ext_i2c, "SCI5 EXT I2C",
		    "CN6 can be used as external I2C bus (R43,R44 = 1K, CN6 pin 3 = #PERI_RST, C27 = 0.1 uF)")

expand_feature_bit (11, rs485_sci6_20mbps, "RS485 SCI6 20 Mbps",
		    "RS485 on SCI6 with ISL3178E (max 20 Mbps)")

expand_feature_bit (12, rs485_sci7_20mbps, "RS485 SCI7 20 Mbps",
		    "RS485 on SCI6 with ISL3178E (max 20 Mbps)")

#undef expand_feature_bit
#endif
