EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_02x20_Odd_Even J12
U 1 1 641701C7
P 4800 3200
F 0 "J12" H 4850 4317 50  0000 C CNN
F 1 "Jetson Xavier NX Developer Kit" H 4850 4226 50  0000 C CNN
F 2 "" H 4800 3200 50  0001 C CNN
F 3 "~" H 4800 3200 50  0001 C CNN
	1    4800 3200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x09 J1
U 1 1 641725B9
P 3050 3400
F 0 "J1" H 3130 3442 50  0000 L CNN
F 1 "BMI088 Breakout Board" H 3130 3351 50  0000 L CNN
F 2 "" H 3050 3400 50  0001 C CNN
F 3 "~" H 3050 3400 50  0001 C CNN
	1    3050 3400
	-1   0    0    1   
$EndComp
Text Label 5100 2300 0    50   ~ 0
5V
Text Label 5100 2400 0    50   ~ 0
5V
Text Label 5100 2500 0    50   ~ 0
GND
Text Label 5100 2600 0    50   ~ 0
UART1_TX
Text Label 5100 2700 0    50   ~ 0
UART1_RX
Text Label 5100 2800 0    50   ~ 0
I2S2_CLK
Text Label 5100 2900 0    50   ~ 0
GND
Text Label 5100 3000 0    50   ~ 0
SPI1_CS1
Text Label 5100 3100 0    50   ~ 0
SPI1_CS0
Text Label 5100 3200 0    50   ~ 0
GND
Text Label 5100 3300 0    50   ~ 0
SPI1_MISO
Text Label 5100 3400 0    50   ~ 0
SPI0_CS0
Text Label 5100 3500 0    50   ~ 0
SPI0_CS1
Text Label 5100 3600 0    50   ~ 0
I2C_GP2_CLK
Text Label 5100 3700 0    50   ~ 0
GND
Text Label 5100 3800 0    50   ~ 0
GPIO9_CAN1_GPIO0_DMIC_CLK
Text Label 5100 3900 0    50   ~ 0
GND
Text Label 5100 4000 0    50   ~ 0
UART1_CTS
Text Label 5100 4100 0    50   ~ 0
I2S_SDIN
Text Label 5100 4200 0    50   ~ 0
I2S_SDOUT
Text Label 4600 2300 2    50   ~ 0
3.3V
Text Label 4600 2400 2    50   ~ 0
I2C_GP5_DAT
Text Label 4600 2500 2    50   ~ 0
I2C_GP5_CLK
Text Label 4600 2600 2    50   ~ 0
MCLK05
Text Label 4600 2700 2    50   ~ 0
GND
Text Label 4600 2800 2    50   ~ 0
UART1_RTS
Text Label 4600 2900 2    50   ~ 0
SPI1_SCK
Text Label 4600 3000 2    50   ~ 0
GPIO27_PWM2
Text Label 4600 3100 2    50   ~ 0
3.3V
Text Label 4600 3200 2    50   ~ 0
SPI0_MOSI
Text Label 4600 3300 2    50   ~ 0
SPI0_MISO
Text Label 4600 3400 2    50   ~ 0
SPI0_SCK
Text Label 4600 3500 2    50   ~ 0
GND
Text Label 4600 3600 2    50   ~ 0
I2C_GP2_DAT
Text Label 4600 3700 2    50   ~ 0
CAN0_DIN
Text Label 4600 3800 2    50   ~ 0
CAN0_DOUT
Text Label 4600 3900 2    50   ~ 0
CAN1_DOUT
Text Label 4600 4000 2    50   ~ 0
I2S_FS
Text Label 4600 4100 2    50   ~ 0
SPI1_MOSI
Text Label 4600 4200 2    50   ~ 0
GND
Text Label 3250 3800 0    50   ~ 0
5V
Text Label 3250 3700 0    50   ~ 0
CS1
Text Label 3250 3600 0    50   ~ 0
CS2
Text Label 3250 3500 0    50   ~ 0
SDI
Text Label 3250 3400 0    50   ~ 0
SCL
Text Label 3250 3300 0    50   ~ 0
SDO
Text Label 3250 3200 0    50   ~ 0
INTAccel
Text Label 3250 3100 0    50   ~ 0
3.3V(out)
Text Label 3250 3000 0    50   ~ 0
GND
NoConn ~ 3250 3100
$Comp
L Connector_Generic:Conn_02x30_Odd_Even J2
U 1 1 641899EA
P 7750 3100
F 0 "J2" H 7800 4717 50  0000 C CNN
F 1 "60PIN HD CONNECTOR FOR MMWAVEICBOOST" H 7800 4626 50  0000 C CNN
F 2 "" H 7750 3100 50  0001 C CNN
F 3 "~" H 7750 3100 50  0001 C CNN
	1    7750 3100
	1    0    0    -1  
$EndComp
NoConn ~ 7550 1700
NoConn ~ 7550 1800
NoConn ~ 7550 1900
NoConn ~ 7550 2000
NoConn ~ 7550 2100
NoConn ~ 8050 1800
NoConn ~ 8050 1900
Text Label 8050 1700 0    50   ~ 0
5V
Text Label 8050 2200 0    50   ~ 0
AR_NRESET
Text Label 8050 2000 0    50   ~ 0
DMM_SYNC
Text Label 8050 2100 0    50   ~ 0
DMM_CLK
Text Label 8050 2300 0    50   ~ 0
MMWAVEICBOOST_PGOOD
Text Label 8050 2400 0    50   ~ 0
SPI_HOSTINTR
Text Label 8050 2500 0    50   ~ 0
MSS_LOGGER
Text Label 8050 2600 0    50   ~ 0
GND
Text Label 8050 2700 0    50   ~ 0
AR_SYNC_IN
Text Label 8050 2800 0    50   ~ 0
AR_SYNC_OUT_SOP1
Text Label 8050 2900 0    50   ~ 0
GND
Text Label 8050 3000 0    50   ~ 0
LVDS_FRCLKP
Text Label 8050 3100 0    50   ~ 0
LVDS_FRCLKN
Text Label 8050 3200 0    50   ~ 0
GND
Text Label 8050 3300 0    50   ~ 0
LVDS_CLKP
Text Label 8050 3400 0    50   ~ 0
LVDS_CLKN
Text Label 8050 3500 0    50   ~ 0
GND
Text Label 8050 3600 0    50   ~ 0
LVDS_1P
Text Label 8050 3700 0    50   ~ 0
LVDS_1N
Text Label 8050 3800 0    50   ~ 0
GND
Text Label 8050 3900 0    50   ~ 0
LVDS_0P
Text Label 8050 4000 0    50   ~ 0
LVDS_0N
Text Label 8050 4100 0    50   ~ 0
GND
Text Label 8050 4200 0    50   ~ 0
AR_NERROR_IN
Text Label 8050 4300 0    50   ~ 0
AR_NERROR_OUT
Text Label 8050 4400 0    50   ~ 0
AR_GPIO_0
Text Label 8050 4500 0    50   ~ 0
AR_GPIO_1
Text Label 8050 4600 0    50   ~ 0
AR_GPIO_2
Text Label 7550 2200 2    50   ~ 0
TDI
Text Label 7550 2300 2    50   ~ 0
TMS
Text Label 7550 2400 2    50   ~ 0
TCK
Text Label 7550 2500 2    50   ~ 0
TDO_SOP0
Text Label 7550 2600 2    50   ~ 0
SPI_CS1
Text Label 7550 2700 2    50   ~ 0
SPI_CLK
Text Label 7550 2800 2    50   ~ 0
SPI_MOSI
Text Label 7550 2900 2    50   ~ 0
SPI_MISO
Text Label 7550 3000 2    50   ~ 0
DMM_DP0
Text Label 7550 3100 2    50   ~ 0
DMM_DP1
Text Label 7550 3200 2    50   ~ 0
DMM_DP2
Text Label 7550 3300 2    50   ~ 0
DMM_DP3
Text Label 7550 3400 2    50   ~ 0
DMM_DP4
Text Label 7550 3500 2    50   ~ 0
DMM_DP5
Text Label 7550 3600 2    50   ~ 0
DMM_DP6
Text Label 7550 3700 2    50   ~ 0
DMM_DP7
Text Label 7550 3800 2    50   ~ 0
BSS_LOGGER
Text Label 7550 3900 2    50   ~ 0
OSC_CLKOUT
Text Label 7550 4000 2    50   ~ 0
MCU_CLKOUT
Text Label 7550 4100 2    50   ~ 0
PMIC_CLKOUT_SOP2
Text Label 7550 4200 2    50   ~ 0
AR_WRMRST
Text Label 7550 4300 2    50   ~ 0
I2C_SDA
Text Label 7550 4400 2    50   ~ 0
I2C_SCL
Text Label 7550 4500 2    50   ~ 0
RS232RX
Text Label 7550 4600 2    50   ~ 0
RS232TX
Wire Wire Line
	3250 3300 4600 3300
Wire Wire Line
	3250 3400 4600 3400
Wire Wire Line
	3250 3200 3600 3200
Wire Wire Line
	3600 3200 3600 3000
Wire Wire Line
	3600 3000 4600 3000
Wire Wire Line
	3250 3800 3800 3800
Wire Wire Line
	3800 3800 3800 2000
Wire Wire Line
	3800 2000 5500 2000
Wire Wire Line
	5500 2000 5500 2300
Wire Wire Line
	5500 2300 5100 2300
Wire Wire Line
	3250 3700 3900 3700
Wire Wire Line
	3900 3700 3900 4500
Wire Wire Line
	3900 4500 6000 4500
Wire Wire Line
	6000 4500 6000 3400
Wire Wire Line
	6000 3400 5100 3400
Wire Wire Line
	3250 3600 3950 3600
Wire Wire Line
	3950 3600 3950 4450
Wire Wire Line
	3950 4450 5950 4450
Wire Wire Line
	5950 4450 5950 3500
Wire Wire Line
	5950 3500 5100 3500
Wire Wire Line
	3700 3200 4600 3200
Wire Wire Line
	3250 3500 3700 3500
Wire Wire Line
	3700 3500 3700 3200
Wire Wire Line
	3250 3000 3550 3000
Wire Wire Line
	3550 3000 3550 2950
Wire Wire Line
	3550 2950 3750 2950
Wire Wire Line
	3750 2950 3750 3500
Wire Wire Line
	3750 3500 4600 3500
$Comp
L power:+5V #PWR?
U 1 1 641F8C2E
P 9400 1000
F 0 "#PWR?" H 9400 850 50  0001 C CNN
F 1 "+5V" H 9415 1173 50  0000 C CNN
F 2 "" H 9400 1000 50  0001 C CNN
F 3 "" H 9400 1000 50  0001 C CNN
	1    9400 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 641F994C
P 9600 1000
F 0 "#PWR?" H 9600 750 50  0001 C CNN
F 1 "GND" H 9605 827 50  0000 C CNN
F 2 "" H 9600 1000 50  0001 C CNN
F 3 "" H 9600 1000 50  0001 C CNN
	1    9600 1000
	-1   0    0    1   
$EndComp
Wire Wire Line
	8050 1700 9400 1700
Wire Wire Line
	9400 1700 9400 1000
Wire Wire Line
	8050 2600 9600 2600
Wire Wire Line
	9600 2600 9600 1000
Wire Wire Line
	5100 3300 6350 3300
Wire Wire Line
	6350 3300 6350 2900
Wire Wire Line
	6350 2900 7550 2900
Wire Wire Line
	4600 4100 4400 4100
Wire Wire Line
	4400 4100 4400 4400
Wire Wire Line
	4400 4400 6400 4400
Wire Wire Line
	6400 4400 6400 2800
Wire Wire Line
	6400 2800 7550 2800
Wire Wire Line
	5100 3100 6550 3100
Wire Wire Line
	6550 3100 6550 2600
Wire Wire Line
	6550 2600 7550 2600
Wire Wire Line
	4600 2900 4100 2900
Wire Wire Line
	4100 2900 4100 1950
Wire Wire Line
	4100 1950 6600 1950
Wire Wire Line
	6600 1950 6600 2700
Wire Wire Line
	6600 2700 7550 2700
Wire Wire Line
	4600 4200 4450 4200
Wire Wire Line
	4450 4200 4450 4350
Wire Wire Line
	4450 4350 6350 4350
Wire Wire Line
	6350 4350 6350 4700
Wire Wire Line
	6350 4700 8700 4700
Wire Wire Line
	8700 4700 8700 4100
Wire Wire Line
	8700 4100 8050 4100
Wire Wire Line
	6300 4750 8750 4750
Wire Wire Line
	8750 4750 8750 2700
Wire Wire Line
	8750 2700 8050 2700
Wire Wire Line
	6300 4750 6300 3800
Wire Wire Line
	6300 3800 5100 3800
$EndSCHEMATC
