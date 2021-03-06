EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ESP32-footprints-Shem-Lib
LIBS:espressif-xess
EELAYER 25 0
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
L AP1117-33 U?
U 1 1 5B3F6443
P 1650 3150
F 0 "U?" H 1500 3275 50  0000 C CNN
F 1 "AP1117-33" H 1650 3275 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-223-3Lead_TabPin2" H 1650 3350 50  0001 C CNN
F 3 "" H 1750 2900 50  0001 C CNN
	1    1650 3150
	1    0    0    -1  
$EndComp
$Comp
L SW_Push_Dual SW?
U 1 1 5B3F65D6
P 8900 4200
F 0 "SW?" H 8950 4300 50  0000 L CNN
F 1 "SW_Push_Dual" H 8900 3930 50  0000 C CNN
F 2 "" H 8900 4400 50  0001 C CNN
F 3 "" H 8900 4400 50  0001 C CNN
	1    8900 4200
	0    1    1    0   
$EndComp
$Comp
L Conn_01x07_Female J?
U 1 1 5B3F665F
P 10750 1800
F 0 "J?" H 10750 2200 50  0000 C CNN
F 1 "CON_I2C" H 10750 1400 50  0000 C CNN
F 2 "" H 10750 1800 50  0001 C CNN
F 3 "" H 10750 1800 50  0001 C CNN
	1    10750 1800
	1    0    0    -1  
$EndComp
$Comp
L ESP32-WROOM U?
U 1 1 5B3F638D
P 6250 4000
F 0 "U?" H 5550 5250 60  0000 C CNN
F 1 "ESP32-WROOM" H 6750 5250 60  0000 C CNN
F 2 "ESP32-footprints-Lib:ESP32-WROOM" H 6600 5350 60  0001 C CNN
F 3 "" H 5800 4450 60  0001 C CNN
	1    6250 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3150 4250 3150
Wire Wire Line
	4250 1500 4250 3150
Wire Wire Line
	4250 3150 4250 3400
Wire Wire Line
	4250 3400 5300 3400
Wire Wire Line
	900  4700 1650 4700
Wire Wire Line
	1650 4700 5300 4700
Wire Wire Line
	1650 4700 1650 3450
Wire Wire Line
	7150 4650 8700 4650
Wire Wire Line
	8700 4650 9900 4650
Wire Wire Line
	8700 4650 8700 4400
Wire Wire Line
	4250 1500 10100 1500
Wire Wire Line
	10100 1500 10550 1500
Connection ~ 4250 3150
Wire Wire Line
	10550 1700 9900 1700
Wire Wire Line
	9900 1700 9900 3900
Wire Wire Line
	9900 3900 9900 4650
Connection ~ 8700 4650
Wire Wire Line
	7150 3350 8350 3350
Wire Wire Line
	8350 3350 8350 1800
Wire Wire Line
	8350 1800 10550 1800
Wire Wire Line
	7150 3650 8900 3650
Wire Wire Line
	7150 3850 8700 3850
Wire Wire Line
	8700 3850 8700 4000
Text Label 7500 3350 0    60   ~ 0
SCL
Text Label 7500 3650 0    60   ~ 0
SDA
Wire Wire Line
	10100 1500 10300 1500
Wire Wire Line
	10300 1500 10300 1600
Wire Wire Line
	10300 1600 10550 1600
Connection ~ 10100 1500
$Comp
L SW_DPDT_x2 SW?
U 1 1 5B3F6F49
P 9300 2300
F 0 "SW?" H 9300 2470 50  0000 C CNN
F 1 "SW_DPDT_x2" H 9300 2100 50  0000 C CNN
F 2 "" H 9300 2300 50  0001 C CNN
F 3 "" H 9300 2300 50  0001 C CNN
	1    9300 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 2300 9100 2300
Wire Wire Line
	9500 2200 9500 1900
Wire Wire Line
	9500 1900 10550 1900
Wire Wire Line
	9500 2400 9700 2400
Wire Wire Line
	9700 2400 9700 2000
Wire Wire Line
	8900 3650 8900 2300
Wire Wire Line
	9700 2000 10550 2000
$Comp
L USB_A J?
U 1 1 5B3F7441
P 900 2400
F 0 "J?" H 700 2850 50  0000 L CNN
F 1 "USB_A" H 700 2750 50  0000 L CNN
F 2 "" H 1050 2350 50  0001 C CNN
F 3 "" H 1050 2350 50  0001 C CNN
	1    900  2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2200 1300 2200
Wire Wire Line
	1300 2200 1300 3150
Wire Wire Line
	1300 3150 1350 3150
Wire Wire Line
	900  2800 900  4700
Connection ~ 1650 4700
Wire Wire Line
	7150 3450 10550 3450
Wire Wire Line
	7150 3550 10550 3550
Wire Wire Line
	10100 1500 10100 3400
$Comp
L Conn_01x06_Female J?
U 1 1 5B3F902D
P 10750 3600
F 0 "J?" H 10750 3900 50  0000 C CNN
F 1 "Flashing" H 10750 3200 50  0000 C CNN
F 2 "" H 10750 3600 50  0001 C CNN
F 3 "" H 10750 3600 50  0001 C CNN
	1    10750 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 3400 10550 3400
Wire Wire Line
	10550 3900 9900 3900
Connection ~ 9900 3900
Wire Wire Line
	10550 3450 10550 3500
Wire Wire Line
	10550 3550 10550 3600
Wire Wire Line
	7150 4450 8400 4450
Wire Wire Line
	8400 4450 8400 3700
Wire Wire Line
	8400 3700 10550 3700
Wire Wire Line
	5300 3500 4950 3500
Wire Wire Line
	4950 3500 4950 2650
Wire Wire Line
	4950 2650 10000 2650
Wire Wire Line
	10000 2650 10000 3800
Wire Wire Line
	10000 3800 10550 3800
$EndSCHEMATC
