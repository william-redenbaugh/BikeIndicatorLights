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
L custom_symbols:HM18_BLE U2
U 1 1 5F7952D0
P 2700 2900
F 0 "U2" H 3228 2996 50  0000 L CNN
F 1 "HM18_BLE" H 3228 2905 50  0000 L CNN
F 2 "custom_footprints:HM18_BLE" H 2300 2600 50  0001 C CNN
F 3 "" H 2300 2600 50  0001 C CNN
	1    2700 2900
	-1   0    0    1   
$EndComp
$Comp
L custom_symbols:MPU605 U3
U 1 1 5F795E22
P 2700 5100
F 0 "U3" H 3028 5096 50  0000 L CNN
F 1 "MPU605" H 3028 5005 50  0000 L CNN
F 2 "custom_footprints:MPU6050_module" H 2500 4800 50  0001 C CNN
F 3 "" H 2500 4800 50  0001 C CNN
	1    2700 5100
	-1   0    0    1   
$EndComp
$Comp
L teensy:Teensy4.0 U1
U 1 1 5F797343
P 4950 4050
F 0 "U1" H 4950 5665 50  0000 C CNN
F 1 "Teensy4.0" H 4950 5574 50  0000 C CNN
F 2 "teensy:Teensy40" H 4550 4250 50  0001 C CNN
F 3 "" H 4550 4250 50  0001 C CNN
	1    4950 4050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J1
U 1 1 5F79C584
P 4600 6200
F 0 "J1" H 4708 6481 50  0000 C CNN
F 1 "Conn_01x03_Male" H 4708 6390 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4600 6200 50  0001 C CNN
F 3 "~" H 4600 6200 50  0001 C CNN
	1    4600 6200
	1    0    0    -1  
$EndComp
$Comp
L custom_symbols:charge_module U4
U 1 1 5F7AC93F
P 9350 3600
F 0 "U4" H 9878 3646 50  0000 L CNN
F 1 "charge_module" H 9878 3555 50  0000 L CNN
F 2 "custom_footprints:charge_module" H 9250 3100 50  0001 C CNN
F 3 "" H 9250 3100 50  0001 C CNN
	1    9350 3600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J2
U 1 1 5F7ADC00
P 2900 3550
F 0 "J2" H 2928 3526 50  0000 L CNN
F 1 "Conn_01x04_Female" H 2928 3435 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 2900 3550 50  0001 C CNN
F 3 "~" H 2900 3550 50  0001 C CNN
	1    2900 3550
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x02_Female J3
U 1 1 5F7AE9B3
P 8300 2800
F 0 "J3" H 8328 2776 50  0000 L CNN
F 1 "Conn_01x02_Female" H 8328 2685 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 8300 2800 50  0001 C CNN
F 3 "~" H 8300 2800 50  0001 C CNN
	1    8300 2800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5F7B8ED6
P 8300 2400
F 0 "J?" H 8328 2376 50  0000 L CNN
F 1 "Conn_01x02_Female" H 8328 2285 50  0000 L CNN
F 2 "" H 8300 2400 50  0001 C CNN
F 3 "~" H 8300 2400 50  0001 C CNN
	1    8300 2400
	1    0    0    -1  
$EndComp
Text Notes 8000 2300 0    50   ~ 0
charge port
Text GLabel 8850 3800 0    50   Input ~ 0
5V_MODULE
Text GLabel 8850 3700 0    50   Input ~ 0
GND_MODULE
Text GLabel 8850 3300 0    50   Input ~ 0
5V_CHRG
Text GLabel 8850 3400 0    50   Input ~ 0
GND_CHRG
Text GLabel 3200 3100 2    50   Input ~ 0
5V_MAIN
Text GLabel 3200 3000 2    50   Input ~ 0
GND_MAIN
Text GLabel 8700 5100 2    50   Input ~ 0
TEENSY_VUSB
Text GLabel 8300 5400 0    50   Input ~ 0
GND_MAIN
$Comp
L Switch:SW_Push_DPDT SW?
U 1 1 5F7BFAF3
P 8500 5200
F 0 "SW?" H 8500 5685 50  0000 C CNN
F 1 "SW_Push_DPDT" H 8500 5594 50  0000 C CNN
F 2 "" H 8500 5400 50  0001 C CNN
F 3 "~" H 8500 5400 50  0001 C CNN
	1    8500 5200
	1    0    0    -1  
$EndComp
Text GLabel 8700 4900 2    50   Input ~ 0
5V_MODULE
Text GLabel 8300 5000 0    50   Input ~ 0
5V_MAIN
Text GLabel 6050 4900 2    50   Input ~ 0
TEENSY_VUSB
Text GLabel 8700 5300 2    50   Input ~ 0
GND_MODULE
$Comp
L Connector:USB_B_Micro J?
U 1 1 5F7B785E
P 9650 2100
F 0 "J?" H 9707 2567 50  0000 C CNN
F 1 "USB_B_Micro_Charge" H 9707 2476 50  0000 C CNN
F 2 "" H 9800 2050 50  0001 C CNN
F 3 "~" H 9800 2050 50  0001 C CNN
	1    9650 2100
	1    0    0    -1  
$EndComp
NoConn ~ 9950 2100
NoConn ~ 9950 2200
NoConn ~ 9950 2300
Text GLabel 9950 1900 2    50   Input ~ 0
5V_CHRG
Text GLabel 9650 2500 3    50   Input ~ 0
GND_CHRG
Wire Wire Line
	3850 2800 3200 2800
Wire Wire Line
	3200 2900 3850 2900
NoConn ~ 9550 2500
Wire Wire Line
	3850 5200 3100 5200
Wire Wire Line
	3850 5100 3100 5100
Text GLabel 3100 5300 2    50   Input ~ 0
GND_MAIN
Text GLabel 3100 5400 2    50   Input ~ 0
5V_MAIN
Wire Wire Line
	3100 4700 3500 4700
Wire Wire Line
	3500 4700 3500 4800
Wire Wire Line
	3500 4800 3850 4800
NoConn ~ 3100 4800
NoConn ~ 3100 4900
NoConn ~ 3100 5000
$Comp
L Device:Battery_Cell BT?
U 1 1 5F7D0442
P 7600 3550
F 0 "BT?" H 7482 3554 50  0000 R CNN
F 1 "Battery_Cell" H 7482 3645 50  0000 R CNN
F 2 "" V 7600 3610 50  0001 C CNN
F 3 "~" V 7600 3610 50  0001 C CNN
	1    7600 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	8850 3600 8250 3600
Wire Wire Line
	8250 3600 8250 3850
Wire Wire Line
	8250 3850 7600 3850
Wire Wire Line
	7600 3850 7600 3750
Wire Wire Line
	7600 3450 7600 3350
Wire Wire Line
	7600 3350 8250 3350
Wire Wire Line
	8250 3350 8250 3500
Wire Wire Line
	8250 3500 8850 3500
Text GLabel 8850 3900 0    50   Input ~ 0
CHRG_KEY
Text GLabel 3850 3700 0    50   Input ~ 0
CHRG_KEY
Text GLabel 3850 4300 0    50   Input ~ 0
GND_MAIN
Wire Wire Line
	3100 3350 3550 3350
Wire Wire Line
	3550 3350 3550 3200
Wire Wire Line
	3550 3200 3850 3200
Wire Wire Line
	3850 3300 3650 3300
Wire Wire Line
	3650 3300 3650 3450
Wire Wire Line
	3650 3450 3100 3450
Wire Wire Line
	3850 3500 3650 3500
Wire Wire Line
	3650 3500 3650 3550
Wire Wire Line
	3650 3550 3100 3550
Wire Wire Line
	3850 3600 3300 3600
Wire Wire Line
	3300 3600 3300 3650
Wire Wire Line
	3300 3650 3100 3650
Text Notes 7550 5800 0    50   ~ 0
1. Everything running on battery power\n2. Battery power disconnected. Usb plugged into teensy powerning everything.
Text GLabel 6050 5100 2    50   Input ~ 0
GND_MAIN
Text GLabel 6050 3400 2    50   Input ~ 0
GND_MAIN
Text GLabel 3850 5300 0    50   Input ~ 0
RGB_D
Text GLabel 4800 6200 2    50   Input ~ 0
RGB_D
Text GLabel 4800 6100 2    50   Input ~ 0
5V_MAIN
Text GLabel 4800 6300 2    50   Input ~ 0
GND_MAIN
Text GLabel 8100 2800 0    50   Input ~ 0
5V_MAIN
Text GLabel 8100 2900 0    50   Input ~ 0
GND_MAIN
Text GLabel 8100 2400 0    50   Input ~ 0
5V_CHRG
Text GLabel 8100 2500 0    50   Input ~ 0
GND_CHRG
NoConn ~ 3850 2700
NoConn ~ 3850 3000
NoConn ~ 3850 3100
NoConn ~ 3850 3800
NoConn ~ 3850 3900
NoConn ~ 3850 4000
NoConn ~ 3850 4100
NoConn ~ 3850 4200
NoConn ~ 3850 4400
NoConn ~ 3850 4500
NoConn ~ 3850 4600
NoConn ~ 3850 4700
NoConn ~ 3850 4900
NoConn ~ 3850 5000
NoConn ~ 3850 5400
NoConn ~ 6050 5400
NoConn ~ 6050 5300
NoConn ~ 6050 5200
NoConn ~ 6050 5000
NoConn ~ 6050 4600
NoConn ~ 6050 4500
NoConn ~ 6050 4400
NoConn ~ 6050 4300
NoConn ~ 6050 4200
NoConn ~ 6050 4100
NoConn ~ 6050 3900
NoConn ~ 6050 3800
NoConn ~ 6050 4000
NoConn ~ 6050 3700
NoConn ~ 6050 3600
NoConn ~ 6050 3500
NoConn ~ 6050 3300
NoConn ~ 6050 3200
NoConn ~ 6050 3100
NoConn ~ 6050 3000
NoConn ~ 6050 2900
NoConn ~ 6050 2800
NoConn ~ 6050 2700
NoConn ~ 3850 3400
NoConn ~ 8700 5500
$EndSCHEMATC
