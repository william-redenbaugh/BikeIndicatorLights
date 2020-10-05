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
P 1850 2100
F 0 "U2" H 2378 2196 50  0000 L CNN
F 1 "HM18_BLE" H 2378 2105 50  0000 L CNN
F 2 "custom_footprints:HM18_BLE" H 1450 1800 50  0001 C CNN
F 3 "" H 1450 1800 50  0001 C CNN
	1    1850 2100
	1    0    0    -1  
$EndComp
$Comp
L custom_symbols:MPU605 U3
U 1 1 5F795E22
P 1250 2900
F 0 "U3" H 1578 2896 50  0000 L CNN
F 1 "MPU605" H 1578 2805 50  0000 L CNN
F 2 "custom_footprints:MPU6050_module" H 1050 2600 50  0001 C CNN
F 3 "" H 1050 2600 50  0001 C CNN
	1    1250 2900
	1    0    0    -1  
$EndComp
$Comp
L teensy:Teensy4.0 U1
U 1 1 5F797343
P 3750 4800
F 0 "U1" H 3750 6415 50  0000 C CNN
F 1 "Teensy4.0" H 3750 6324 50  0000 C CNN
F 2 "teensy:Teensy40" H 3350 5000 50  0001 C CNN
F 3 "" H 3350 5000 50  0001 C CNN
	1    3750 4800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J1
U 1 1 5F79C584
P 6300 2450
F 0 "J1" H 6408 2731 50  0000 C CNN
F 1 "Conn_01x03_Male" H 6408 2640 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6300 2450 50  0001 C CNN
F 3 "~" H 6300 2450 50  0001 C CNN
	1    6300 2450
	1    0    0    -1  
$EndComp
$Comp
L custom_symbols:charge_module U4
U 1 1 5F7AC93F
P 6950 3650
F 0 "U4" H 7478 3696 50  0000 L CNN
F 1 "charge_module" H 7478 3605 50  0000 L CNN
F 2 "custom_footprints:charge_module" H 6850 3150 50  0001 C CNN
F 3 "" H 6850 3150 50  0001 C CNN
	1    6950 3650
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J2
U 1 1 5F7ADC00
P 1400 4700
F 0 "J2" H 1428 4676 50  0000 L CNN
F 1 "Conn_01x04_Female" H 1428 4585 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 1400 4700 50  0001 C CNN
F 3 "~" H 1400 4700 50  0001 C CNN
	1    1400 4700
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J3
U 1 1 5F7AE9B3
P 1450 4000
F 0 "J3" H 1478 3976 50  0000 L CNN
F 1 "Conn_01x02_Female" H 1478 3885 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x02_P2.54mm_Vertical" H 1450 4000 50  0001 C CNN
F 3 "~" H 1450 4000 50  0001 C CNN
	1    1450 4000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J?
U 1 1 5F7B8ED6
P 3600 1900
F 0 "J?" H 3628 1876 50  0000 L CNN
F 1 "Conn_01x02_Female" H 3628 1785 50  0000 L CNN
F 2 "" H 3600 1900 50  0001 C CNN
F 3 "~" H 3600 1900 50  0001 C CNN
	1    3600 1900
	1    0    0    -1  
$EndComp
Text Notes 3300 1800 0    50   ~ 0
charge port
Text GLabel 6450 3850 0    50   Input ~ 0
5V_MODULE
Text GLabel 6450 3750 0    50   Input ~ 0
GND_MODULE
Text Notes 4150 3250 0    50   ~ 0
Cut VIN-VUSB trace
Text GLabel 6450 3350 0    50   Input ~ 0
5V_CHRG
Text GLabel 6450 3450 0    50   Input ~ 0
GND_CHRG
Text GLabel 1350 1900 0    50   Input ~ 0
5V_MAIN
Text GLabel 1350 2000 0    50   Input ~ 0
GND_MAIN
Text GLabel 7700 5000 2    50   Input ~ 0
5V_MAIN
Text GLabel 7300 5300 0    50   Input ~ 0
5V_USB
$Comp
L Switch:SW_Push_DPDT SW?
U 1 1 5F7BFAF3
P 7500 5100
F 0 "SW?" H 7500 5585 50  0000 C CNN
F 1 "SW_Push_DPDT" H 7500 5494 50  0000 C CNN
F 2 "" H 7500 5300 50  0001 C CNN
F 3 "~" H 7500 5300 50  0001 C CNN
	1    7500 5100
	1    0    0    -1  
$EndComp
Text GLabel 7700 4800 2    50   Input ~ 0
5V_CHRG
Text GLabel 7300 4900 0    50   Input ~ 0
5V_VUSB
Text GLabel 4850 5650 2    50   Input ~ 0
5V_USB
Text GLabel 7700 5200 2    50   Input ~ 0
5V_MODULE
Text GLabel 4850 5750 2    50   Input ~ 0
5V_MAIN
Text Notes 8500 4600 0    50   ~ 0
Modes\n1. plug in usb to charge\n2. plug in usb to program\n3. run off of battery\n
$EndSCHEMATC
