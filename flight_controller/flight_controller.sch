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
L RF_Module:ESP32-WROOM-32 U?
U 1 1 6144AADB
P 7800 3450
F 0 "U?" H 7800 5031 50  0000 C CNN
F 1 "ESP32-WROOM-32" H 7800 4940 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 7800 1950 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 7500 3500 50  0001 C CNN
	1    7800 3450
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Motion:MPU-9250 U?
U 1 1 6144F491
P 4950 3450
F 0 "U?" H 4950 2461 50  0000 C CNN
F 1 "MPU-9250" H 4950 2370 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_3x3mm_P0.4mm" H 4950 2450 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf" H 4950 3300 50  0001 C CNN
	1    4950 3450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J?
U 1 1 61452008
P 2500 3750
F 0 "J?" H 2608 4031 50  0000 C CNN
F 1 "Conn_01x03_Male" H 2608 3940 50  0000 C CNN
F 2 "" H 2500 3750 50  0001 C CNN
F 3 "~" H 2500 3750 50  0001 C CNN
	1    2500 3750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Top_Bottom J?
U 1 1 614531FB
P 2500 4800
F 0 "J?" H 2550 5117 50  0000 C CNN
F 1 "Conn_02x03_Top_Bottom" H 2550 5026 50  0000 C CNN
F 2 "" H 2500 4800 50  0001 C CNN
F 3 "~" H 2500 4800 50  0001 C CNN
	1    2500 4800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x03_Top_Bottom J?
U 1 1 61453F05
P 2500 5500
F 0 "J?" H 2550 5817 50  0000 C CNN
F 1 "Conn_02x03_Top_Bottom" H 2550 5726 50  0000 C CNN
F 2 "" H 2500 5500 50  0001 C CNN
F 3 "~" H 2500 5500 50  0001 C CNN
	1    2500 5500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
