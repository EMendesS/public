# public
Dedicated to show some projects made by me.

# 1. Tupan Project

|!["ESP32 Inicialização"](https://github.com/EMendesS/Tupan/blob/TupanV3/Images/Board_preview.jpg)|
|:--:|
| *Fig. 1: Top preview* |
|!["ESP32 Inicialização"](https://github.com/EMendesS/Tupan/blob/TupanV3/Images/Board_preview_bottom.jpg)|
| *Fig. 2: Bottom preview* |

# 1.1 DESCRIPTION
The programmable circuit based on STM32f405 works as a Driver to provide power to a brushless motor, or to a brushed motor (using two terminals, and suppressing the third terminal). The MorpheusV9 module has short and long distance telemetry: It has an ESP32 - for short distances of up to 10m, enabling communication via IP, and has a LoRa SX1276 radio - for long distances of up to 15km. The system can be powered with voltage from 6 to 60 volts and has a 5v BEC, with a maximum capacity of 2.5A. Programming is done through an st-link programmer external to the module.

***
# 1.2 HARDWARES EM USO
## 1.2.1 PINOUT DO MÓDULO
## 1.2.2 INTERFACE BETWEEN MICROCONTROLLER AND DRIVER IC
### 1.2.2.1 ANALOG LINES
This video [Crosstalk Explained by Eric Bogatin](https://www.youtube.com/watch?v=EF7SxgcDfCo) summarizes the advantages of _striplines_ over _microstrips_ in terms of reducing _crosstalk_: It is recommended to use _striplines_ to reduce the effect of _crosstalk_. 

***
# 1.3 SCHEMATIC
## 1.3.1 SYSTEM POWER SUPPLY
It is important to note in this topic a current discussion: why place capacitors in parallel. In order to reduce the inductive characteristic of the PTH capacitor combination, for example, large, medium and small capacitors are used in parallel - Ex.: 10uF, 1uF and 0.10uF (to choose the decoupling capacitor, [see the calculation in the video](https://youtu.be/y4REmZlE7Jg?t=827)).
For PTH capacitors, higher capacitance values ​​indicate a larger inductive loop, since the physical construction of each type of capacitor in this combination implies a different behavior in relation to high frequencies.
However, SMD capacitors do not present the same behavior. According to [Mr. Eric Bogatin](https://www.colorado.edu/faculty/bogatin/) in this video: ["You must Unlearn what You have Learned"](https://youtu.be/y4REmZlE7Jg?t=1917), for SMD capacitors of the same size, for example 0603, each of them will present the same reactivity value at a given frequency, regardless of the capacitance value. Therefore, associating 3 0603 capacitors of different values ​​is not necessary, although the association of only 2 of them will both increase the total capacitance and decrease the inductive loop. Observe the figure below taken [from this article](https://www.slideshare.net/genarova/capacitores-35235005): 
|!["Capacitor Association"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/Capacitor_association.JPG)|
|:--:|
| *Fig. 1: Capacitor Association* |

Note above that the combination of 0805 capacitors **does not change the inductive characteristic** of the set: using only one of these 0805 capacitors the curve in the **inductive region** would remain unchanged. In the figure to the side, associating capacitors of different sizes - 0805, 0603 and 0402 - we obtain a resulting inductive characteristic that covers a larger frequency band, shown in yellow.

[Nesse vídeo](https://youtu.be/icAZlvpiJCo?t=2463), note the recommendation to place the vias between the pads and the decoupling capacitor in the case of a 4-layer-board.
|!["Capacitor Placing"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/Capacitor%20placing.JPG)|
|:--:|
| *Fig. 1: Capacitor placing* |


see LDO: https://www.analog.com/en/analog-dialogue/articles/understand-ldo-concepts.html e https://www.analog.com/en/analog-dialogue/articles/ldo-operational-corners.html

see BEAD FERRITE: https://www.analog.com/en/analog-dialogue/articles/ferrite-beads-demystified.html#:~:text=A%20ferrite%20bead%20is%20a,in%20the%20form%20of%20heat.&text=This%20forms%20a%20low%2Dpass,high%20frequency%20power%20supply%20noise.

see Power Supply Noise Rejection : https://www.idt.com/br/en/document/apn/806-power-supply-noise-rejection

see PASSIVE FILTERS: https://blog.mbedded.ninja/electronics/circuit-design/filters/passive-filters/

see Filter Technique: https://www.ti.com/lit/an/scaa048/scaa048.pdf?ts=1596749179778&ref_url=https%253A%252F%252Fwww.google.com%252F

see [essa série de 4 artigos sobre Bulk Capacitors para ponte H com cálculos](https://www.embeddedrelated.com/showarticle/588.php)

## 1.3.2 DRV8353S
### 1.3.2.1 BULK CAPACITOR
The images below compare the system performance without any and then with 6x10uF bulk capacitors.
A 29.86kHz pulse at 25% duty cycle was used on the INHA pin.
Note the reduction in the ringing level at the **low level** and the horizontalization of the **high level**, after the addition of the bulk capacitors.
|!["Capacitor Placing"](https://github.com/EMendesS/Tupan/blob/main/Images/DRV8353%20OSC%20IMG/scope_64.png)|
|:--:|
| *Fig. 1: No bulk capacitors added* |

|!["Capacitor Placing"](https://github.com/EMendesS/Tupan/blob/main/Images/DRV8353%20OSC%20IMG/scope_66.png)|
|:--:|
| *Fig. 1: With 6x10uF bulk capacitors added* |

### 1.3.2.2 SWITCHING
Note that during the switching time, the operation occurs with some peculiar behaviors, as shown below, meaning:

* Region 1 - SHA and GHA are equal because in this short period, both A-phase FETs are OFF. Also, there's a leakage current flowing through the high-side FET's body diode, which explains the slightly higher voltage level in the region 1. Thus, in there, _SHA = Vdrain voltage + 0.7v_ , which Vdrain is 15v; 
* Region 2 - When the difference between GHA and SHA reaches Vgs(th), the FET is switched on and consequently SHA goes to VM level, which explains the slightly lower level on SHA, when compared to SHA in Region 1. 


| <img src="https://github.com/EMendesS/Tupan/blob/main/Images/DRV8353%20OSC%20IMG/switching_1.png" width="450" height="280"> |
|:--:|
| *Fig 1 - Files to exclude* |

## 1.3.3 USB-C DESIGN
The use of type A and B connectors is becoming increasingly scarce. The industry is migrating to USB-C, following the trend of using this new technology. [Here](https://www.beyondlogic.org/usbnutshell/usb2.shtml) we see a compilation of important information about USB technology, in general, to introduce ideas on the subject. ([reference.](https://dubiouscreations.com/2021/04/06/designing-with-usb-c-lessons-learned/))

([USB-C connector datasheet](https://github.com/EMendesS/Tupan/blob/TupanV3/Documents/datasheets/Global_Connector_Technology_usb4105-3106202.pdf))

### 1.3.3.1 CHOICE OF RESISTORS
In the document [USB Type-C Specs](https://www.usb.org/sites/default/files/USB%20Type-C%20Spec%20R2.0%20-%20August%202019.pdf), at 2.5 Vconn, on page 36, it reads:


> VCONN functionally differs from VBUS in that it is isolated from the other end of the
> cable. VCONN is independent of VBUS and, unlike VBUS which can use USB PD to support
> higher voltages, VCONN voltage stays within the range of 3.0 to 5.5 V (vVCONNValid).

Therefore, to have a dynamic, changeable voltage, VBUS is used, since VCONN remains unchanged between 3v and 5.5v.

In 4.2 of the same document, there is a description of the USBC pins.

In [Overview of USB Type-C and Power Delivery technologies](https://www.st.com/resource/en/technical_article/dm00496853-overview-of-usb-type-c-and-power-delivery-technologies-stmicroelectronics.pdf), see page 8, item 5, table 3 shows the Rd values. The resistor Rd is used on both DC pins (C1 and C2) to inform the host of the system's voltage and current capacity.
In addition, these resistors indicate whether the cable has been plugged in, as well as its orientation, according to item 7 of the same document.
The base values ​​of 5k1Ω were chosen for the resistors on both DC pins.

_Note: Solder the 5k1Ω resistors only if there is no controller for these pins._

### 3.3.2 TRANSIENT PROTECTION
In the Texas Instruments video [What is a Transient Voltage Suppressor (TVS) Diode?](https://training.ti.com/what-transient-voltage-suppressor-tvs-diode#:~:text=The%20reverse%20stand%2Doff%20voltage%20is%20the%20operating%20voltage%20you,to%20ensure%20proper%20data%20transmission.), we have a lesson on the most important parameters for protecting circuits against ESD, using TVS (Transient Voltage Suppressor) diodes.
Some of the important points to be observed in the datasheet:

* Vrwm - Reverse Stand-off Voltage: This is the highest voltage that the data pin, for example, will reach without being influenced by the action of the TVS diode. If a data line goes from 0 to 5v, the Vrwm value must be at least 5v, to maintain signal integrity.

* Vbr - Breaking Down Voltage: Voltage at which the TVS diode begins to conduct significantly, providing transient protection (ESD, EFT and Surge). Choose Vrb slightly higher than Vrwm, so that protection begins immediately after the voltage rises above the level determined by Vrwm.

* Vclamp - Clamp Voltage: This is the voltage at which the TVS diode will be at the moment a transient occurs.

* Capacitance: This is the "amount" of capacitance added to the line by the TVS diode. Typically, for 5Gbps, the recommended added capacitance should be less than 0.5pF. For 3Gbps, less than 0.8pF, according to the table in the previous link.

According to the STM32F405 datasheet, in [Table 58. USB OTG FS DC electrical characteristics](https://www.st.com/resource/en/datasheet/dm00037051.pdf), note that the operating limits for the **FULL SPEED USB** mode of the _data line_ are up to 3.6v, so a TVS diode with Vrwm = 3.6v is sufficient - like the [TPD1E01B04](https://www.ti.com/lit/ds/symlink/tpd1e01b04.pdf?HQS=TI-null-null-digikeymode-df-pf-null-wwe&ts=1598591481142).

For the transient protection of the VBUS pin, a TVS diode with Vrwm = 20.0v was chosen.

For the DC pins, it was chosen to use zener diodes that protect such pins from cables with incorrect specifications, according to [AN5086](https://www.onsemi.cn/pub/Collateral/AN-5086-D.PDF). At the end of this document, there is [AN6102](https://www.onsemi.com/pub/Collateral/AN-6102.pdf.pdf), which suggests zener diodes, after considering the voltages on the DC line. In the worst case - where Rp = 4k7, Rd = 5k1 and VBUS = 20v - the voltage on the DC line would be given by the equation [(1)](https://www.onsemi.com/pub/Collateral/AN-6102.pdf.pdf):

><img src="https://render.githubusercontent.com/render/math?math=Vcc = 20 * (5k1/9k8)">

Therefore, since Vcc is the voltage on the DC bus and is equal to 10.4v according to the calculation above, we need to add a zener diode to limit the DC voltage (absolute max < 6v).

A zener diode with Vz = 5v1 and 200mW power was chosen.

_Note: The diode chosen must have an intrinsic capacitance of 200pF to 600pF_

Varistors and zeners are used to eliminate ESD, but the response of these components is less efficient. On the one hand, varistors eliminate transients more quickly than zener diodes, but reach higher voltages. On the other hand, zeners take longer to dissipate the energy of transients, but reach lower voltages than varistors.
In any case, TVS diodes have a faster and more efficient response to transients, compared to varistors and zeners.

### 1.3.3.3 HOST VBUS VOLTAGE DETECTION

See [AN4879 - Introduction to USB hardware and PCB guidelines using STM32 MCUs](https://www.st.com/resource/en/application_note/dm00296349-usb-hardware-and-pcb-guidelines-using-stm32-mcus-stmicroelectronics.pdf) for a hardware guideline. Note that item 2.6 - sensing detection mentions that VBUS must be used on pin A9 to detect the cable plug. In the Reference Manual [RM0090](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf) there is a complementary description of the use in item 34.5. To this end, a common-drain circuit with MOSFET was used to establish a voltage limit lower than 4v on pin PA9, as discussed in [item 2.6](https://www.st.com/resource/en/application_note/dm00296349-usb-hardware-and-pcb-guidelines-using-stm32-mcus-stmicroelectronics.pdf).

_Note: The same item above states: "When VBUS sensing feature is enabled, PA9 should be left at their default state (floating input), not as alternate
function."_

To [calculate the value of the zener limiting resistor](https://www.electronics-notes.com/articles/electronic_components/diode/zener-diode-circuits-design.php), considering that the VBUS voltage can reach 20 volts, a 3.3v zener diode was selected (read the previous subitem, where there is an upper voltage limitation of 4v for PA9). Thus, considering the maximum dissipation power of the diode of 500mW, we have:

For the zener diode [BZT52C3V3](https://www.mouser.ca/datasheet/2/258/BZT52C2V4_7eBZT52C75(500mW)(SOD-123)-V1-1626089.pdf), having understood the [main specifications of zener diodes](https://www.electronics-notes.com/articles/electronic_components/diode/zener-diode-datasheet-specifications-parameters.php#:~:text=Current%20%3A%20The%20current%2C%20IZM%2C,typical%20leaded%20400%20mW%20device.):

><img src="https://render.githubusercontent.com/render/math?math=0.500 > 3.3 * i, with i < 150mA">, therefore, to limit dissipation, consider half the maximum current:

><img src="https://render.githubusercontent.com/render/math?math=i < 150mA/2, therefore: i < 75mA">;
>Therefore, <img src="https://render.githubusercontent.com/render/math?math=20.0 - 3.3 < R * 75mA">;

>For the case of VBUS equal to 5v, for the diode to work correctly, the current through it must typically be greater than 5mA. We will have:
><img src="https://render.githubusercontent.com/render/math?math=5.0 - 3.3 > R * (5mA)">;

>Then, <img src="https://render.githubusercontent.com/render/math?math=223 < R < 340">. A 270Ω resistor was chosen.

A 10k resistor was placed to limit the current on the PA9 input pin. This resistor also ensures that the current through the zener diode will be practically the same as that passing through the 270Ω resistor - this ensures that the current in the zener diode will always be above 5mA.

_Note: Depending on the chosen value, different effects occur: if the resistor has a low value, the dissipation in the diode increases; if the value is high, the current in the zener decreases, and the minimum current required for the diode to function correctly may not be reached. Accordingly, the current available to supply the load will also be increasingly limited as the resistance value increases._

### 1.3.3.4 DATA LINE
In order to eliminate noise, a _COMMON MODE CHOKE_ type inductor was chosen, explained in detail [In This Article](https://industrial.panasonic.com/tw/ss/technical/n4)

## 1.3.4. ESP32
 • [Datasheet ESP32](https://www.mouser.ca/datasheet/2/891/esp32_datasheet_en-1223853.pdf)
 
 • [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)

 • [Datasheet WROOM 32](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf)
 
 • [Hardware Design Guidelines](https://www.espressif.com/sites/default/files/documentation/esp32_hardware_design_guidelines_en.pdf)
 
 • [ESP32 AT Instruction Set and Examples](https://www.espressif.com/sites/default/files/documentation/esp32_at_instruction_set_and_examples_en.pdf)
 
 • [ESP32 LoRa V2](https://resource.heltec.cn/download/WiFi_LoRa_32/V2/WiFi_LoRa_32_V2(433%2C470-510).PDF) 

The [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf) provides detailed information about the chip's operation and also important data for configuring the hardware, in addition to its [Datasheet](https://www.mouser.ca/datasheet/2/891/esp32_datasheet_en-1223853.pdf). The design features chosen from the manufacturer's data will be discussed below.

### 1.3.4.1 COMUNICAÇÃO SPI
Observe the figure below, with the spatial arrangement of the pins corresponding to the SPI communication available on the chip:
|!["SPI Pins"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/SPI_ORG_ESP32.png)|
|:--:|
| *Fig. 1: SPI Pins* |

According to the [Datasheet](https://www.mouser.ca/datasheet/2/891/esp32_datasheet_en-1223853.pdf), the chip supports QSPI, which was used for protocol communication with the external FLASH memory, as shown in the image below:
|!["FLASH Pinout"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/SPI%20Flash.JPG)|
|:--:|
| *Fig. 1: FLASH SPI Conection* |


According to [AN4760 - Quad-SPI interface on STM32 microcontrollers and microprocessors](https://www.st.com/resource/en/application_note/dm00227538-quadspi-interface-on-stm32-microcontrollers-and-microprocessors-stmicroelectronics.pdf) the STM32F405RG processor is not included in Table 1 of this document, as it does not support QSPI communication. Therefore, the physical connection of the SPI bus between the STM32 and the ESP32 was established following the pinout table below:
|!["Pinout ESP32"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/Pinout%20ESP32.JPG)|
|:--:|
| *Fig. 1: ESP32 SPI Pinout* |

Consecutively, the pattern as shown in the image below was used:

|!["dominating_sets_example2"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/Tabela%2027%20ESP32%20Technical%20Reference%20Manual.JPG)|
|:--:|
| *Fig. 2: ESP32 SPI Types and Correspondences* |

### 1.3.4.2 INICIALIZATION
Na inicialização do chip ESP32 deve-se garantir um determinado estado para os pinos da figura abaixo, retirada do datasheet:
|!["ESP32 Inicialização"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/ESP32%20Strapping%20Pins.JPG)|
|:--:|
| *Fig. 2: ESP32 Initialization Pins* |

### 1.3.4.3 COMUNICAÇÃO RÁDIO
A [Pro-Ob-440](https://datasheet.octopart.com/PRO-OB-440-Proant-AB-datasheet-138897411.pdf) type antenna and a [MHF3 connector](https://www.mouser.ca/datasheet/2/398/RECE_20369_001E_01-2005142.pdf) were used in the 2.4GHz transmission line in order to tune/check the wave propagation ([here](http://www.wellshow.com/products/mini-coax-connector/ipex-connector/) there is a compatibility table and the types of IPX connectors).

## 1.3.5 STM32F405RG
### 1.3.5.1 COMUNICAÇÃO SPI
In order to simplify the design, it is important to know the processor hardware and highlight the SPI communication pins, as in the figure below:
|!["STM32 SPI Organization"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/SPI%20Org.JPG)|
|:--:|
| *Fig. 2: STM32 SPI Organization* |

### 1.3.5.2 COMUNICAÇÃO I2C
I2C communication is done through a clock line (SCL) and a single data line (SDA), which only allows sending information in the _HALF-DUPLEX_ configuration, unlike the SPI and UART protocols. However, one of the advantages of the I2C protocol is the possibility of using several chips "hanging" on the bus, so that data collisions do not imply electrical conflict, since the implementation of the I2C communication block consists of transistors in _OPEN DRAIN_ mode, as shown in the schematic below:

|!["I2C Block"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/Esquema%20I2C.JPG)|
|:--:|
| *Fig. 2: I2C Block* |

A fim de simplificar o design, é importante conhecer o hardware do processador e evidenciar os pinos de comunicação I2C, como na figura abaixo: 
|!["STM32 I2C Organization"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/LQFP64%20I2C%20Pins.png)|
|:--:|
| *Fig. 2: STM32 I2C Organization* |

### 1.3.5.3 BOOT
There are 3 Boot modes:

|!["Boot Modes"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/Boot%20Table.JPG)|
|:--:|
| *Fig. 1: Boot Modes* |

The _FLASH_ memory has a main block (_Main FLASH_) divided into sectors, from where the Boot comes in the first option. In addition, there is also a region of the _FLASH_ designated as the System Memory (_System Memory_), which can be selected for Boot. The _FLASH_ is accessed by AHB I-Code and D-Code, to erase/program and read/write protection mechanisms.

_Note: In STM32F42xxx and STM32F43xxx devices, when booting from the main Flash memory,
the application software can either boot from bank 1 or from bank 2. By default, boot from
bank 1 is selected._

The _SRAM_ has 3 divisions, of which _SRAM1_ and _SRAM2_ are accessed by all AHB masters and SRAM3 is accessed only by the CPU. When SRAM is selected for Boot, the CPU accesses this memory through I-Code and D-Code.

_Obs.: When the device boots from SRAM, in the application initialization code, you have to
relocate the vector table in SRAM using the NVIC exception table and the offset register._
 
### 1.3.5.4 BOOTLOADER EMBARCADO
The Bootloader is written to System Memory by the manufacturer during production. It is used to reprogram the _FLASH_ memory from the below interfaces according to [RM0090](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf):

• USART1 (PA9/PA10)

• USART3 (PB10/11 and PC10/11)

• CAN2 (PB5/13)

• USB OTG FS (PA11/12) in Device mode (DFU: device firmware upgrade).

Therefore, to program the System Memory using the USART3 connection, it was chosen to connect the ESP32 and STM32 via the PB10/11 pair, in order to allow the STM32 to be reprogrammed via WIFI.
In Table 22 of [AN2606](https://bin.jvnv.net/f/Qjbxx), there are details on how to use USART3 to reprogram the System Memory. In the same document, note that:

> The RX pins of the peripheral unused in this bootloader have to be kept at a known (low
or high) level, and should not be left floating during the detection phase as described
below:
    If USART3 (on PB10/PB11) is used to connect to the bootloader: the USART1_RX
  (PA10), USART3_RX (PC11), CAN2_RX (PB05), OTG_FS_DM (PA11) and
  OTG_FS_DP (PA12) have to be kept at a high or low level and must not be left
  floating during the detection phase

*TUTORIAL:* [Program STM32F4 with UART](http://stm32f4-discovery.net/2014/09/program-stm32f4-with-uart/)  

### 1.3.5.5 FEEDBACK SIGNAL CAPTURE
see [OpAmp Filter Design: Do's and Don'ts](http://iowahills.com/A6OpAmpFilterHelpFile.html)

## 1.3.6 H-BRIDGE
### 1.3.6.1 PROTECTIONS
#### 1.3.6.2.1 SNUBBERs
* see [Power Tips: Calculate an R-C snubber in seven steps](https://e2e.ti.com/blogs_/b/powerhouse/posts/calculate-an-r-c-snubber-in-seven-steps)

* see [[FAQ] Proper RC Snubber Design for Motor Drivers](https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/991693/faq-proper-rc-snubber-design-for-motor-drivers)

## 1.3.7 SX1276 LoRa Module
### 1.3.7.1 RF Design
In this article [Using Via Fences for Crosstalk Reduction in PCB
Circuits](https://github.com/EMendesS/Tupan/blob/TupanV2/Bibliography/Using%20via%20fences%20for%20crosstalk%20reduction%20in%20PCB%20circuits.pdf) the study shows how the via fencing is effective to couple with the microstrip, considering trace width, via diameter etc. From this article, the images below show the relation between via configurations and S parameters, where signal is applied to P1 and collected from P2 terminal:   

For 3 vias case, consider the distance of adjacent vias equal 23mm. Executing the same calculations from above, a via spacing a=23mm leads to fr = 3.54GHz, depicted in the figure b). In addition, for the 3-via case, the resonances (minima in S21 signature) become weaker as the line spacing increases, but for the 12-via case, the spacing does not affect the magnitudes of S21 significantly other than a small ripple in the insertion loss when S = 0.5mm;

This comparison is made from D/Wt = 0.169, 0.339, 0.677, and 0.847, where Wt is fixed to 3mm. It is observed that the resonances are shifted towards
lower frequencies, as D/Wt decreases. Also, a wider resonance is also observed when via diameter (D) is much smaller than the via fence width (Wt).

For the same study, now consider two microstrips, one active and the other a victim, to whom _S_ parameters are valid, as shown below:
|!["Microstrip"](https://github.com/EMendesS/Tupan/blob/TupanV2/Images/microstrip_via_study4.png)|
|:--:|
| *Fig. 2: TIMERs* |

Some times few vias between two microstrips can improve signal integrity, compered to a no-via fencing, as shown below:
|!["Microstrip"](https://github.com/EMendesS/Tupan/blob/TupanV2/Images/microstrip_via_study5.png)|
|:--:|
| *Fig. 2: TIMERs* |

***
# 1.4 PINOUT
|!["TIMERs"](https://github.com/EMendesS/Morpheus-V9/blob/MorpheusV9/Images/Timer%20Pins.jpg)|
|:--:|
| *Fig. 2: TIMERs* |

***

# 1.9 REFERENCES
[1] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[2] https://resource.heltec.cn/download/WiFi_LoRa_32/V2/WiFi_LoRa_32_V2(433%2C470-510).PDF
[3] https://www.murata.com/en-us/products/emc/emifil/library/knowhow/basic/chapter01-p1
[4] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[5] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[6] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[7] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[8] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[9] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[10] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[11] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[12] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[13] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[14] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[15] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[16] https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lines
[17] https://www.digipart.com/

***


