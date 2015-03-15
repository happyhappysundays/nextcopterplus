# Getting Started Guide #
Please read the ['Frequently Ask Questions'](OpenAero2_FAQ.md) before loading OpenAero2 onto a KK2 board.

## Where can I find OpenAero2 firmware? ##
The latest files are always listed at the bottom of the first post in the RCGroups OpenAero2 thread.
  * [Version 1.2 beta 3 - test release](http://nextcopterplus.googlecode.com/files/OpenAero2%20V1.2%20Beta%203.zip)
  * [Version 1.1.1 - Final release](http://nextcopterplus.googlecode.com/files/OpenAero2%20V1.1.1.zip)
  * [Version 1.0 - Final release](https://nextcopterplus.googlecode.com/files/OpenAero2%20V1.0.zip)

A link to the latest beta version of OpenAero2 can be found on [RC Groups.com](http://www.rcgroups.com/forums/showpost.php?p=22394780&postcount=1)

## How do I reflash my KK2 board? ##
  1. Download the latest version of [OpenAero2](http://www.rcgroups.com/forums/showpost.php?p=22394780&postcount=1).
  1. Extract the 'OpenAero2.hex' file an 'OpenAero2' folder on your desktop,
  1. Download the latest [KKMulticopter Flashtool](http://lazyzero.de/en/modellbau/kkmulticopterflashtool#download).
  1. Extract the contents of the zip file to a 'flash tool' folder on your desktop.
  1. Connect the 6 pin plug of the [USBasp](http://www.hobbyking.com/hobbyking/store/uh_viewitem.asp?idproduct=21321) to the ISP programming header labeled JP1, in the lower left of the KK2 board, with pin 1 towards the edge of the board.
  1. Connect the USBasp to a free port on your computer. A green light on the USBasp should now be lit and the screen of KK2 board should now display text. If the screen is lit without any text, follow the [designers tips](http://www.rcgroups.com/forums/showpost.php?p=23511469&postcount=8334), else you can [return](http://www.hobbyking.com/hobbyking/store/supportform.asp) the board to your nearest HobbyKing warehouse.
  1. Run 'kkMulticopterFlashTool.cmd' from the 'flash tool' folder on your desktop,
  1. Select 'USBasp' from the programmer drop-down menu,
  1. Select 'Hobbyking KK2.0 (32kB flash)' from the controller drop-down menu,
  1. On the 'Flashing firmware' tab, click on the floppy disc icon,
  1. Navigate to the 'OpenAero2' folder on your desktop,
  1. Select the 'OpenAero2.hex' file and click 'open',
  1. Maximize the application window to see the writing and reading progress in the next step,
  1. Click the green icon next to the floppy disc to start the writing process.
  1. Once complete, ensure the read process has a 100% success rate.
  1. Close the 'KKMulticopter Flashtool' and remove the USBasp from the computer.

## How do I mount the KK2 board in my aeroplane? ##
For best results, the KK2 board should be securely mounted with [M3 nylon spacers](http://www.hobbyking.com/hobbyking/store/RC_PRODUCT_SEARCH.asp?searchType=10&strSearch=M3+Nylon+Threaded+Spacer&location=HK&idCategory=&sortBy=Price1&NumPerPage=20&currentPage=) and [nylon screws](http://www.hobbyking.com/hobbyking/store/__17847__M3x8_Nylon_Screws_10pcs_bag_.html).

As shown in the following diagram, the two piezo buzzer output pins indicate the front of the KK2 board, while the four buttons indicate the rear of the board.

![http://i.imgur.com/tIzoUNQ.jpg](http://i.imgur.com/tIzoUNQ.jpg)

OpenAero2 now supports five different board orientations. See the [Basic User Guide](OpenAero2_UserGuide_V2.md) for more info.

## How do I connect the KK2 board in my model? ##
The KK2 board should be installed within your aircraft between your receiver and servos, to stabilize an aeroplane or a camera gimbal.

## How do I power the KK2 board? ##
Unlike other boards, the KK2 board benefits from the stability of two regulated power supplies, one to prevent receiver brown-outs and a second to power the servos.

**Do not connect the 5V supply from M1, or the receiver to the servos on M2-M8, else the electrical noise from the servos in flight will cause the KK2 board to reboot and crash your aircraft.**

### KK2 Board ###
A 5 volt regulated power supply must be connected to the pin marked M1. Typically, this will be the 5 volt supply from the ESC's internal BEC.
This will provide power to both the KK2 board and the receiver.

### Servos ###
A second BEC must be connected to any of the pins marked M2-M8, to power the servos.
Depending on the specification of your servos, typically a 5 to 6 volt external BEC will be used.

We recommend the [Turnigy 3A 5V/6V](http://www.hobbyking.com/hobbyking/store/__4319__TURNIGY_3A_UBEC_w_Noise_Reduction.html) & [HobbyKing HKU5 5A 5V](http://www.hobbyking.com/hobbyking/store/__16663__HobbyKing_HKU5_5V_5A_UBEC.html) external UBECs from Hobbyking.

## How do I connect my receiver? ##
The KK2 board supports either common multi-wire PWM receivers or single-wire CCPM or Serial receivers (S.Bus, Satellite, Xtreme).

Simple wiring can be achieved when the ESC is connected to the pins marked M1 and throttle pass-through is selected. This allows the ESC's internal 5 volt BEC to power the KK2 board and receiver while OpenAero2 can control the ESC's throttle.

### PWM Receivers ###
The KK2 board limits common multi-wire PWM receivers to the following inputs.
  * Aileron - Required
  * Elevator - Required
  * Throttle - Optional
  * Rudder - Required
  * Auxiliary - Flight mode switch (required)

Since V1.1 Beta 6, the throttle input channel can be used for the following purposes.
  * **Throttle** - Pass-through to M1 with receivers failsafe,
  * **Throttle** - Pass-through to M1 with OpenAero2's internal failsafe. Orange [R610](http://www.hobbyking.com/hobbyking/store/__11965__OrangeRx_R610_Spektrum_DSM2_6Ch_2_4Ghz_Receiver_w_Sat_Port_.html) receiver only.
  * **Second Aileron** - For use with pre-mixed flaperons from transmitter.

### CPPM Receivers ###
Single-wire receivers can be connected to the **rudder** input for both signal and power.
While there are many brands available, the following have been tested.

  * [FrSky V8R7-SP](http://www.frsky-rc.com/ShowProducts.asp?id=48)
  * [FrSky D4R-II](http://www.frsky-rc.com/ShowProducts.asp?id=119)
  * [FrSky D8R-XP](http://www.frsky-rc.com/ShowProducts.asp?id=126)

### Serial receivers ###
Serial receivers connect to the KK2 board via the **Throttle** input. Some of these require a cable adapter to function.

For the Spektrum Satellite, please use the following cable available from HobbyKing.
[ZYX-S DSM2/DSMJ Satellite Receiver Cable](http://www.hobbyking.com/hobbyking/store/__24524__ZYX_S_DSM2_DSMJ_Satellite_Receiver_Cable.html)

![http://www.hobbyking.com/hobbyking/store/catalog/24524-main.jpg](http://www.hobbyking.com/hobbyking/store/catalog/24524-main.jpg)

**Spektrum Satellite setup**

![http://i.imgur.com/At2S8zS.jpg](http://i.imgur.com/At2S8zS.jpg)

For the Futaba/OrangeRx S.Bus, please use the following cable available from HobbyKing.
[ZYX-S S.BUS Connection Cable](http://www.hobbyking.com/hobbyking/store/__24523__ZYX_S_S_BUS_Connection_Cable.html)

**Futaba S.bus setup**

![http://i.imgur.com/BMlI60q.jpg](http://i.imgur.com/BMlI60q.jpg)

### Binding satellite receivers ###
OpenAero2 makes it easy to bind Satellite receivers without the need for a separate host receiver.
  1. Ensure that the Satellite receiver is connected via the adapter cable.
  1. Hold down button 4 of the KK2 board.
  1. While holding down button 4, apply power to the KK2 board. The attached Satellite receiver should start to blink rapidly. Release button 4.
  1. Switch on the TX in binding mode. The is usually done by holding down the Bind button on the TX module. The TX module should flash.
  1. After a few seconds the Satellite will complete the binding process and will show a solid lit LED.

<a href='http://www.youtube.com/watch?feature=player_embedded&v=c_x0GtdCm6o' target='_blank'><img src='http://img.youtube.com/vi/c_x0GtdCm6o/0.jpg' width='425' height=344 /></a>

### Channel Order ###
OpenAero2 supports the JR/Spektrum, Fubata and Spektrum Satellite channel allocations.

| **JR/Spektrum** | **Futaba**| **Satellite** |
|:----------------|:----------|:--------------|
|CH1: Throttle |CH1: Aileron #1| CH1: Flap |
|CH2: Aileron #1 |CH2: Elevator| CH2: Aileron |
|CH3: Elevator |CH3: Throttle| CH3: Elevator |
|CH4: Rudder |CH4: Rudder| CH4: Rudder |
|CH5: Gear switch |CH5: Gear switch| CH5: Gear |
|CH6: Aux 1 |CH6: Aileron #2| CH6: Throttle |
|CH7: Aux 2 |CH7: Aux 1| CH7: AUX1 |
|CH8: Aux 3  |CH8: Aux 2 | CH8: AUX2 |

## How do I connect my servos? ##
The OpenAero2 mixer presets support either conventional 'aeroplane mode' or 'flying-wing mode'.

All unassigned channels, designated as 'none' by the mixer presets, can be manually customized for other purposes.

### Aeroplane Mode ###
The following channel assignment will apply to most conventional aeroplanes.

  * M1: Throttle - for CPPM users
  * M2: None
  * M3: None
  * M4: None
  * M5: Elevator
  * M6: Left aileron
  * M7: Right aileron
  * M8: Rudder

Where a single aileron servo is required, M6 will become the master and M7 will remain unassigned as 'none'.

### Flying-Wing Mode ###
  * M1: Throttle - for CPPM users
  * M2: None
  * M3: None
  * M4: None
  * M5: Elevator
  * M6: Left elevon
  * M7: Right elevon
  * M8: Rudder

### Servo Rate ###
Depending on the servo rate selected in the General Menu, the type of receiver you have, and whether Camstab mode is activated, the servo output rate varies. The various rates are illustrated in the table below.
The rate varies from as low as your receiver sends, to up to 300Hz.

Do not select a servo rate of "HIGH" unless you are sure that your servos are rated for it. Typically, analog servos should always be used with the servo rate set to "Low".

| **Receiver type** | **Servo rate setting**| **Actual servo rate** |
|:------------------|:----------------------|:----------------------|
| None (Camstab ON)| HIGH | ~300Hz `*`|
| Satellite (11ms) | HIGH | 90Hz `*`|
| S.Bus (11ms) | HIGH | 90Hz `*`|
| 50Hz PWM | N/A | 50Hz |
| None (Camstab ON)| LOW | 50Hz |
| Satellite (Any)| LOW| 45Hz |
| S.Bus (Any) | LOW| 45Hz |
| 27Hz CPPM | N/A | 27Hz |

**`*`Warning: Do not use these rates with analog servos.**

