## About this Repository
The repo contains the hardware and software of the Small Antenna for Vital Emergency Response (SAVER) for APS SDC 2022. It consists of pluggable blocks for miniaturized antennas, an antenna testbench, a wristband which monitors vital signs and a mobile phone app which collects, analyzes, and plots the vital signs from the wristband.

## Folder Structure
* Antenna Testbench
    * Antenna Hardware
        - STL and STEP files of the pluggable antenna blocks
    * GUI
        - GUI of the antenna testbench run on a Raspberry Pi 4 which is also connected to PocketVNA.
* Mobile Phone App
    - MIT App Inventor Project File and the Released APK file
* Wristband
    * Hardware
        - Gerber Files, schematics, soldering guide, and bill of materials of the Wristband PCB
        * Wristband support printing
            - STL and STEP files of the wristband case
    * Software
        - PlatformIO project folder of the wristband firmware
