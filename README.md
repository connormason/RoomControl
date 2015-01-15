# RoomControl
(Work in progress) Code for the Arduinos that control my room

# Consists of 4 parts:
  1. **RelayModule**: boxes that contain 4 outlets on them, each outlet being individually controllable by an internal microprocessor through an RF signal sent from the DoorPanel
  2. **DoorPanel**: the control module that sits next to my door like a light switch, allowing for me to choose between different modes for the lighting in the room (turning on and off relays as well as sending lighting signals to the DeskModule
  3. **DeskModule**: the Arduino that controls the grid of LEDs contructed on my ceiling (the main source of light in the room) which sits in a custom made enclosure with a 12v power supply and breadboard with a series of MOSFETs for power control
  4. **DeskModuleRF**: an Arduino (on a breadboard) that sits in the same enclosure as the DeskModule that receives the RF signals from the DoorPanel and transmits them to the DeskModule Arduino (since SPI was not playing nicely with the TLC5940 chips used for LED strip control alongside the SPI RF module)
  
