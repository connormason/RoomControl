# RoomControl
(Work in progress) Code for the Arduinos that control my room

Consists of 3 parts:
  1) RelayModule: boxes that contain 4 outlets on them, each outlet being individually controllable by an internal microprocessor through an RF signal sent from the DoorPanel
  2) DoorPanel: the control module that sits next to my door like a light switch, allowing for me to choose between different modes for the lighting in the room (turning on and off relays as well as sending lighting signals to the DeskModule
  3) DeskModule: the Arduino that controls the grid of LEDs contructed on my ceiling (the main source of light in the room)
  
