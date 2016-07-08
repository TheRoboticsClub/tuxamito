For integration in the Arduino IDE:

1- Add the RBot board description to the arduino_installation_directory/hardware/arduino/avr/boards.txt file.
The description is found at the end of the boards.txt file found in this repository. You could also overwrite the file with the one provided here, but keep in mine that this file is based on Arduino 1.6.8 and things, or other boards, might have change.
If you are using an ATMega328p (instead of the ATMega328), then change the value of rbot.menu.cpu.8MHzatmega328.build.mcu=atmega328 to atmega328p.

2- Create the directory arduino_installation_directory/hardware/arduino/avr/variants/rbot and copy the file pins_arduino.h.

The board should be selectable for 
