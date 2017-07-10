# Nerdbrew
Arduino-based beer brewing automation

# Schematics
![schematics](https://github.com/danp3d/nerdbrew/blob/master/sketch.png)

## Connections
### LCD
* CLK -> Pin 7  
* DIN -> Pin 8  
* DC -> Pin 10  
* RST -> Pin 12  
* CE -> Pin 11  

### Buttons
* On/Off/Cancel -> Pin 3  
* Right arrow -> Pin 4  
* Up -> Pin 5  
* Down -> Pin 6  
* Set -> Pin 2  

### Others
* Temperature sensor -> Pin 9  
* Relay -> Pin 13  

## Installation
The Arduino IDE is great for beginners. That said, as soon as you try to do anything that remotely resembles a production-grade application, it goes from useful to painful very quickly - even things like breaking the application into multiple files can be a big pain.  
To try to remediate that, I'm using Arduino-Makefile here. It's not perfect - the structure still needs to be very 'arduinesque' and isn't very flexible, but it's MUCH better than writing everything in one massive file.


### Dependencies
The Arduino IDE needs to be installed. The following libraries need to be installed via the library manager (`Sketch > Include Library > Manage libraries`)  
* PCD8544  
* OneWire  
* Bounce2  
* PID  
* PID AutoTune  
* DallasTemperature  
* TimerOne  
  
You'll also need to install `avr-gcc`, `binutils`, `avr-libc`, `avrdude` and `pySerial`. For OSX, run the following commands:  

```Bash
  $ brew tap osx-cross/avr
  $ brew install avr-libc
  $ brew install avrdude
  $ brew install python
  $ pip install pyserial
```

### Makefile
Change the contents of the `PROJECT_DIR`, `ARDMK_DIR`, `ARDUINO_DIR`, `ARDUINO_LIB_PATH`, `AVR_TOOLS_DIR` and `AVRDUDE` to whatever you're using in your system (this is extra important if you're not using OSX - less chances of paths being the same).  
If you're not using an Arduino Pro Mini, make sure you change the contents of the `BORD_TAG` and `BOARD_SUB` variables, too.

### Make
After installing the dependencies (previous sections), run the following commands

```Bash
  $ cd /path/to/code
  $ make
  $ make upload
```

`make upload` requires your board to be plugged into your computer.
