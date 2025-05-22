# ESP32_HD44780-PCF8574T-
An implementation of a HD44780 1602 16x2 Serial LCD display using a PCF8574T backpack to connect to an ESP32 via I2C

## Parts Used
- ESP32
- HD44780
- PCF8574T backpack

## Project Directory
```
wireless-display-driver/
├── main/ 
│ ├── main.c
│ └── CMakeLists.txt
│
├── README.md
├── CMakeLists.txt
└── LICENSE
```
  
## Wiring

 - Connect the relavent power pins, only the first display has to be connected to the ESP32 SPI pins, the rest are daisy chained. Note that with >2 displays, power may become an issue due to the displays high power draw
   - `GPIO 21` — SDA
   - `GPIO 22` — SCLK
---

## License
MIT License

---

## Author
[Sparrowehawk](https://github.com/Sparrowehawk)

