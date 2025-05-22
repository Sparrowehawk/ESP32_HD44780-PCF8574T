# ESP32_HD44780-PCF8574T
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

 - Connect the relavent power pins and I2C pins; note multiple can be linked together with seperate I2C addresses if the soldering on the back of the packpack to short A0, A1, A2 is completed
   - `GPIO 21` — SDA
   - `GPIO 22` — SCLK
---

## License
MIT License

---

## Author
[Sparrowehawk](https://github.com/Sparrowehawk)

