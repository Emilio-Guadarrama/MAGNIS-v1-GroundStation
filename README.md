# VESTA v1 Ground Station
### Ignitia Rocket Lab · Tecnológico de Monterrey GDL

---

## What is this?

VESTA v1 is the ground station of Ignitia Rocket Lab's IGNIS avionics family. It receives the
real-time LoRa telemetry from the IGNIS flight computer and the team's CanSat, logs every packet
to a microSD card and streams it over USB to a live dashboard on the laptop.

It is designed to be **simple, reliable and easy to fix in the field**: the microcontroller is a
plug-in dev board, the GPS is a plug-in breakout, and every part can be swapped in seconds
without soldering. The same RF hardware, footprints and drivers as the IGNIS flight computer are
used, so the firmware and the lessons learned carry over directly.

*Vesta is the Roman goddess of the hearth fire: the home base that keeps the fire of IGNIS.*

---

## Competitions

VESTA is used in the competitions Ignitia Rocket Lab takes part in or plans to join:

| Competition | Name | Location |
|---|---|---|
| **IREC** | Intercollegiate Rocket Engineering Competition, Spaceport America Cup | New Mexico, USA |
| **LASC** | Latin American Space Challenge | Brazil |
| **ENMICE** | Encuentro Mexicano de Ingeniería en Cohetería Experimental | Mexico |

---

## Design Philosophy

v1 is built around four rules:

- **Swappable:** the ESP32 dev board and the GPS sit on female headers. If one fails, it is replaced in seconds.
- **Shared with IGNIS:** same RFM95W, SMA, microSD socket and LED footprints as the flight computer, and the same drivers; only the pin defines change.
- **Assembled in Mexico:** almost every part is sourced from **UNIT Electronics**, with LCSC, Mouser and DigiKey as alternates.
- **Cheap to build:** 2-layer, 100 × 100 mm board, inside the low-cost PCB size class.

---

## Hardware at a Glance

| Block | Component |
|---|---|
| MCU | ESP32-DevKitC V4 · ESP32-WROOM-32D · plug-in on 2 × 19 female headers · CP2102 USB-UART |
| RF Telemetry | RFM95W · 915 MHz LoRa · SMA edge connector |
| GNSS | Adafruit Ultimate GPS breakout (PA1616S) · plug-in · PPS and FIX to input-only pins |
| Storage | MicroSD push-push socket (same as IGNIS v5) + 6-pin header for an external SPI module (own CS and MISO) |
| SPI | One shared bus on the native VSPI pins (RFM95W + microSD) |
| Power | USB 5 V through the dev board · 3.3 V from the dev board regulator |
| Indicators | 3× WS2812B RGB (3.3 V) · MLT-8530 magnetic buzzer (5 V, MOSFET low-side, PWM tone) |
| Host link | USB to laptop · PySide6 + pyqtgraph live dashboard · per-session CSV logging |
| PCB | 2-layer · 100 × 100 mm · Altium Designer · JLCPCB fabrication, assembled in Mexico |

### Link profiles

| Profile | Spreading factor | Bandwidth | Sync word |
|---|---|---|---|
| Rocket (IGNIS flight computer) | SF9 | 125 kHz | 0x12 |
| CanSat | SF9 | 125 kHz | 0xAB |

The default profile is set in the firmware (`LINK_PROFILE_DEFAULT`) and can be switched from the laptop
at any time by sending `PROFILE:ROCKET` or `PROFILE:CANSAT` over USB.

> In Brazil (LASC) the channel must sit fully inside the 915–928 MHz band allowed by ANATEL. The
> frequency is set in the firmware configuration.

One RFM95W listens to one profile at a time, so two identical boards are used for the two
missions or for antenna diversity.

---

## System Architecture

```mermaid
flowchart TD
    PC[Laptop<br/>PySide6 dashboard + CSV]
    PC <-->|USB · CP2102| MCU[ESP32-DevKitC V4<br/>WROOM-32D · plug-in]

    MCU --> RAIL5[5 V rail<br/>from USB]
    MCU --> RAIL33[3.3 V rail<br/>dev board regulator]

    RAIL33 --> LORA[RFM95W LoRa<br/>915 MHz]
    RAIL33 --> SD[MicroSD socket<br/>push-push]
    RAIL33 --> EXTSD[External microSD<br/>module header]
    RAIL33 --> GNSS[Adafruit Ultimate GPS<br/>plug-in]
    RAIL33 --> LED[3 × WS2812B]
    RAIL5 --> BUZ[MLT-8530 buzzer<br/>NTS4001 low-side]

    MCU <-->|VSPI shared| LORA
    MCU <-->|VSPI shared| SD
    MCU <-->|VSPI, own CS + MISO| EXTSD
    LORA --> ANT[SMA antenna]
    GNSS -->|UART2 + PPS + FIX| MCU
    MCU --> LED
    MCU --> BUZ
```

---

## Pin Map

Matches schematic VESTA-v1-SCH-01. The net names follow the IGNIS flight computer, so porting the
firmware only means changing the pin defines.

| Net | GPIO | Note |
|---|---|---|
| SNS_SPI_SCK / MISO / MOSI | 18 / 19 / 23 | native VSPI · pull-ups on MISO and MOSI |
| RFM95W_CS / RESET / INT | 5 / 25 / 32 | CS is VSPI CS0 (idles high at boot) |
| SD_CS | 21 | onboard slot · pull-up |
| SD_CS_EXT / SD_MISO_EXT | 4 / 39 | external module · MISO on an input-only pin |
| GPS_UART_TX / GPS_UART_RX | 16 / 17 | UART2 · `Serial2.begin(9600, SERIAL_8N1, 16, 17)` |
| GPS_PPS / FIX_GPS | 34 / 35 | input-only pins |
| WS2812B_DATA | 26 | 33 Ω series resistor at LED1 |
| BUZZER | 27 | MOSFET gate · 10 kΩ pull-down · PWM tone |
| **Do not connect** | 0, 1, 2, 3, 6–11, 12, 15 | strapping, UART0 (USB link), internal SPI flash |

> GPIO12 must never be pulled high at boot: it selects 1.8 V flash and the board will not start.

---

## Project Structure

```
VESTA-v1-GroundStation/
├── PCB_Project_VESTA-v1-GroundStation/
│   ├── Project_Group_VESTA-v1-GroundStation.DsnWrk
│   ├── PCB_Project_VESTA-v1-GroundStation.PrjPcb
│   ├── Sch_VESTA-v1.SchDoc
│   └── PCB_VESTA-v1.PcbDoc
├── Firmware/
│   └── VESTA_GS_v1/
│       └── VESTA_GS_v1.ino
├── BOM/
│   ├── VESTA_v1_Parts_List.xlsx
│   └── VESTA_v1_Netlist.xlsx
├── LICENSE
└── README.md
```

The schematic is a **single sheet**: the board is simple enough that each block (Main, Power,
RF_LoRa, GPS, Storage, Indicators) is separated by a drawn frame on the same page. The netlist
uses these block names.

---

## Design & Development

**Lead Designer & Author:** [Emilio Guadarrama](https://github.com/Emilio-Guadarrama)  
Vice President, Ignitia Rocket Lab · Avionics & Electronics Lead

**Team President:** Maximiliano Funoy Serrano  
**Faculty Advisor:** Dr. José Luis Henríquez Mercado

## Team

**Ignitia Rocket Lab**  
Tecnológico de Monterrey, Campus Guadalajara

📧 ignitia.rocketlab@gmail.com

---

## License

This project is open source under the [MIT License](LICENSE). You are free to use, modify and
share the design, including for your own rockets, as long as the original copyright notice is kept.

---

**Contact the author:** Emilio Guadarrama · [emilio.guadarrama@outlook.com](mailto:emilio.guadarrama@outlook.com)
