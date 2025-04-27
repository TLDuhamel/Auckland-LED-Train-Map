# Auckland Live LED Train Map

![PCB Overview Render](Images/Auckland-LED-Train-Map-Render.avif) A project to create a physical map of the Auckland train network where train movements are displayed in real-time using individually addressable LEDs. The map is controlled by an ESP32-C3 microcontroller, fetching live data over Wi-Fi.

## Features

* **Real-time Train Tracking:** Displays the approximate locations of trains on the Auckland network.
* **Addressable LEDs:** Uses approximately 150 small (1.6x1.5mm) WS2812B-compatible RGB LEDs for vibrant display.
* **Wi-Fi Connectivity:** Leverages the ESP32-C3's built-in Wi-Fi to fetch live train data.
* **Custom PCB:** Designed specifically for this project, fitting within common PCB manufacturer limits (JLCPCB).
* **Open Source:** Hardware design files and (eventually) firmware are open source.

![ESP32-C3 PCB Render](Images/ESP32C3-PCB-Render.avif)

## Hardware

The core components of the map include:

* **Microcontroller:** ESP32-C3
  * **CPU:** RISC-V single-core @ 160 MHz
  * **Wireless:** 2.4 GHz Wi-Fi (802.11b/g/n) and Bluetooth 5 (LE)
  * **Flash:** 4 MB internal Flash
  * **Package:** QFN32 (5x5 mm)
* **LEDs:** ~150 x XL-1615RGBC-WS2812B (1.6mm x 1.5mm Addressable RGB LEDs)
* **PCB:**
  * **Dimensions:** 249mm x 66mm
  * **Manufacturer Friendly:** Designed to fit within JLCPCB's standard 250mm x 250mm maximum size.
* **Antenna:** On-board PCB antenna based on the [Texas Instruments CC2430DB Demo Board design (SWCU125)](https://www.ti.com/lit/ug/swru125/swru125.pdf).

![Images\Pukekohe-PCB-Render.avif](Images/ESP32C3-PCB-Render.avif)

## PCB Design

The Printed Circuit Board (PCB) was designed using KiCad V9.0 and utilizes my custom [JLCPCB KiCad Library](https://github.com/CDFER/jlcpcb-kicad-library)

* **View Online:** You can view the PCB layout interactively using the Kicanvas web viewer:
    [https://kicanvas.org/?github=https%3A%2F%2Fgithub.com%2FCDFER%2FAuckland-LED-Train-Map%2Ftree%2FV1-PCB%2FPCB](https://kicanvas.org/?github=https%3A%2F%2Fgithub.com%2FCDFER%2FAuckland-LED-Train-Map%2Ftree%2FV1-PCB%2FPCB)
* **Source Files:** The KiCad project files (schematic, PCB layout) can be found in the `/PCB` directory of this repository.

## Software / Firmware

*(This section is a placeholder - details about the specific firmware, data source, setup instructions, and required libraries will be added here when it is completed.)*

The firmware running on the ESP32-C3 is responsible for:

1. Connecting to a Wi-Fi network.
2. Fetching live train data from my cached GTFS API <https://github.com/CDFER/GTFS-Realtime-Cache-Server>
3. Processing the data to determine train locations on the map segments.
4. Controlling the WS2812B LEDs to display the train positions.

## Contributing

Contributions are welcome! If you have improvements, bug fixes, or feature suggestions, please feel free to open an issue or submit a pull request.

## License

This library is released under the MIT license

© 2025 Chris Dirks
