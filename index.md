---
layout: page
hide_title: true
hide: true
---

<div align="center" style="display: flex; flex-wrap: nowrap; justify-content: center; align-items: center;"> 
    <a href="https://github.com/xyzroe/ZigUSB_C6/releases"><img src="https://img.shields.io/github/release/xyzroe/ZigUSB_C6.svg" alt="GitHub version"></a>
    <a href="https://github.com/xyzroe/ZigUSB_C6/actions/workflows/build.yml"><img src="https://img.shields.io/github/actions/workflow/status/xyzroe/ZigUSB_C6/build.yml" alt="GitHub Actions Workflow Status"></a>
    <a href="https://github.com/xyzroe/ZigUSB_C6/releases/latest"><img src="https://img.shields.io/github/downloads/xyzroe/ZigUSB_C6/total.svg" alt="GitHub download"></a>
    <a href="https://github.com/xyzroe/ZigUSB_C6/issues"><img src="https://img.shields.io/github/issues/xyzroe/ZigUSB_C6" alt="GitHub Issues or Pull Requests"></a>
</div>

The ZigUSB C6 project is an innovative solution designed to enhance the control and monitoring of USB-powered devices through Zigbee communication. This project aims to provide a seamless integration for smart home enthusiasts and professionals alike, enabling remote control, automation, and monitoring of USB devices in a Zigbee-enabled ecosystem.

This project is based on the original ZigUSB but uses a modern chip and custom-developed firmware.

Using this device, you can remotely control the power of the USB port to turn on or off the connected device. Additionally, you can monitor the current voltage and current. It also functions as a reliable Zigbee network router.

Frequent use cases include converting a "dumb" USB lamp into a "smart" one, connecting modems/sticks/adapters that sometimes require a power reset, and monitoring the current consumption of any connected device.

### Key Features

- **USB Power Control**: Remotely manage the power supply to USB devices, allowing for energy savings and enhanced device management.
- **Zigbee Integration**: Fully compatible with Zigbee networks, facilitating easy integration into existing smart home setups.
- **OTA Updates**: Support for Over-The-Air (OTA) firmware updates, ensuring the device remains up-to-date with the latest features and security enhancements.
- **USB data transfer** is available. This may be needed when connecting a USB modem to a router that does not know how to manage USB power, and the modem may need to be rebooted.
- Monitoring of voltage and current - INA219 chip.
- For power management, USB switch - AP22804AW5-7 chip.
- WT0132C6-S5 module ​​was used as the Zigbee chip. It's ESP32 C6 based module.
- Designed for AK-N-12 case.

#### To re-pairing or reset to factory defaults:

**Hold touch button for more than 5 seconds**

### Project Goals

ZigUSB C6 was created with the vision of making smart home automation more accessible and versatile. By providing a bridge between USB devices and Zigbee networks, it opens up new possibilities for device automation and control. Whether you're looking to remotely manage lighting, charge devices on a schedule, or integrate USB devices into complex automation routines, ZigUSB C6 offers the flexibility and reliability needed for modern smart homes.

Stay tuned for updates as we continue to expand the capabilities of ZigUSB C6, and feel free to contribute to the project or suggest new features through our GitHub repository.

### Overview

<div align="center">
<img width="40%" src="./images/top.png">
<img width="40%" src="./images/bottom.png">
</div>

### Photos

<div align="center">
<img width="40%" src="./images/top_case.jpeg">
<img width="40%" src="./images/back_case.jpeg">
</div>

### Schematic

<div align="center"><img width="90%" src="./hardware/Schematic.png"></div>
  
#### zigbee2mqtt overview

<div align="center">
<img width="40%" src="./images/dash.png">
<img width="55%" src="./images/exposes.png">
</div>

#### Home Assistant overview

<div align="center">
<img width="60%" src="./images/ha.png">
</div>

### Hardware files

- [iBOM page](./hardware/iBOM.html) 🌍
- [BOM file](./hardware/BOM.csv) 📃
- [Gerber zip](./hardware/Gerber.zip) 🗂

This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>

### Web Flasher

Flash your device using next options:

<ol>
  <li>Connect your device using USB-TTL adapter.</li>
  <li id="coms">Hit "Connect" and select the correct COM port. <a onclick="showSerialHelp()">No device found?</a></li>
  <li>Get firmware installed and connected in less than 3 minutes!</li>
</ol>

<div class="web-install">
  <div class="pick-variant">
    <select id="firmwareVersion">
      <!-- Options will be populated dynamically -->
    </select>
  </div>

 <div class="button-group">
    <button id="connectButton" class="md-button md-button--primary">🔌 CONNECT</button>
    <button id="flashButton" class="md-button md-button--primary" disabled>⚡ FLASH</button>
  </div>

<progress id="progressBar" value="0" max="100" style="display: none;"></progress>

<div id="statusMessage" class="status-message"></div>
<div id="poweredBy">Powered by <a href="https://github.com/espressif/esptool-js/" target="_blank">espressif/esptool-js</a></div>

</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/crypto-js/4.0.0/crypto-js.min.js"></script>

<script type="module">

import { ESPLoader, Transport } from 'https://unpkg.com/esptool-js/bundle.js';

async function loadFirmwareVersions() {
    try {
        const response = await fetch('https://api.github.com/repos/xyzroe/ZigUSB_C6/releases');
        const data = await response.json();
        const select = document.getElementById('firmwareVersion');
        select.innerHTML = '';
        data.forEach(release => {
            const option = document.createElement('option');
            option.value = release.assets[0].browser_download_url; 
            option.textContent = release.name;
            select.appendChild(option);
        });
    } catch (error) {
        console.error('Error fetching releases:', error);
    }
}

let esploader = null;
let transport = null;
let device = null;

document.getElementById('connectButton').addEventListener('click', async () => {
    const statusMessage = document.getElementById('statusMessage');
    const connectButton = document.getElementById('connectButton');
    try {
        connectButton.disabled = true;
        statusMessage.textContent = 'Select port';

        device = await navigator.serial.requestPort({});
        transport = new Transport(device);
        statusMessage.textContent = 'Connecting...';

        const flashOptions = {
            transport: transport,
            baudrate: parseInt(460800),
            enableTracing: true,
            debugLogging: true,
        };

        esploader = new ESPLoader(flashOptions);

        let chip = await esploader.main();

        document.getElementById('flashButton').disabled = false;
        statusMessage.textContent = 'Connected to: '  + chip;

    } catch (error) {
        console.error('Error connecting to device:', error);
        statusMessage.textContent = 'Failed to connect to device.';
        connectButton.disabled = false;
    }
});


async function closeDevicePort(device) {
    if (!device) return;

    try {

        if (device.readable && device.readable.locked) {
            console.log("Releasing readable stream...");
            await device.readable.cancel();
        }

        if (device.writable && device.writable.locked) {
            console.log("Releasing writable stream...");
            await device.writable.getWriter().releaseLock();
        }

        console.log("Closing the device port...");
        await device.close();
        console.log("Device port closed successfully.");
    } catch (error) {
        console.error("Error closing device port:", error);
 
    }
}
    
document.getElementById('flashButton').addEventListener('click', async () => {
    const statusMessage = document.getElementById('statusMessage');
    const firmwareUrl = document.getElementById('firmwareVersion').value;
    const progressBar = document.getElementById('progressBar');

    try {
        if (!esploader) {
            throw new Error('ESPLoader is not initialized.');
        }

        progressBar.style.display = 'block';
        statusMessage.textContent = 'Flashing...';

        const response = await fetch(firmwareUrl);
        const firmwareArrayBuffer = await response.arrayBuffer();
        const uint8Array = new Uint8Array(firmwareArrayBuffer);
        let firmwareString = '';
        for (let i = 0; i < uint8Array.length; i++) {
            firmwareString += String.fromCharCode(uint8Array[i]);
        }

        const flashOptions = {
            fileArray: [{ data: firmwareString, address: 0x0000 }],
            flashSize: "keep",
            eraseAll: false,
            compress: true,
            reportProgress: (fileIndex, written, total) => {
                progressBar.value = (written / total) * 100;
            },
        };

        await esploader.writeFlash(flashOptions);
        statusMessage.textContent = 'Firmware flashed successfully!';
        alert('Firmware flashed successfully!');
    } catch (error) {
        console.error('Error flashing firmware:', error);
        statusMessage.textContent = 'Failed to flash firmware.';
        alert('Failed to flash firmware.');
    } finally {
        progressBar.style.display = 'none';

        await closeDevicePort(device);

        esploader = null;
        transport = null;
        device = null;

        document.getElementById('connectButton').disabled = false;
        document.getElementById('flashButton').disabled = true;
    }
});


function showSerialHelp() {
    document.getElementById('coms').innerHTML = `Hit "Connect" and select the correct COM port.<br><br>
    You might be missing the drivers for your board.<br>
    Here are drivers for one of the most popular chip:
    <a href="https://sparks.gogo.co.nz/ch340.html" target="_blank">CH340C</a><br><br>
    Make sure your USB cable supports data transfer.<br><br>
    `;
}

loadFirmwareVersions();
</script>

<style>
.web-install {
    display: flex;
    flex-direction: column;
    align-items: center;
}

.pick-variant, .md-button, #progressBar, .status-message {
    margin: 10px 0;
}

.status-message {
    font-size: 1.2em;
    color: #007BFF;
}

.md-button--primary {
    background-color: #007BFF;
    color: white;
    border: none;
    padding: 10px 20px;
    cursor: pointer;
}

.md-button--primary:disabled {
    background-color: #CCCCCC;
    cursor: not-allowed;
}

#progressBar {
    width: 100%;
    max-width: 300px;
    height: 60px; 
}

#firmwareVersion {
    font-size: 1.5em;
}

#poweredBy {
    margin-top: 2.5em;
    font-size: 0.75em;
}

.button-group {
    display: flex;
    gap: 20px;
}
</style>

#### Firmware Files

All source files are available in this repository. Pre-built firmware files can be found in the [releases section](https://github.com/xyzroe/ZigUSB_C6/releases).

### OTA

##### zigbee2mqtt

1. Put firmware update (.\*ota) file next to config file
2. Create `index.json` file:

```
[
    {
        "url": "ZigUSB_C6.ota",
        "force": true
    }
]
```

3. Add config option to you zigbee2mqtt configuration.yaml file

```
ota:
  zigbee_ota_override_index_location: index.json
```

4. Open OTA tab in z2m and click check update next to your device.
5. Check update process via web UI

##### homed

1. Put firmware update (.\*ota) file to "ota" folder next to config file
2. Open device page and click OTA button
3. Click refresh and then update

### Verified Supported Zigbee Systems

- [zigbee2mqtt](https://www.zigbee2mqtt.io/) - Full support, no longer requires an [external converter](https://github.com/xyzroe/ZigUSB_C6/tree/main/external_converter/ZigUSB_C6.js) ⭐⭐⭐⭐⭐
- [HOMEd](https://wiki.homed.dev/page/HOMEd) - Partial support ⭐⭐⭐⭐
- [ZHA](https://www.home-assistant.io/integrations/zha/) - Partial support ⭐⭐⭐⭐
- Other systems must be tested. The device uses standard clusters and attributes, so most coordinators can support it out of the box.

### Where to buy?

<a href="https://www.tindie.com/stores/mind/?ref=offsite_badges&utm_source=sellers_xyzroe&utm_medium=badges&utm_campaign=badge_large"><img src="https://d2ss6ovg47m0r5.cloudfront.net/badges/tindie-larges.png" alt="I sell on Tindie" height="120"></a>

### Like ♥️?

[![badges](https://badges.aleen42.com/src/buymeacoffee.svg)](https://www.buymeacoffee.com/xyzroe)
[![badges](https://badges.aleen42.com/src/github.svg)](https://github.com/sponsors/xyzroe)
[![badges](https://badges.aleen42.com/src/paypal.svg)](http://paypal.me/xyzroe)

### Contribute 🚀

- [How-to](./CONTRIBUTE.md)

<br>  
ZigUSB_C6 is licensed under the <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>
