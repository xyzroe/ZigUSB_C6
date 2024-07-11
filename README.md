# ZigUSB C6

The ZigUSB C6 project is an innovative solution designed to enhance the control and monitoring of USB-powered devices through Zigbee communication. This project aims to provide a seamless integration for smart home enthusiasts and professionals alike, enabling remote control, automation, and monitoring of USB devices in a Zigbee-enabled ecosystem.

## Key Features

- **USB Power Control**: Remotely manage the power supply to USB devices, allowing for energy savings and enhanced device management.
- **Zigbee Integration**: Fully compatible with Zigbee networks, facilitating easy integration into existing smart home setups.
- **OTA Updates**: Support for Over-The-Air (OTA) firmware updates, ensuring the device remains up-to-date with the latest features and security enhancements.

## Project Goals

ZigUSB C6 was created with the vision of making smart home automation more accessible and versatile. By providing a bridge between USB devices and Zigbee networks, it opens up new possibilities for device automation and control. Whether you're looking to remotely manage lighting, charge devices on a schedule, or integrate USB devices into complex automation routines, ZigUSB C6 offers the flexibility and reliability needed for modern smart homes.

Stay tuned for updates as we continue to expand the capabilities of ZigUSB C6, and feel free to contribute to the project or suggest new features through our GitHub repository.

## OTA
  
### homed

1. Put firmware update (.*0ta) file next to config file 
2. Run MQTT command on topic homed/command/zigbee
```
{
  "action": "otaUpgrade",
  "device": "IEEE_OF_DEVICE",
  "endpointId": 1,
  "fileName": "/config/ZigUSB_C6.ota"
}
``` 
3. Check update process via log files

### zigbee2mqtt

1. Put firmware update (.*0ta) file next to config file 
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