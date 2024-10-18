#ifndef CONST_H
#define CONST_H

/* Text constants */
#define T_STATUS_FAILED "Failed!"
#define T_STATUS_DONE "Done"

/* Number constants */
#define LED_ON_STATE 0
#define LED_OFF_STATE 1

/* Zigbee configuration */
#define OTA_UPGRADE_MANUFACTURER 0x3443 /* The attribute indicates the value for the manufacturer of the device */
#define OTA_UPGRADE_IMAGE_TYPE 0x1011   /* The attribute indicates the type of the image */
#define OTA_UPGRADE_HW_VERSION 0x0101   /* The parameter indicates the version of hardware */
#define OTA_UPGRADE_MAX_DATA_SIZE 64    /* The parameter indicates the maximum data size of query block image */

#define HW_MANUFACTURER "xyzroe" /* The parameter indicates the manufacturer of the device */
#define HW_MODEL "ZigUSB_C6"     /* The parameter indicates the model of the device */

#define SENSOR_ENDPOINT 1  /* the endpoint number for the sensor */
#define INT_LED_ENDPOINT 2 /* the endpoint number for the internal LED */
#define EXT_LED_ENDPOINT 3 /* the endpoint number for the external LED */
#define INV_USB_ENDPOINT 4 /* the endpoint number for the USB switch (inverted logic) */

#define OTA_FW_VERSION 0x0000013D /* The attribute indicates the version of the firmware */
#define FW_BUILD_DATE "20241018"  /* The parameter indicates the build date of the firmware */

/* GPIO configuration */
#define BTN_GPIO_1 5                 /* Button from v0.3 */
#define BTN_GPIO_2 9                 /* Button till v0.2 */
#define ALARM_GPIO 10                /* INA219 alert pin */
#define EXT_LED_GPIO 7               /* External LED - yellow LED */
#define INT_LED_GPIO 8               /* Internal LED - blue LED */
#define USB_GPIO 3                   /* USB switch control */
#define I2C_SDA_GPIO 0               /* I2C SDA */
#define I2C_SCL_GPIO 1               /* I2C SCL */
#define I2C_PORT 0                   /* I2C port number */
#define I2C_ADDR INA219_ADDR_GND_GND /* I2C address */
#define SHUNT_RESISTOR_MILLI_OHM 100 /* Shunt resistor value */

/* Time constants */
#define INA219_INTERVAL 1000             /* Reading interval */
#define CPU_TEMP_INTERVAL 10000          /* Reading interval */
#define LONG_PRESS_TIME 5000             /* to make factory reset */
#define SHORT_PRESS_TIME 150             /* to toggle USB power */
#define UPDATE_ATTRIBUTE_INTERVAL 600000 /* 10 minutes to FORCE update all states */
#define WAIT_BEFORE_FIRST_UPDATE 15000   /* 15 seconds to wait before first update */

#endif // CONST_H
