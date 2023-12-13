# ModbusGW
ModbusGW is a Modbus TCP and MQTT gateway to Modbus rs485 for RTU and ASCII.

# Features
ModbusGW is an easy-to-use Python application,
which is a gateway from
.. ModbusTCP to Modbus rs-485 RTU and ASCII
.. MQTT to Modbus rs-485 for RTU and ASCII

the application can be run in a container or as a process.

# Installation
## Run as a process
```
sudo pip install git+https://https://github.com/EHerzog76/ModbusGW.git

python3 -m pyModbusGW /dev/ttyUSB0
```

## Run in a container
1. Run as a process:
  ```sh
  docker run --rm -p 502:502 -e SERIAL_DEVICE=/dev/ttyUSB0 eherzog/modbus-gw:alpine-latest
  ```
2. Run as daemon:
  ```sh
  docker run -d \
       --restart unless-stopped \
       --privileged \
       -p 502:502 \
       -e SERIAL_DEVICE=/dev/ttyUSB0 \
       eherzog/modbus-gw:alpine-latest
  ```
  3. Run with docker compose:
    Edit the ```compose.yml``` to your needs and use it with:
    ```sh
    docker compose up -d
    ```

# Usage example

## Environment variables / Parameters
|Env-Name|Param-Name|Default Value|Description|
|---|---|---|---|
|SERIAL_DEVICE | |/dev/ttyUSB0 | Serial device name
|LISTEN_ADDRESS | -L --listen|0.0.0.0 |listen-address
|PORT | -p --port|502 |Listen on TCP Port
|SERIAL_BAUDRATE | -b --baudrate|19200 |serial baudrate
|SERIAL_BYTESIZE | -c --bytesize|8 |Use 5,6,7 or 8 Bits per Char
|SERIAL_PARITY | -P --parity|N |serial parity N..None, O..Odd, E..Even
|SERIAL_STOPBITS | -f --stopbits|1 |serial Stopbits 0, 1, 1.5
|SERIAL_FRAMETYPE | -F --frametype|RTU |Serial Frametype RTU or ASCII
|SERIAL_TIMEOUT | -t --timeout|1.0 |Serial bus timeout delay for response
|SERIAL_WAITTIME | -w --waittime|0 |Serial waittime befor sending next frame
|SERIAL_EOF | -e --eof|0.05 |Not used
|MQTT_HOST | -M --mqtthost| |Connect to MQTT-Host only when set
|MQTT_PORT | -q --mqttport|1883 |
|MQTT_TOPIC | -T --mqtttopic|modbus |MQTT topic to use
|MQTT_CLIENTID | -i --mqttclientid|pyRS485GW |MQTT client-id prefix
|MQTT_USER | -u --mqttuser|usr |Only login to MQTT-Host if set
|MQTT_PWD | -s --mqttpwd| |
|MQTT_VERSION | -v --mqttversion|MQTTv5 |MQTT-Version: MQTTv5, MQTTv311, MQTTv31
|LOG_LEVEL | -g --log|INFO |Loglevel: Debug, Info, Warning, Error, Critical see python logging 
|DEBUG | -d --debug|False |Activate Debug-Output

All serial timing parameters are calculated based on the baudrate and the Bits per Char.
So SERIAL_WAITTIME is only needed if your modbus devices need a longer time delay between the read/write requests.

To use more then one serial bus you can seperate all SERIAL_ parameters with a ```;```
e.g.:
```
SERIAL_DEVICE=/dev/ttyUSB0;/dev/ttyUSB1
SERIAL_BAUDRATE=19200;9600
```

# MQTT
To receive and publish the modbus commands over MQTT use the ```MQTT_``` parameters.
## MQTT message syntax
### MQTT subscribe message format
topic/DeviceID/SlaveID/Function-Code
     payload: {"Adr": 123, "Value": "..."}

     topic...........is defined by MQTT_TOPIC
     DeviceID........The index of your serial device. (The 1st Device e.g. /dev/ttyUSB0 has the index: 0)
     SlaveID.........The SlaveID of the modbus device, with which you would like to communicate.
     Function-Code...The modbus Function code e.g.: 01, 03, 05, 06, ...

### MQTT pulish message format
topic/DeviceID/SlaveID/Function-Code
     payload: {"value": "PDU", "slave_id": "SlaveID", "fc": FC}


# for Developers
This project is written in python and is based on the project:
https://github.com/sourceperl/pyModbusTCP/

As starting point I used the serial-gw example:
https://github.com/sourceperl/pyModbusTCP/blob/master/examples/server_serial_gw.py

I rewrite the pyModbusTCP to use the asyncio library instead of the threading-server implementation.

# Dependencies
pyserial
paho-mqtt


In the future it is posible that
    paho-mqtt  will be replaced with: asyncio-mqtt

In python you have 2 projects for serial-port communication.
.. serial
.. pyserial
But in this project only pyserial works well, because the serial project is not comFILE0  PXz     8  �                � g|         `           H      ���5������5������5����i�����                    5                  0   h          L     �    ���5������5������5������5���                        h e a d s     �   �        �       $ I 3 0 0               �   �       �    ` N     �    ���5������5������5������5����       �                m a s t e r                 �����yG                                                                      g|                                                                                                                                                                                                                                                                                  