# ModbusGW
ModbusGW is a Modbus TCP and MQTT gateway to Modbus rs485 for RTU and ASCII.

# Features
ModbusGW is an easy-to-use Python application,
which is a gateway from
.. ModbusRTU-TCP to Modbus rs485 RTU and ASCII
.. MQTT to Modbus rs-485 for RTU and ASCII

the application can be run in a container or as a process.

# Installation
## Run as a process
```
sudo pip install git+https://https://github.com/EHerzog76/ModbusGW.git

python3 pyModbusGW.py /dev/ttyUSB0
```

## Run in a container
1. Run as a daemon process:
  ```sh
  docker run --rm -it -p 502:502 -e 
  docker run -d -p 502:502 ... -t eherzog/modbus-gw
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
|MQTT_TOPIC | -T --mqtttopic|modbus |
|MQTT_CLIENTID | -i --mqttclientid|pyRS485GW |MQTT client-id prefix
|MQTT_USER | -u --mqttuser|usr |Only login to MQTT-Host if set
|MQTT_PWD | -s --mqttpwd| |
|MQTT_VERSION | -v --mqttversion|MQTTv5 |MQTT-Version: MQTTv5, MQTTv311, MQTTv31
|LOG_LEVEL | -g --log|INFO |Loglevel: Debug, Info, Warning, Error, Critical see python logging 
|DEBUG | -d --debug|False |Activate Debug-Output

# Other Modbus - Projects
https://github.com/M3m3M4n/modbus-fuzz-note
https://github.com/Bouni/ModBusGateway
https://pymodbustcp.readthedocs.io/en/latest/examples/server_serial_gw.html
https://github.com/sourceperl/pyModbusTCP/tree/master
https://github.com/RT-Thread-packages/freemodbus/tree/master
https://github.com/eModbus/eModbus/tree/master
https://github.com/epsilonrt/mbpoll
https://github.com/burakdereli/Modbus-Slave-RTU-ASCII/tree/master
https://github.com/ClassicDIY/ModbusTool/tree/master
https://github.com/3cky/mbusd/tree/master
