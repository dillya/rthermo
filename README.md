# RThermo: remote Thermostat basde on nRF24L01+ wireless tranciever

RThermo is an Open source and Open Hardware project which consists on a wireless network for house temperature control. It is composed of a main controller running on a Linux powered device (connected to a nRF24L01(+)) and many remote devices based on Low power Nordic Semiconductor nRF24LE1.

## Details

RThermo is composed of a main controller and up to 5 remote control devices:
* main controller which manages global temperature in House by controlling all remote devices in House,
* remote devices which can measure local temperature and control a heater or an house boiler.

### Main controller

It monitors local temperature and remote temperature through data send periodically by associated devices.
It regulates house temperature globally or independently by responding to status request send by devices with a heaters activation flag.
So, main controller only respond to requests send by devices.

### Remote device

It measures local temperature and control heater activation.
A status request is sent periodically to main controller with measured temperature and status response is expected in order to enable or disable heater.
When device is not requesting status, it sleeps in order to save power.
If it is connected to a battery, its level is also send in status requests to notify main controller of a low battery level status.

## Example

Below, an example of a temperature control in a house with a boiler for heating of most room of house and an extra electric heater for attic not warmed by boiler:

       __________________________
      |       (Living room)      |  _______________________________
      |  _____________________   | |           (Basement)          |
      | |                     |  | |   __________                  |
      | |                     |  | |  |          |                 |
      | |                     | <===> | Device 0 | -> House Bolier |
      | |                     |  | |  |__________|                 |
      | |                     |  | |_______________________________|
      | |   Main controller   |  |  __________________________________
      | |     (T° sensor)     |  | |              (Attic)             |
      | |                     |  | |   __________                     |
      | |                     |  | |  |          |                    |
      | |                     | <===> | Device 1 | -> Electric heater |
      | |_____________________|  | |  |__________| <- Local T° sensor |
      |    v           v         | |__________________________________|
      |   Wifi   Electronic UI   |
      |__________________________|

## License

RThermo is licensed under the GPLv2. For more details about the license, please visit the following website: http://www.gnu.org/licenses/old-licenses/gpl-2.0.en.html

