Yet another garage door Arduino project, unlike most projects this project controls the garage door through separate to open and to close. 

### Features:
- 1 relay to open the garage door
- 1 relay to close the garage door
- Using a reed contact to detect if the garage door is open
- Integrates into Home Assistant as cover device through MQTT and is discoverable. Device id is based on MAC address.

### Commands:
| command | description |
| --- | --- |
| 'open' | triggers the open relay |
| 'close' | triggers the close relay |
| 'stop' | stops opening or closing the door by triggering the open relay if closing or the close relay when opening |

### States:
| state | description |
| --- | --- |
| 'open' | when reed contact is open |
| 'closed' | when reed contact is closed |
| 'opening' | waits for 20 seconds and then checks the actual open/close state |
| 'closing' | waits for up to 20 seconds to detect if the door is closed. If the door is not closed after 20 seconds state is set to 'open' |
| 'stopping' | reporting only 1 cycle and then replaced with actual open/close state |

### Hardware:
- Hormann Suprametic Garage Door
- Arduino MKR 1010 WiFi





