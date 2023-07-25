# MotionCal-BLE

Among the available PC tools for calibration, MotionCal by Paul Stoffregen is considered the best. It provides users with the ability to visualize data and check important statistical information such as gap, fit error, and variance. 

However, I have found it inconvenient to connect my board to a PCB every time I need to calibrate the magnetometer. This can be particularly problematic for PCBs inside closed enclosures or in mass production, where connecting via USB to a PC may become impossible. Due to this challenge, I have come up with a trick to reuse MotionCal over a BLE (Bluetooth Low Energy) connection. 

![image](https://github.com/yahyatawil/MotionCal-BLE/assets/1148381/096a37a7-0e22-43c8-9e30-72b9c3b05c35)

## Steps to use this hack
### Pre-requests: 

- A firmware with 3 characteristics in one BLE service. One characteristic for Acc. and Gyro. data, one characteristic for Mag. data and one characteristic for receiving calibration results. A reference Arduino firmware is provided in the repo.
- NRF52 USB dongle with Python pc-ble-driver-py library installed. Flash the dongle with connectivity firmware (if needed). I use connectivity_4.1.4_usb_with_s132_5.1.0. 
- Linux/Ubuntu OS. Although, Windows can be supported in a future update with few tweaks related to virtual ports on Windows (contributions as a PR are welcomed). 

### Steps: 

- Download the repository of my MotionCal fork. Run the provided MotionCal build.
- Run the following command inside the terminal to create the virtual ports. 
`sudo socat -d -d pty,link=/dev/ttyACM11,echo=0,perm=0777 pty,link=/dev/ttyACM10,echo=0,perm=0777`
- In MotionCal, select ttyACM11 in port list. 
- Run the Python script after connecting the NRF52 USB dongle to PC.  Change ttyACM0 in the command according to your port assigned to the dongle. 
`python3 nRF_dongle_imu_cal.py NRF52 /dev/ttyACM0`
- After the calibration is caluclated, press ‘Send Cal’ button in MotionCal, the python script will read the message from the tool and send it back to the firmware via BLE.

- Complete documentation is found in my [blog Atadiat](https://atadiat.com/en/e-magnetometer-calibration-wirelessly-over-ble/). 
