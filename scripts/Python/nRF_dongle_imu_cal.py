import sys
import time
import logging
from queue import Queue, Empty
from pc_ble_driver_py.observers import *
import time, threading
import numpy as np
import struct
import json
import serial

ser = serial.Serial('/dev/ttyACM10', 115200, timeout=0)
print(ser.name)

TARGET_DEV_NAME = "Mag Cal"
CONNECTIONS = 1
CFG_TAG = 1

Acc_x = []
Acc_y = []
Acc_z = []

Gyro_x = []
Gyro_y = []
Gyro_z = []

Mag_x = []
Mag_y = []
Mag_z = [] 

H=[]
S=[]

sampletime=[]

newdata = False

No = 0
sampleNo = [] 

startFPSCalc = False
LastFPS = 0
FPS= 0

def CalculateFPS():
    global FPS
    global sampleNo
    global LastFPS
    FPS = len(sampleNo) - LastFPS
    LastFPS=len(sampleNo)
    print("==============FPS:{}==================".format(FPS))
    threading.Timer(1, CalculateFPS).start()

    
def init(conn_ic_id):
    # noinspection PyGlobalUndefined
    global config, BLEDriver, BLEAdvData, BLEEvtID, BLEAdapter, BLEEnableParams, BLEGapTimeoutSrc, BLEUUIDBase, BLEUUID, BLEConfigCommon, BLEConfig, BLEConfigConnGatt, BLEGapScanParams
    from pc_ble_driver_py import config

    config.__conn_ic_id__ = conn_ic_id
    # noinspection PyUnresolvedReferences
    from pc_ble_driver_py.ble_driver import (
        BLEDriver,
        BLEAdvData,
        BLEEvtID,
        BLEEnableParams,
        BLEGapTimeoutSrc,
        BLEUUIDBase,
        BLEUUID,
        BLEGapScanParams,
        BLEConfigCommon,
        BLEConfig,
        BLEConfigConnGatt,
    )

    # noinspection PyUnresolvedReferences
    from pc_ble_driver_py.ble_adapter import BLEAdapter

    global nrf_sd_ble_api_ver
    nrf_sd_ble_api_ver = config.sd_api_ver_get()


class BLEDevice(BLEDriverObserver, BLEAdapterObserver):
    def __init__(self, adapter):
        super(BLEDevice, self).__init__()
        self.adapter = adapter
        self.conn_q = Queue()
        self.adapter.observer_register(self)
        self.adapter.driver.observer_register(self)
        self.adapter.default_mtu = 250

    def open(self):
        self.adapter.driver.open()
        if config.__conn_ic_id__.upper() == "NRF51":
            self.adapter.driver.ble_enable(
                BLEEnableParams(
                    vs_uuid_count=1,
                    service_changed=0,
                    periph_conn_count=0,
                    central_conn_count=1,
                    central_sec_count=0,
                )
            )
        elif config.__conn_ic_id__.upper() == "NRF52":
            gatt_cfg = BLEConfigConnGatt()
            gatt_cfg.att_mtu = self.adapter.default_mtu
            gatt_cfg.tag = CFG_TAG
            self.adapter.driver.ble_cfg_set(BLEConfig.conn_gatt, gatt_cfg)
            #19b10001-e8f2-537e-4f6c-d104768a1214
            BASE_UUID = BLEUUIDBase([0x19, 0xb1, 0x00, 0x00, 0xe8, 0xf2, 0x53, 0x7e, 0x4f, 0x6c, 0xd1, 0x04, 0x76, 0x8a, 0x12, 0x14])  
            self.X4_UUID = BLEUUID(0x0001, BASE_UUID)
            self.X4_UUID_1 = BLEUUID(0x0002, BASE_UUID)
            self.X4_UUID_2 = BLEUUID(0x0003, BASE_UUID)
            self.adapter.driver.ble_enable()
            self.adapter.driver.ble_vs_uuid_add(BASE_UUID)
            
    def close(self):
        self.adapter.driver.close()

    def connect_and_discover(self):
        scan_duration = 10
        params = BLEGapScanParams(interval_ms=200, window_ms=150, timeout_s=scan_duration)

        self.adapter.driver.ble_gap_scan_start(scan_params=params)

        try:
            new_conn = self.conn_q.get(timeout=scan_duration)
            self.adapter.service_discovery(new_conn)

            self.adapter.enable_notification(new_conn, self.X4_UUID) #0xF000aa8104514000b000000000000000
            self.adapter.enable_notification(new_conn, self.X4_UUID_1) #0xF000aa8104514000b000000000000000
            
            self.conn_h = new_conn
            
            self.adapter.att_mtu_exchange(new_conn,64)
            
            return new_conn
        except Empty:
            print(f"No collector advertising with name {TARGET_DEV_NAME} found.")
            return None

    def on_gap_evt_connected(
        self, ble_driver, conn_handle, peer_addr, role, conn_params
    ):
        print("New connection: {}".format(conn_handle))
        self.conn_q.put(conn_handle)

    def on_gap_evt_disconnected(self, ble_driver, conn_handle, reason):
        print("Disconnected: {} {}".format(conn_handle, reason))

    def on_gap_evt_adv_report(
        self, ble_driver, conn_handle, peer_addr, rssi, adv_type, adv_data
    ):
        if BLEAdvData.Types.complete_local_name in adv_data.records:
            dev_name_list = adv_data.records[BLEAdvData.Types.complete_local_name]

        elif BLEAdvData.Types.short_local_name in adv_data.records:
            dev_name_list = adv_data.records[BLEAdvData.Types.short_local_name]

        else:
            return

        dev_name = "".join(chr(e) for e in dev_name_list)
        address_string = "".join("{0:02X}".format(b) for b in peer_addr.addr)
        print(
            "Received advertisment report, address: 0x{}, device_name: {}".format(
                address_string, dev_name
            )
        )

        if TARGET_DEV_NAME in dev_name:
            print("FOUND THE DEVICE!\r\n")
            self.adapter.connect(peer_addr, tag=CFG_TAG)
            
    def write(self,data):
        byte_array = bytes(data, 'UTF-8')
        ret = self.adapter.write_req(self.conn_h, self.X4_UUID_2,byte_array)
        print(ret)
    
    def on_notification(self, ble_adapter, conn_handle, uuid, data):
        global Acc_x 
        global Acc_y 
        global Acc_z 
        global Gyro_x 
        global Gyro_y 
        global Gyro_z 
        global Mag_x
        global Mag_y
        global Mag_z
        
        global No 
        global sampleNo 
  
        global startFPSCalc
        
        global H
        global S
        global newdata
        
        line = ser.readline()   # read a '\n' terminated line        
        print(len(line))
        print(line)
        if len(line) == 68:
                print(line[0])
                print(line[1])
                V1 = float('.'.join(str(ele) for ele in struct.unpack('f', line[26:30])))
                V2 = float('.'.join(str(ele) for ele in struct.unpack('f', line[30:34])))
                V3 = float('.'.join(str(ele) for ele in struct.unpack('f', line[34:38])))

                H=[V1,V2,V3]
                
                index = 42
                hh=[]
                for i in range(6):
                        h = float('.'.join(str(ele) for ele in struct.unpack('f', line[index:index+4])))
                        hh.append(h)
                        index = index + 4
                
                S=[hh[0],hh[3],hh[4],-hh[3],hh[1],hh[5],-hh[4],-hh[5],hh[2]]
                        
                newdata = True        
                print(H)       
                print(S) 
                if line[0] == 177 and line[1] == 84:
                        print("signature")
                        quit()
                
                #np.array(int.from_bytes(data[0:2],byteorder='little', signed=True),dtype="int16")
                    
        #print(data)
        if uuid.value == 0x0001:
                #print("ACC_GYRO:")
                #if startFPSCalc is False:
                    #startFPSCalc = True
                    #CalculateFPS()
                    
                acc_x = np.array(int.from_bytes(data[0:2],byteorder='little', signed=True),dtype="int16")
                Acc_x.append(acc_x)

                acc_y = np.array(int.from_bytes(data[2:4],byteorder='little', signed=True),dtype="int16")
                Acc_y.append(acc_y)

                acc_z = np.array(int.from_bytes(data[4:6],byteorder='little', signed=True),dtype="int16")
                Acc_z.append(acc_z) 

                gyro_x = np.array(int.from_bytes(data[6:8],byteorder='little', signed=True),dtype="int16")
                Gyro_x.append(gyro_x)

                gyro_y = np.array(int.from_bytes(data[8:10],byteorder='little', signed=True),dtype="int16")
                Gyro_y.append(gyro_y)

                gyro_z = np.array(int.from_bytes(data[10:12],byteorder='little', signed=True),dtype="int16")
                Gyro_z.append(gyro_z)

                print("Gyro=({},{},{}), Acc=({},{},{}) ".format(gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z))
                
                sampleNo.append(No)
                No = No + 1
                
        elif uuid.value == 0x0002:
                #print("Mag")
                mag_x = np.array(int.from_bytes(data[0:2],byteorder='little', signed=True),dtype="int16")
                Mag_x.append(mag_x)

                mag_y = np.array(int.from_bytes(data[2:4],byteorder='little', signed=True),dtype="int16")
                Mag_y.append(mag_y)

                mag_z = np.array(int.from_bytes(data[4:6],byteorder='little', signed=True),dtype="int16")
                Mag_z.append(mag_z) 
                
                print("Mag=({},{},{})) ".format(mag_x,mag_y,mag_z))
                
                to_serial_port = 'Raw:{},{},{},{},{},{},{},{},{}\r\n'.format(Acc_x[-1],Acc_y[-1],Acc_z[-1],Gyro_x[-1],Gyro_y[-1],Gyro_z[-1],mag_x,mag_y,mag_z)
                print(to_serial_port)
                ser.write(bytes(to_serial_port,'UTF-8'))
                
            

#x = ser.read()          # read one byte
#s = ser.read(10)        # read up to ten bytes (timeout)
#line = ser.readline()   # read a '\n' terminated line
        
def main(selected_serial_port):
    print("Serial port used: {}".format(selected_serial_port))
    driver = BLEDriver(
        serial_port=selected_serial_port, auto_flash=False, baud_rate=1000000, log_severity_level="info"
    )

    adapter = BLEAdapter(driver)
    ble_device = BLEDevice(adapter)
    ble_device.open()
    conn = ble_device.connect_and_discover()
    global newdata 
    if conn is not None:
        #time.sleep(30)
        while True:
                if newdata is True:
                        data =  "HardI:{:.2f},{:.2f},{:.2f}".format(H[0],H[1],H[2])
                        print("write hardiron cal. ({}) with length {}".format(data,len(data)))
                        ble_device.write(data)
                        data =  "SoftI:{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}".format(S[0],S[1],S[2],S[3],S[4],S[5],S[6],S[7],S[8])
                        print("write softiron cal. ({}) with length {}".format(data,len(data)))
                        ble_device.write(data)
                        newdata = False
                pass
    #ble_device.close()


def item_choose(item_list):
    for i, it in enumerate(item_list):
        print("\t{} : {}".format(i, it))
    print(" ")

    while True:
        try:
            choice = int(input("Enter your choice: "))
            if (choice >= 0) and (choice < len(item_list)):
                break
        except Exception:
            pass
        print("\tTry again...")
    return choice


if __name__ == "__main__":
    logging.basicConfig(
        level="ERROR",
        format="%(asctime)s [%(thread)d/%(threadName)s] %(message)s",
    )
    serial_port = None
    if len(sys.argv) < 2:
        print("Please specify connectivity IC identifier (NRF51, NRF52)")
        exit(1)
    init(sys.argv[1])
    if len(sys.argv) > 3:
        serial_port = sys.argv[2]
        TARGET_DEV_NAME = sys.argv[3]
    elif len(sys.argv) == 3:
        serial_port = sys.argv[2]
    else:
        descs = BLEDriver.enum_serial_ports()
        choices = ["{}: {}".format(d.port, d.serial_number) for d in descs]
        print(choices)
        choice = item_choose(choices)
        serial_port = descs[choice].port
        
    main(serial_port)
	
    quit()
