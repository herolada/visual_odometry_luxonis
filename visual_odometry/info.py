import depthai as dai
from typing import List

print('Searching for all available devices...\n')
# Query all available devices (USB and POE OAK cameras)
infos: List[dai.DeviceInfo] = dai.DeviceBootloader.getAllAvailableDevices()

if len(infos) == 0:
    print("Couldn't find any available devices.")
    exit(-1)


for info in infos:
    # Converts enum eg. 'XLinkDeviceState.X_LINK_UNBOOTED' to 'UNBOOTED'
    state = str(info.state).split('X_LINK_')[1]

    print(f"Found device '{info.name}', MxId: '{info.deviceId}', State: '{state}'")

device = dai.Device(infos[0])
print("Available camera sensors: ", device.getCameraSensorNames())
calib = device.readCalibration()
eeprom = calib.getEepromData()
print(f"Product name: {eeprom.productName}, board name {eeprom.boardName}")
