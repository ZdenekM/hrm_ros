#!/usr/bin/python

import struct
from bluepy.btle import Peripheral, ADDR_TYPE_RANDOM, AssignedNumbers
import rospy
from hrm_msgs.msg import HrmData


# https://github.com/IanHarvey/bluepy/issues/53
# https://github.com/rlangoy/bluepy_examples_nRF51822_mbed/blob/master/readButton1Notify.py
# https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.heart_rate_measurement.xml

# sudo apt-get install libcap2-bin
# sudo setcap 'cap_net_raw,cap_net_admin+eip' `which gatttool`

class HRM(Peripheral):
    def __init__(self, addr):
        Peripheral.__init__(self, addr, addrType=ADDR_TYPE_RANDOM)


if __name__ == "__main__":

    rospy.init_node("hrm_node")

    pub = rospy.Publisher("/hrm/polar", HrmData, queue_size=1)

    cccid = AssignedNumbers.client_characteristic_configuration
    hrmid = AssignedNumbers.heart_rate
    hrmmid = AssignedNumbers.heart_rate_measurement

    hrm = None
    try:
        hrm = HRM(rospy.get_param("~mac"))

        service, = [s for s in hrm.getServices() if s.uuid == hrmid]
        ccc, = service.getCharacteristics(forUUID=str(hrmmid))

        if 0:  # This doesn't work
            ccc.write('\1\0')

        else:
            desc = hrm.getDescriptors(service.hndStart,
                                      service.hndEnd)
            d, = [d for d in desc if d.uuid == cccid]

            hrm.writeCharacteristic(d.handle, '\1\0')


        def print_hr(cHandle, data):

            flags = struct.unpack("B", data[0])[0]

            msg = HrmData()
            msg.timestamp = rospy.Time.now()

            msg.heart_rate = struct.unpack("B", data[1])[0]

            if flags & (1 << 4):
                # should be in interval 0.6-1.2 seconds
                msg.rr_interval = struct.unpack("h", data[-2:])[0] / 1024.0

            pub.publish(msg)


        hrm.delegate.handleNotification = print_hr

        while not rospy.is_shutdown():
            hrm.waitForNotifications(3.)

    finally:
        if hrm:
            hrm.disconnect()
