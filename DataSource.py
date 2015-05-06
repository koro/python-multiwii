import threading
import sys, time
import numpy as np

# sys.path.insert(0, "/home/src/QK/mavlink/")
sys.path.insert(0, "/home/src/QK/mavlink/pymavlink/")
# sys.path.insert(0, "/home/x75/tmp")

# import pymavlink
import mavutil
import mavlinkv10 as mavlink
# import pymavlink as mavlink

# baseflight stuff
# - status
# - raw_imu
# - attitude
# - global position int (contains altitude)
# - rc
# - motors

class DataSource(threading.Thread):
    def __init__(self, parent, out_queue, sysid=39):
        threading.Thread.__init__(self)
        self.sysid = sysid
        # use dest port 32002, sysid 39
        # use dest port 32001, sysid 43
        self.mavs = mavutil.mavlink_connection("udp:127.0.0.1:17779", input=True, source_system=self.sysid)
        self.mavo = mavutil.mavlink_connection("udp:127.0.0.1:32001", input=False, source_system=self.sysid)
        # self.mav2 = mavutil.mavlink_connection("udp:127.0.0.1:32002", input=False, source_system=self.sysid)
        self.mavqgc = mavutil.mavlink_connection("udp:127.0.0.1:14550", input=False, source_system=self.sysid)
        # self.mavs = mavutil.mavlink_connection("udp:127.0.0.1:17779", input=True, source_system=self.sysid)
        # self.mavo = mavutil.mavlink_connection("udp:127.0.0.1:32002", input=False, source_system=self.sysid)
        self.out_queue = out_queue
        self.running = True
        self.daemon = True
        self.parent = parent
        print "DS isDaemon", self.isDaemon()

    def run(self):
        while self.running:
            try:
                msg = self.mavs.recv_msg()
                # print type(msg)
                # print msg.get_msgId()
            except:
                # print "Couldn't recv_msg() mavlink msg"
                msg = ""

            if not(msg) or msg == "":
                # print "invalid msg"
                time.sleep(0.0001)
                continue

            # some debugging output
            # print type(msg), msg.get_type()

            # timestamp = time.time()
            # self.out_queue.put((x, timestamp))
            # FIXME: use some callback method here as in oscsrv
            if msg.get_msgId() == mavlink.MAVLINK_MSG_ID_SYS_STATUS:
                self.parent.mavlink_sys_status_handler(msg)
            elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_RAW_IMU:
                self.parent.mavlink_raw_imu_handler(msg)
            elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_ATTITUDE:
                self.parent.mavlink_attitude_handler(msg)
            elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                self.parent.mavlink_global_position_int_handler(msg)
            elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                self.parent.mavlink_rc_channels_raw_handler(msg)
            elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                self.parent.mavlink_servo_output_raw_handler(msg)

            elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_DATA64:
                pass
            elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_DEBUG:
                pass
            elif msg.get_msgId() == mavlink.MAVLINK_MSG_ID_PARAM_VALUE:
                # print "param value", msg.param_id, msg.param_value
                # if self.pid_opt:
                # self.parent.mavlink_lateral_pid_set_params(msg)
                # print msg
                pass
