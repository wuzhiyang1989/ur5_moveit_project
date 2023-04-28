import serial  # 导入模块
import threading
import time
import logging
import sys

dh_projectstart_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0x01, 0xE8]
dh_prjectend_buf = [0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0x03, 0xEA]

class ts_control(object):
    """docstring for ts_control"""
    def __init__(self):
        self.data_buf = ''
        self.ser_ = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)
            # 判断是否打开成功
        if(self.ser_.is_open):
            print("串口打开成功")
        # except Exception as e:
            # print("---异常---：", e)


    def ts_write(self, data):
        result = self.ser_.write(data)  # 写数据
        print('data = ', data)
        logging.info("Write %s(%d)" % (data.hex(), result))


    def ts_read(self):
        # if self.ser_.in_waiting != 0:
        #     print(f'len = {self.ser_.in_waiting}')
        #     self.data_buf = self.ser_.read(self.ser_.in_waiting)
        while self.ser_.in_waiting == 0:
            pass

        print(f'len = {self.ser_.in_waiting}')
        self.data_buf = self.ser_.read(self.ser_.in_waiting)
        return self.data_buf



# 测试函数



# # writebuf = bytearray([0x55, 0xAA, 0x04, 0xE2, 0x00, 0x00, 0x02, 0x00, 0x01, 0xE8])
# writebuf = bytearray(dh_prjectend_buf)
# ts = ts_control()
# ts.ser_.flushInput()
# ts.ts_write(writebuf)

# data = ts.ts_read()
# print(len(data))