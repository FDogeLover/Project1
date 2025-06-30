import serial
import time

def send_takeoff_command(port_name, baudrate=230400):
    # 构造65字节的数据包
    dataBuf = bytearray(65)
    dataBuf[0] = 0x55         # 帧头1
    dataBuf[1] = 0xAA         # 帧头2
    dataBuf[2] = 0x40         # 起飞命令
    # 其余字节可按需要设置，基本用0填充即可
    dataBuf[63] = 0xAA        # 帧尾
    # 校验和：0~62字节累加 & 0xFF
    checksum = sum(dataBuf[:63]) & 0xFF
    dataBuf[64] = checksum

    # 打开串口
    with serial.Serial(port_name, baudrate, timeout=1) as ser:
        # 发送数据包
        ser.write(dataBuf)
        print("Takeoff command sent.")

        # 可选：等待飞控板反馈
        # start = time.time()
        # while time.time() - start < 3:
        #     if ser.in_waiting:
        #         resp = ser.readline()
        #         if b"Departures" in resp:
        #             print("Takeoff acknowledged by flight controller.")
        #             break
        # else:
        #     print("No takeoff acknowledgment received within timeout.")

if __name__ == "__main__":
    # 修改为你的串口名（如 Windows: 'COM3', Linux: '/dev/ttyUSB0'）
    port = '/dev/ttyAMA0'
    send_takeoff_command(port)