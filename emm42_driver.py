try:
    from maix import uart, time
    MAIX_MODE = True
except ImportError:
    import serial
    import time as pytime
    MAIX_MODE = False

class Emm42Driver:
    # 修改参数命令
    def set_microstep(self, store=True, value=16):
        # 修改细分：地址 + 0x84 + 0x8A + 是否存储标志 + 细分值 + 校验
        cmd = [self.addr, 0x84, 0x8A, 0x01 if store else 0x00, value]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_id_addr(self, store=True, id_addr=1):
        # 修改ID地址：地址 + 0xAE + 0x4B + 是否存储标志 + ID地址 + 校验
        cmd = [self.addr, 0xAE, 0x4B, 0x01 if store else 0x00, id_addr]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_open_closed_mode(self, store=True, mode=2):
        # 切换开环/闭环模式：地址 + 0x46 + 0x69 + 是否存储标志 + 模式 + 校验
        # mode: 1=开环, 2=闭环
        cmd = [self.addr, 0x46, 0x69, 0x01 if store else 0x00, mode]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_open_current(self, store=False, current=1000):
        # 修改开环模式电流：地址 + 0x44 + 0x33 + 是否存储标志 + 电流(2字节) + 校验
        current_h = (current >> 8) & 0xFF
        current_l = current & 0xFF
        cmd = [self.addr, 0x44, 0x33, 0x01 if store else 0x00, current_h, current_l]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_driver_config(self, store=True, params=None):
        # 修改驱动配置参数：地址 + 0x48 + 0xD1 + 是否存储标志 + 驱动参数(多字节) + 校验
        # params为list或bytes，长度和内容需按协议
        if params is None:
            params = []
        cmd = [self.addr, 0x48, 0xD1, 0x01 if store else 0x00] + list(params)
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_pid_params(self, store=False, kp=62000, ki=100, kd=62000):
        # 修改位置环PID参数：地址 + 0x4A + 0xC3 + 是否存储标志 + Kp(4字节) + Ki(4字节) + Kd(4字节) + 校验
        kp_bytes = kp.to_bytes(4, 'big')
        ki_bytes = ki.to_bytes(4, 'big')
        kd_bytes = kd.to_bytes(4, 'big')
        cmd = [self.addr, 0x4A, 0xC3, 0x01 if store else 0x00]
        cmd += list(kp_bytes) + list(ki_bytes) + list(kd_bytes)
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_speed_mode_params(self, store=True, direction=0, speed=1500, acc=10, en_ctrl=True):
        # 存储速度模式参数：地址 + 0xF7 + 0x1C + 存储/清除标志 + 方向 + 速度(2字节) + 加速度 + En引脚控制启停标志 + 校验
        speed_h = (speed >> 8) & 0xFF
        speed_l = speed & 0xFF
        cmd = [self.addr, 0xF7, 0x1C, 0x01 if store else 0x00, direction, speed_h, speed_l, acc, 0x01 if en_ctrl else 0x00]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_comm_speed_scale(self, store=True, enable=True):
        # 修改通讯控制输入速度缩小10倍：地址 + 0x4F + 0x71 + 是否存储标志 + 是否缩小10倍标志 + 校验
        cmd = [self.addr, 0x4F, 0x71, 0x01 if store else 0x00, 0x01 if enable else 0x00]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))
    # 读取参数命令
    def read_firmware_version(self):
        # 地址 + 0x1F + 校验
        cmd = [self.addr, 0x1F]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_motor_resistance_inductance(self):
        # 地址 + 0x20 + 校验
        cmd = [self.addr, 0x20]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_pid_params(self):
        # 地址 + 0x21 + 校验
        cmd = [self.addr, 0x21]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_bus_voltage(self):
        # 地址 + 0x24 + 校验
        cmd = [self.addr, 0x24]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_phase_current(self):
        # 地址 + 0x27 + 校验
        cmd = [self.addr, 0x27]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_encoder_value(self):
        # 地址 + 0x31 + 校验
        cmd = [self.addr, 0x31]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_input_pulses(self):
        # 地址 + 0x32 + 校验
        cmd = [self.addr, 0x32]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_target_position(self):
        # 地址 + 0x33 + 校验
        cmd = [self.addr, 0x33]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_real_target_position(self):
        # 地址 + 0x34 + 校验
        cmd = [self.addr, 0x34]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_real_speed(self):
        # 地址 + 0x35 + 校验
        cmd = [self.addr, 0x35]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_real_position(self):
        # 地址 + 0x36 + 校验
        cmd = [self.addr, 0x36]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_position_error(self):
        # 地址 + 0x37 + 校验
        cmd = [self.addr, 0x37]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_motor_status(self):
        # 地址 + 0x3A + 校验
        cmd = [self.addr, 0x3A]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_driver_config(self):
        # 地址 + 0x42 + 0x6C + 校验
        cmd = [self.addr, 0x42, 0x6C]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_system_status(self):
        # 地址 + 0x43 + 0x7A + 校验
        cmd = [self.addr, 0x43, 0x7A]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))
    # 触发动作命令
    def trigger_calibrate_encoder(self):
        # 触发编码器校准：地址 + 0x06 + 0x45 + 校验
        cmd = [self.addr, 0x06, 0x45]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def trigger_clear_position(self):
        # 清零位置：地址 + 0x0A + 0x6D + 校验
        cmd = [self.addr, 0x0A, 0x6D]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def trigger_release_clog(self):
        # 解除堵转保护：地址 + 0x0E + 0x52 + 校验
        cmd = [self.addr, 0x0E, 0x52]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def trigger_restore_factory(self):
        # 恢复出厂设置：地址 + 0x0F + 0x5F + 校验
        cmd = [self.addr, 0x0F, 0x5F]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def __init__(self, port, baudrate=115200, addr=1):
        self.addr = addr
        if MAIX_MODE:
            self.ser = uart.UART(port, baudrate)
        else:
            self.ser = serial.Serial(port, baudrate, timeout=0.5)

    def send_cmd(self, cmd_bytes):
        self.ser.write(cmd_bytes)
        if MAIX_MODE:
            time.sleep_ms(2)
            return self.ser.read()
        else:
            pytime.sleep(0.01)
            return self.ser.read(32)

    def checksum(self, data):
        # 默认校验方式：最后一个字节为0x6B
        return 0x6B

    def enable_motor(self, enable=True, sync=False):
        # 地址 + 0xF3 + 0xAB + 使能状态 + 多机同步标志 + 校验
        cmd = [self.addr, 0xF3, 0xAB, 0x01 if enable else 0x00, 0x01 if sync else 0x00]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_speed(self, direction=0, speed=1500, acc=10, sync=False):
        # 地址 + 0xF6 + 方向 + 速度(2字节) + 加速度 + 同步标志 + 校验
        speed_h = (speed >> 8) & 0xFF
        speed_l = speed & 0xFF
        cmd = [self.addr, 0xF6, direction, speed_h, speed_l, acc, 0x01 if sync else 0x00]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def set_position(self, direction=0, speed=1500, acc=0, pulses=3200, abs_mode=False, sync=False):
        # 地址 + 0xFD + 方向 + 速度(2字节) + 加速度 + 脉冲数(4字节) + 绝对/相对 + 同步标志 + 校验
        speed_h = (speed >> 8) & 0xFF
        speed_l = speed & 0xFF
        pulses_bytes = pulses.to_bytes(4, 'big')
        cmd = [self.addr, 0xFD, direction, speed_h, speed_l, acc]
        cmd += list(pulses_bytes)
        cmd += [0x01 if abs_mode else 0x00, 0x01 if sync else 0x00]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def stop(self, sync=False):
        # 地址 + 0xFE + 0x98 + 同步标志 + 校验
        cmd = [self.addr, 0xFE, 0x98, 0x01 if sync else 0x00]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def clear_position(self):
        # 地址 + 0x0A + 0x6D + 校验
        cmd = [self.addr, 0x0A, 0x6D]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def calibrate_encoder(self):
        # 地址 + 0x06 + 0x45 + 校验
        cmd = [self.addr, 0x06, 0x45]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def release_clog(self):
        # 地址 + 0x0E + 0x52 + 校验
        cmd = [self.addr, 0x0E, 0x52]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def restore_factory(self):
        # 地址 + 0x0F + 0x5F + 校验
        cmd = [self.addr, 0x0F, 0x5F]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_version(self):
        # 地址 + 0x1F + 校验
        cmd = [self.addr, 0x1F]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_voltage(self):
        # 地址 + 0x24 + 校验
        cmd = [self.addr, 0x24]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_current(self):
        # 地址 + 0x27 + 校验
        cmd = [self.addr, 0x27]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_encoder(self):
        # 地址 + 0x31 + 校验
        cmd = [self.addr, 0x31]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_status(self):
        # 地址 + 0x3A + 校验
        cmd = [self.addr, 0x3A]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    # 回零相关方法
    def set_origin(self, store=True):
        # 设置单圈回零零点：地址 + 0x93 + 0x88 + 是否存储标志 + 校验
        cmd = [self.addr, 0x93, 0x88, 0x01 if store else 0x00]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def trigger_origin(self, mode=0, sync=False):
        # 触发回零：地址 + 0x9A + 回零模式 + 同步标志 + 校验
        cmd = [self.addr, 0x9A, mode, 0x01 if sync else 0x00]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def abort_origin(self):
        # 强制中断回零：地址 + 0x9C + 0x48 + 校验
        cmd = [self.addr, 0x9C, 0x48]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_origin_params(self):
        # 读取回零参数：地址 + 0x22 + 校验
        cmd = [self.addr, 0x22]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def read_origin_status(self):
        # 读取回零状态标志：地址 + 0x3B + 校验
        cmd = [self.addr, 0x3B]
        cmd.append(self.checksum(cmd))
        return self.send_cmd(bytes(cmd))

    def close(self):
        if not MAIX_MODE:
            self.ser.close()

if __name__ == "__main__":
    # 自动选择串口设备
    if MAIX_MODE:
        device = "/dev/ttyS0"
    else:
        device = "COM3"
    driver = Emm42Driver(port=device, baudrate=115200, addr=1)
    driver.enable_motor(enable=True)
    driver.set_speed(direction=1, speed=1500, acc=10, sync=False)
    driver.close()
