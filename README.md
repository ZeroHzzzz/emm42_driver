# Emm42_V5.0 步进闭环驱动器 Python 串口驱动

本项目为张大头 Emm42_V5.0 步进闭环驱动器的 Python 串口通讯驱动，支持所有官方说明书串口命令，包括控制动作、参数读取、参数修改、回零操作等。

## 依赖环境

本项目依赖包为`pyserial`，建议在 uv 环境下运行。

```bash
uv sync
```

## 文件说明
- `emm42_driver.py`：主驱动代码，包含所有串口命令方法。

## 快速使用
```python
from emm42_driver import Emm42Driver

driver = Emm42Driver(port="COM3", baudrate=115200, addr=1)
# 使能电机
driver.enable_motor(True)
# 设置速度模式
driver.set_speed(direction=0, speed=1500, acc=10)
# 位置模式控制
driver.set_position(direction=0, speed=1500, acc=0, pulses=3200, abs_mode=False)
# 立即停止
driver.stop()
# 清零位置
driver.clear_position()
# 校准编码器
driver.calibrate_encoder()
# 读取电压
driver.read_voltage()
# 读取电流
driver.read_current()
# 回零相关
driver.set_origin(store=True)
driver.trigger_origin(mode=0)
driver.abort_origin()
# 修改参数
driver.set_microstep(value=16)
driver.set_id_addr(id_addr=2)
driver.set_open_closed_mode(mode=2)
# 关闭串口
driver.close()
```

## 主要功能方法
- 控制动作命令：使能/失能、速度模式、位置模式、立即停止等
- 读取参数命令：固件版本、电压、电流、编码器、脉冲数、目标/实时位置、速度、误差、状态等
- 修改参数命令：细分、ID地址、开环/闭环模式、电流、驱动配置、PID参数、速度模式参数、通讯速度缩放等
- 回零相关命令：设置零点、触发回零、强制中断回零、读取回零参数和状态

## 串口参数说明
- `port`：串口号，如 `COM3`
- `baudrate`：波特率，默认 `115200`
- `addr`：驱动器地址，默认 `1`

## 参考资料
- [Emm42_V5.0 步进闭环驱动说明书](https://blog.csdn.net/zhangdatou666/article/details/132644047)
