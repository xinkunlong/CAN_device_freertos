import argparse
from can.interfaces.slcan import slcanBus
from can import Message

# 创建一个函数，将通道参数值强制转换为大写
def channel_uppercase(value):
    return value.upper()

# 创建命令行参数解析器
parser = argparse.ArgumentParser(description="CAN Communication with slcanBus")

# 添加--channel选项，使用type参数指定处理函数为channel_uppercase
parser.add_argument("-c","--channel", type=channel_uppercase, required=True, help="channel name eg: COM22")

args = parser.parse_args()

msg = Message(arbitration_id=0x666, dlc=8, data=[11, 22, 33, 44, 55, 66, 77, 88], is_extended_id=False)

# 使用命令行参数中的通道创建can_dev
can_dev = slcanBus(channel=args.channel, bitrate=500000)

while True:
    try:
        can_dev.send(msg=msg)
        rec_msg = can_dev.recv(0.1)
        if rec_msg:
            print(rec_msg)
    except KeyboardInterrupt:
        can_dev.shutdown()
        break
