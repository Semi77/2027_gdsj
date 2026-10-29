"""预留的 K230 JSON 行串口桥，未来由模型程序按文档输出检测结果。"""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import serial
import rclpy
from std_msgs.msg import String

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from shared.results import validate_detection


def main():
    """@brief --port 为独立 USB 串口，--baud 为与 K230 程序一致的波特率。"""
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',required=True)
    parser.add_argument('--baud',type=int,default=115200)
    args=parser.parse_args()
    resolved=os.path.realpath(args.port)
    if not (resolved.startswith('/dev/ttyUSB') or resolved.startswith('/dev/ttyACM')):
        parser.error('请选择 K230 独立 USB TTL 串口，不能占用 GPIO 雷达 UART')
    rclpy.init()
    node=rclpy.create_node('k230_serial_bridge')
    publisher=node.create_publisher(String,'/k230/detections_json',10)
    port=None
    buffer=bytearray()
    dropping=False
    last_warning=0.
    try:
        while rclpy.ok():
            try:
                if port is None:
                    port=serial.Serial(args.port,args.baud,timeout=.1,exclusive=True)
                    port.reset_input_buffer()
                    buffer.clear()
                    dropping=False
                chunk=port.read(4096)
                # 有界保留半行，串口超时不会误丢合法的拆包消息。
                for value in chunk:
                    if value==10:
                        raw=bytes(buffer)
                        buffer.clear()
                        if dropping:
                            dropping=False
                            continue
                        try:
                            data=validate_detection(json.loads(raw))
                            msg=String()
                            msg.data=json.dumps(data,ensure_ascii=False,allow_nan=False)
                            publisher.publish(msg)
                        except (ValueError,UnicodeError) as error:
                            if time.monotonic()-last_warning>1:
                                node.get_logger().warning(str(error))
                                last_warning=time.monotonic()
                    elif not dropping:
                        buffer.append(value)
                        if len(buffer)>65536:
                            buffer.clear()
                            dropping=True
                rclpy.spin_once(node,timeout_sec=0)
            except (OSError,serial.SerialException) as error:
                node.get_logger().error(f'K230 串口失效：{error}')
                if port:
                    port.close()
                port=None
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        if port:
            port.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
