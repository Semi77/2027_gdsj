#!/usr/bin/env python3
"""保留桌面交互输入，通过原ROS角度接口控制STM32 PA8。"""
import subprocess


def main():
    """@brief 接收0至270度输入，退出后PA8保持最后PWM。"""
    print('STM32 PA8：输入0–270°，输入q退出，退出后保持最后输出。')
    while True:
        value = input('角度> ').strip()
        if value.lower() in {'q', 'quit', 'exit'}:
            return
        subprocess.run(['/home/xu/servo-angle-set', value], check=False)


if __name__ == '__main__':
    main()
