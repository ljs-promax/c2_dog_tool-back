import threading
import socket
import json
import time
import sys
import tty
import termios

# 配置
REPORT_PORT = 9876
COMMAND_PORT = 9877
TARGET_IP = "127.0.0.1"

def status_listener():
    """ 监听 9876 端口并格式化打印 """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", REPORT_PORT))
    while True:
        data, _ = sock.recvfrom(2048)
        try:
            msg = json.loads(data.decode('utf-8'))
            dev = msg.get("device", "")
            ver = msg.get("version", "")
            prog = msg.get("progress", 0)

            # ========================
            # 👉 IMU / 电池 只显示成功/失败
            # ========================
            if dev in ["imu", "battery"]:
                if prog == -1:
                    print(f"\n✅ [成功] 设备: {dev}")
                elif prog == -2:
                    print(f"\n❌ [失败] 设备: {dev}")
                continue  # 不再走下面逻辑

            # 原来的版本/升级逻辑（其他设备）
            if prog == -1:
                print(f"\n[结果] 设备: {dev} | 版本: {ver}")
            elif prog == -2:
                print(f"\n[失败] 设备: {dev} | 原因: {ver}")
            elif 0 <= prog <= 100:
                bar = "#" * (prog // 5)
                print(f"\r[升级] {dev}: [{bar:<20}] {prog}% ({ver})", end="")
                if prog == 100:
                    print()
        except:
            pass
def input_linux(prompt):
    """ Linux 下完美支持退格、删除、无乱码 """
    print(prompt, end='', flush=True)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        buf = []
        while True:
            c = sys.stdin.read(1)
            if c == '\n' or c == '\r':
                print()
                return ''.join(buf)
            # 退格
            if c == '\x7f' or c == '\b':
                if buf:
                    buf.pop()
                    print('\b \b', end='', flush=True)
            else:
                buf.append(c)
                print(c, end='', flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)
def send(cmd):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(cmd.encode('utf-8'), (TARGET_IP, COMMAND_PORT))
    sock.close()

def main_menu():
    threading.Thread(target=status_listener, daemon=True).start()
    
    while True:
        print("\n" + "="*30)
        print("  OTA 交互控制台")
        print("="*30)
        print("1. 查询所有设备版本       → 输入: check_version all")
        print("2. 查询 MCU 版本          → 输入: check_version mcu")
        print("3. 查询单个关节版本       → 输入: check_version joint <ch> <id>")
        print("4. 升级 MCU               → 输入: upgrade mcu <固件路径>")
        print("5. 升级单个关节           → 输入: upgrade joint <ch> <id> <固件路径>")
        print("6. 控制单个关节电机       → 输入: control joint <ch> <id> <kp> <kd> <pos> <vel> <tor>")
        print("7. 查询单个关节错误码     → 输入: query joint <ch> <id>")
        print("8. 查询 IMU 数据          → 输入: query imu")
        print("9. 查询电池               → 输入: query battery")
        print("10. 查询单个关节序列号     → 输入: query joint_sn <ch> <id>")
        print("11. 写入单个关节序列号     → 输入: set joint_sn <ch> <id> <sn>")
        print("0. 退出")

        # 只保留一次输入，支持退格
        cmd = input_linux("\n请输入完整指令: ").strip()

        if cmd == "0":
            print("退出程序")
            break

        if cmd:
            send(cmd)  # 直接发送完整指令
        else:
            print("无效输入，请重新输入")

        time.sleep(0.5)
        

if __name__ == "__main__":
    main_menu()