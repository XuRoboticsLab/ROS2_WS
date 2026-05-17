import subprocess
import re

def check_realsense():
    try:
        result = subprocess.run(
            ["rs-enumerate-devices", "-S"],
            capture_output=True, text=True, timeout=10
        )
    except FileNotFoundError:
        print("未找到 rs-enumerate-devices，请确认 librealsense 已正确安装。")
        return

    output = result.stdout.strip()
    if not output:
        print("未检测到 RealSense 相机。")
        return

    print(output)

if __name__ == "__main__":
    check_realsense()