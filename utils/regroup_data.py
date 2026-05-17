import os
import shutil

src_dir = "/home/xuroboticslab/ws/new2/data/0425"
dst_odd = "/home/xuroboticslab/ws/new2/data/open_bottom_drawer"
dst_even = "/home/xuroboticslab/ws/new2/data/close_bottom_drawer"

# 创建目标目录（如果不存在）
os.makedirs(dst_odd, exist_ok=True)
os.makedirs(dst_even, exist_ok=True)

# 获取所有子文件夹（只要第一层）
folders = [
    f for f in os.listdir(src_dir)
    if os.path.isdir(os.path.join(src_dir, f))
]

# 排序（时间戳字符串本身就可排序）
folders.sort()

print(f"Total folders: {len(folders)}")

# 遍历并按奇偶分配（从1开始）
for idx, folder in enumerate(folders, start=1):
    src_path = os.path.join(src_dir, folder)

    if idx % 2 == 1:
        dst_path = os.path.join(dst_odd, folder)
        print(f"[{idx}] ODD  -> {folder}")
    else:
        dst_path = os.path.join(dst_even, folder)
        print(f"[{idx}] EVEN -> {folder}")

    shutil.move(src_path, dst_path)

print("Done!")