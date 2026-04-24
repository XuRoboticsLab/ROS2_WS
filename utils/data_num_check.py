import os

def count_first_level(path):
    files = 0
    dirs = 0

    # 只列出第一层
    for entry in os.listdir(path):
        full_path = os.path.join(path, entry)

        if os.path.isfile(full_path):
            files += 1
        elif os.path.isdir(full_path):
            dirs += 1

    return files, dirs


if __name__ == "__main__":
    target_path = "/home/xuroboticslab/ws/new2/data/open_top_drawer"  # 替换成你的路径

    files, dirs = count_first_level(target_path)

    print(f"文件数量: {files}")
    print(f"文件夹数量: {dirs}")