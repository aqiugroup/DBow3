import sys
# sys.path.remove('/home/qiuzc/Documents/1_code/2_github/kimera/catkin_ws/devel/lib/python2.7/dist-packages')
# import sh
# sh.zsh('-c', 'source /home/qiuzc/Documents/1_code/2_github/kimera/python3_ros_ws/devel_isolated/setup.zsh')
import os
import openpyxl
import pandas as pd
import collections

# 3. 广度优先遍历，按层遍历  了解
def traverse_by_brand(path, prefix, suffix, depth=100):
    allfile = []
    LogFileNames = []
    LogDirPath = []
    root_depth = len(path.split(os.path.sep))

    queue = collections.deque()
    queue.append(path)  # 先入队
    while len(queue) > 0:
        item = queue.popleft() #左边出队
        childs = os.listdir(item)  # 获取所有的项目（目录，文件）
        for current in childs:
            abs_path = os.path.join(item, current)  # 绝对路径

            # 大于给定深度，返回
            dir_depth = len(abs_path.split(os.path.sep))
            if dir_depth > root_depth + depth + 1:
                return allfile, LogFileNames, LogDirPath

            if os.path.isdir(abs_path):  # 如果是目录，则入队
                queue.append(abs_path)
            else:
                # print(current)  # 输出
                if current.startswith(prefix) and current.endswith(suffix):
                    allfile.append(abs_path)
                    LogFileNames.append(current)
                    LogDirPath.append(item)

    return allfile, LogFileNames, LogDirPath

if __name__ == "__main__":
    logPath="/media/qiuzc/Document/4_data/kimera_data/true_data/47_floor/220114/debug/2202208001/image"
    all_left_images_out = logPath + "/all_images.txt"
    only_left = False

    [logFiles, Filenames, LogDirPaths] = traverse_by_brand(logPath, "", "left.png", 1)  # 0代表当前目录
    with open(all_left_images_out, 'w') as f:  # 设置文件对象
        for idx in range(0, len(logFiles)):
            log_file = logFiles[idx]
            file_name = Filenames[idx]
            LogDirPath = LogDirPaths[idx]
            print("log_file: ", log_file)
            print("file_name: ", file_name)
            print("LogDirPath: ", LogDirPath)
            f.write(log_file)  # 将字符串写入文件中
            f.write('\n')  # 将字符串写入文件中

            if only_left == False:
                right_image_file = LogDirPath + "/" + file_name[:-8] + "right.png"
                print("right_image_file: ", right_image_file)
                f.write(right_image_file)  # 将字符串写入文件中
                f.write('\n')  # 将字符串写入文件中

            # prefix = LogDirPath.split("\\")[-1][11:]
            # save_xlsx = prefix + "_ate.xlsx"
            #
            # csv = pd.read_csv(log_file, encodeing="utf-8")
            # csv.to_excel(save_xlsx, sheet_name="origin")

    print("Finish!")
    print("Finish!")