# 手眼校准

## 安装Python依赖

```shell
pip install -r requirements.txt
```

## 手眼设置

### 准备数据

**Instructions:**
- 1.拍摄棋盘的照片和记录相应的机器人末端姿态
- 2.确保机器人末端姿态包括所有六个维度（6D）的变化。
- 3.使用以下文件结构组织捕获的图像和机器人姿态。
- 4.确保图像的时间戳格式与`datetime.now().strftime('%Y-%m-%d_%H-%M-%S')`匹配：

```shell
DI0226/DI
├── DI0225.csv #x,y,z,rx,ry,rz  (rx,ry,rz euler angles from -180 to 180)
├── frame2025-02-25_10-20-09.jpg
├── frame2025-02-25_10-26-04.jpg
├── frame2025-02-25_10-27-20.jpg
├── frame2025-02-25_10-27-51.jpg
├── frame2025-02-25_10-28-22.jpg
├── frame2025-02-25_10-29-08.jpg
├── frame2025-02-25_10-29-51.jpg
├── frame2025-02-25_10-30-15.jpg
├── frame2025-02-25_10-31-08.jpg
├── frame2025-02-25_10-31-35.jpg
├── frame2025-02-25_10-32-25.jpg
├── frame2025-02-25_10-33-09.jpg
├── frame2025-02-25_10-33-39.jpg
├── frame2025-02-25_10-34-48.jpg
└── frame2025-02-25_10-35-03.jpg
```

生成手眼标定的输入文件：

```shell
cp generate_calibrate_input_config_template.yaml generate_calibrate_input_config.yaml
```

打开刚创建的YAML文件，并根据您的设置更改输入生成配置部分：

```yaml
pose_file: 'DI0226/DI/6_data.csv' # 包含姿态数据的文件路径：x,y,z,rx,ry,rz
src_dir: 'DI0226/DI' # 包含图像的目录
dst_base: './input_data'
camera_params:
  - 1123.9  # fx
  - 1123.4  # fy
  - 982.364 # cx
  - 567.264 # cy
  - 0.0769521  # k1
  - -0.105434  # k2
  - 6.25417e-05  # p1
  - 3.9459e-5  # p2
  - 0.0428337  # k3
square_size: 100  # 棋盘格的大小
pattern_size: [7, 5]  # 棋盘格的列数和行数
corner_color: 'black'  # 角点颜色，'black'或'white'
error_threshold: 0.9
```

运行输入生成脚本：

```shell
python generate_calibrate_input.py --config-file generate_calibrate_input_config.yaml
```

用法：

```shell
usage: generate_calibrate_input.py [-h] [--config-file CONFIG_FILE]

处理棋盘图像并生成校准输入。

选项:
  -h, --help            显示此帮助消息并退出
  --config-file CONFIG_FILE
                        配置YAML文件的路径
```

您可以在`input_data`中找到示例输出，路径如`input_data/2025-02-26_14-26-39`和`input_data/filtered_pose_data_2025-02-26_14-26-39.csv`作为参考。

### 运行校准脚本以获取相机到基座的变换矩阵

```shell
cp hand_to_eye_config_template.yaml hand_to_eye_config.yaml
```

根据您的设置更改手眼标定配置部分：

```yaml
pose_file: 'input_data/filtered_pose_data_2025-02-26_14-26-39.csv'
image_folder: 'input_data/2025-02-26_14-26-39'
camera_params:
  - 1123.9  # fx
  - 1123.4  # fy
  - 982.364 # cx
  - 567.264 # cy
  - 0.0769521  # k1
  - -0.105434  # k2
  - 6.25417e-05  # p1
  - 3.9459e-5  # p2
  - 0.0428337  # k3
square_size: 100  # 棋盘格的大小
pattern_size: [7, 5]  # 棋盘格的列数和行数
corner_color: 'black'  # 角点颜色，'black'或'white'
result_file: 'result_matrix.txt' 
```

运行标定脚本：

```shell
python hand_to_eye.py --config-file hand_to_eye_config.yaml
```

用法：

```shell
usage: hand_to_eye.py [-h] [--config_file CONFIG_FILE]

使用棋盘图像进行手眼标定。

选项:
  -h, --help            显示此帮助消息并退出
  --config_file CONFIG_FILE
                        YAML配置文件的路径。
```

#### 示例输出

```shell
WARNING:__main__:使用默认配置模板。内部的参数设置可能不适应您的当前设置。
INFO:__main__:从CSV加载的姿态向量：input_data/filtered_pose_data_2025-02-26_14-26-39.csv
INFO:__main__:使用来自input_data/2025-02-26_14-26-39的输入图像
INFO:__main__:相机到基座的旋转矩阵：
INFO:__main__:[[ 0.99128407 -0.12835439 -0.02968225]
 [-0.12758403 -0.99147255  0.02654234]
 [-0.03283596 -0.02252402 -0.99920692]]
INFO:__main__:相机到基座的平移向量：
INFO:__main__:[[1924.24637018]
 [-774.55760843]
 [2664.42951569]]
INFO:__main__:相机到基座的姿态矩阵：
INFO:__main__:[[   0.99128407   -0.12835439   -0.02968225 1924.24637018]
 [  -0.12758403   -0.99147255    0.02654234 -774.55760843]
 [  -0.03283596   -0.02252402   -0.99920692 2664.42951569]
 [   0.            0.            0.            1.        ]]
INFO:__main__:相机到基座的姿态矩阵已保存到result_matrix.txt
```

### 进一步验证

您可以使用`eye_hand_validation.py`脚本将记录的点云转换到机器人手臂的框架中，使用刚刚从校准中计算出的矩阵：

```shell
usage: eye_hand_validation.py [-h] [--points_dir POINTS_DIR] [--matrix_file MATRIX_FILE]

将点云转换到机器人基座框架。

Usage:
  -h, --help            显示此帮助消息并退出
  --points_dir POINTS_DIR
                        包含点的PLY文件的路径。
  --matrix_file MATRIX_FILE
                        包含cam2base矩阵的文件路径。

```
使用点云可视化软件选取兴趣点，将机器人移动到对应位置进行验证