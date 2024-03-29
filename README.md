# 编译环境配置

- **LCM**
- **OpenCV-4.1.1**: 在使用cmake时确保包含了gstreamer。如果没有，请按照官方文档进行安装。
- **Paddle Inference**: 使用预编译的C++版本，部署到ARM设备上请使用ARM版本。
- **GStreamer**: 依赖项。

# 需要修改的内容

## paddle_detection/CMakeLists.txt：

- `WITH_MKL`: 是否使用MKL或OpenBLAS，对于TX2设备需要设置为OFF。
- `WITH_GPU`: 是否使用GPU预测，可选。
- `PADDLE_DIR`: 修改双引号中的内容为C++版本Paddle的路径。
- `PADDLE_LIB_NAME`: 预测库的名称，不同平台和版本的预测库名称可能不同，请查看所下载的预测库中`paddle_inference/lib/`文件夹下的lib名称。
- `CUDA_LIB`: CUDA路径。
- `CUDNN_LIB`: CuDNN路径。
- `TENSORRT_INC_DIR`: TensorRT的include路径。
- `TENSORRT_LIB_DIR`: TensorRT的lib路径。

## tactile_paving/scripts/signalLampDetection.py：

- 将第一行修改为本地Python路径。

# 功能包说明

- **face_light_sdk_nano**: 机器狗头部灯光。
- **paddle_detection**: 飞桨目标识别C++版本。
- **robot_voice**: 科大迅飞语言识别，仅适用于x86平台。
- **slamware_ros_sdk、slamware_sdk**: 雷达发布话题。
- **tactile_paving**: 导盲场景。
- **unitree_legged_msgs、unitree_legged_real、unitree_legged_sdk-master**: 宇树机器狗控制功能包。

# 主要运行命令

- `roslaunch tactile_paving Start_control.launch`: 启动导盲场景，自动开启寻迹、雷达避障、机器狗前灯。

# 主要业务代码

- `tactile_paving/src/example_Tracing.cpp`: 寻迹场景。
- `tactile_paving/src/lidar_node.cpp`: 雷达避障。
- `tactile_paving/src/unitree_put_cmd.cpp`: 订阅/cmd_vel中Twist内容并转换为unitree发送的高位控制。
- `tactile_paving/scripts/signalLampDetection.py`: Python版本的飞桨目标识别，速度较慢且存在bug。
- `robot_voice/src/voice_control.cpp`: 语音识别的主要逻辑。
- `paddle_detection/src/main.cc`: C++版本的飞桨目标识别。
- `face_light_sdk_nano/src/main.cpp`: 机器狗头部灯光。

# 可修改的启动文件

- `tactile_paving/launch/Start_control.launch`。
