#!/bin/bash
set -e

sudo apt update -y
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-teleop-twist-keyboard
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers
sudo apt install -y ros-humble-pcl-ros
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-joint-state-publisher-gui
sudo apt install -y ros-humble-rplidar-ros
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-interactive-markers
sudo apt install -y ros-humble-nav2-msgs ros-humble-rosidl-default-generators
sudo apt install -y qtbase5-dev libyaml-cpp-dev libtinyxml2-dev ros-humble-tinyxml2-vendor libpcl-dev
sudo apt install -y ros-dev-tools
sudo apt install -y python3-pip
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-serial git
sudo apt install -y python3-argcomplete
sudo apt install -y pcl-tools

# Kinect2 CPU processing, calibration and RTAB-Map (install libfreenect2 separately).
sudo apt install -y \
  libusb-1.0-0-dev \
  libturbojpeg0-dev \
  libeigen3-dev \
  libopencv-dev \
  libboost-dev \
  ros-humble-depth-image-proc \
  ros-humble-rclcpp-components \
  ros-humble-image-transport \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-rtabmap-ros \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-ament-cmake-pytest \
  python3-opencv \
  python3-numpy

# Offline Chinese/English speech recognition and text-to-speech.
# python3-pip and python3-numpy are installed above.
sudo apt install -y alsa-utils pulseaudio-utils python3-pytest
sudo apt install -y ros-humble-joy ros-humble-tf2-ros ros-humble-pcl-conversions
sudo apt install -y libatlas-base-dev liblapack-dev python3-dev build-essential cmake
sudo apt install -y ros-humble-rclpy ros-humble-std-msgs ros-humble-launch-ros \
  ros-humble-ament-index-python ros-humble-ros2run ros-humble-ros2launch
python3 -m pip install --user 'vosk==0.3.45' 'sherpa-onnx==1.13.8'
python3 -m pip install --user 'face-recognition==1.3.0'

# Find the model downloader relative to this script, independent of the working directory.
install_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
model_downloader="$install_script_dir/../../../wp_speech/wp_speech/download_model.py"
if [[ -f "$model_downloader" ]]; then
  python3 "$model_downloader" --language all
  python3 "$install_script_dir/../../../wp_speech/wp_speech/download_tts_model.py"
else
  python3 -m wp_speech.download_model --language all
  python3 -m wp_speech.download_tts_model
fi

# Build every source package needed by the experiments, including speech.
workspace_dir="$install_script_dir"
while [[ "$workspace_dir" != / && ! -f "$workspace_dir/src/wp_speech/package.xml" ]]; do
  workspace_dir="$(dirname -- "$workspace_dir")"
done
if [[ "$workspace_dir" != / ]]; then
  source /opt/ros/humble/setup.bash
  (cd -- "$workspace_dir" && colcon build --symlink-install)
  printf '工作空间编译完成。请在使用终端执行：\nsource "%s/install/setup.bash"\n' "$workspace_dir"
else
  printf '依赖和语音模型已安装；未找到源码工作空间，请在工作空间执行 colcon build --symlink-install。\n'
fi
