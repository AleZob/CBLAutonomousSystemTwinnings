#! /bin/zsh
xhost +si:localuser:root
sudo docker run -it \
  -e DISPLAY=:0 \
  -e WAYLAND_DISPLAY=wayland-1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket \
  -e XDG_RUNTIME_DIR=/mnt/linux/runtime-dir \
  --device /dev/snd \
  ros2-humble-gazebo-2irr10
#  -v /run/desktop/mnt/host/linux:/mnt/linux \
#  -e PULSE_SERVER=/mnt/linux/PulseServer \
xhost -si:localuser:root
