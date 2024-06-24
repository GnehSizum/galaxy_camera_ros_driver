# Galaxy Camera ROS Driver

A ROS package for Daheng Imaging Galaxy USB 3.0 industrial camera.

## Dependency

- ROS noetic
- image_transport
- camera_info_manager
- OpenCV

## Usage

Modify camera information in `config/config.yaml` and `config/camera_info.yaml`.

And run:

```
roslaunch galaxy_camera gxcam.launch
```

## Thanks

Thanks for [campus-guo](https://github.com/campus-guo).