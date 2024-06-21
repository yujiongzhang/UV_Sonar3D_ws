# UV_SONAR3D
Underwater robots using sonar for 3D reconstruction

## Packages

### oculus_ros2 && oculus_driver
the ROS2 driver for oculus m750d sonar, which depends on `oculus_interface`.

### fls_3d
3D reconstruction of underwater structures using forward looking sonar (multibeam sonar).

### m750d_3D
Convert m750d sonar image data `/oculus_m1200d/sonar_image` into a 3D points cloud


## How to use 3D reconstruction

##### step1: play ros2 bag

##### step2: transmit ping to pointcloud
```
ros2 run mbs m750d_ping_to_pointcloud
```

##### step3: vertical scan

```
ros2 run m750d_3d vertical_scan_reconstruction
```