### how to use
step 1 
```
ros2 launch cirs_girona_cala_viuda cirs_msg_process.launch.py
```

step 2 : publish static TF
```
ros2 launch cirs_girona_cala_viuda cirs_girona_cala_viuda_static_tf.launch.py
```

step 3 : publish topic 
```
ros2 bag play record/cirs/cirs.db3 --topics /depth_sensor /dvl_linkquest /imu_adis_ros /odometry --clock
```