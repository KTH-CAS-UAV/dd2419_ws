**Readme about the detection part of the detection part of Milestone 1:**

__For playing back the rosbag run the following nodes /launch files:__
```bash
pixi run rviz2
pixi run ros2 run odometry odometry  #important to have transform from base_link to odom
pixi run ros2 launch robp_launch frames_launch.xml
pixi run ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/dd2419_ws/rosbags/obj_det_odom_opti_rosbag
pixi run ros2 run detection detection
```

__ssh into the Robot in Clemens Hotspot:__
```bash
sshpass -p 'group3' ssh group3@10.28.211.242
```

__Record rosbag:__
```bash
pixi run ros2 bag record -o obj_detection /phidgets/motor/encoders /realsense/depth/color/points /realsense/depth/image_rect_raw /realsense/color/image_raw/compressed /realsense/color/image_raw
```

__Some words about the Code:__

- The core concept is a simple thresholding for the 3d-Color-Points
- First we threshold for distance and height and already apply that geometric filter to the points
    - this is currently done in the frame of the 3d point cloud
    - one could also do that in the map frame, that would be a bit easier to understand would require tf lookups though
- Afterwards we threshold the color using oklab color space
    - oklab color space is an optimized color space (similar to rgb or hsv) but optimized for purposes like color detection using cameras
    - it has 3 Components
        - L: perceptual lightness ranging from 0% to 100%
        - a: for green and red values (green: -0.5, red: 0.5)
        - b: for blue and yellow values (blue: -0.5, yellow: 0.5)
    - This color space does better at handling differences in illumniation 
        - when thresholding in rgb space the a color A that is close to another color B in terms of their coordinates (e.G. (150, 10, 10) and (140, 10, 20)) are not actually close to each other in terms of how they look in different lightings, OKlab is supposed to handle this better
        - https://en.wikipedia.org/wiki/Oklab_color_space
    - As of right now we only threshold the Components a and b but that can and probably must be changed for future detection algorithms
    - This detection currently outperforms the rgb detection I used in the bootcamp

- rgb values of object, measured with the realsense (2 measurements at 2 different distances)
    - red: 137 55 50  / 145 37 17
    - green: 0 72  58 / 1 67 56 
    - blue: 1 90 134 / 2 76 117
    - wood: 111 77 49 / 90 73 54
    --> theese values are roughly averaged and are used to calculate oklab threshold values 

__What's next?__

- we need to filter out points outside the workspace (important!! and has to be ratehr accurate otherwise we will mistake cardboard for wood)
- we need to cluster the points that we output to further reduce noise
- we need to calculate the position of the objects in the map frame and save it (in the map file for example)

  
