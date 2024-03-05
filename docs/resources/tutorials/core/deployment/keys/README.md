# Fast DDS - Vulcanexus Topic Keys Demo

This project contains code for demonstrating the how-to-use topic keys feature.

### Building

Standard build using colcon. Open a terminal and run

```bash
colcon build
```

### Launching the talker

This runs ROS 2 node that publishes a custom-generated message with one of its field, keyed.
First, source the workspace by running:

```bash
# Open new terminal
source install/setup.bash
```

Now, run the right_sensor:

```bash
ros2 run demo_keys_cpp right_sensor
```

In a new terminal, source the worskace
```bash
# Open new terminal
source install/setup.bash
```

Run the left_sensor:

```bash
ros2 run demo_keys_cpp left_sensor
```

Finally, in a third terminal, source the worskace
```bash
# Open new terminal
source install/setup.bash
```

And run the controller

```bash
ros2 run demo_keys_cpp controller
```
