# Setup Docker Image and Container (ROS2 Foxy)

## Install Docker
Instructions on how to install [Docker](https://docs.docker.com/engine/install/)

## Build Docker Image
Make sure you are in the directory where the Dockerfile is located.
```
docker build -t user/ros2-desktop-vnc:foxy .
```
`user/ros2-desktop-vnc:foxy` will be the tag for the image.
<br>
_If the build fails, try running it again. This issue sometimes occurs, but is fixed by the second build._
Ensure that the _setup_script.sh_ is located in the same folder as the _DockerFile_.

## Create/Run Docker Container
The following command will create/start the docker container.
```
docker run -it -p 6080:80 --name ros2_auto user/ros2-desktop-vnc:foxy
```
`ros2_auto` will be the name of the container.<br>
Visit [http://127.0.0.1:6080/](http://127.0.0.1:6080/) to view the virtual online linux environment.

## Build Workspace
```
cd auto_parking/car_ws/
colcon build --symlink-install
```

## Source Workspace
```
source install/setup.bash
```

## Run Simulation
In a terminal run the following command to launch the simulation environment:
```
ros2 launch auto_park2 launch_sim.launch.py world:=./src/auto_park2/worlds/parking_lot_right.world
```
Then, in another terminal run the following command to launch the parking space tracker:
```
ros2 run line_tracker detect_line
```
Then, in another terminal run the following command to launch the car navigation:
```
ros2 run line_tracker follow_line
```

## TROUBLESHOOTING
When running the command:
```
ros2 launch auto_park2 launch_sim.launch.py world:=./src/auto_park2/worlds/parking_lot_right.world
```
for the first time, it will not properly load the controllers.<br>
To fix this, when running the command for the first time, let it load the environment in Gazebo, then close it and rerun the previous command. If the controllers are properlly loaded, the terminal will show the controller names highlighted in blue and green.
