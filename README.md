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
## Create/Run Docker Container
The following command will create/start the docker container.
```
docker run -it -p 6080:80 --name ros2_auto user/ros2-desktop-vnc:foxy
```
`ros2_auto` will be the name of the container.<br>
Visit [http://127.0.0.1:6080/](http://127.0.0.1:6080/) to view the virtual online linux environment.

## Install python dependencies
* Click the icon located at the bottom left corner of the screen.
* Under `System Tools`, click `LXTerminal` to open the linux terminal.
```
pip install xacro
```
## Clone repo to VOLE
```
cd
git clone https://github.com/TheAndrew-O/auto_parking.git
```

## Build Workspace
```
cd .../auto_parking/car_ws/
colcon build --symlink-install
```

## Source Workspace
```
source install/setup.bash
```

## Run Simulation
In a terminal run the following command to launch the simulation environment:
```
ros2 launch auto_park2 launch_sim.launch.py world:=./src/auto_park2/worlds/parking_lot5.world
```
Then, in another terminal run the following command to launch the parking space tracker:
```
ros2 run line_tracker detect_line --ros-args -p tuning_mode:=false
```
Then, in another terminal run the following command to launch the car navigation:
```
ros2 run line_tracker follow_line --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```
