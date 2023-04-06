# Setup Docker Image and Container (ROS2 Foxy)

## Install Docker
Instructions on how to install [Docker](https://docs.docker.com/engine/install/)

## Build Docker Image
Make sure you are in the directory where the Dockerfile is located.
```
docker build -t user/ros2-desktop-vnc:foxy .
```
`user/ros2-desktop-vnc:foxy` will be the tag for the image.

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
git clone https://github.com/TheAndrew-O/auto_parking.git
```

## Move parking lot model to gazebo-models
```
sudo cp -r /home/ubuntu/auto_parking/turtlebot3_ws/src/auto_park/worlds/parking_lot/ /usr/share/gazebo-11/models/
```

## Build Workspace
```
cd turtlebot3_ws/
colcon build --symlink-install
```

## Source Workspace
```
source install/setup.bash
```

## Run Simulation
```
ros2 launch auto_park launch_sim.launch.py world:=./src/auto_park/worlds/parking_lot1.world
```
