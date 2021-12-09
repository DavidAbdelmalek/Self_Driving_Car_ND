# Scan Matching Localization

  
## Project Overview
In this project,  the goal will be to localize a car driving in simulation for at least 170m from the starting position and never exceeding a distance pose error of 1.2m. The simulation car is equipped with a lidar, provided by the simulator at regular intervals are lidar scans. There is also a point cloud map  `map.pcd`  already available, and by using point registration matching between the map and scans localization for the car can be accomplished. This point cloud map has been extracted from the  [CARLA simulator](https://carla.org/).


  

## Usage

  

Press the blue button "Desktop". Start one terminal. Run the Carla simulator by using these Unix commands:

  

```
su - student # Ignore Permission Denied, if you see student@ you are good

cd /home/workspace/c3-project

./run_carla.sh
```

  

Start another terminal. Compile the project by using these Unix commands:

  

```
cd /home/workspace/c3-project

cmake .

make
```


Run the project:
```
./cloud_loc
```
There will be output message, shown below, ask the user to choose between NDT and ICP for scan matching algorithm:

```
Please press:
1: Iterative Closest Point (ICP) algorithm 
2: Normal Distributions Transform (NDT)
```

Lastly, the project will be running, click on the map and tap the UP key 3 times, with delays of 1 second between taps. If the green car gets left behind, run the project again and tap the UP key 3 times again. The second run or the third run usually produce better results than the results of the first run.


## Video Demonstrations
  

### Scan Matching Localization with LIDAR Point Clouds - Algorithm 1: Normal Distributions Transform NDT

https://user-images.githubusercontent.com/22666537/145484017-1c1c7416-c2f2-407c-ae89-6a1db7d5791d.mov


  

### Scan Matching Localization with LIDAR Point Clouds - Algorithm 2: Iterative Closest Point (ICP) **

https://user-images.githubusercontent.com/22666537/145483935-9ef773b4-557c-405c-8866-0d2ad1aa2140.mov


