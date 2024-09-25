# UAV Motion Planning

## 0. Quick Installation within 3 Minutes

*Tested on ubuntu 20.04 LTS with ROS Noetic.*

1. Install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full Install *Recommended*).

2. Install git.

    ```bash
    sudo apt install git
    ```

3. Clone the repository.

    ```bash
    git clone git@github.com:peiyu-cui/uav_motion_planning.git
    ```

4. Install dependences.

    ```bash
    # eigen
    sudo apt install libeigen3-dev
    
    # osqp and osqp-eigen
    cd uav_motion_planning
    git submodule update --init --recursive

    ## osqp
    cd 3rd/osqp
    mkdir build
    cd build
    cmake ..  # NOTE: if error occurs on cmake version, just change the cmake_minimum_required in the first line of CMakeLists.txt
    make
    sudo make install

    ## osqp-eigen
    cd 3rd/osqp-eigen
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

5. Compile the code.

    ```bash
    catkin_make -DCMAKE_CXX_STANDARD=14
    ```

## 1. Search-Based Methods

### 1.1. A*

- Quick start:

  ```bash
  # in one terminal
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # in another terminal
  source devel/setup.bash
  roslaunch test test_astar_searching.launch
  ```

- Parameters:

  ```xml
  <!-- astar parameters -->
  <param name="astar/resolution" value="0.1"/>
  <param name="astar/lambda_heu" value="1.5"/>
  <param name="astar/allocated_node_num" value="1000000"/>
  ```

  - **astar/resolution:** astar search resolution, control the search resolution

  - **astar/lambda_heu:** $f = g(n) + \lambda * h(n)$

  - **astar/allocated_node_num:** pre-allocated search node num, avoid too many nodes

- Methods:

  - tie_breaker: enhance the speed of searching

  - weighted A*: 
    
    ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/equation1.png?raw=true)

- Simulation:

  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/astar.gif?raw=true)

### 1.2. Kinodynamic A*

- Quick start:

  ```bash
  # in one terminal
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # in another terminal
  source devel/setup.bash
  roslaunch test test_kino_astar_searching.launch
  ```

- Parameters: **NOTE: Some parameters need to change.**

  - **kino_astar/collision_check_type:** 1: **kino_astar planning**, 2: **kino_se(3) planning**

  - **(map_type change)** simulator.xml: **map/fix_map_type:** 0: generate random map, 1: generate fix wall map

  ```xml
   <!-- kino_astar parameters -->
  <param name="kino_astar/rou_time" value="20.0"/>
  <param name="kino_astar/lambda_heu" value="3.0"/>
  <param name="kino_astar/allocated_node_num" value="100000"/>
  <param name="kino_astar/goal_tolerance" value="2.0"/>
  <param name="kino_astar/time_step_size" value="0.075"/>
  <param name="kino_astar/max_velocity" value="7.0"/>
  <param name="kino_astar/max_accelration" value="10.0"/>
  <param name="kino_astar/acc_resolution" value="4.0"/>
  <param name="kino_astar/sample_tau" value="0.3"/>
  <!-- collision check type 1: kino_astar, 2: kino_se3 -->
  <param name="kino_astar/collision_check_type" value="2"/>
  <!-- robot ellipsoid parameters -->
  <param name="kino_se3/robot_r" value="0.4"/>
  <param name="kino_se3/robot_h" value="0.1"/>
  ```

- Methods:

  - **Paper:** B. Zhou, F. Gao, L. Wang, C. Liu and S. Shen, ["Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight](https://arxiv.org/pdf/1907.01531)

- Simulation:

  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/kino_astar.gif?raw=true)

### 1.3. SE(3) Planning

- Quick start:

  ```bash
  # in one terminal
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # in another terminal
  source devel/setup.bash
  roslaunch test test_kino_astar_searching.launch
  ```

- Parameters: **NOTE: Some parameters need to change.**

  - **kino_astar/collision_check_type:** 1: **kino_astar planning**, 2: **kino_se(3) planning**

  - **(map_type change)** simulator.xml: **map/fix_map_type:** 0: generate random map, 1: generate fix wall map

  ```xml
   <!-- kino_astar parameters -->
  <param name="kino_astar/rou_time" value="20.0"/>
  <param name="kino_astar/lambda_heu" value="3.0"/>
  <param name="kino_astar/allocated_node_num" value="100000"/>
  <param name="kino_astar/goal_tolerance" value="2.0"/>
  <param name="kino_astar/time_step_size" value="0.075"/>
  <param name="kino_astar/max_velocity" value="7.0"/>
  <param name="kino_astar/max_accelration" value="10.0"/>
  <param name="kino_astar/acc_resolution" value="4.0"/>
  <param name="kino_astar/sample_tau" value="0.3"/>
  <!-- collision check type 1: kino_astar, 2: kino_se3 -->
  <param name="kino_astar/collision_check_type" value="2"/>
  <!-- robot ellipsoid parameters -->
  <param name="kino_se3/robot_r" value="0.4"/>
  <param name="kino_se3/robot_h" value="0.1"/>
  ```

- Methods:

  - **Paper:** S. Liu, K. Mohta, N. Atanasov, and V. Kumar, [“Search-based motion planning for aggressive flight in SE(3)”](https://arxiv.org/pdf/1710.02748)

- Simulation:

  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/kino_se3.gif?raw=true)

## 2. Sampling-Based Methods

### 2.1. RRT

- Quick start:

  ```bash
  # in one terminal
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # in another terminal
  source devel/setup.bash
  roslaunch test test_rrt_searching.launch
  ```

- Parameters:

  ```xml
  <!-- rrt parameters -->
  <param name="rrt/max_tree_node_num" value="100000"/>
  <param name="rrt/step_length" value="0.5"/>
  <param name="rrt/max_allowed_time" value="5"/>
  <param name="rrt/search_radius" value="1.0"/>
  ```

​  - **rrt/max_tree_node_num:** rrt max tree node num, avoid too many tree nodes

​	 - **rrt/step_length:** rrt step(x1, x2, length), control the step length

​  - **rrt/max_allowed_time:** rrt max allowed time, avoid too long search time

​  - **rrt/search_radius:** rrt goal tolerance, control the error between search end and real_end

- Methods:
  - **kdtree-acceleration:** find nearest tree node

- Simulation:

  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/rrt.gif?raw=true)

### 2.2. RRT*:

- Quick start:

  ```bash
  # in one terminal
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # in another terminal
  source devel/setup.bash
  roslaunch test test_rrt_star_searching.launch
  ```

- Parameters:

  ```xml
  <param name="rrt_star/max_tree_node_num" value="100000"/>
  <param name="rrt_star/step_length" value="0.5"/>
  <param name="rrt_star/search_radius" value="1.0"/>
  ```

- Methods:
  - **kdtree-acceleration:** find nearest tree node

- Simulation:

  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/rrt_star.gif?raw=true)

## 3. Trajectory Optimization

### 3.1. RRT* + Minimum Snap :

- Quick start:

  ```bash
  # in one terminal
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch

  # in another terminal
  source devel/setup.bash
  roslaunch test test_minimum_jerk.launch
  ```

- Methods:
  - **Front-End (Path Finding)**: RRT* or other search-based method, this project is based on RRT*
  
  - **Back-End (Trajectory Optimization)**: construct a Quadratic Program(QP) based on the discrete waypoints obtained by front RRT*, I use **OSQP Solver** to solve the QP
  
  - **Paper:**: D. Mellinger and V. Kumar, [“Minimum snap trajectory generation and control for quadrotors”](https://web.archive.org/web/20120713162030id_/http://www.seas.upenn.edu/~dmel/mellingerICRA11.pdf)

- Simulation:
  
  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/minimum_jerk.gif?raw=true)

  - **Red Line**: RRT* Path
  - **Red Sphere**: RRT* Waypoints
  - **Purple Line**: minimum jerk trajectory
