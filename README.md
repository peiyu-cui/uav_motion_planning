# UAV motion-planning

## Quick start

```shell
git clone https://github.com/peiyu-cui/motion-planning.git
cd motion-planning
catkin_make -DCMAKE_CXX_STANDARD=14
```

### 1. Search-Based Methods

#### (1) A*：

* quick start:

  * one terminator

  ```shell
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch
  ```

  * another terminator

  ```shell
  source devel/setup.bash
  roslaunch test test_astar_searching.launch
  ```

* parameters:

  ```xml
  <!-- astar parameters -->
  <param name="astar/resolution" value="0.1"/>
  <param name="astar/lambda_heu" value="1.5"/>
  <param name="astar/allocated_node_num" value="1000000"/>
  ```

  **"astar/resolution":**     astar search resolution, control the search resolution

  **"astar/lambda_heu":**     $f = g(n) + \lambda * h(n)$

  **"astar/allocated_node_num":**  pre-allocated search node num, avoid too many nodes

* methods:

  * tie_breaker：enhance the speed of searching

  * weighted A*: 
    
    ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/equation1.png?raw=true)

* simulation:

![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/astar.gif?raw=true)

#### (2) Kinodynamic A*

* quick start

  * one terminator

  ```shell
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch
  ```

  * another terminator

  ```shell
  source devel/setup.bash
  roslaunch test test_kino_astar_searching.launch
  ```

* parameters:(**need some parameters to change!!!)**

  * kino_astar/collision_check_type = 1 : **kino_astar planning**
  * kino_astar/collision_check_type = 2 : **kino_se(3) planning**
  * **(map_type change)** simulator.xml : "map/fix_map_type" = 0 : generate random map
  * **(map_type change)**simulator.xml : "map/fix_map_type" = 1 : generate fix wall map

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

* methods:

  * **Paper:** B. Zhou, F. Gao, L. Wang, C. Liu and S. Shen, ["Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight](https://arxiv.org/pdf/1907.01531)   

* simulation:

  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/kino_astar.gif?raw=true)

#### (3) SE(3) Planning

* quick start

  * one terminator

  ```shell
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch
  ```

  * another terminator

  ```shell
  source devel/setup.bash
  roslaunch test test_kino_astar_searching.launch
  ```

* parameters(**need some parameters to change!!!!**):

  * "kino_astar/collision_check_type" = 1 : **kino_astar planning**
  * "kino_astar/collision_check_type" = 2 : **kino_se(3) planning**
  * **(map_type change)** simulator.xml : "map/fix_map_type" = 0 : generate random map
  * **(map_type change)**simulator.xml : "map/fix_map_type" = 1 : generate fix wall map

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

* methods:

  * **Paper:** S. Liu, K. Mohta, N. Atanasov, and V. Kumar, [“Search-based motion planning for aggressive flight in SE(3)”](https://arxiv.org/pdf/1710.02748)

* simulation:

  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/kino_se3.gif?raw=true)

### 2. Sampling-Based Methods

#### (1) RRT:

* quick start:

  * one terminator

    ```shell
    source devel/setup.bash
    roslaunch plan_manage single_run_in_sim.launch
    ```

  * another terminator

    ```shell
    source devel/setup.bash
    roslaunch test test_rrt_searching.launch
    ```

* parameters:

```xml
<!-- rrt parameters -->
<param name="rrt/max_tree_node_num" value="100000"/>
<param name="rrt/step_length" value="0.5"/>
<param name="rrt/max_allowed_time" value="5"/>
<param name="rrt/search_radius" value="1.0"/>
```

​    **"rrt/max_tree_node_num":** rrt max tree node num, avoid too many tree nodes

​	**"rrt/step_length":** rrt step(x1, x2, length), control the step length

​	**"rrt/max_allowed_time":** rrt max allowed time, avoid too long search time

​	**"rrt/search_radius":** rrt goal tolerance, control the error between search end and real_end

* method:
  * **kdtree-acceleration:** find nearest tree node

* simulation:

![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/rrt.gif?raw=true)



#### (2) RRT*:

* quick start:

  * one terminator

  ```shell
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch
  ```

  * another terminator

  ```shell
  source devel/setup.bash
  roslaunch test test_rrt_star_searching.launch
  ```

* parameters:

```xml
<param name="rrt_star/max_tree_node_num" value="100000"/>
<param name="rrt_star/step_length" value="0.5"/>
<param name="rrt_star/search_radius" value="1.0"/>
```

* method:
  * **kdtree-acceleration:** find nearest tree node
* simulation:

![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/rrt_star.gif?raw=true)

### 3. Trajectory Optimization

#### (1) RRT* + Minimum Snap :

* quick start:

  * install [osqp](https://github.com/osqp/osqp) and [osqp-eigen](https://github.com/robotology/osqp-eigen)(**important!!**)

  * one terminator

    ```shell
    source devel/setup.bash
    roslaunch plan_manage single_run_in_sim.launch
    ```

  * another terminator

    ```shell
    source devel/setup.bash
    roslaunch test test_minimum_jerk.launch
    ```

* method:
  * **front(Path Finding)**: RRT* or other search-based method, this project is based on RRT*
  * **end(Trajectory Optimization)**: construct a Quadratic Program(QP) based on the discrete waypoints obtained by front RRT*, I use **OSQP Solver** to solve the QP
  * reference paper: [D. Mellinger and V. Kumar, “Minimum snap trajectory generation and control for quadrotors”](https://web.archive.org/web/20120713162030id_/http://www.seas.upenn.edu/~dmel/mellingerICRA11.pdf)
* simulation:
  * **red Line**: RRT* Path
  * **red Sphere**: RRT* Waypoints
  * **purple Line**: minimum jerk trajectory


![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/minimum_jerk.gif?raw=true)

