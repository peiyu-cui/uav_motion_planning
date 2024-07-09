# UAV motion-planning

## Quick start

```shell
git clone git@github.com:peiyu-cui/motion-planning.git
cd motion-planning
catkin_make -DCMAKE_CXX_STANDARD=14
```

### 1. Search-Based Methods

* A*：

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

* Kinodynamic A*

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

  * parameters:

    ```xml
    <!-- kino_astar parameters -->
    <param name="kino_astar/rou_time" value="10.0"/>
    <param name="kino_astar/lambda_heu" value="5.0"/>
    <param name="kino_astar/allocated_node_num" value="100000"/>
    <param name="kino_astar/goal_tolerance" value="5.0"/>
    <param name="kino_astar/time_step_size" value="0.05"/>
    <param name="kino_astar/max_velocity" value="5.0"/>
    <param name="kino_astar/max_accelration" value="7.0"/>
    <param name="kino_astar/sample_tau" value="0.5"/>
    ```

  * methods:

    * Paper: B. Zhou, F. Gao, L. Wang, C. Liu and S. Shen, "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight   https://arxiv.org/pdf/1907.01531

  * simulation:

    ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/kino_astar.gif?raw=true)


### 2. Sampling-Based Methods

* RRT:

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



* RRT*:

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



