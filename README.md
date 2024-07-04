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

  ```shell
  source devel/setup.bash
  roslaunch plan_manage single_run_in_sim.launch
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

    **"astar/lambda_heu":**     $f = g(n) + lambda_{heu} * h(n)$

    **"astar/allocated_node_num":**  pre-allocated search node num, avoid too many nodes

  * methods:

    * tie_breaker：enhance the speed of searching
  
    * weighted A*: 
      $$
      \begin{equation}
      f(n)=\left\{
      \begin{aligned}
      g(n) & , & \lambda =0,Dijkstra \\
      g(n)+h(n) & , &\lambda=1,A* \\
      g(n)+\lambda h(n) & , & \lambda>1,A* Greedy
      \end{aligned}
      \right.
      \end{equation}
      $$
      
  
  * simulation:
  
  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/astar.gif?raw=true)

### 2. Sampling-Based Methods

* RRT:

  * quick start:

    ```shell
    source devel/setup.bash
    roslaunch plan_manage single_run_in_sim.launch
    roslaunch test test_rrt_searching.launch
    ```

  * parameters:

  ```xml
  <!-- rrt parameters -->
  <param name="rrt/max_tree_node_num" value="100000"/>
  <param name="rrt/step_length" value="0.5"/>
  <param name="rrt/max_allowed_time" value="5"/>
  <param name="rrt/goal_tolerance" value="0.2"/>
  ```

  ​    **"rrt/max_tree_node_num":** rrt max tree node num, avoid too many tree nodes

  ​	**"rrt/step_length":** rrt step(x1, x2, length), control the step length

  ​	**"rrt/max_allowed_time":** rrt max allowed time, avoid too long search time

  ​	**"rrt/goal_tolerance":** rrt goal tolerance, control the error between search end and real_end

  * method:
    * **kdtree-acceleration:** find nearest tree node
  
  * simulation:
  
  ![Image](https://github.com/peiyu-cui/motion-planning/blob/main/pic/rrt.gif?raw=true)



