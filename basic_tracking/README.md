Basic tracking
===============

Beginner project that uses the C++ interface of Mujoco to create a simple application that does the following:

1. Load an scene containing a table and a red box.
2. Loads either a Panda arm or a UR10 manipulator (user's choice)
3. Uses the keyboard to select a few actions to control the arm such that it moves to a few default poses and then tracks the cube's position.


Build
------
1. First off, go to the CMakeLists.txt file and edit the following variables:
   * MUJOCO_DIR : Set it to your local mujoco installation (in my case, I install it in my local folder)
   * MUJOCO_MENAGERIE_DIR_VAR: Similar to above, set it to your local mujoco_menagerie folder.
   
2. Standard cmake process:

   ```
   cd basic_tracking
   mkdir build
   cd build
   cmake .. && make
   ```
   
3. You should have a ./basic_tracking executable in your build folder.

Run
---

1. In the build folder, call the executable with one of the two robot names supported (ur10 or panda), e.g.:

   ```
   cd basic_tracking/build
   ./basic_tracking panda
   ```
   
2. A Mujoco window will show up with the robot on top of a table and a cube. You can control it by:

   0: Moves arm to a zero configuration (all joints to zero)
   1: Moves arm to a default pose (up for UR10)
   2: Moves the arm to a pose that has its EE pointing down
   o: Turns on tracking mode Arm will move its EE so it is place on top of the cube.
   f: Turns off tracking mode
   a/d: Move cube to the left/right.
   w/x: Move cube to up/down.
   s: Stop cube's motion.
   
   
Miscellaneous learning:
-----------------------

I tried to load a table mesh that had rails on the tabletop so the cube wouldn't fall. It didn't work because Mujoco requires convex meshes. If you give a non-convex mesh (like the table with rails), Mujoco silently will create a convex hull of the mesh. I ended up breaking the mesh in pieces. No cool!






