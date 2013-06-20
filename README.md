robotKin
========

In order to parse URDFs:

In some directory that you don't care about, run the following commands:

      hg clone https://bitbucket.org/osrf/urdfdom_headers
      mkdir urdfdom_headers/build && cd urdfdom_headers/build
      cmake ..
      sudo make install
      cd ../..

      git clone https://github.com/ros/console_bridge.git
      mkdir console_bridge/build && cd console_bridge/build
      cmake ..
      sudo make install
      cd ../..

      hg clone https://bitbucket.org/osrf/urdfdom
      mkdir urdfdom/build && cd urdfdom/build
      cmake ..
      sudo make install
      cd ../..

If you encounter an error about visual_array or collision_array missing in the last step, you probably have ROS Groovy installed and sourced. Try removing it from your ~/.bashrc or ~/.zshrc (you may need to log out and log back in).

Personally, I had to resort to manually deleting certain headers from the ROS source tree. Let me know if you get hung up at this stage: mxgrey@gatech.edu

After the above steps are complete, RobotKin should automatically detect the installed packages. Simply run cmake again and recompile.

A robot kinematics C++ library.

To take this library for a test drive do the following terminal commands:

Clone the repository:

      git clone https://github.com/rowoflo/robotKin.git

CD into build directory:

      cd robotKin/build

Cmake to the parent directory:

      cmake ..

Make from this director:

      make

CD into bin directory:

      cd ../bin

Run program:

      ./robotTest

This will run a tutorial program that is helpful if you are also looking at the code located at:

      robotKin/test/robotTest.cpp

