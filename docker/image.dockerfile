# This file tells docker what image must be created
# in order to be ahble to test this library
FROM rafa606/ros_noetic_vim

RUN apt clean
ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    python3-pip git iputils-ping net-tools netcat screen build-essential lsb-release gnupg2 curl less \
                    python3-sympy coinor-libipopt-dev sudo valgrind \
                    build-essential pkg-config git \
                    liblapack-dev liblapack3 libopenblas-base libopenblas-dev \
                    libgfortran-7-dev cmake libgsl-dev gdb python3-tk libeigen3-dev \
                    libboost-math-dev \
                    terminator \
    && pip3 install setuptools matplotlib scipy quadpy six cython tk \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    ros-noetic-ifopt exuberant-ctags ros-noetic-ros-base python3-catkin-tools \
                    ros-noetic-plotjuggler-ros \
                    ros-noetic-joint-trajectory-controller \
                    ros-noetic-joint-trajectory-action \
                    ros-noetic-xacro \
                    ros-noetic-gazebo-ros \
                    ros-noetic-gazebo-ros-control \
                    ros-noetic-joint-state-controller \
                    ros-noetic-position-controllers \
                    ros-noetic-robot-state-publisher \
                    ros-noetic-joint-state-publisher \
                    ros-noetic-rqt \
                    ros-noetic-rqt-graph \
                    ros-noetic-roslint \
                    ros-noetic-plotjuggler-ros \
                    ros-noetic-rqt-gui \
                    ros-noetic-rqt-gui-py \
                    ros-noetic-rqt-py-common \
                    ros-noetic-moveit-msgs \
                    ros-noetic-rqt-joint-trajectory-controller \
                    ros-noetic-jsk-rviz-plugins \
                    ros-noetic-kdl-conversions \
                    rviz \
                    python3-tf2-kdl \
                    python3-tf-conversions \
                    libtf-conversions-dev \
    && echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list \
    && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -

# https://amir-yazdani.github.io/post/pykdl/
#RUN git clone -b noetic-devel https://github.com/amir-yazdani/hrl-kdl.git /kdl \
#    && cd /kdl/pykdl_utils && python3 setup.py build && sudo python3 setup.py install \
#    && cd /kdl/pykdl_utils && python3 setup.py build && sudo python3 setup.py install \
#    && cd /kdl/pykdl_utils && python3 setup.py build && sudo python3 setup.py install \

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends -o Dpkg::Options::="--force-confnew" \
                    robotpkg-py38-pinocchio \
    && echo "export PATH=/opt/openrobots/bin:$PATH" >> /etc/bash.bashrc \
    && echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH" >> /etc/bash.bashrc \
    && echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH" >> /etc/bash.bashrc \
    && echo "export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH" >> /etc/bash.bashrc \
    && echo "export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH" >> /etc/bash.bashrc \
    && mkdir -p /aux_ws/src \
    && git clone https://github.com/tork-a/rqt_joint_trajectory_plot.git /aux_ws/src/rqt_joint_trajectory_plot \
    && bash -c 'source /opt/ros/noetic/setup.bash && cd /aux_ws && catkin config --install --install-space /opt/ros/noetic/ --extend /opt/ros/noetic/ && catkin build' \
    && rm -rf /var/lib/apt/lists/* \
    && echo 'source /opt/ros/noetic/setup.bash' > /etc/bash.bashrc \
    && chmod 777 /workspace \
    && echo '\
#!/bin/bash\n\
main(){\n\
export ROS_MASTER_URI=http://172.0.0.1:11311\n\
export ROS_IP=172.0.0.1\n\
screen -S roscore -d -m roscore \n\
terminator >/dev/null 2>&1 & \n\
bash \n\
}\n\
main $@' > /entrypoint.bash
ENTRYPOINT ["bash", "/entrypoint.bash"]
