# syntax=docker/dockerfile:1
FROM ros:noetic
ARG GRB_VERSION=11.0.2
ARG GRB_SHORT_VERSION=11.0
ARG NLOPT_VERSION=2.7.1
ARG IPOPT_VERSION=3.14.4

LABEL Maintainer="Nick Mohammad <nm9ur@virginia.edu>" \
      Description="ROS Trajectory Planner and MPC"

# dependency layer
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends git wget vim build-essential && \
    apt-get install -y gcc g++ gfortran patch pkg-config liblapack-dev && \
    apt-get install -y libmetis-dev cppad ca-certificates ginac-tools libginac-dev && \
    apt-get install -y python3-catkin-tools

WORKDIR /home

# install IPOPT
RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-${IPOPT_VERSION}.tar.gz && \
    tar -xvf Ipopt-${IPOPT_VERSION}.tar.gz && \
    rm Ipopt-${IPOPT_VERSION}.tar.gz && \
    mv Ipopt-releases-${IPOPT_VERSION} ipopt && \
    mkdir ipopt/build && \
    git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git && \
    mv ThirdParty-Mumps ipopt/

WORKDIR /home/ipopt/ThirdParty-Mumps

RUN ./get.Mumps && \
    ./configure && \
    make && \
    make install

WORKDIR /home/ipopt/build

RUN ../configure && \
    make && \
    make install && \
    cp -r /usr/local/include/coin-or /usr/local/include/coin
    
# install GUROBI
WORKDIR /opt

RUN export GRB_PLATFORM="linux64" && \
    update-ca-certificates && \
    wget -v https://packages.gurobi.com/${GRB_SHORT_VERSION}/gurobi${GRB_VERSION}_$GRB_PLATFORM.tar.gz && \
    tar -xvf gurobi${GRB_VERSION}_$GRB_PLATFORM.tar.gz  && \
    rm -f gurobi${GRB_VERSION}_$GRB_PLATFORM.tar.gz && \
    mv -f gurobi* gurobi && \
    rm -rf gurobi/$GRB_PLATFORM/docs && \
    mv -f gurobi/$GRB_PLATFORM*  gurobi/linux

WORKDIR /opt/gurobi/linux/src/build

RUN make && \
    cp libgurobi_c++.a /opt/gurobi/linux/lib && \
    mkdir -p /home/catkin_ws/src

ENV GUROBI_HOME /opt/gurobi/linux
ENV PATH $PATH:$GUROBI_HOME/bin
ENV LD_LIBRARY_PATH $GUROBI_HOME/lib

# install NLOPT
WORKDIR /home

RUN wget https://github.com/stevengj/nlopt/archive/refs/tags/v${NLOPT_VERSION}.tar.gz && \
    tar -xvf v${NLOPT_VERSION}.tar.gz && \
    mv nlopt-${NLOPT_VERSION} nlopt && \
    mkdir nlopt/build

WORKDIR /home/nlopt/build

RUN cmake .. && \
    make && \
    make install

# install ROS packages and dependencies
RUN apt-get install -y --no-install-recommends ros-noetic-costmap-2d ros-noetic-tf2-geometry-msgs ros-noetic-gmapping && \
    apt-get install -y --no-install-recommends ros-noetic-rviz ros-noetic-backward-ros ros-noetic-pcl-ros sqlite3 python3-pip

WORKDIR /home/catkin_ws

RUN git clone https://github.com/artivis/distance_map.git && \
    rm -rf distance_map/distance_map_rviz distance_map/distance_map_opencv && \
    mv distance_map src/distance_map && \
    sed -i 's/LaserOdometryInstanDistanceMapInstantiatertiater/DistanceMapConverterInstantiater/' \
    /home/catkin_ws/src/distance_map/distance_map_core/include/distance_map_core/distance_map_converter_instantiater.h && \
    git clone https://github.com/rail-berkeley/rlkit.git && \
    mv rlkit /home


# copy the package to the workspace and build
COPY . ./src/cbf_tracking

WORKDIR /home/catkin_ws/src/cbf_tracking

RUN mv ./amrl_logging /home/catkin_ws/src && \
    git checkout feature/sac_learning && \
    pip install -e /home/rlkit && \
    pip install -r requirements.txt

WORKDIR /home/catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /home/catkin_ws/devel/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash"]
