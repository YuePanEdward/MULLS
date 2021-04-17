FROM ubuntu:16.04

RUN apt-get update && \
      apt-get -y install sudo python3-pip apt-utils build-essential git checkinstall \
          protobuf-compiler libprotobuf-dev libgoogle-glog-dev libgflags-dev \
          libeigen3-dev libboost-thread-dev libpcl-dev libproj-dev libatlas-base-dev libsuitesparse-dev \
          libgeotiff-dev libopencv-dev libhdf5-serial-dev libopenmpi-dev openmpi-bin libhdf5-openmpi-dev && \
      apt-get dist-upgrade -y


RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo

WORKDIR /mulls

# install newest cmake
RUN apt-get install -y apt-transport-https ca-certificates gnupg software-properties-common wget && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
    apt-add-repository 'deb https://apt.kitware.com/ubuntu/ xenial main' && \
    apt-get update && \
    apt-get install -y cmake cmake-curses-gui

# install newer Eigen for TEASER++ and others
RUN wget 'http://de.archive.ubuntu.com/ubuntu/pool/universe/e/eigen3/libeigen3-dev_3.3.7-2_all.deb' && \
    dpkg -i libeigen3-dev_3.3.7-2_all.deb && \
    rm *.deb

# last one supporting Python 3.5
RUN python3 -m pip install pip==20.3.4

ADD script/tools/install_dep_lib.sh script/tools/install_dep_lib.sh
ADD python/requirements.txt python/requirements.txt

RUN DEBIAN_FRONTEND=noninteractive bash script/tools/install_dep_lib.sh

ADD . .

RUN rm -rf build && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j

RUN apt-get install -y xvfb
ENTRYPOINT ["/usr/bin/xvfb-run", "-a", "-s", "-screen 0 1024x768x24"]
CMD ["script/run_mulls_slam.sh"]

# /usr/bin/xvfb-run -a -s '-screen 0 1024x768x24' script/run_mulls_slam.sh
