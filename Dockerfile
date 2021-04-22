ARG UBUNTU_VERSION=20.04
FROM ubuntu:${UBUNTU_VERSION} AS prereqs

RUN apt-get update && \
      DEBIAN_FRONTEND=noninteractive apt-get -y install sudo python3-pip apt-utils build-essential git wget checkinstall \
          protobuf-compiler libprotobuf-dev libgoogle-glog-dev libgflags-dev \
          libeigen3-dev libboost-thread-dev libpcl-dev libproj-dev libatlas-base-dev libsuitesparse-dev \
          libgeotiff-dev libopencv-dev libhdf5-serial-dev libopenmpi-dev openmpi-bin libhdf5-openmpi-dev

RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo

WORKDIR /mulls

# install newest cmake
RUN apt-get install -y apt-transport-https ca-certificates gnupg software-properties-common wget && \
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null && \
    apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" && \
    apt-get update && \
    apt-get install -y cmake cmake-curses-gui

# install newest clang
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    apt-add-repository "deb http://apt.llvm.org/$(lsb_release -cs)/ llvm-toolchain-$(lsb_release -cs) main" && \
    apt-get update && \
    apt-get install -y clang

# install newer Eigen for TEASER++ and others
RUN wget 'http://de.archive.ubuntu.com/ubuntu/pool/universe/e/eigen3/libeigen3-dev_3.3.7-2_all.deb' && \
    dpkg -i libeigen3-dev_3.3.7-2_all.deb && \
    rm *.deb

RUN if [ $(lsb_release -cs) = xenial ]; then \
        python3 -m pip install pip==20.3.4; \
    else \
        python3 -m pip install --upgrade pip; \
    fi

ADD script/tools/install_dep_lib.sh script/tools/install_dep_lib.sh
ADD python/requirements.txt python/requirements.txt

ARG NPROC=""

RUN DEBIAN_FRONTEND=noninteractive NPROC=${NPROC} bash script/tools/install_dep_lib.sh

FROM prereqs

ADD . .

ARG CXX_COMPILER=clang++
RUN rm -rf build && \
    mkdir build && \
    cd build && \
    cmake .. -DBUILD_WITH_SOPHUS=ON -DBUILD_WITH_PROJ4=ON -DBUILD_WITH_LIBLAS=ON -DCMAKE_CXX_COMPILER=${CXX_COMPILER} && \
    make -j${NPROC}

RUN apt-get install -y xvfb
RUN sed -i 's/real_time_viewer_on=1/real_time_viewer_on=0/g' script/run_mulls_slam.sh
ENTRYPOINT ["/usr/bin/xvfb-run", "-a", "-s", "-screen 0 1024x768x24"]
CMD ["script/run_mulls_slam.sh"]

# /usr/bin/xvfb-run -a -s '-screen 0 1024x768x24' script/run_mulls_slam.sh
