unamestr=`uname`
if [[ "$unamestr" == 'Linux' ]]; then
  echo "**** Installing necessary packes as listed in https://github.com/ethz-asl/mav_tools/wiki/Install-the-ASL-MAV-framework#manual-installation ****"
  rosversionstr=`rosversion -d`
  echo "** Installing packages for ROS ${rosversionstr} **"
  sudo apt-get install -y protobuf-c-compiler python-wstool python-catkin-tools \
  ros-${rosversionstr}-angles \
  ros-${rosversionstr}-octomap-ros ros-${rosversionstr}-ompl \
  ros-${rosversionstr}-bfl \
  ros-${rosversionstr}-dynamic-edt-3d \
  ros-${rosversionstr}-mavlink \
  ros-${rosversionstr}-geographic-msgs \
  libssh2-1-dev unzip \
  liblog4cplus-dev libv4l-dev cimg-dev python-pip \
  liblapacke-dev protobuf-compiler libsuitesparse-dev \
  libopenblas-dev \
  libgflags-dev libgoogle-glog-dev \
  libgmp-dev libmpfr-dev \
  libnlopt-dev \
  libgtk-3-dev \
  libmgl-dev libx11-dev numactl \
  libgdal-dev 

  if [[ "$rosversionstr" != 'kinetic' ]]; then
    sudo apt-get install -y ros-${rosversionstr}-apriltag
  fi
  if [[ `lsb_release -rs` == "16.04" ]]; then
    sudo apt-get install -y libglfw3-dev libgeographic-dev
  fi
  pip install --user future
elif [[ "$unamestr" == 'Darwin' ]]; then
  echo "This is a Mac, everything still has to be set up by hand."
else
  echo "Platform $unamestr is not supported! Go away!"
  exit -1
fi