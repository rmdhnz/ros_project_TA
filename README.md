sudo apt install -y git cmake python3-pip libboost-all-dev libjpeg-dev libtiff5-dev \
    libpng-dev libglib2.0-dev libgtk-3-dev libcurl4-openssl-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav


git clone https://git.libcamera.org/libcamera/libcamera.git
cd libcamera

mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install

cd ~
git clone https://github.com/raspberrypi/libcamera-apps.git
cd libcamera-apps

mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install

export PATH=$PATH:/usr/local/bin

sudo apt install cmake git libboost-dev libgnutls28-dev libjpeg-dev \
libsdl2-dev python3-yaml python3-pip python3-jinja2 python3-pyqt5 \
qtbase5-dev g++ meson ninja-build

sudo apt install git cmake build-essential python3-pip libboost-all-dev libudev-dev libjpeg-dev libtiff5-dev libpng-dev libyaml-dev -y

network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4


tes
