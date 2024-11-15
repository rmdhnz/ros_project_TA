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


tes
