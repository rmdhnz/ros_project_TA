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


network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      routes:
        - to: 0.0.0.0/0
          via: 192.168.1.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4


import rclpy
from rclpy.node import Node
import board
import busio
import adafruit_bno055
from sensor_msgs.msg import Imu

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        # Inisialisasi I2C dan sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.get_logger().info('BNO055 Node Started')

    def publish_imu_data(self):
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Membaca data orientasi dari sensor
            euler = self.sensor.euler
            if euler is not None:
                imu_msg.orientation.x = euler[0]
                imu_msg.orientation.y = euler[1]
                imu_msg.orientation.z = euler[2]

            # Membaca data angular velocity
            gyro = self.sensor.gyro
            if gyro is not None:
                imu_msg.angular_velocity.x = gyro[0]
                imu_msg.angular_velocity.y = gyro[1]
                imu_msg.angular_velocity.z = gyro[2]

            # Membaca data linear acceleration
            accel = self.sensor.linear_acceleration
            if accel is not None:
                imu_msg.linear_acceleration.x = accel[0]
                imu_msg.linear_acceleration.y = accel[1]
                imu_msg.linear_acceleration.z = accel[2]

            # Publish data
            self.publisher.publish(imu_msg)
            self.get_logger().info('IMU Data Published')
        except Exception as e:
            self.get_logger().error(f'Error reading sensor: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

