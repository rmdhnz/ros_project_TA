import board
import busio
import adafruit_bno055
import time
import os

# Inisialisasi I2C dan sensor BNO055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# File untuk menyimpan data kalibrasi
CALIBRATION_FILE = "calibration_data.txt"


def read_calibration_status():
    """Membaca status kalibrasi sensor."""
    sys, gyro, accel, mag = sensor.calibration_status
    print(f"Calibration Status -> Sys: {sys}, Gyro: {gyro}, Accel: {accel}, Mag: {mag}")
    return sys, gyro, accel, mag


def save_calibration():
    """Menyimpan data kalibrasi ke file."""
    calibration_data = sensor.calibration
    with open(CALIBRATION_FILE, "w") as f:
        f.write(str(calibration_data))
    print("Calibration data saved!")


def load_calibration():
    """Memuat data kalibrasi dari file."""
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, "r") as f:
            calibration_data = eval(f.read())
        sensor.calibration = calibration_data
        print("Calibration data loaded!")
    else:
        print("No calibration data file found. Starting fresh calibration.")


def perform_calibration():
    """Melakukan proses kalibrasi manual."""
    print("Starting calibration process...")
    while True:
        sys, gyro, accel, mag = read_calibration_status()
        if sys == 3 and gyro == 3 and accel == 3 and mag == 3:
            print("Calibration complete!")
            save_calibration()
            break
        else:
            print("Move the sensor as needed to calibrate (Refer to documentation).")
        time.sleep(1)


def read_sensor_data():
    """Membaca dan menampilkan data sensor."""
    print("Reading IMU data...")
    while True:
        print(f"Temperature: {sensor.temperature} °C")
        print(f"Accelerometer: {sensor.acceleration} m/s²")
        print(f"Gyroscope: {sensor.gyro} rad/s")
        print(f"Magnetometer: {sensor.magnetic} microtesla")
        print(f"Euler angles: {sensor.euler}")
        print(f"Quaternion: {sensor.quaternion}")
        print(f"Linear acceleration: {sensor.linear_acceleration} m/s²")
        print(f"Gravity: {sensor.gravity} m/s²\n")

        time.sleep(1)  # Delay untuk membaca data lagi


if __name__ == "__main__":
    print("IMU BNO055 Data Reader and Calibration Tool")
    load_calibration()
    read_calibration_status()

    print("\nChoose an option:")
    print("1. Perform Calibration")
    print("2. Read Sensor Data")
    choice = input("Enter your choice (1/2): ").strip()

    if choice == "1":
        perform_calibration()
    elif choice == "2":
        read_sensor_data()
    else:
        print("Invalid choice. Exiting.")
