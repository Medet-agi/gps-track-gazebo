#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import csv
import os
from datetime import datetime


class GPSTracker:
    def __init__(self):
        # Инициализация узла
        rospy.init_node('gps_tracker', anonymous=True)

        # Создаем директорию для сохранения данных
        self.data_dir = os.path.expanduser("~/gps_data")
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        # Создаем файл для записи GPS-данных
        self.session_dir = os.path.join(self.data_dir, datetime.now().strftime("%Y%m%d_%H%M%S"))
        if not os.path.exists(self.session_dir):
            os.makedirs(self.session_dir)

        self.gps_file = os.path.join(self.session_dir, "gps_data.csv")

        # Создаем CSV-файл с заголовками
        with open(self.gps_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude', 'status'])

        # Подписываемся на GPS-топик
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)

        rospy.loginfo(f"GPS Tracker запущен. Данные сохраняются в: {self.gps_file}")

    def gps_callback(self, msg):
        # Получаем данные GPS
        timestamp = rospy.get_time()
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        status = msg.status.status

        # Выводим информацию
        rospy.loginfo(f"GPS: Lat: {latitude:.6f}, Lon: {longitude:.6f}, Alt: {altitude:.2f}")

        # Записываем в CSV
        with open(self.gps_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, latitude, longitude, altitude, status])

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        tracker = GPSTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass