#!/usr/bin/env python3

from PyQt5 import QtCore, QtGui, QtWidgets
from math import radians, sin, cos
import random
import socket
import struct
import time


class TableBotGUI:

    system_time = 0
    system_voltage = 12.0
    system_current = 1.0
    servo_current = 0.0

    cliff_left = 0
    cliff_center = 1
    cliff_right = 1

    accel_x = 0
    accel_y = 0
    accel_y = 0
    gyro_x = 0
    gyro_y = 0
    gyro_y = 0
    mag_x = 0
    mag_y = 0
    mag_y = 0

    motor1_vel = 0
    motor2_vel = 0
    motor1_pos = 0
    motor2_pos = 0
    motor1_current = 0
    motor2_current = 0

    pose_x = 0
    pose_y = 0
    pose_th = 0

    laser_data = [0 for i in range(450)]  # millimeters
    laser_angle = [0 for i in range(450)]  # 0.01 degree steps

    def __init__(self):

        # TODO: remove this fake data for test
        self.laser_data = [500]
        for i in range(449):
            self.laser_data.append(self.laser_data[-1] + 1)
        self.laser_angle = [0.0]
        for i in range(449):
            self.laser_angle.append(self.laser_angle[-1] + 0.8)

        # Setup comms to robot
        self.ip = "192.168.0.42"
        self.port = 6707
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.conn.bind(("", 0))
        self.conn.setblocking(False)

        # Create variables
        self.state_label = QtWidgets.QLabel(text="<b>System State</b>")
        self.state_label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
        self.state_label.setMargin(10)

        self.time_value = QtWidgets.QLabel(text="0")
        self.time_value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.voltage_value = QtWidgets.QLabel(text="0")
        self.voltage_value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.current_value = QtWidgets.QLabel(text="0")
        self.current_value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.left_cliff = QtWidgets.QLabel(text="0")
        self.left_cliff.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.center_cliff = QtWidgets.QLabel(text="0")
        self.center_cliff.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.right_cliff = QtWidgets.QLabel(text="0")
        self.right_cliff.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        # TODO: IMU state
        # TODO: motor state

        # State variables in vertical column on right of screen
        self.right_column = QtWidgets.QWidget()
        self.right_column.setFixedWidth(200)
        self.right_column_layout = QtWidgets.QVBoxLayout()
        self.right_column.setLayout(self.right_column_layout)
        self.right_column_layout.addWidget(self.state_label)

        # Layout
        self.status_layout = QtWidgets.QGridLayout()
        self.status_layout.addWidget(QtWidgets.QLabel(text="System Time (ms)"), 0, 0)
        self.status_layout.addWidget(self.time_value, 0, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="System Voltage (V)"), 1, 0)
        self.status_layout.addWidget(self.voltage_value, 1, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="System Current (A)"), 2, 0)
        self.status_layout.addWidget(self.current_value, 2, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="Left Cliff"), 3, 0)
        self.status_layout.addWidget(self.left_cliff, 3, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="Center Cliff"), 4, 0)
        self.status_layout.addWidget(self.center_cliff, 4, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="Right Cliff"), 5, 0)
        self.status_layout.addWidget(self.right_cliff, 5, 1)
        self.right_column_layout.addLayout(self.status_layout)
        self.right_column_layout.addStretch()

        self.map = QtWidgets.QLabel()
        canvas = QtGui.QPixmap(600, 600)
        canvas.fill(QtCore.Qt.gray)
        self.map.setPixmap(canvas)

        self.layout = QtWidgets.QGridLayout()
        self.layout.addWidget(self.map, 0, 0, 1, 1)
        self.layout.addWidget(self.right_column, 0, 1, 1, 1)

        self.window = QtWidgets.QWidget()
        self.window.setLayout(self.layout)
        self.window.setWindowTitle("TableBot")
        self.window.show()

        self.color = QtCore.Qt.red

    def update(self):

        self.conn.sendto(b"\xffBOT", 0, (self.ip, self.port))

        # Read data back
        t = time.time()
        while True:
            try:
                packet = self.conn.recv(1024)

                # TODO confirm header
                
                # Remove header
                packet = packet[4:]
                #if len(packet) != 1872:
                #    print("Invalid packet size")
                #    break

                self.system_time = struct.unpack_from("<L", packet, 0)[0]
                self.system_voltage = struct.unpack_from("<f", packet, 4)[0]
                self.system_current = struct.unpack_from("<f", packet, 8)[0]
                self.servo_current = struct.unpack_from("<f", packet, 12)[0]

                self.cliff_left = struct.unpack_from("<H", packet, 16)[0]
                self.cliff_center = struct.unpack_from("<H", packet, 18)[0]
                self.cliff_right = struct.unpack_from("<H", packet, 20)[0]

                self.accel_x = struct.unpack_from("<h", packet, 22)[0]
                self.accel_y = struct.unpack_from("<h", packet, 24)[0]
                self.accel_z = struct.unpack_from("<h", packet, 26)[0]
                self.gyro_x = struct.unpack_from("<h", packet, 28)[0]
                self.gyro_y = struct.unpack_from("<h", packet, 30)[0]
                self.gyro_z = struct.unpack_from("<h", packet, 32)[0]
                self.mag_x = struct.unpack_from("<h", packet, 34)[0]
                self.mag_y = struct.unpack_from("<h", packet, 36)[0]
                self.mag_z = struct.unpack_from("<h", packet, 38)[0]

                self.motor1_vel = struct.unpack_from("<h", packet, 40)[0]
                self.motor2_vel = struct.unpack_from("<h", packet, 42)[0]
                self.motor1_pos = struct.unpack_from("<i", packet, 44)[0]
                self.motor2_pos = struct.unpack_from("<i", packet, 48)[0]
                self.motor1_current = struct.unpack_from("<h", packet, 52)[0]
                self.motor2_current = struct.unpack_from("<h", packet, 54)[0]

                self.state = struct.unpack_from("<H", packet, 56)[0]
                # 58 is unused

                self.pose_x = struct.unpack_from("<f", packet, 60)[0]
                self.pose_y = struct.unpack_from("<f", packet, 64)[0]
                self.pose_th = struct.unpack_from("<f", packet, 68)[0]

                return;

                for i in range(450):
                    self.laser_data[i] = struct.unpack_from("<H", packet, 72 + i * 2)[0]
                    self.laser_angle[i] = float(struct.unpack_from("<H", packet, 972 + i * 2)[0]) * 0.01

            except socket.error as err:
                #print(err)
                if time.time() - t > 10:
                    print("Failed to get return packet!")
                    break

    def refresh(self):

        # Convert millimeters to pixels
        SCALE = 0.2

        pen = QtGui.QPen()
        pen.setWidth(4)
        pen.setBrush(self.color)

        paint = QtGui.QPainter(self.map.pixmap())
        paint.setPen(pen)
        for i in range(450):
            angle = radians(self.laser_angle[i])
            x = int(SCALE * self.laser_data[i] * sin(angle))
            y = int(SCALE * self.laser_data[i] * cos(angle))
            paint.drawPoint(300 + x, 300 - y)
        paint.end()

        self.time_value.setText("%i" % self.system_time)
        self.voltage_value.setText("%.3f" % self.system_voltage)
        self.current_value.setText("%.3f" % self.system_current)
        self.left_cliff.setText(self.getCliffValue(self.cliff_left))
        self.center_cliff.setText(self.getCliffValue(self.cliff_center))
        self.right_cliff.setText(self.getCliffValue(self.cliff_right))

        self.window.update()

    def getCliffValue(self, value):
        if value == 0:
            # Table is there
            return "OK"
        else:
            return "<b><font color='red'>CLIFF</font></b>"


if __name__ == "__main__":

    app = QtWidgets.QApplication([])
    gui = TableBotGUI()

    # Read data at 1hz
    read_board = QtCore.QTimer()
    read_board.timeout.connect(gui.update)
    read_board.start(1000)

    # Start refresh timer at 10hz
    update_map = QtCore.QTimer()
    update_map.timeout.connect(gui.refresh)
    update_map.start(100)

    app.exec_()
