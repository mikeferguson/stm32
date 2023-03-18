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

    run_state = 0
    neck_angle = 0

    pose_x = list()
    pose_y = list()
    pose_th = list()

    # Table size in meters
    # TODO: read this from firmware
    TABLE_LENGTH = 1.2192
    TABLE_WIDTH = TABLE_LENGTH / 2.0

    laser_data = list()

    # Used to convert meters into pixels
    SCALE = 200.0

    def __init__(self):

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

        self.run_state_value = QtWidgets.QLabel(text="NONE")
        self.run_state_value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.cliff_left_value = QtWidgets.QLabel(text="0")
        self.cliff_left_value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.cliff_center_value = QtWidgets.QLabel(text="0")
        self.cliff_center_value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.cliff_right_value = QtWidgets.QLabel(text="0")
        self.cliff_right_value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.neck_angle_value = QtWidgets.QLabel(text="0")
        self.neck_angle_value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        # TODO: IMU state
        # TODO: motor state

        # State variables in vertical column on right of screen
        self.right_column = QtWidgets.QWidget()
        self.right_column.setFixedWidth(250)
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
        self.status_layout.addWidget(QtWidgets.QLabel(text="System State"), 3, 0)
        self.status_layout.addWidget(self.run_state_value, 3, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="Left Cliff"), 4, 0)
        self.status_layout.addWidget(self.cliff_left_value, 4, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="Center Cliff"), 5, 0)
        self.status_layout.addWidget(self.cliff_center_value, 5, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="Right Cliff"), 6, 0)
        self.status_layout.addWidget(self.cliff_right_value, 6, 1)
        self.status_layout.addWidget(QtWidgets.QLabel(text="Neck Angle"), 7, 0)
        self.status_layout.addWidget(self.neck_angle_value, 7, 1)
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

    def request_updates(self):
        # Request updates
        self.conn.sendto(b"\xffBOT", 0, (self.ip, self.port))

    def handle_packets(self):
        # Read data back
        while True:
            try:
                packet = self.conn.recv(1024)

                # TODO confirm header

                # Remove header
                packet = packet[4:]

                if len(packet) == 76:
                    # System state packet
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

                    self.run_state = struct.unpack_from("<H", packet, 56)[0]
                    self.neck_angle = struct.unpack_from("<H", packet, 58)[0]

                    pose_x = struct.unpack_from("<f", packet, 60)[0]
                    pose_y = struct.unpack_from("<f", packet, 64)[0]
                    pose_th = struct.unpack_from("<f", packet, 68)[0]

                    if len(self.pose_x) == 0 or \
                            abs(pose_x - self.pose_x[-1]) > 0.01 or \
                            abs(pose_y - self.pose_y[-1]) > 0.01 or \
                            abs(pose_th - self.pose_th[-1]) > 0.1:
                        print(pose_x, pose_y, pose_th)
                        self.pose_x.append(pose_x)
                        self.pose_y.append(pose_y)
                        self.pose_th.append(pose_th)

                elif len(packet) == 47:
                    if len(self.pose_x) == 0:
                        # Need a pose before we process laser data
                        continue

                    # Laser packet
                    start_angle = (struct.unpack_from("<H", packet, 4)[0]) * 0.01
                    end_angle = (struct.unpack_from("<H", packet, 42)[0]) * 0.01
                    step = (end_angle - start_angle) / 11.0

                    for i in range(12):
                        angle = -radians(start_angle)

                        range_m = (struct.unpack_from("<H", packet, 6 + (i * 3))[0]) * 0.001
                        x = range_m * cos(angle)
                        y = range_m * sin(angle)

                        # Get global coordinates
                        gx = self.pose_x[-1] + (cos(self.pose_th[-1]) * x - sin(self.pose_th[-1]) * y)
                        gy = self.pose_y[-1] + (sin(self.pose_th[-1]) * x + cos(self.pose_th[-1]) * y)

                        # Add the global point
                        self.laser_data.append([gx, gy])

                        # Increment for next point
                        start_angle += step

                    if len(self.laser_data) > 600:
                        self.laser_data = self.laser_data[-600:]

                else:
                    print("Got packet of length %i" % len(packet))

            except socket.error as err:
                # Not an error to not have packets
                break

    def refresh(self):
        self.time_value.setText("%i" % self.system_time)
        self.voltage_value.setText("%.3f" % self.system_voltage)
        self.current_value.setText("%.3f" % self.system_current)
        self.run_state_value.setText(self.getRunStateValue(self.run_state))
        self.cliff_left_value.setText(self.getCliffValue(self.cliff_left))
        self.cliff_center_value.setText(self.getCliffValue(self.cliff_center))
        self.cliff_right_value.setText(self.getCliffValue(self.cliff_right))
        self.neck_angle_value.setText(str(self.neck_angle))

        # Create a new canvas and painter
        canvas = QtGui.QPixmap(600, 600)
        canvas.fill(QtCore.Qt.gray)
        paint = QtGui.QPainter(canvas)
        paint.setRenderHint(QtGui.QPainter.HighQualityAntialiasing)

        # Draw table
        table_pen = QtGui.QPen()
        table_pen.setWidth(4)
        table_pen.setBrush(QtCore.Qt.darkGray)
        paint.setPen(table_pen)
        paint.setBrush(QtGui.QBrush(QtCore.Qt.darkGray, QtCore.Qt.SolidPattern))
        x = int(self.TABLE_LENGTH * self.SCALE / 2.0)
        y = int(self.TABLE_WIDTH * self.SCALE / 2.0)
        paint.drawRoundedRect(300 - y, 300 - x, 2 * y, 2 * x, 5, 5)

        # If we have pose and laser data
        if len(self.pose_x) > 0:
            # Y is always cented in view (and on table)
            # X needs to shift down a bit to center table in view
            self.offset_x = int(300 + self.TABLE_LENGTH * self.SCALE / 2.0)

            # Draw odometry poses
            odom_pen = QtGui.QPen()
            odom_pen.setWidth(4)
            odom_pen.setBrush(QtCore.Qt.blue)
            paint.setPen(odom_pen)
            for i in range(len(self.pose_x)):
                x = int(self.SCALE * self.pose_x[i])
                y = int(self.SCALE * self.pose_y[i])
                paint.drawPoint(300 - y, self.offset_x - x)

            # Draw robot origin axis
            self.drawAxis(self.pose_x[-1], self.pose_y[-1], self.pose_th[-1], paint)

            # Draw laser data
            data_pen = QtGui.QPen()
            data_pen.setWidth(4)
            data_pen.setBrush(QtCore.Qt.red)
            paint.setPen(data_pen)
            for point in self.laser_data:
                x = int(self.SCALE * point[0])
                y = int(self.SCALE * point[1])
                paint.drawPoint(300 - y, self.offset_x - x)

        paint.end()
        self.map.setPixmap(canvas)
        self.window.update()

    def drawAxis(self, x_m, y_m, th, paint):
        length = self.SCALE * 0.2

        x_pen = QtGui.QPen()
        x_pen.setWidth(2)
        x_pen.setBrush(QtCore.Qt.red)
        paint.setPen(x_pen)

        x = int(self.SCALE * x_m)
        y = int(self.SCALE * y_m)
        xx = x + int(length * cos(th))
        yy = y + int(length * sin(th))
        paint.drawLine(300 - y, self.offset_x - x, 300 - yy, self.offset_x - xx)

        y_pen = QtGui.QPen()
        y_pen.setWidth(2)
        y_pen.setBrush(QtCore.Qt.green)
        paint.setPen(y_pen)

        xx = x + int(-length * sin(th))
        yy = y + int(length * cos(th))
        paint.drawLine(300 - y, self.offset_x - x, 300 - yy, self.offset_x - xx)

    def getCliffValue(self, value):
        if value < 1500:
            # Table is there
            return "OK"
        else:
            return "<b><font color='red'>CLIFF</font></b>"

    def getRunStateValue(self, value):
        if value == 0:
            return "NONE"
        elif value == 1:
            return "Phase 1"
        elif value == 2:
            return "Phase 2"
        elif value == 3:
            return "Phase 3"
        elif value == 255:
            return "DONE"
        else:
            return str(value)


if __name__ == "__main__":

    app = QtWidgets.QApplication([])
    gui = TableBotGUI()

    # Request data at 10hz
    read_board = QtCore.QTimer()
    read_board.timeout.connect(gui.request_updates)
    read_board.start(100)

    # Process data at 100hz
    handle_packets = QtCore.QTimer()
    handle_packets.timeout.connect(gui.handle_packets)
    handle_packets.start(10)

    # Start refresh timer at 10hz
    update_map = QtCore.QTimer()
    update_map.timeout.connect(gui.refresh)
    update_map.start(100)

    app.exec_()
