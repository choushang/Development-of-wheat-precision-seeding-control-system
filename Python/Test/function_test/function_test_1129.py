# 版本说明：
# 1128：1.GNSS串口通讯 2.播种串口通讯
# 1129：1.速度检测窗口通讯 2.播种种箱检测

import sys
import os
import threading
import time
import numpy as np
import serial
from datetime import datetime
from openpyxl import Workbook
import webbrowser
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
from ui import Ui_Form
from WelcomeScreen import WelcomeScreen
from fun_lib import creat_excel
from fun_lib import wgs84_to_gcj02
from BDS_plot_function import BDS_plot
from BDS_distance_function import BDS_distance
from deleter_fun import del_xlsx
from pynmea_fun import *
import serial.tools.list_ports
import logging
import socket
import base64

logging.basicConfig(
    filename='error_log.txt',  # 日志文件名
    level=logging.ERROR,  # 设置日志记录级别，ERROR 表示只记录错误及以上级别的日志
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',  # 日志记录格式
    datefmt='%Y-%m-%d %H:%M:%S'  # 日期和时间格式
)


def handle_collected_gnss_data(gnss_data, radar_data, acc_x_data, acc_y_data, yaw_data, estimated_Speed_data,
                               time_data):
    wb = Workbook()
    ws = wb.active
    ws.title = "Speed Data"
    ws.append(["Time Stamp", "Radar Data", "Acc X Data", "Acc Y Data", "Yaw", "GNSS Speed", "Estimated Speed"])

    for time_stamp, radar, acc_x, acc_y, yaw, speed, estimated_speed in zip(time_data, radar_data, acc_x_data,
                                                                            acc_y_data, yaw_data, gnss_data,
                                                                            estimated_Speed_data):
        ws.append([time_stamp.strftime("%Y-%m-%d %H:%M:%S"), radar, acc_x, acc_y, yaw, speed, estimated_speed])

    file_name = datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_gnss_speed.xlsx"
    wb.save(file_name)
    print(f"Data saved to {file_name}")
    pass


class WelcomeWindow(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.stacked_widget = stacked_widget
        self.ui = WelcomeScreen()
        self.ui.setupUi(self)
        self.setFixedSize(self.size())
        self.ui.pushButton.clicked.connect(self.switch_window)

    def switch_window(self):
        self.stacked_widget.setCurrentIndex(1)


class InformationShowWindow(QWidget):
    def __init__(self, stacked_widget):
        super().__init__()
        self.width_control = None
        self.seed_box_value_spinbox = None
        self.box_value_two_show = None
        self.box_value_one_show = None
        self.close_box_sensor_pushButton = None
        self.open_box_sensor_pushButton = None
        self.close_speed_sensor_pushButton = None
        self.open_speed_sensor_pushButton = None
        self.worker_seedbox = None
        self.seed_box_second = None
        self.seed_box = None
        self.sowing_one_number = None
        self.worker_radar = None
        self.ntrip_work = None
        self.worker_gnss = None
        self.worker_sow = None
        self.sowing_paused = False
        self.open_sow_pushButton = None
        self.pause_sow_pushButton = None
        self.close_sow_pushButton = None
        self.open_gnss_pushButton = None
        self.close_gnss_pushButton = None
        self.stacked_widget = stacked_widget
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.ui_process()
        self.setFixedSize(1500, 770)

    def ui_process(self):
        # 控件定义
        self.open_sow_pushButton = self.ui.pushButton
        self.pause_sow_pushButton = self.ui.pushButton_2
        self.close_sow_pushButton = self.ui.pushButton_3
        self.plot_map_pushButton = self.ui.pushButton_4
        self.upload_sow_information_pushButton = self.ui.pushButton_5
        self.calculate_area_pushButton = self.ui.pushButton_6
        self.stop_pushButton = self.ui.pushButton_7
        self.open_gnss_pushButton = self.ui.pushButton_8
        self.close_gnss_pushButton = self.ui.pushButton_9
        self.open_speed_sensor_pushButton = self.ui.pushButton_10
        self.close_speed_sensor_pushButton = self.ui.pushButton_11
        self.open_box_sensor_pushButton = self.ui.pushButton_12
        self.close_box_sensor_pushButton = self.ui.pushButton_13
        self.box_value_one_show = self.ui.progressBar
        self.box_value_two_show = self.ui.progressBar_2
        self.seed_box_value_spinbox = self.ui.doubleSpinBox_3
        self.width_control = self.ui.doubleSpinBox_4

        # GNSS模块
        self.open_gnss_pushButton.clicked.connect(lambda: self.open_gnss_port_selection())
        # self.open_gnss_pushButton.clicked.connect(self.ntrip_start)
        self.close_gnss_pushButton.clicked.connect(self.close_gnss_port)
        # self.close_gnss_pushButton.clicked.connect(self.ntrip_stop)

        # 速度检测模块
        self.open_speed_sensor_pushButton.clicked.connect(lambda: self.open_speed_sensor_port_selection())
        self.close_speed_sensor_pushButton.clicked.connect(self.close_speed_sensor_port)

        # 播种模块
        self.sowing_one_number = self.ui.doubleSpinBox
        self.sowing_one_number.editingFinished.connect(self.update_worker_sowing_value)
        self.open_sow_pushButton.clicked.connect(lambda: self.open_sow_port_selection())
        self.pause_sow_pushButton.clicked.connect(self.toggle_pause_sow)
        self.close_sow_pushButton.clicked.connect(self.close_sow_port)

        # 种箱检测模块
        self.seed_box = self.ui.progressBar
        self.seed_box_second = self.ui.progressBar_2
        self.open_box_sensor_pushButton.clicked.connect(lambda: self.open_box_sensor_port_selection())
        self.close_box_sensor_pushButton.clicked.connect(self.close_seedbox_port)
        self.seed_box.setValue(0)
        self.seed_box_second.setValue(0)
        self.seed_box_value_spinbox.valueChanged.connect(self.update_seedbox_calibration_value)

    # GNSS模块
    def open_gnss_port_selection(self):
        # 串口占用检测
        try:
            # 获取可用的串口列表
            ports = list(serial.tools.list_ports.comports())
            port_names = [port.description for port in ports] if ports else ['无可用串口']
            port_description, ok = QInputDialog.getItem(self, "选择GNSS串口", "可用的串口列表：", port_names, 0, False)
            if ok and port_description:
                selected_port = next((port.device for port in ports if port.description == port_description), None)
                # 创建并启动 Worker_gnss 线程
                self.worker_gnss = Worker_gnss()
                self.worker_gnss.set_port(selected_port)
                self.worker_gnss.start()
                # 连接信号
                self.worker_gnss.mistake_message_transmit.connect(self.mistake_message_show)
                # 改变图片
                self.ui.label_19.setPixmap(QPixmap("./icon/滑动开关-开.png"))
        except serial.serialutil.SerialException as error:
            logging.error(f"GNSS串口启动失败: {str(error)}", exc_info=True)
            self.mistake_message_show(error)

    def close_gnss_port(self):
        if self.worker_gnss is not None:
            if self.worker_gnss.isRunning():
                self.worker_gnss.should_run = False
                self.worker_gnss.quit()
                self.worker_gnss.wait()
            self.worker_gnss = None
        self.ui.label_19.setPixmap(QPixmap("./icon/滑动开关-关.png"))

    def ntrip_start(self):
        try:
            self.ntrip_work = Worker_Ntrip(self.worker_gnss)
            self.ntrip_work.mistake_message_transmit.connect(self.mistake_message_show)
            self.ntrip_work.start()
        except serial.serialutil.SerialException as error:
            logging.error({str(error)}, exc_info=True)
            self.mistake_message_show(error)

    def ntrip_stop(self):
        if hasattr(self, 'ntrip_work') and self.ntrip_work is not None:
            self.ntrip_work.should_run = False
            self.ntrip_work.quit()
            self.ntrip_work.wait()
            self.ntrip_work = None

    # 速度检测模块
    def open_speed_sensor_port_selection(self):
        try:
            # 获取可用的串口列表
            ports = list(serial.tools.list_ports.comports())
            port_descriptions = [port.description for port in ports] if ports else ['无可用串口']
            # 创建一个对话框供用户选择串口
            port, ok = QInputDialog.getItem(self, "选择速度传感器串口", "可用的串口列表：", port_descriptions, 0, False)
            if ok and port:
                selected_port = next((port.device for port in ports if port.description == port), None)
                if selected_port:
                    self.worker_radar = Worker_radar()
                    self.worker_radar.set_port(port)
                    self.worker_radar.start()
                    # 连接信号
                    self.worker_radar.mistake_message_transmit.connect(self.mistake_message_show)
                    self.ui.label_16.setPixmap(QPixmap("./icon/滑动开关-开.png"))
        except serial.serialutil.SerialException as error:
            logging.error(f"速度传感器串口启动失败: {str(error)}", exc_info=True)
            self.mistake_message_show(str(error))

    def close_speed_sensor_port(self):
        if self.worker_radar is not None:
            if self.worker_radar.isRunning():
                self.worker_radar.should_run = False
                self.worker_radar.quit()
                self.worker_radar.wait()
            self.worker_radar = None
        self.ui.label_16.setPixmap(QPixmap("./icon/滑动开关-关.png"))

    # 种箱检测模块
    def open_box_sensor_port_selection(self):
        try:
            # 获取可用的串口列表
            ports = list(serial.tools.list_ports.comports())
            port_descriptions = [port.description for port in ports] if ports else ['无可用串口']
            port_description, ok = QInputDialog.getItem(self, "选择种箱信息传感器串口", "可用的串口列表：",
                                                        port_descriptions, 0, False)
            if ok and port_description:
                selected_port = next((port.device for port in ports if port.description == port_description), None)
                print(selected_port)
                # 弹窗询问是否进行种箱标定
                reply = QMessageBox.question(self, '种箱标定', '是否进行种箱标定？', QMessageBox.Yes | QMessageBox.No,
                                             QMessageBox.No)
                if selected_port:
                    print(selected_port)
                    self.worker_seedbox = Worker_seedbox()
                    self.worker_seedbox.set_port(selected_port)
                    if reply == QMessageBox.Yes:
                        self.worker_seedbox.calibrate = True
                    else:
                        self.worker_seedbox.calibrate = False
                    self.worker_seedbox.start()
                    # 连接信号
                    self.worker_seedbox.mistake_message_transmit.connect(self.mistake_message_show)
                    self.ui.label_24.setPixmap(QPixmap("./icon/滑动开关-开.png"))
                    self.worker_seedbox.send_box_value.connect(self.show_box_value)
                    self.worker_seedbox.send_ratio_one.connect(self.show_ratio_one)
                    self.worker_seedbox.send_ratio_two.connect(self.show_ratio_two)
        except serial.serialutil.SerialException as error:
            logging.error(f"种箱传感器串口启动失败: {str(error)}", exc_info=True)
            self.mistake_message_show(str(error))

    def close_seedbox_port(self):
        if self.worker_seedbox is not None:
            if self.worker_seedbox.isRunning():
                self.worker_seedbox.should_run = False
                self.worker_seedbox.quit()
                self.worker_seedbox.wait()
            self.worker_seedbox = None
        self.ui.label_24.setPixmap(QPixmap("./icon/滑动开关-关.png"))

    def update_seedbox_calibration_value(self):
        if self.worker_seedbox is not None:
            self.worker_seedbox.box_value = self.seed_box_value_spinbox.value()

    def show_box_value(self, value):
        self.seed_box_value_spinbox.setValue(value)

    def show_ratio_one(self, ratio):
        self.box_value_one_show.setValue(ratio)

    def show_ratio_two(self, ratio):
        self.box_value_two_show.setValue(ratio)

    def pop_two(self, which):
        if which == 1:
            QMessageBox.information(self, "确认", "种箱1种料不足")
            pass
        if which == 2:
            QMessageBox.information(self, "确认", "种箱2种料不足")
            pass
        pass

    # 播种控制模块
    def open_sow_port_selection(self):
        # 获取可用的串口列表
        ports = list(serial.tools.list_ports.comports())
        port_names = [port.description for port in ports] if ports else ['无可用串口']
        port_description, ok = QInputDialog.getItem(self, "选择播种串口", "可用的串口列表：", port_names, 0, False)

        if ok and port_description:
            selected_port = next((port.device for port in ports if port.description == port_description), None)
            if selected_port:
                # 创建并启动 Worker_sow 线程
                self.worker_sow = Worker_sow()
                self.worker_sow.set_port(selected_port)
                self.worker_sow.start()
                # 连接信号
                self.worker_sow.mistake_message_transmit.connect(self.mistake_message_show)

    def toggle_pause_sow(self):
        """暂停或恢复播种"""
        if self.worker_sow is not None and self.worker_sow.isRunning():
            try:
                if self.sowing_paused:
                    # 恢复播种
                    sowing_value = self.sowing_one_number.value()
                    self.worker_sow.update_sowing_value(sowing_value)  # 恢复之前的转速
                    self.pause_sow_pushButton.setText("暂停播种")
                    self.sowing_paused = False
                else:
                    # 暂停播种，发送转速为0
                    self.worker_sow.update_sowing_value(0.0)
                    self.pause_sow_pushButton.setText("恢复播种")
                    self.sowing_paused = True
            except Exception as e:
                logging.error(f"Error occurred during toggle pause: {str(e)}", exc_info=True)
                QMessageBox.critical(self, "错误", f"暂停/恢复播种时发生错误：{str(e)}")
                pass
            pass
        pass

    def close_sow_port(self):
        if self.worker_sow is not None and self.worker_sow.isRunning():
            try:
                # 停止线程运行
                self.worker_sow.stop()
                self.worker_sow.wait()  # 等待线程完全结束
                self.worker_sow = None
            except Exception as e:
                logging.error(f"Error occurred during closing sow port: {str(e)}", exc_info=True)
                QMessageBox.critical(self, "错误", f"关闭播种串口时发生错误：{str(e)}")

    def update_worker_sowing_value(self):
        if self.worker_sow is not None:
            sowing_value = self.sowing_one_number.value()
            self.worker_sow.update_sowing_value(sowing_value)
            pass
        pass

    # 错误提示
    def mistake_message_show(self, message):
        QMessageBox.information(self, "错误", "错误原因{}".format(message))
        pass

    # 后处理模块
    def open_file(self):
        self.filename = QFileDialog.getOpenFileNames(self, '选择文件', os.getcwd(), "All Files(*)")
        self.path = self.filename[0]

    def plot_map(self):
        BDS_plot(self.path[0])
        webbrowser.open_new_tab('loc_map.html')

    def distance(self):
        n = self.width_control.value()
        m = BDS_distance(self.path[0], n)
        QMessageBox.information(self, "工作结果", "工作距离为{}米, 工作面积为{}平方米".format(m[0], m[1]),
                                QMessageBox.Ok)

    def pop(self):
        QMessageBox.information(self, "确认", "地图生成")


class Worker_gnss(QThread):
    finished_signal = pyqtSignal()
    mistake_message_transmit = pyqtSignal(str)
    message_generated = threading.Event()

    def __init__(self):
        super(Worker_gnss, self).__init__()
        self.ser = None
        self.port = None
        self.baudrate = 115200
        self.should_run = True
        self.gnss_messages_send = None

    def set_port(self, port):
        self.port = port
        pass

    def stop(self):
        """停止线程运行"""
        self.should_run = False
        if self.ser and self.ser.is_open:
            self.ser.close()  # 确保关闭串口

    # def run(self):
    #     try:
    #         lat, lon, alt, speed, rtk_fix_quality = 0, 0, 0, 0.0, 0
    #         gngga_messages = []
    #         if self.baudrate is not None and self.port is not None:
    #             self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
    #             self.ser.write("gnrmc 0.05\r\n".encode())
    #             self.ser.write("gngga 0.05\r\n".encode())
    #             while self.should_run:
    #                 try:
    #                     current_timestamp = datetime.now()
    #                     if self.previous_timestamp:
    #                         self.time_difference = (current_timestamp - self.previous_timestamp).total_seconds()
    #                     self.previous_timestamp = current_timestamp
    #                     raw_message_gngga = self.ser.readline().decode('utf-8').rstrip()
    #                     if raw_message_gngga.startswith("$GNGGA"):
    #                         gngga_messages.append(raw_message_gngga)
    #                         fields = raw_message_gngga.split(",")
    #                         rtk_fix_quality = fields[6]
    #                         print(rtk_fix_quality)
    #                         if len(gngga_messages) % 20 == 0:
    #                             self.gnss_messages_send = "\r\n".join(gngga_messages) + "\r\n"
    #                             gngga_messages = []
    #                             self.message_generated.set()
    #                     raw_message_gnrmc = self.ser.readline().decode('utf-8').rstrip()
    #                     if raw_message_gnrmc.startswith("$GNRMC"):
    #                         print(raw_message_gnrmc)
    #                         if rtk_fix_quality == '4' or rtk_fix_quality == '5':
    #                             # todo GNSS速度未使用
    #                             speed = extract_speed_mps(raw_message_gnrmc)
    #
    #                 except Exception as err_1:
    #                     self.mistake_message_transmit.emit(str(err_1))
    #                     logging.error(str(err_1), exc_info=True)
    #                     continue
    #     except Exception as err_1:
    #         logging.error(f"{str(err_1)}", exc_info=True)
    #         self.mistake_message_transmit.emit(str(err_1))
    #     finally:
    #         if self.ser and self.ser.is_open:
    #             self.ser.close()
    #         self.finished_signal.emit()

    def run(self):
        # 测试通讯
        try:
            if self.baudrate is not None and self.port is not None:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                while self.should_run:
                    try:
                        # 读取 Arduino 发送的 "helloworld" 并打印
                        raw_message = self.ser.readline().decode('utf-8').rstrip()
                        if raw_message == "helloworld":
                            print("Received from Arduino: helloworld")
                    except Exception as err_1:
                        self.mistake_message_transmit.emit(str(err_1))
                        logging.error(str(err_1), exc_info=True)
                        continue
        except Exception as err_1:
            logging.error(f"{str(err_1)}", exc_info=True)
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.finished_signal.emit()


class Worker_Ntrip(QThread):
    finished_signal = pyqtSignal()
    mistake_message_transmit = pyqtSignal(str)

    def __init__(self, gnss_worker):
        super(Worker_Ntrip, self).__init__()
        self.port = "COM7"
        self.baudrate = 115200
        self.should_run = True
        self.gnss_worker = gnss_worker
        self.ser = None
        self.s = None

        # todo 更新账号密码
        self.Ntrip_host = "39.107.207.235"
        self.Ntrip_port = 8002
        self.Ntrip_mountpoint = "AUTO"
        self.Ntrip_user = "qxyjjd001"
        self.Ntrip_password = "c3934ef"

    def stop(self):
        """停止线程运行"""
        self.should_run = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.s:
            self.s.close()

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.reconnect()
            while self.should_run:
                gnss_messages_send = self.gnss_worker.gnss_messages_send
                if gnss_messages_send == None:
                    continue
                if self.RTK_singal_OK == 1:
                    self.s.send(gnss_messages_send.encode())
                    rtcm_data = self.s.recv(1024000)
                    if rtcm_data is not None:
                        self.ser.write(rtcm_data)
                        self.ser.flush()
        except Exception as err_1:
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            if self.s:
                self.s.close()
            self.finished_signal.emit()

    def reconnect(self):
        while True:
            self.s = socket.socket()
            self.s.settimeout(10)
            self.s.connect((self.Ntrip_host, self.Ntrip_port))
            self.ntrip_request = f"GET /{self.Ntrip_mountpoint} HTTP/1.0\r\n"
            self.ntrip_request += "User-Agent: NTRIP PythonClient/1.0\r\n"
            self.ntrip_request += f"Authorization: Basic {base64.b64encode((self.Ntrip_user + ':' + self.Ntrip_password).encode()).decode()}\r\n"
            self.ntrip_request += "\r\n"

            self.s.send(self.ntrip_request.encode())
            response = self.s.recv(1024)

            if b"ICY 200 OK" in response:
                self.RTK_singal_OK = 1
                print('ICY 200 OK')
                break
            else:
                print('链接失败')


class Worker_sow(QThread):
    finished_signal = pyqtSignal()
    mistake_message_transmit = pyqtSignal(str)

    def __init__(self):
        super(Worker_sow, self).__init__()
        self.previous_sowing_value = None
        self.ser = None
        self.port = None
        self.baudrate = 115200
        self.should_run = True
        self.sowing_value = 0.0

    def set_port(self, port):
        self.port = port
        pass

    def stop(self):
        """停止线程运行"""
        self.should_run = False
        if self.ser and self.ser.is_open:
            try:
                # 发送停止命令
                self.ser.write("0\r\n".encode())
            except Exception as e:
                logging.error(f"Error occurred while sending stop command: {str(e)}", exc_info=True)
            finally:
                self.ser.close()  # 关闭串口

    @pyqtSlot(float)
    def update_sowing_value(self, value):
        self.sowing_value = value

    def run(self):
        try:
            if self.baudrate is not None and self.port is not None:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                while self.should_run:
                    try:
                        if self.sowing_value != self.previous_sowing_value:
                            self.previous_sowing_value = self.sowing_value
                            message_to_send = f"{self.sowing_value}\r\n"
                            if self.ser.is_open:  # 检查串口是否打开
                                self.ser.write(message_to_send.encode())
                    except Exception as err_1:
                        logging.error(f"播种模块出错: {str(err_1)}", exc_info=True)
                        self.mistake_message_transmit.emit(str(err_1))
                        continue
        except Exception as err_1:
            logging.error(f"播种模块出错: {str(err_1)}", exc_info=True)
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write("0\r\n".encode())  # 发送停止命令
                except Exception as e:
                    logging.error(f"播种模块出错: {str(e)}", exc_info=True)
                finally:
                    self.ser.close()
            self.finished_signal.emit()


class Worker_radar(QThread):
    finished_signal = pyqtSignal()
    mistake_message_transmit = pyqtSignal(str)

    def __init__(self):
        super(Worker_radar, self).__init__()
        self.ser = None
        self.should_run = True
        self.port = None
        self.baudrate = 115200
        self.serial_connection = None

    def run(self):
        try:
            global radar_speed, acc_x, acc_y, yaw
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=1)
            while self.should_run:
                try:
                    line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("R "):
                        speed = line[1:]
                        speed = float(speed)
                        radar_speed = speed
                except Exception as err_1:
                    logging.error(f"速度传感器有误: {str(err_1)}", exc_info=True)
                    self.mistake_message_transmit.emit(str(err_1))
                    continue
        except Exception as err_1:
            logging.error(f"速度传感器有误: {str(err_1)}", exc_info=True)
            self.mistake_message_transmit.emit(err_1)
        finally:
            self.finished_signal.emit()


class Worker_seedbox(QThread):
    finished_signal = pyqtSignal()
    mistake_message_transmit = pyqtSignal(str)
    send_ratio_one = pyqtSignal(int)
    send_ratio_two = pyqtSignal(int)
    send_box_value = pyqtSignal(float)

    def __init__(self):
        super(Worker_seedbox, self).__init__()
        self.should_run = True
        self.port = None
        self.baudrate = 9600
        self.box_value = None
        self.ser = None
        self.calibrate = False

    def set_port(self, port):
        self.port = port

    def stop(self):
        self.should_run = False
        if self.ser and self.ser.is_open:
            self.ser.close()  # 确保关闭串口

    def run(self):
        try:
            if self.port is not None:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                while self.should_run:
                    if self.calibrate:
                        response = None
                        # 通知单片机进行标定
                        self.ser.write(b'CALIBRATE\n')
                        data = self.ser.readline().decode().strip()
                        if data.startswith('0,0,'):
                            response = data
                        if response:
                            try:
                                Identifier, Type, Data = response.split(',')
                                Identifier = int(Identifier)
                                Type = int(Type)
                                Data = float(Data)
                                if Identifier == 0 and Type == 0:
                                    self.box_value = Data
                                    self.send_box_value.emit(self.box_value)
                                    self.calibrate = False
                            except Exception as err_1:
                                self.mistake_message_transmit.emit(str(err_1))
                                logging.error(f"种箱标定错误: {str(err_1)}", exc_info=True)
                    elif not self.calibrate:
                        data = self.ser.readline().decode().strip()
                        print(data)
                        try:
                            Identifier, Type, Data = data.split(',')
                            Identifier = int(Identifier)
                            Type = int(Type)
                            Data = float(Data)
                            if Identifier == 0:
                                if Type == 1:
                                    ratio = int(((self.box_value - Data) / self.box_value) * 100)
                                    self.send_ratio_one.emit(ratio)
                                elif Type == 2:
                                    ratio = int(((self.box_value - Data) / self.box_value) * 100)
                                    self.send_ratio_two.emit(ratio)
                        except ValueError:
                            logging.error(f"种箱模块数据错误", exc_info=True)
                            continue
                        except Exception as err_1:
                            logging.error(f"种箱模块错误：{str(err_1)}", exc_info=True)
        except Exception as err_1:
            logging.error(f"种箱模块错误：{str(err_1)}", exc_info=True)
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.send_ratio_one.emit(0)
            self.send_ratio_two.emit(0)
            self.finished_signal.emit()


if __name__ == '__main__':
    try:
        # 定义全局变量 radar_speed
        radar_speed = 0.0
        acc_y = 0.0
        acc_x = 0.0
        yaw = 0.0

        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
        app = QApplication(sys.argv)
        stacked_widget = QStackedWidget()

        # 创建欢迎窗口和信息显示窗口
        welcome_window = WelcomeWindow(stacked_widget)
        information_show_window = InformationShowWindow(stacked_widget)

        # 将窗口添加到stacked_widget中
        stacked_widget.addWidget(welcome_window)
        stacked_widget.addWidget(information_show_window)
        stacked_widget.setCurrentIndex(0)

        # 创建主窗口
        main_window = QMainWindow()  # 创建一个主窗口
        main_window.setWindowTitle("果园作业机械研究团队")  # 设置标题
        main_window.setCentralWidget(stacked_widget)  # 将stacked_widget设置为主窗口的中心小部件
        main_window.show()  # 主窗口默认最大化显示

        sys.exit(app.exec_())
    except Exception as err:
        print(err)
