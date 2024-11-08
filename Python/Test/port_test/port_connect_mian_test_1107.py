import sys
import os
import threading
import time

import numpy as np
import serial
import datetime
import openpyxl
import webbrowser
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
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

logging.basicConfig(
    filename='error_log.txt',  # 日志文件名
    level=logging.ERROR,       # 设置日志记录级别，ERROR 表示只记录错误及以上级别的日志
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',  # 日志记录格式
    datefmt='%Y-%m-%d %H:%M:%S'  # 日期和时间格式
)



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
        self.worker_sow = None
        self.sowing_paused = False
        self.open_sow_pushButton = None
        self.pause_sow_pushButton = None
        self.close_sow_pushButton = None
        # self.plot_map_pushButton = None
        # self.upload_sow_information_pushButton = None
        # self.calculate_area_pushButton = None
        # self.stop_pushButton = None
        # self.open_gnss_pushButton = None
        # self.close_gnss_pushButton = None
        # self.open_speed_sensor_pushButton = None
        # self.close_speed_sensor_pushButton = None
        # self.open_box_sensor_pushButton = None
        # self.close_box_sensor_pushButton = None
        # self.sow_selected_port = None
        # self.box_sersor_selected_port = None
        # self.speed_sersor_selected_port = None
        # self.gnss_selected_port = None
        self.stacked_widget = stacked_widget
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.ui_process()
        self.setFixedSize(1500, 770)

    def ui_process(self):
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

        self.open_gnss_pushButton.clicked.connect(lambda: self.open_gnss_port_selection())
        self.open_speed_sensor_pushButton.clicked.connect(lambda: self.open_speed_sensor_port_selection())
        self.open_box_sensor_pushButton.clicked.connect(lambda: self.open_box_sensor_port_selection())


        self.sowing_one_number = self.ui.doubleSpinBox
        self.sowing_one_number.editingFinished.connect(self.update_worker_sowing_value)

        self.open_sow_pushButton.clicked.connect(lambda: self.open_sow_port_selection())
        self.pause_sow_pushButton.clicked.connect(self.toggle_pause_sow)
        self.close_sow_pushButton.clicked.connect(self.close_sow_port)

    def open_gnss_port_selection(self):
        # 获取可用的串口列表
        ports = list(serial.tools.list_ports.comports())
        port_descriptions = [port.description for port in ports] if ports else ['无可用串口']

        # 创建一个对话框供用户选择串口
        port, ok = QInputDialog.getItem(self, "选择GNSS串口", "可用的串口列表：", port_descriptions, 0, False)
        if ok and port:
            self.gnss_selected_port = port
            pass
        pass

    def open_speed_sensor_port_selection(self):
        # 获取可用的串口列表
        ports = list(serial.tools.list_ports.comports())
        port_descriptions = [port.description for port in ports] if ports else ['无可用串口']

        # 创建一个对话框供用户选择串口
        port, ok = QInputDialog.getItem(self, "选择速度传感器串口", "可用的串口列表：", port_descriptions, 0, False)
        if ok and port:
            self.speed_sersor_selected_port = port
            pass
        pass

    def open_box_sensor_port_selection(self):
        # 获取可用的串口列表
        ports = list(serial.tools.list_ports.comports())
        port_descriptions = [port.description for port in ports] if ports else ['无可用串口']

        # 创建一个对话框供用户选择串口
        port, ok = QInputDialog.getItem(self, "选择种箱信息传感器串口", "可用的串口列表：", port_descriptions, 0, False)
        if ok and port:
            self.box_sersor_selected_port = port
            pass
        pass

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
                self.worker_sow.finished_signal.connect(self.on_worker_sow_finished)
                self.worker_sow.mistake_message_transmit.connect(self.on_worker_sow_error)
                pass
            pass
        pass

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


    def on_worker_sow_finished(self):
        QMessageBox.information(self, "信息", "播种已完成")
        pass

    def on_worker_sow_error(self, message):
        QMessageBox.critical(self, "错误", f"播种线程错误：{message}")
        pass

    def update_worker_sowing_value(self):
        if self.worker_sow is not None:
            sowing_value = self.sowing_one_number.value()
            self.worker_sow.update_sowing_value(sowing_value)
            pass
        pass


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

    def run(self):
        try:
            lat, lon, alt, speed, rtk_fix_quality = 0, 0, 0, 0.0, 0
            gngga_messages = []
            if self.baudrate is not None and self.port is not None:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                self.ser.write("gnrmc 0.05\r\n".encode())
                self.ser.write("gngga 0.05\r\n".encode())
                while self.should_run:
                    try:
                        current_timestamp = datetime.now()
                        if self.previous_timestamp:
                            self.time_difference = (current_timestamp - self.previous_timestamp).total_seconds()
                        self.previous_timestamp = current_timestamp
                        raw_message_gngga = self.ser.readline().decode('utf-8').rstrip()
                        if raw_message_gngga.startswith("$GNGGA"):
                            gngga_messages.append(raw_message_gngga)
                            fields = raw_message_gngga.split(",")
                            rtk_fix_quality = fields[6]
                            print(rtk_fix_quality)
                            if len(gngga_messages) % 20 == 0:
                                self.gnss_messages_send = "\r\n".join(gngga_messages) + "\r\n"
                                gngga_messages = []
                                self.message_generated.set()
                        raw_message_gnrmc = self.ser.readline().decode('utf-8').rstrip()
                        if raw_message_gnrmc.startswith("$GNRMC"):
                            print(raw_message_gnrmc)
                            if rtk_fix_quality == '4' or rtk_fix_quality == '5':
                                # todo GNSS速度未使用
                                speed = extract_speed_mps(raw_message_gnrmc)

                    except Exception as err_1:
                        self.mistake_message_transmit.emit(str(err_1))
                        continue
        except Exception as err_1:
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            self.finished_signal.emit()
            self.ser.close()
            pass
        pass
    pass

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
                        logging.error(f"Error occurred in SOW worker thread: {str(err_1)}", exc_info=True)
                        self.mistake_message_transmit.emit(str(err_1))
                        continue
        except Exception as err_1:
            logging.error(f"Failed to initialize SOW worker: {str(err_1)}", exc_info=True)
            self.mistake_message_transmit.emit(str(err_1))
        finally:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write("0\r\n".encode())  # 发送停止命令
                except Exception as e:
                    logging.error(f"Error occurred while sending stop command in finally block: {str(e)}",
                                  exc_info=True)
                finally:
                    self.ser.close()
            self.finished_signal.emit()


if __name__ == '__main__':
    try:
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