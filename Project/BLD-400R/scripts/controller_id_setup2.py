#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import math
import os
import sys
import time
import threading
import signal  # signal 모듈 추가
import ntplib  # current time
import ntplib
import glob
import serial
import serial.tools.list_ports


# PyQt5 imports
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QTabWidget, QAction
from PyQt5.QtGui import *
from PyQt5.QtCore import QEvent
from PyQt5.QtCore import QTimer

import numpy as np

ui_file_path = os.path.join(os.path.dirname(__file__), 'controller_id_set.ui')
form_class = uic.loadUiType(ui_file_path)[0]

# SerialReaderThread의 첫 번째 정의만 유지하고 두 번째는 삭제
class SerialReaderThread(threading.Thread):
    def __init__(self, motor_control, exit_event=None):  # exit_event 파라미터 추가
        super().__init__()  # Thread의 __init__ 호출 방식 수정
        self.motor_control = motor_control
        self.running = False
        self.read_buf = bytearray([0] * 57)  # 최대 57바이트 버퍼
        self.exit_event = exit_event if exit_event else threading.Event()

    def run(self):
        print("Serial reader thread started")
        self.running = True
        
        while self.running and not self.exit_event.is_set():
            try:
                if self.motor_control.serial_port and self.motor_control.serial_port.is_open:
                    if self.motor_control.serial_port.in_waiting > 0:
                        # 1바이트씩 읽기
                        data = self.motor_control.serial_port.read()
                        
                        if not data:
                            continue

                        # 데이터 길이 결정
                        if self.motor_control.serial_command_type == 0:
                            data_length = 57
                        else:
                            data_length = 8

                        # 버퍼 시프트 및 새 데이터 추가
                        for i in range(data_length - 1):
                            self.read_buf[i] = self.read_buf[i + 1]
                        self.read_buf[data_length - 1] = data[0]

                        # 응답 패킷 확인 (0x01, 0x06)
                        if self.read_buf[0] == 0x01 and self.read_buf[1] == 0x06:
                            print("Received: ", end='')
                            for i in range(data_length):
                                print(f"0x{self.read_buf[i]:02X} ", end="")
                            print()

                        # 모니터링 데이터 처리 (57바이트)
                        if self.motor_control.serial_command_type == 0:
                            crc = self.motor_control.CRC16_MODBUS(self.read_buf[:55], 55)
                            if (self.read_buf[55] == (crc & 0xFF) and 
                                self.read_buf[56] == ((crc >> 8) & 0xFF)):
                                
                                # 현재 속도 추출
                                current_speed = (self.read_buf[4] << 8) | self.read_buf[5]
                                print(f"Current target speed: {current_speed}")

                                # 엔코더 값 추출
                                encoder_val = ((self.read_buf[49] << 24) | 
                                             (self.read_buf[50] << 16) |
                                             (self.read_buf[47] << 8) | 
                                             self.read_buf[48])
                                print(f"Current encoder value: {encoder_val}")

            except Exception as e:
                print(f"Serial reading error: {e}")
                time.sleep(0.1)

    def stop(self):
        self.running = False
        if self.exit_event:
            self.exit_event.set()
            
class MotorControl:
	def __init__(self):
		self.serial_port = None
		self.serial_command_type = 0
		
	def CRC16_MODBUS(self, data, length):
		wCRCTable = [
			0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780,
			0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1,
			0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801,
			0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40,
			0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680,
			0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0,
			0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501,
			0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
			0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981,
			0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1,
			0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200,
			0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141,
			0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480,
			0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0,
			0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01,
			0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
			0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381,
			0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0,
			0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01,
			0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40,
			0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81,
			0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1,
			0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341, 0X4100,
			0X81C1, 0X8081, 0X4040
		]

		wCRCWord = 0xFFFF
		i = 0
		
		while length > 0:
			nTemp = data[i] ^ (wCRCWord & 0xFF)
			wCRCWord >>= 8
			wCRCWord ^= wCRCTable[nTemp]
			i += 1
			length -= 1
			
		return wCRCWord

	def connect_serial(self, port, baudrate=19200):
		"""시리얼 포트 연결"""
		try:
			self.serial_port = serial.Serial(
				port=port,
				baudrate=baudrate,
				timeout=1
			)
			return True
		except serial.SerialException as e:
			print(f"Serial connection error: {e}")
			return False

	def disconnect_serial(self):
		"""시리얼 포트 연결 해제"""
		if self.serial_port and self.serial_port.is_open:
			self.serial_port.close()

	def modbus_enable(self, motor_id):
		"""모드버스 활성화"""
		if self.serial_port and self.serial_port.is_open:
			
			protocol = bytearray([0] * 8)
			self.serial_command_type = 0
			
			protocol[0] = int(motor_id)
			protocol[1] = 0x06
			protocol[2] = 0x00
			protocol[3] = 0x00
			protocol[4] = 0x00
			protocol[5] = 0x01
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()			
			self.serial_port.write(protocol)
			print("Modbus Enable ")
			return True
		return False
		
	def save_parameters(self):
		"""Save parameter number (내부적으로 이전에 설정된 파라미터 번호 저장)"""
		if self.serial_port and self.serial_port.is_open:
			self.serial_command_type = 14
			protocol = bytearray([0] * 8)
			
			protocol[0] = 0x01  # Device ID
			protocol[1] = 0x06  # Function code (Write)
			protocol[2] = 0x00  # Parameter address high byte
			protocol[3] = 0x14  # Parameter address low byte
			protocol[4] = 0x00  # Data high byte
			protocol[5] = 0x01  # Data low byte
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF        # CRC low byte
			protocol[7] = (crc >> 8) & 0xFF # CRC high byte
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False	
		
	def send_motor_drive_output_enable(self, set):
		if self.serial_port and self.serial_port.is_open:
			self.serial_command_type = 1
			protocol = bytearray([0] * 8)
			self.serial_command_type = 0
			
			protocol[0] = 0x01
			protocol[1] = 0x06
			protocol[2] = 0x00
			protocol[3] = 0x01
			protocol[4] = 0x00	
			
			if set is True:
				protocol[5] = 0x01
			else:	
				protocol[5] = 0x00
				
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			print("Motor Output Enable")	
			self.serial_port.write(protocol)
			return True
		return False
		
		
	def send_motor_speed(self, motor_id, speed):
		"""모터 속도 설정"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 2
			
			protocol[0] = motor_id  # Motor ID 설정
			protocol[1] = 0x06
			protocol[2] = 0x00
			protocol[3] = 0x02
			protocol[4] = (speed >> 8) & 0xFF  # High byte
			protocol[5] = speed & 0xFF         # Low byte
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False

	def monitor_command(self):
		"""모니터 명령 전송"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 0
			
			protocol[0] = 0x01
			protocol[1] = 0x03
			protocol[2] = 0x00
			protocol[3] = 0x00
			protocol[4] = 0x00
			protocol[5] = 0x1A
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
	
	def read_system_voltage(self, motor_id):
		"""시스템 전압 읽기"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 4
			
			protocol[0] = motor_id  # Device ID 사용
			protocol[1] = 0x03
			protocol[2] = 0x00
			protocol[3] = 0x11  # 전압 레지스터 주소
			protocol[4] = 0x00
			protocol[5] = 0x01  # 읽을 레지스터 수
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
		
		
	def change_motor_id(self, current_id, new_id):
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			
			protocol[0] = current_id  # 현재 모터 ID (0x01)
			protocol[1] = 0x06        # Function code
			protocol[2] = 0x00        # Register address high byte
			protocol[3] = 0x15        # Register address low byte
			protocol[4] = 0x00        # Data high byte
			protocol[5] = new_id      # Data low byte (새로운 ID)
			
			# CRC 계산
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			self.serial_port.write(protocol)	
		return False
		
		
	def find_connected_motors(self):
		"""연결된 모터의 ID를 스캔"""
		found_motors = []
		
		if self.serial_port and self.serial_port.is_open:
			# ID 1부터 10까지 스캔 (필요에 따라 범위 조정 가능)
			for motor_id in range(1, 11):
				protocol = bytearray([0] * 8)
				
				protocol[0] = motor_id  # 테스트할 모터 ID
				protocol[1] = 0x03      # Read holding registers
				protocol[2] = 0x00      
				protocol[3] = 0x15      # Device Address register
				protocol[4] = 0x00
				protocol[5] = 0x01      # Read 1 register
				
				# CRC 계산
				crc = self.CRC16_MODBUS(protocol[:6], 6)
				protocol[6] = crc & 0xFF
				protocol[7] = (crc >> 8) & 0xFF
				
				try:
					# 이전 데이터 클리어
					self.serial_port.reset_input_buffer()
					# 명령어 전송
					self.serial_port.write(protocol)
					# 응답 대기
					time.sleep(0.1)
					
					# 응답 읽기
					if self.serial_port.in_waiting >= 5:
						response = self.serial_port.read(self.serial_port.in_waiting)
						if len(response) >= 5 and response[0] == motor_id:
							found_motors.append(motor_id)
							print(f"Found motor with ID: {motor_id}")
				
				except Exception as e:
					print(f"Error scanning ID {motor_id}: {str(e)}")
					
			return found_motors	
	
	def read_motor_speed(self, motor_id):
		"""모터 속도 읽기"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 3
			
			protocol[0] = motor_id  # Motor ID 설정
			protocol[1] = 0x03
			protocol[2] = 0x00
			protocol[3] = 0x10  # 속도 레지스터 주소
			protocol[4] = 0x00
			protocol[5] = 0x01  # 읽을 레지스터 수
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
    	

class WindowClass(QDialog, form_class):
	def __init__(self):
		super(WindowClass, self).__init__()
		self.setupUi(self)
		self.setWindowTitle("Controller ID setup GUI2")
		
		# 종료 이벤트 추가
		self.exit_event = threading.Event()
		
		# MotorControl 인스턴스 생성
		self.motor_control = MotorControl()
        
		# 시리얼 리더 스레드
		self.serial_reader = None
        
		# 시리얼 포트 관련 변수
		self.serial_port = None
		self.is_connected = False
		self.checkBox_USB.setChecked(True)
		
		# 커멘트 관련 변수
		self.motor_command_type = 0
		# UI 초기화
		self.init_ui()		
		
		# Motor ID 관련 변수
		
		self.textEdit_Motor_ID1.setText("1")
		self.textEdit_Motor_ID2.setText("2")
		self.textEdit_Motor_ID3.setText("3")
		
		self.textEdit_Motor_Speed1.setText("100")		
		self.textEdit_Motor_Speed2.setText("100")
		self.textEdit_Motor_Speed3.setText("100")
		# TabWidget의 탭 이름 변경
		self.tabWidget.setTabText(0, "motor control 1")  # 첫 번째 탭의 이름 변경
		self.tabWidget.setTabText(1, "motor control 2")  # 두 번째 탭의 이름 변경
		self.tabWidget.setTabText(2, "motor control 3")  # 세 번째 탭의 이름 변경
		self.tabWidget.setTabText(3, "motor control 4")  # 네 번째 탭의 이름 변경
		
		self.tabWidget.setTabText(4, "motor comm. setting")  # 다섯 번째 탭의 이름 변경
		
		# 시그널 연결
		self.pushButton_Connect.clicked.connect(self.connect_serial)
		self.checkBox_USB.stateChanged.connect(self.update_serial_ports)  # 체크박스 상태 변경 시그널 연결
		
		self.pushButton_Modbus_Enable.clicked.connect(self.modbus_enable_function)
		self.pushButton_Set_Motor_Speed1.clicked.connect(self.set_motor_speed)		
		self.pushButton_Set_Motor_Speed2.clicked.connect(self.set_motor_speed)		
		self.pushButton_Set_Motor_Speed3.clicked.connect(self.set_motor_speed)		
		
		self.pushButton_Set_Motor_Stop1.clicked.connect(self.set_motor_stop)
		self.pushButton_Set_Motor_Stop2.clicked.connect(self.set_motor_stop)
		self.pushButton_Set_Motor_Stop3.clicked.connect(self.set_motor_stop)
		
		self.pushButton_Read_Motor_Speed1.clicked.connect(self.read_motor_speed)
		self.pushButton_Read_Motor_Speed2.clicked.connect(self.read_motor_speed)
		self.pushButton_Save_Parameter.clicked.connect(self.save_parameters)
		self.pushButton_Find_ID.clicked.connect(self.scan_motors)
		 
		 # 탭 변경 시그널 연결
		self.tabWidget.currentChanged.connect(self.tab_changed)		
		self.buttonBox.accepted.connect(self.handle_accept)
		self.buttonBox.rejected.connect(self.handle_reject)
		
	def init_ui(self):
		# 시리얼 포트 검색 및 콤보박스에 추가
		self.update_serial_ports()	
		
	def tab_changed(self, index):
		print(f"Current tab index: {index}")
		# 또는 현재 탭 이름 출력
		current_tab_name = self.tabWidget.tabText(index)
		print(f"Current tab name: {current_tab_name}")
		
	def update_serial_ports(self):
		"""시리얼 포트 목록을 업데이트합니다."""
		self.comboBox_Serial_Port_No.clear()
		
		if self.checkBox_USB.isChecked():  # USB만 검색
			# ttyUSB 장치만 추가 (Linux)
			for usb in glob.glob('/dev/ttyUSB*'):
				self.comboBox_Serial_Port_No.addItem(usb)
		else:  # 모든 포트 검색
			# Linux와 Windows 모두 지원하도록 구현
			ports = list(serial.tools.list_ports.comports())
			
			for port in ports:
				self.comboBox_Serial_Port_No.addItem(port.device)
				
			# ttyUSB 장치 추가 (Linux)
			for usb in glob.glob('/dev/ttyUSB*'):
				if usb not in [port.device for port in ports]:
					self.comboBox_Serial_Port_No.addItem(usb)
					
			# ttyACM 장치 추가 (Linux)
			for acm in glob.glob('/dev/ttyACM*'):
				if acm not in [port.device for port in ports]:
					self.comboBox_Serial_Port_No.addItem(acm)
				
	def connect_serial(self):
		"""시리얼 포트 연결/해제를 처리합니다."""
		if not self.is_connected:
			try:
				port = self.comboBox_Serial_Port_No.currentText()
				if self.motor_control.connect_serial(port):
					self.is_connected = True
					self.pushButton_Connect.setText("Disconnect")
					self.comboBox_Serial_Port_No.setEnabled(False)
					
					# 시리얼 리더 스레드 시작 (exit_event 파라미터 전달)
					self.serial_reader = SerialReaderThread(self.motor_control, self.exit_event)
					self.serial_reader.start()
					
					QMessageBox.information(self, "Connection", f"Successfully connected to {port}")
					
					# 연결 후 모드버스 활성화
					try:
						self.motor_control.modbus_enable(1)
					except Exception as e:
						QMessageBox.warning(self, "Warning", f"Modbus enable failed: {str(e)}")
						
					# 연결 후 드라이버 출력 활성화
					try:    
						self.motor_control.send_motor_drive_output_enable(True)
					except Exception as e:
						QMessageBox.warning(self, "Warning", f"Drive Output enable failed: {str(e)}")        
							
				else:
					QMessageBox.critical(self, "Error", "Failed to connect")
					
			except serial.SerialException as e:
				QMessageBox.critical(self, "Error", f"Serial connection error: {str(e)}")
				self.is_connected = False
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Unexpected error: {str(e)}")
				self.is_connected = False
		else:
			try:
				# 스레드 정지
				if self.serial_reader:
					self.serial_reader.stop()
					self.serial_reader.join()
					self.serial_reader = None
					
				self.motor_control.disconnect_serial()
				self.is_connected = False
				self.pushButton_Connect.setText("Connect")
				self.comboBox_Serial_Port_No.setEnabled(True)
				QMessageBox.information(self, "Disconnection", "Successfully disconnected")
				
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Failed to disconnect: {str(e)}")

	def modbus_enable_function(self):
		try:
			# 현재 선택된 탭의 Motor ID 사용
			current_tab = self.tabWidget.currentIndex()
			if current_tab == 0:
				motor_id = int(self.textEdit_Motor_ID1.toPlainText())
			elif current_tab == 1:
				motor_id = int(self.textEdit_Motor_ID2.toPlainText())
			elif current_tab == 2:
				motor_id = int(self.textEdit_Motor_ID3.toPlainText())
			else:
				motor_id = 1  # 기본값

			self.motor_control.modbus_enable(motor_id)
		except Exception as e:
			QMessageBox.warning(self, "Warning", f"Modbus enable failed: {str(e)}")
		
		
	def set_motor_speed(self):
		"""모터 속도 설정"""
		try:
			# 현재 선택된 탭 인덱스 가져오기
			current_tab = self.tabWidget.currentIndex()
			
			# 탭 인덱스에 따라 적절한 ID와 speed textEdit 선택
			if current_tab == 0:
				motor_id = int(self.textEdit_Motor_ID1.toPlainText())
				speed = int(self.textEdit_Motor_Speed1.toPlainText())
			elif current_tab == 1:
				motor_id = int(self.textEdit_Motor_ID2.toPlainText())
				speed = int(self.textEdit_Motor_Speed2.toPlainText())
			elif current_tab == 2:
				motor_id = int(self.textEdit_Motor_ID3.toPlainText())
				speed = int(self.textEdit_Motor_Speed3.toPlainText())
			else:
				QMessageBox.warning(self, "Warning", "Invalid tab selected")
				return
				
			if self.is_connected:
				if self.motor_control.send_motor_speed(motor_id, speed):
					print(f"Motor speed set to {speed}")
				else:
					QMessageBox.warning(self, "Warning", "Failed to set motor speed")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Motor ID and speed")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Unexpected error: {str(e)}")
	
	def set_motor_stop(self):
		try:
			# Get current tab index
			current_tab = self.tabWidget.currentIndex()
			speed = 0
			
			# Get the appropriate motor ID based on current tab
			if current_tab == 0:
				motor_id = int(self.textEdit_Motor_ID1.toPlainText())
			elif current_tab == 1:
				motor_id = int(self.textEdit_Motor_ID2.toPlainText())
			elif current_tab == 2:
				motor_id = int(self.textEdit_Motor_ID3.toPlainText())
			else:
				QMessageBox.warning(self, "Warning", "Invalid tab selected")
				return
				
			if self.is_connected:
				if self.motor_control.send_motor_speed(motor_id, speed):
					print(f"Motor {motor_id} stopped (speed set to {speed})")
				else:
					QMessageBox.warning(self, "Warning", "Failed to stop motor")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid Motor ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error stopping motor: {str(e)}")	
			
		
		
	def save_parameters(self):
		"""파라미터 저장 버튼 핸들러"""
		try:
			if self.is_connected:
				if self.motor_control.save_parameters():
					QMessageBox.information(self, "Success", "Parameters saved successfully")
				else:
					QMessageBox.warning(self, "Warning", "Failed to save parameters")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error saving parameters: {str(e)}")
			
	def scan_motors(self):
		"""GUI에서 모터 스캔 버튼 핸들러"""
		try:
			if self.is_connected:
				found_motors = self.motor_control.find_connected_motors()  # Changed this line to use motor_control instance
				if found_motors:
					message = "Found motors with IDs: " + ", ".join(map(str, found_motors))
					QMessageBox.information(self, "Scan Result", message)
				else:
					QMessageBox.warning(self, "Scan Result", "No motors found")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error scanning motors: {str(e)}")


	def read_motor_speed(self):
		"""모터 속도 읽기"""
		try:
			if self.is_connected:
				# 현재 선택된 탭 인덱스 가져오기
				current_tab = self.tabWidget.currentIndex()
				
				# 탭 인덱스에 따라 적절한 Motor ID 선택
				if current_tab == 0:
					motor_id = int(self.textEdit_Motor_ID1.toPlainText())
				elif current_tab == 1:
					motor_id = int(self.textEdit_Motor_ID2.toPlainText())
				elif current_tab == 2:
					motor_id = int(self.textEdit_Motor_ID3.toPlainText())
				else:
					QMessageBox.warning(self, "Warning", "Invalid tab selected")
					return
					
				if self.motor_control.read_motor_speed(motor_id):
					print(f"Reading motor speed for Motor ID {motor_id}...")
				else:
					QMessageBox.warning(self, "Warning", "Failed to read motor speed")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid Motor ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error reading motor speed: {str(e)}")
			
			
		
	def cleanup(self):
		"""리소스 정리"""
		try:
			# 종료 이벤트 설정
			self.exit_event.set()
			
			# 시리얼 연결 해제
			if self.is_connected:
				if self.serial_reader:
					self.serial_reader.stop()
					self.serial_reader.join(timeout=1.0)
				self.motor_control.disconnect_serial()
				self.is_connected = False
				
		except Exception as e:
			print(f"Cleanup error: {e}")
            						
	def closeEvent(self, event):
		"""프로그램 종료시 처리"""
		self.cleanup()
		event.accept()
		QApplication.quit()
		os._exit(0)
		
	def handle_accept(self):
		"""OK 버튼 클릭 시 처리"""
		self.cleanup()
		self.accept()
		QApplication.quit()
		os._exit(0)
		
	def handle_reject(self):
		"""Cancel 버튼 클릭 시 처리"""
		self.cleanup()
		self.reject()
		QApplication.quit()
		os._exit(0)
			
def main():
	app = QApplication(sys.argv)

	# 예외 처리 추가
	try:
		myWindow = WindowClass()
		myWindow.show()

		# SIGINT 핸들러 설정
		signal.signal(signal.SIGINT, lambda signum, frame: app.quit())
		
		# 이벤트 루프 시작
		exit_code = app.exec_()
		
		# 프로그램 종료
		sys.exit(exit_code)
		
	except Exception as e:
		print(f"Error in main: {str(e)}")
		sys.exit(1)

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print(f"Error: {str(e)}")
		
		
