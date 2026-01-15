from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.uic import loadUiType
import sys, os
import pyqtgraph.opengl as gl
import numpy as np
import serial.tools
import serial.tools.list_ports
from rotations import Euler2Rotation, Quaternion2Euler
from draw_mav import DrawMav  
import serial, time
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import folium, io
import csv
import datetime
from stl import mesh
import random

MainUI, _ = loadUiType('gcs.ui')

class State:
    def __init__(self, q0, q1, q2, q3, north=0, east=0, altitude=0, temp=0, lat=0, lon=0):
        self.q0 = q0             # must not equal to zero for initialization == 1 for rocket pointing to the sky
        self.q1 = q1        
        self.q2 = q2            
        self.q3 = q3            
        self.north = north        # North position
        self.east = east          # East position
        self.altitude = altitude  # Altitude
        self.temp = temp          # Temp
        self.lat = lat
        self.lon = lon

class Main(QMainWindow, MainUI):
    def __init__(self, parent=None):
        super(Main, self).__init__(parent)
        self.setupUi(self)
        self.ui_changes()
        self.handle_button()
        # Initialize the state (quat, pos, temp, lat, lon) + State class instance initialization in the Main class
        self.state = State(1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0) # Normal quaternion
        self.connected = False
        self.speed = 0.0
        self.sat = 0.0
        self.apogee = 0.0
        date = datetime.datetime.now()
        date = date.strftime('%Y-%m-%d-%H-%M-%S')
        self.log_file = 'Logs/' + f'Flight_Log{date}.csv'
        
    def ui_changes(self):
        self.showMaximized()
        self.setWindowTitle('USTO Rocketry Team Ground Control Station V1.3')
        self.load_and_rotate_image()

    def handle_button(self):
        #self.pushButton.clicked.connect(self.start)
        #self.pushButton_2.clicked.connect(self.stop)
        self.pushButton_3.clicked.connect(self.connect)
        self.pushButton_4.clicked.connect(self.populate_ports)

    def init_3d_visualization(self):
        self.opengl_widget = gl.GLViewWidget()
        self.opengl_widget.setCameraPosition(distance=60, azimuth=45, elevation=35)

        # Get the layout of groupBox_7 and add the OpenGL widget
        layout = QVBoxLayout(self.groupBox_6)
        layout.addWidget(self.opengl_widget)

        # Create and add the rocket to the OpenGL scene in the main
        self.rocket = DrawMav(self.state, self.opengl_widget)

        # Displaying reference axis like 3D modeling softwares
        self.add_reference_axes()


    def update_rocket_orientation(self):
        """ Update the rocket's orientation with the latest serial data. """
        self.state.q0 = self.q0
        self.state.q1 = self.q1 
        self.state.q2 = self.q2 
        self.state.q3 = self.q3 
        
        # extract euler from quaternions then roll, pitch and yaw angles:
        self.euler = Quaternion2Euler(np.array([[self.q0], [self.q1], [self.q2], [self.q3]]))
        phi, theta, psi = self.euler 

        # display and update the euler angles:
        self.label_18.setText(f"roll : {round(np.rad2deg(psi), 2)} °")
        self.label_19.setText(f"Pitch : {round(np.rad2deg(theta), 2)} °")
        self.label_20.setText(f"Yaw : {round(np.rad2deg(phi), 2)} °")
            
        # Update the 3D model with the new orientation
        self.rocket.update(self.state)


    def read_serial_data(self):
        """ Continuously read serial data and update angles from Recieved data from the rocket by the GCS reciever """
        while True:
            try:
                arduinoData = self.ser.readline().decode('ascii').rstrip().split(',')
                if len(arduinoData) == 13:
                    q0, q1, q2, q3, altitude, temp, sat, speed, lat, lon, deploy_drogue, deploy_main, apogee= arduinoData
                    self.lat = float(lat)
                    self.lon = float(lon)
                    self.speed = 0
                    self.sat = 7
                    self.apogee = apogee
                    self.label_12.setText('GPS Status : ON')
                    self.label_13.setText(f'Satellites : {int(float(self.sat))}')
                    self.label_14.setText(f'GPS Speed : {self.speed} Km/h')
                    self.label_15.setText(f"Apogee : {self.apogee} m")
                    if deploy_drogue == '1':
                        self.label_16.setText("Drogue Parachute Deployed !")
                    else :
                        self.label_16.setText("")
                    if deploy_main == '1':
                        self.label_17.setText("Main Parachute Deployed !")
                    else :
                        self.label_17.setText("")
                    self.q0, self.q1, self.q2, self.q3, self.altitude, self.temp = float(q0), float(q1), float(q2), float(q3), round(float(altitude),2), round(float(temp),2)
                    self.datalog()
            except Exception as e:
                print(e)
                self.q0, self.q1, self.q2, self.q3, self.altitude, self.temp, self.lat, self.lon, self.speed = 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 35.66755304896952, -0.6279940516322225, 0.0 #35.66755304896952, -0.6279940516322225

    
    def add_reference_axes(self):
        """ Add a 3D reference axis in the bottom left of the OpenGL view. """
        
        scale_factor = 4  # Adjust this to make the axes smaller or bigger
        
        # Translate the axes to the bottom left corner of the screen
        translation = np.array([12, -5, -5])  
        # X-axis (red)
        x_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [scale_factor, 0, 0]]) + translation, 
                                color=(1, 0, 0, 1), width=7)
        self.opengl_widget.addItem(x_axis)
        
        # Y-axis (green)
        y_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, scale_factor, 0]]) + translation, 
                                color=(0, 1, 0, 1), width=7)
        self.opengl_widget.addItem(y_axis)
        
        # Z-axis (blue)
        z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, scale_factor]]) + translation, 
                                color=(0, 0, 1, 1), width=7)
        self.opengl_widget.addItem(z_axis)

    def load_and_rotate_image(self):
        """Load an image, rotate it 90 degrees, and set it to label_2."""
        # Load the image as a QPixmap
        pixmap = QPixmap(r"C:\Users\VIVOBOOK\Pictures\Screenshots\rocket.png")
        pixmap2 = QPixmap(r"C:\Users\VIVOBOOK\Downloads\logo.png")
        pixmap3 = QPixmap(r"C:\Users\VIVOBOOK\Downloads\logoustormvbg.png")
        
        # Rotate the image by 90 degrees
        transform = QTransform().rotate(90)
        rotated_pixmap = pixmap.transformed(transform)
        
        # Set the rotated pixmap to the label_2
        self.label_2.setPixmap(rotated_pixmap)
        self.label_2.setScaledContents(True)  # fitting the image to the label size
        self.label_3.setPixmap(pixmap2)
        self.label_3.setScaledContents(True)
        self.label_4.setPixmap(pixmap3)
        self.label_4.setScaledContents(True)


    def plot_realtime_data(self):
        # Create a figure and canvas for the plot
        try:
            self.fig, self.ax1 = plt.subplots(figsize=(6, 4), dpi=100, constrained_layout=True)
            self.canvas = FigureCanvas(self.fig)

            # Add the canvas to the groupBox_6 layout in the UI
            self.layout = QVBoxLayout(self.groupBox_5)
            self.layout.addWidget(self.canvas)

            self.ax2 = self.ax1.twinx() # creating a twin ax of ax1 for temp plotting

            self.ax1.set_xlabel('Time (s)')
            self.ax1.set_ylabel('Altitude (m)', color='b')
            self.ax2.set_ylabel('Temperature (°C)', color='r')
        except Exception as e:
            print(e)
        

    def update_plot(self):
        """ Update the altitude plot with new data. """
        try :
            current_time = time.time() - self.start_time

            self.time_data.append(current_time)
            self.altitude_data.append(float(self.altitude))
            self.temp_data.append(float(self.temp))

            self.ax1.clear()
            self.ax2.clear()

            # Plot altitude data on ax1
            self.ax1.plot(self.time_data, self.altitude_data, label="Altitude", color='b', linewidth=2)  # Adjust linewidth
            self.ax1.set_xlabel('Time (s)', fontsize=10)
            self.ax1.set_ylabel('Altitude (m)', color='b', fontsize=10)
            self.ax1.set_title("Altitude & Temperature", fontsize=16)
            self.ax1.tick_params(axis='y', labelcolor='b', width=2)  # Tick thickness
            self.ax1.set_ylim(min(self.altitude_data) - 0.01, max(self.altitude_data) + 0.01)
                
            self.ax1.spines['left'].set_linewidth(2)
            self.ax1.spines['left'].set_color('b')
            self.ax1.spines['bottom'].set_linewidth(1)
            self.ax1.spines['top'].set_linewidth(0)

            # Plot temperature data on ax2 (secondary axis)
            self.ax2.plot(self.time_data, self.temp_data, label="Temperature", color='r', linewidth=2)  # Adjust linewidth
            self.ax2.set_ylabel('Temperature (°C)', color='r', fontsize=10)
            self.ax2.tick_params(axis='y', labelcolor='r', width=2)  # Tick thickness
            
            self.ax2.spines['right'].set_linewidth(2)
            self.ax2.spines['right'].set_color('r')
            self.ax2.spines['top'].set_linewidth(0)

            self.ax2.yaxis.set_label_position('right')
            self.ax2.yaxis.tick_right()

            self.ax1.relim()           
            self.ax1.autoscale_view()  
            self.ax2.relim()           
            self.ax2.autoscale_view()

            self.canvas.draw()
        except Exception as e: 
            print(e)


    """def start(self):
        try:
            self.ser.write(('START\n').encode())
            time.sleep(0.5)
        except Exception as e:
            print(e)"""

    """def stop(self):
        try :
            self.ser.write(('STOP\n').encode())
            time.sleep(0.5)
            self.q0, self.q1, self.q2, self.q3, self.altitude, self.temp, self.lat, self.lon = 1.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0
        except Exception as e:
            print(e)"""

    def populate_ports(self):
        try :
            ports = serial.tools.list_ports.comports()
            self.comboBox.clear()
            self.comboBox.addItem('Select PORT')
            for port, desc, hwid in sorted(ports):
                self.comboBox.addItem(f'{port} : {desc}')
        except Exception as e:
            print(e)

    def connect(self):
        try :
            com = self.comboBox.currentText()
            com = com[:4]
            self.ser = serial.Serial(com, 115200, timeout=0.1)
            self.connected = True
            if self.connected :
                # initializing main state instances to be used by main functions
                self.q0 = 1.0
                self.q1 = 0
                self.q2 = 0
                self.q3 = 0
                self.altitude = 0
                self.temp = 0
                self.lat = 0
                self.lon = 0
                self.speed = 0
                # Initialize list to hold time and altitude data
                self.time_data = []
                self.altitude_data = []
                self.temp_data = []
                self.init_3d_visualization()
                self.plot_realtime_data()
                self.init_map()
                self.start_time = time.time()
                self.label_12.setStyleSheet("color: rgb(0, 255, 0); font: 87 15pt 'Arial Black';")
                self.label_13.setStyleSheet("color: rgb(0, 255, 0); font: 87 15pt 'Arial Black';")
                self.label_14.setStyleSheet("color: rgb(56, 152, 255); font: 87 15pt 'Arial Black';")
    

                # Start thread to continuously read from serial
                self.data_thread = threading.Thread(target=self.read_serial_data)
                self.data_thread.daemon = True
                self.data_thread.start()

                # Timer to update the rocket's orientation
                self.orientation_timer = QTimer()
                self.orientation_timer.timeout.connect(self.update_rocket_orientation)
                self.orientation_timer.start(10)  # Every 25 ms

                # Timer to update altitude plot
                self.plot_timer = QTimer()
                self.plot_timer.timeout.connect(self.update_plot)
                self.plot_timer.start(100)  # Every 500 ms (0.5 seconds)

                # Timer to update GPS location plot
                self.gps_timer = QTimer()
                self.gps_timer.timeout.connect(self.update_location)
                self.gps_timer.start(3000)  # Every 1 s
        except Exception as e:
            print(e)

    def init_map(self):
        """ Initialize the map and embed it in groupBox_4 using a webengine view """
        try:
            self.map_widget = QWebEngineView()

            # Generate an initial map at the default location at IGCMO Senia
            coordinates = (35.66745173062603, -0.6280350157767245) 

            self.map = folium.Map(title='Rocket Location',
                    zoom_start=17,
                    location=coordinates)
            
            self.marker = folium.Marker(location=coordinates)
            self.marker.add_to(self.map)
            
            # Save map data to a data object
            data = io.BytesIO()
            self.map.save(data, close_file=False)
            
            self.map_widget.setHtml(data.getvalue().decode())
            # Add the map to the groupBox_4 layout
            layout = QVBoxLayout(self.groupBox_4)
            layout.addWidget(self.map_widget)
        except Exception as e:
            print(e)


    def update_location(self):
        """ Update the GPS location on the map from gps data """
        try :
            coordinates = (self.lat, self.lon)
            self.map = folium.Map(title='Rocket Location',
                    zoom_start=17,
                    location=coordinates)
            
            self.marker = folium.Marker(location=coordinates)
            self.marker.add_to(self.map)
            
            data = io.BytesIO()
            self.map.save(data, close_file=False)
            
            self.map_widget.setHtml(data.getvalue().decode())
        except Exception as e:
            print(e)


    def datalog(self):
        data = {"Q":self.q0,
                "Qx":self.q1,
                "Qy":self.q2,
                "Qz":self.q3,
                "Altitude":self.altitude,
                "Temperature":self.temp,
                "Latitude":self.lat,
                "Longitude":self.lon}
        
        columns = ['Q','Qx','Qy','Qz','Altitude','Temperature','Latitude','Longitude']
        
        file_exists = os.path.exists(self.log_file)

        with open(self.log_file, "a", newline="") as f:
            writer = csv.DictWriter(f, columns)

            if not file_exists and os.stat(self.log_file).st_size == 0:
                writer.writeheader()

            writer.writerow(data)


def main():
    app = QApplication(sys.argv)
    main_instance = Main()  # Create an instance of the Main class
    main_instance.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
