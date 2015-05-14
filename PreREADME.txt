***************************************************************************
AUTHORS:
Dhruva Kumar, dhruva.kumar08@gmail.com
Aditya GOurav, adityagoyrav@gmail.com

****************************************************************************

Youtube link for setting up the mount : https://www.youtube.com/watch?v=DccbVNSroy0

Folders:
1. The "server" folder contains all the libraries for Kinect and the push gesture recognition programs
2. The "pi" folder contains the code to be run on Pi

Server Setup ::
1. We refer to the "server" as the system which can run a kinect and perform gesture recognition. After the gesture recognition, it relays the co-ordinates of the gesture to the Pi for proper actuation.
2. The server uses OpenNi drivers to interface with the kinect.
3. The files used for gesture recognition are in the directory : server/OpenNI_NITE_Installer-Linux64-0.27/OpenNI-Bin-Dev-Linux-x64-v1.5.4.0/Samples/NiUserTracker
	(This is our "current" folder for the rest of this readme)
4. The Push gesture recognition parameter can be tweaked in the push() function in SceneDrawer.cpp. The way a gesture is recognissed can be tweaked by changing the threashold velocity of the hand and other parameters. Even the co-ordinate axes (Y and Z, currently) can be changed in case the system setup changes. 
5. Pi's address and the port on which it listens is hardcoded in sendPacket() in SceneDrawer.cpp
6. There are two modes of operation which the system supports currently:
	Mode 1 : The panels of a columns flutter whenever a user comes in front of them. The server continuously sends location data to the Pi in this mode.
	Mode 2 : A wave effect emanates outwards from the location or the panel in front of which the push gesture is performed.
7. Selecting the modes :
	$ ./NiUserTracker 1
	$ ./NiUserTracker 2
8. A new executable can be compiled by the Makefile in the current folder. The location of the executable after the compilation can be set by editing the makefile. For now, the executable is built in ../Bin/x64-Release/ and is named as NiUserTracker

Panel Setup :
1. Each Panel is represented by an Object "Panel". Each panel object can be assigned a center co-ordinate depending upon its relative position from 
	the kinect. This is typically hard-coded in main.cpp::initPanels()
3. The PushPoint panel object has the cordinates of the push gesture.
2. The panels are sorted out according to their distance from the PushPoint object.
4. The panels are categorized into ZONE 1,2. These zones are decided based on the distane from the PushPoint.
	The int array distIndices (size = 2) records the index of the change of zones in the sorted array of Panel objects. So till distIndices[0], all the panels in the sorted array are within zone 1 whereas those after distIndices[0] and before disIndices[1] remain in zone 2. Panels with indices > distIndices[1] lie in zone 2.
	All panels belonging to a zone are actuated simulteneously. The ripple effect is produced when the panels in different zones are actuated by an incrementing delay (cureently linear increment, can be made fancy by including exponential delay).
5. However, if the distance between the PushPoint center and the center of one of the panels is less than RADIUS_PANEL, the panel is not actuated.
6. The direction in which a panel moves is always away from the PushPoint, along the line joining the PushPoint center and the center of the panel. This is the angle between the PushPoint center and the panel center in the kinect's frame of reference.

Communication between server and the Pi :
1. The server communicates with the pi by using the ip address specified in the "sendToPi" characted array.
2. Th Pi listens on a port specified as an argument to the recievePacket() function. It returs int>0 when it recieves something.
3. The Pi communicates with the server by using the ip address specified in the "sendto_ip" char array. This is mainly used for debugging purposes.

Motor Driver:
1. Adafruit's 16-channel 12-bit PWM servo driver was used for this project (http://www.adafruit.com/products/815) These drivers work on I2C.
2. There are two of these to support the 20 motors we are using (2 for each panel). These are addressed as 0x40 and 0x41, depending upon which bits are soldered in the motor driver boards.
3. The two drivers are connected in master-slave configuration.
4. The VCC is the power source for the driver's logic (+5V or +3.3V).
5. The power supply for the motors is connected at the V+ pin.

Interfacing the Pi with the motor drivers:
1. To enable I2C communication on Pi, follow the steps as listed in http://www.instructables.com/id/Raspberry-Pi-I2C-Python/
2. Connect the master motor driver with the Pi by connecting it to Pi's I2C ports. (SDA,SCL,GND and VCC need to be connected)
3. Its possible to get the address of the motor drivers on the I2C bus by excuting the commands listed in the above mentioned site.


Advanced ::
1. There are NITE libraries for extended gesture functionality (server/OpenNI_NITE_Installer-Linux64-0.27/NITE-Bin-Dev-Linux-x64-v1.5.2.21). One can also use these for gesture recognition.
