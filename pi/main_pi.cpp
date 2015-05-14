/*
    AUTHORS:
    Dhruva Kumar, dhruva.kumar08@gmail.com
    Aditya GOurav, adityagoyrav@gmail.com
*/

/* Additions made by Michelle Adjangba*/

#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <strings.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include "cordinates.h"
#include <cmath>
#include <pthread.h>
#include <sstream>
#include "PCA9685.h"
#include "iostream"
#include <time.h>

#define MINPULSE 1.0
#define MAXPULSE 2.0
#define MID MINPULSE + (MAXPULSE - MINPULSE)/2.0
#define FLOWERCLOSE 2.6 // pulse sent to set pwm
#define ROTATEXY 0.4 // the desired distance for rotating left/right/up/down
#define NO_OF_STEPS 10 // No. of steps from start (MINPULSE/MAX) to end (MAXPULSE/MIN) point
#define DELAY_BW_PULSES 0 // Delay in ms between each setServoPulse() call
#define DELAY_BW_PULSES2 0.6 
// [Deprecated] #define INC_BEW_PULSES 0.02 // for loop increment from (min)MINPULSE - (max)MAXPULSE
#define DELAY_BW_MOTIONS 100 // Delay between Shoot & Recoil

#define DELAY_BW_PANELZONES 150 // For mode 2, delay of panel actuation between zones created
#define RADIUS_PANEL 130 // Radius of each panel (Used to ignore the panel actuation for which the push point lies on that panel)

#define NUMPANELS 10
#define NUMCOLS 4

#define DELAY_BW_COLS 100

/*
-----------Summary of all the functions----------------

void init() : Initializes the PWM servo drivers (Prescale. Set I2C freq.)
void setServoPulse(uint8_t n, float pulse) : Maps panel no. to servos and feeds them PWM (ms -> 0-4094)
~void angleDisplay(char * buffer) : Displays the angle received from serial data [Deprectaed/Debugging] 
int quadrant(int angle) : Input: angle (0-360). Returns quadrant it lies in (1.2.3.4)
float gaussian(float x, float mu, float sig, float a)

void xyShoot(void const *args) : Part 1 { panelActuate() } (Moves panel from center to desired point)
void waitBWMotions(void const *args) : Part 2 { panelActuate() } (Waits)
void xyRecoil(void const *args) : Part 3 { panelActuate() } (Moves panel from desired point to center)

void actuatePanel(void const *panel) : Given panel object (Panel). With specified delay (for zones) actuate panel (Shoot.wait.Recoil) 
~void push(char * buffer) : Pushes serial data onto global queue (TEMP)
~void serialCom() : Listener thread which keeps looking for oncoming serial data and enqueues into global data q. 
void actuatePanels(int n, ...) Input: n (no. of arguments), all the panel objects that need to be actuated | Function: For all the panel objects passed, it creates threads and actuates them. [Used for Mode 1]

void mode1(int col, int dir) : Operates Mode 1 (Walk). Input: Column. Direction| Function: Actuates corresponding panels in that column [Dependencies: actuatePanels()]
void mode2(int y, int z): Mode 2 (PUSH): Actuate panel according to (y,z) of push origin

void initPanels(): Initializes the panels with ids, delay, center (y,z) of each panel, This need to be calibrated a/c to the position of the kinect from the board

void main() : 
    - init()
    - initPanels()
    - while(1)
        - Listens for TCP packet from kinect.
        - If packet received, enters into mode1() or mode2()
-------------------------------------------------------
*/

/*
Parameters to tweak: 
- Delay between panel zones (Currently linear. Can be changed.)
- Speed of panel actuation - (Delay b/w pulses | No. of steps)
- Center positions of panels 
*/

// NOTE: Unfortunately, the threads required for this project = atleast the number of panels used. Here we were using 10.
// On testing, it was found that the mbed doesn't run more than 6 threads efficiently (excluding the main thread). On testing with 10, the program crashes.
// Solution: Port to a faster processor! With a good multithreaded support! Pi! C++ pthread!

using namespace std;

PCA9685 pwm1;    // nothing soldered
PCA9685 pwm2;  // A0 soldered
PCA9685 pwm3;

// Global variables
Panel panels[NUMPANELS]; // Array of panel (named Panel) objects
Panel pushPoint(0,0); // Center push point
// Stack to store the commands from serial data
char *stack[10]; int stackPtr = 0; int stackFlag = 0;
int distIndices[5] = {0}; // Indices of start of panel zones. 
int petal = 0;

// for recieving udp packets
char buffer[100];
char *data = &buffer[0];
char sendto_ip[20] = "158.130.62.97";


int recievePacket(int port)
{
    // returns a positive integer if it recieves something, else returns a negative int (-1)
    int recv_sock;
    struct sockaddr_in myAddr;
    recv_sock = socket(AF_INET,SOCK_DGRAM,0);
    myAddr.sin_family = AF_INET;
    myAddr.sin_port = port;
    myAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(recv_sock, (struct sockaddr*)&myAddr, sizeof(myAddr));
    int n = recvfrom(recv_sock, buffer,100, 0, (struct sockaddr*)0, 0);
    close(recv_sock);
    return n; 
}

int sendPacket(char* ip, int port, const char* message)
{
    int sock;
    struct sockaddr_in serverAdd;
    sock = socket(AF_INET,SOCK_DGRAM,0);
    serverAdd.sin_family = AF_INET;
    serverAdd.sin_port = port;
    serverAdd.sin_addr.s_addr = inet_addr(ip);
    int n = sendto(sock, message, 100, 0, (struct sockaddr*)&serverAdd, sizeof(serverAdd)); 
    close(sock);
    return n;
}

struct YZ 
{
    int y;
    int z;
};
///////////////////////////////////
void nanoWait(int ms)
{
    // nanosleep because its higher presicion system call and does not interfere with the signals in the system. A/C to man nanosleep
	//changed from long to double   
    double ns = ms * 1000000;
    timespec sleepValue = {0};
    sleepValue.tv_nsec = ns;
    nanosleep(&sleepValue, NULL);
}

//CHANGE:
// Initializes the PWM servo driver
void init()
{
   pwm1.init(1,0x40);
   pwm2.init(1,0x41);           // address is determined by soldering the bits on the pwm driver
   pwm3.init(1,0x42);
   pwm1.setPWMFreq(100); 
   pwm2.setPWMFreq(100);     //This value is decided for 10ms interval - 100 Hz PWM frequency 
   pwm3.setPWMFreq(100);	
}





// Maps panel no. to servo no. and sets the servo pulse accordingly
// TEMP: Panel numbers from L:R: 2.3.4.0.1.7.8.9.5.6
void setServoPulse(uint8_t n, float pulse) // pulse in ms 
{

    pulse =4095 * pulse / 10; // 10 ms ; period of pwm

	if(petal == 1||petal==2){
		pwm3.setPWM(n,0,pulse);
	}   
	else {
		if (n < 10) { pwm1.setPWM(n, 0, pulse); }
  		else {n = n - 10; pwm2.setPWM(n, 0, pulse); }
	}
	
}

// [Deprecated/debugging] Displays the serial input buffer
int angleDisplay(char * buffer)
{
   int p = 0;
   int angle = 0;
   while(*(buffer+p) != '\r') { angle = (angle * 10) + *(buffer+(p++)) - 48; }  
   stringstream ss;
   ss<<angle;
   sendPacket(sendto_ip, 5000, ss.str().c_str());
   return angle;
}

float gaussian(float x, float mu, float sig, float a){
	return a*(exp(-pow((x-mu),2)/2*pow(sig,2)));
}

// Fully pulling the flower closed
void flowerPull(void const *args)
{
    // Extract id & angle
    int servo = static_cast<const Panel*>(args)->id; 
    int angle = static_cast<const Panel*>(args)->angle;
	 
    // Debugging
    float xStep = static_cast<const Panel*>(args)->xStepShoot;
    float yStep = static_cast<const Panel*>(args)->yStepShoot;


        float count = 0; 
	float diffx;
        float gx;
	int changex = 1;
	float x = -3;
	float negx; //check direction of change
	if (xStep > 0){
		negx = 0;
	}
	else if(xStep < 0){
		negx = 1;
	}
	else{
		changex = 0;

	}
	
	diffx = FLOWERCLOSE; // for closing the flower	
	printf("Servo #%d has diffx = %f, diffy = %f mid is %f\n",servo,diffx,diffy,MID);
	float magnitudeX;
	float magnitudeY;

	while (x <=3){
	if (negx == 0){magnitudeX = diffx;}
	else {magnitudeX = (diffx);} 
        
        if (changex==1)
	gx= gaussian(x,0,.5,magnitudeX)+.1; //pulse to send	
	
	if (changex == 1)
	setServoPulse(servo, gx);
	x+=.008; // amount to change the x from -3 to 3
	}

// make sure all servos are off	 
setServoPulse(servo*2, 0); 
setServoPulse(servo*2 + 1, 0);
setServoPulse(servo, 0); 

}




//  Part 1 of actuatePanel(): Shoot! (Moves panel from center to desired point) horizontal and vertical movement
void xyShoot(void const *args)
{
    // Extract id & angle
    int servo = static_cast<const Panel*>(args)->id; 
    int angle = static_cast<const Panel*>(args)->angle;

//printf("SERVO %d has angle: %d \n", servo, angle);
	 
    // Debugging
    float xStep = static_cast<const Panel*>(args)->xStepShoot;
    float yStep = static_cast<const Panel*>(args)->yStepShoot;
    float xEnd = static_cast<const Panel*>(args)->xEnd;
    float yEnd = static_cast<const Panel*>(args)->yEnd;

    float count = 0; 
//	float x = MID; 
	float y = MID;
	float diffx;
	float diffy;
        float gx;
	float gy;
	int changex = 1;
	int changey = 1;
	float x = -3;
	float negx;
	float negy;
	if (xStep > 0){
		diffx =  xStep*NO_OF_STEPS;
		negx = 0;
	}
	else if(xStep < 0){
		diffx =  -(xStep*NO_OF_STEPS);
		negx = 1;
	}
	else{
	        changex = 0;
	}
	if (yStep > 0){
		diffy = yStep*NO_OF_STEPS;
		negy = 0;
	}
	else if(yStep < 0){
		diffy = -(yStep*NO_OF_STEPS);
		negy = 1;
	}
	else{
		changey = 0;
		printf("mid %f", -(MID));
	}
	


	//printf("Servo #%d has diffx = %f, diffy = %f mid is %f\n",servo,diffx,diffy,MID);
	float magnitudeX;
	float magnitudeY;
	while ( x <=3){
        //swapped signs bc of servo orientation change
	diffx = ROTATEXY;
	diffy = ROTATEXY;	
	float shiftyx; 
	if (negx == 0){magnitudeX = -(diffx); shiftyx = 1.7; }
	else {magnitudeX = diffx; shiftyx = 1.5;}
	float shiftyy;
	if (negy == 0){magnitudeY = -(diffy); }
	else {magnitudeY = (diffy);shiftyy = 2.5;} 



        if (changex==1)
	gx= gaussian(x,0,.5,magnitudeX)+shiftyx; //pulse to send .2,1.5	
	if (changey==1)
	gy= gaussian(x,0,.5,-.4)+2; //pulse to send .2,1.5
	petal = 0;

	if (petal == 0){
	if (changex == 1)
	setServoPulse(servo*2, gx);
        if (changey == 1)
	setServoPulse(servo*2 + 1, gy);
	}
	//printf("petel ummmmm xyshoot %f\n", x);


	x+=.07; //0.06
	}
	 setServoPulse(servo*2, 0); setServoPulse(servo*2 + 1, 0);
    //if (petal == 1) setServoPulse(servo, 0); 

//old
/*

   
    while (count < NO_OF_STEPS)
    {
        setServoPulse(servo*2, x);
        setServoPulse(servo*2 + 1, y);
        x += xStep; y+= yStep;
        count++;
        //CHANGE::
        nanoWait(DELAY_BW_PULSES);
    }*/
}


//  Part 2 of actuatePanel: Wait!
void waitBWMotions(void const *args)
{
    nanoWait(DELAY_BW_MOTIONS);
}

//  Part 3 of actuatePanel: Recoil! (Moves panel from desired point to center)
void xyRecoil(void const *args)
{
    // Extract id & angle
    int servo = static_cast<const Panel*>(args)->id; 
    int angle = static_cast<const Panel*>(args)->angle;

    float xStart = static_cast<const Panel*>(args)->xStart; 
    float yStart = static_cast<const Panel*>(args)->yStart;  
    float xStep = static_cast<const Panel*>(args)->xStepRecoil; 
    float yStep= static_cast<const Panel*>(args)->yStepRecoil; 
    
    float count = 0; float x = xStart; float y = yStart;
    while (count < NO_OF_STEPS)
    {
        setServoPulse(servo*2, x);
        setServoPulse(servo*2 + 1, y);
        x += xStep; y += yStep;
        count++;
        nanoWait(DELAY_BW_PULSES);
    }
    // Bring to rest/stationary position after a short delay
    nanoWait(350);
    setServoPulse(servo*2, 0); setServoPulse(servo*2 + 1, 0);
    stringstream ss;
    ss<<servo<<" xyRecoil Done!";
    sendPacket(sendto_ip, 5000, ss.str().c_str());
}

void flowerClose(void const *args)
{
printf("in flower close\n");
    // Extract id & angle
    int servo = static_cast<const Panel*>(args)->id; 
    int angle = static_cast<const Panel*>(args)->angle;

//printf("SERVO %d has angle: %d \n", servo, angle);
	 
    // Debugging
    float xStep = static_cast<const Panel*>(args)->xStepShoot;
    float yStep = static_cast<const Panel*>(args)->yStepShoot;
//printf("SERVO %d xstep %f\n ", servo,` xStep);
    float count = 0; 
//	float x = MID; 
	float y = MID;
	float diffx;
	float diffy;
        float gx;
	float gy;
	int changex = 1;
	int changey = 1;
	float x = -3;
//printf("xstep %f\n",xStep);	
//printf("change %f\n", (-MID+yStep*NO_OF_STEPS));
	float negx;
	float negy;
	if (xStep > 0){
		diffx =  xStep*NO_OF_STEPS;
		negx = 0;
	}
	else if(xStep < 0){
		diffx =  -(xStep*NO_OF_STEPS);
		negx = 1;
	}
	else{
	//	changex = 0;
		diffx = 2;
	}
	if (yStep > 0){
		diffy = yStep*NO_OF_STEPS;
		negy = 0;
	}
	else if(yStep < 0){
		diffy = -(yStep*NO_OF_STEPS);
		negy = 1;
	}
	else{
		//changey = 0;
		printf("mid %f", -(MID));
	        diffy = 1.5;
	}
	
	printf("Servo #%d has diffx = %f, diffy = %f mid is %f\n",servo,diffx,diffy,MID);
	float magnitudeX;
	float magnitudeY;
	while ( x <=3){
	if (negx == 0){magnitudeX = diffx;}
	else {magnitudeX = -(diffx);}

	if (negy == 0){magnitudeY = diffy;}
	else {magnitudeY = -(diffy);} 
        
        if (changex==1)
	gx= gaussian(x,0,.5,magnitudeX/2)+1.7; //pulse to send .2,1.5	
	if (changey==1)
	gy= gaussian(x,0,.5,magnitudeY/2)+1.6; //pulse to send .2,1.5	
		
	setServoPulse(servo, gy);
	   
	
	//nanoWait(DELAY_BW_PULSES2);
	x+=.07; //0.06
	}
      setServoPulse(servo, 0); 

}


// Input: Panel object (which contains servo number and angle - dereferenced later) | Function: Actuates panel
void* actuatePanel(void *panel)
{
    // CHANGE:
    // Delay before the current panel should actuate.
    int delay = static_cast<Panel*>(panel)->delay; 
    nanoWait(delay);
    //printf("actuate delay = %d\n", delay);
    if (petal == 0)    
    xyShoot(panel);
    else if (petal == 1) flowerClose(panel);
    else flowerPull(panel);
  //   waitBWMotions(panel);
  //  xyRecoil(panel); not using

}

// [Not Used!] Pushes the new command onto stack and updates pointer. Didn't work on it. Currently doing it one at a time.    
void push(char * buffer)
{
    *stack = buffer;
    stackFlag = 1;
}

// [Used for Mode 1] Input: argc (no. of arguments), all the panel objects that need to be actuated | Function: For all the panel objects passed, it creates threads and actuates them.
void* actuatePanels(int n, ...)
{
    va_list args;
    va_start(args, n);
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    // Init all the threads with arguments
    for (int i = 0;  i<n; i++)
    {
        //p[i].init(actuatePanel, va_arg(args, void *)); // Create and initialize threads
        pthread_create(&p[i], NULL, &actuatePanel, va_arg(args, void *));
    }
    // Wait until all threads finish executing
    for (int i = 0; i<n; i++)
    {
        pthread_join(p[i], NULL);
    }
    sendPacket(sendto_ip, 5000, "actuatePanels:: All spawned threads terminated");
}

// Mode 1: Actuate panel according to column & direction
void mode1(int col, int dir) {
    petal = 0;
    dir = (dir == 0) ? 0 : 180;
    setAllAngles(panels, NUMPANELS, dir); // Set angles of all panels as direction
    initPanels(panels); // Initialize all variables w.r.t panel actuation (Call after setting angle!)

    if (col == 1) { actuatePanels(3, (void *)&panels[2], (void *)&panels[3], (void *)&panels[4]); }
    if (col == 2) { actuatePanels(2, (void *)&panels[0], (void *)&panels[1]); }
    if (col == 3) { actuatePanels(3, (void *)&panels[7], (void *)&panels[8], (void *)&panels[9]); }
    if (col == 4) { actuatePanels(2, (void *)&panels[5], (void *)&panels[6]); }
    if (col == 5) { actuatePanels(10, (void *)&panels[0], (void *)&panels[1], (void *)&panels[5], (void *)&panels[6], (void *)&panels[7], (void *)&panels[8], (void *)&panels[9], (void *)&panels[2], (void *)&panels[3], (void *)&panels[4]); }
}
// Mode 10: Actuate panel according to column & direction, flutter
void mode10(int col, int dir) {
    petal = 1;
    dir = (dir == 0) ? 0 : 180;
    setAllAngles(panels, NUMPANELS, dir); // Set angles of all panels as direction
    initPanels(panels); // Initialize all variables w.r.t panel actuation (Call after setting angle!)

    if (col == 1) { actuatePanels(3, (void *)&panels[2], (void *)&panels[3], (void *)&panels[4]); }
    if (col == 2) { actuatePanels(2, (void *)&panels[0], (void *)&panels[1]); }
    if (col == 3) { actuatePanels(3, (void *)&panels[7], (void *)&panels[8], (void *)&panels[9]); }
    if (col == 4) { actuatePanels(2, (void *)&panels[5], (void *)&panels[6]); }
    if (col == 5) { actuatePanels(10, (void *)&panels[0], (void *)&panels[1], (void *)&panels[5], (void *)&panels[6], (void *)&panels[7], (void *)&panels[8], (void *)&panels[9], (void *)&panels[2], (void *)&panels[3], (void *)&panels[4]); }
}



// Mode 2 (PUSH): Actuate panel according to (y,z) of push origin
//void mode2(int y, int z)
void* mode2(void *yz) 
{   //printf("HI MODE 2!!\n");
    petal = 0;
    //cout<<"MODE 2 TESTT \n";
    // struct for points y,z to be passed to MODE2
    struct YZ *hello = (struct YZ*)yz;

    int y = -((hello)->y); int z = -((hello)->z);
    // Set push point
    pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
    setAngles(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
    // Sort panels according to Push point. 
    merge_sort(panels, 0, NUMPANELS-1, pushPoint);
    // Create zones from sorted array. Get zone indices.
    getIndices(panels, distIndices, pushPoint, RADIUS_PANEL);
    
    //CHANGE:
    // Debugging - Sorted array
    int j = 0;
    stringstream ss;
    for (int i = 0; i < NUMPANELS; i++) 
    { 
        std::stringstream ss;
        ss<<panels[i].id<<" | ";
        cout << panels[i].id << "("<< panels[i].angle << "-"<< panels[i].distance(pushPoint)<<") | ";  
        sendPacket(sendto_ip,5000,ss.str().c_str());
        if (i == distIndices[j]-1) 
        {
            cout << " || ";
            ss<<"|| "; 
            sendPacket(sendto_ip,5000,ss.str().c_str());
            j++;
        }
    }
    cout << endl;
    
    int n = 10; int delay = 0; int distIndex = 0;
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    int threadInd = 0;
    for (int i = 0;  i<NUMPANELS; i++)
    {  //delay = DELAY_BW_PANELZONES*exp(i); 
        // If Push origin on panel, ignore the actuation on the ith panel!
      if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
      	//printf("petall %d\n",petal);    
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        
	if (i == distIndices[distIndex]) { delay += DELAY_BW_PANELZONES*exp(distIndex+1) ; distIndex++; }
        panels[i].delay = delay;
        pthread_create(&p[i], NULL, &actuatePanel, (void*)&panels[i]);
    }

    // Wait until all threads finish executing
    for (int i = 0; i<NUMPANELS; i++)
    {
        pthread_join(p[i], NULL);
    }
}




// Mode 2 (PUSH): Actuate panel according to (y,z) of push origin
//void mode2(int y, int z)
void* mode11(void *yz) 
{printf("HI MODE 2!!\n");
    petal = 1;
    //cout<<"MODE 2 TESTT \n";
    // struct for points y,z to be passed to MODE2
    struct YZ *hello = (struct YZ*)yz;

    int y = -((hello)->y); int z = -((hello)->z);
    // Set push point
    pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
    setAngles(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
    // Sort panels according to Push point. 
    merge_sort(panels, 0, NUMPANELS-1, pushPoint);
    // Create zones from sorted array. Get zone indices.
    getIndices(panels, distIndices, pushPoint, RADIUS_PANEL);
    
    //CHANGE:
    // Debugging - Sorted array
    int j = 0;
    stringstream ss;
    for (int i = 0; i < NUMPANELS; i++) 
    { 
        std::stringstream ss;
        ss<<panels[i].id<<" | ";
        cout << panels[i].id << "("<< panels[i].angle << "-"<< panels[i].distance(pushPoint)<<") | ";  
        sendPacket(sendto_ip,5000,ss.str().c_str());
        if (i == distIndices[j]-1) 
        {
            cout << " || ";
            ss<<"|| "; 
            sendPacket(sendto_ip,5000,ss.str().c_str());
            j++;
        }
    }
    cout << endl;
    
    int n = 10; int delay = 0; int distIndex = 0;
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    int threadInd = 0;
    for (int i = 0;  i<NUMPANELS; i++)
    {  //delay = DELAY_BW_PANELZONES*exp(i); 
        // If Push origin on panel, ignore the actuation on the ith panel!
      if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
    printf("petall %d\n",petal);    
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        if (i == distIndices[distIndex]) { delay += DELAY_BW_PANELZONES*exp(distIndex+1) ; distIndex++; }
        panels[i].delay = delay;
	printf("pan %d has delay %d\n",i,delay);
	//cout <<"actuatttee \n";
        pthread_create(&p[i], NULL, &actuatePanel, (void*)&panels[i]);
    }
//actuatePanel((void*)&panels[7]);

    // Wait until all threads finish executing
    for (int i = 0; i<NUMPANELS; i++)
    {
        pthread_join(p[i], NULL);
    }
}


//petalrandom flutter
void* mode3(void *yz){
  	petal = 1;
	//pthread_t p[1];
	//pthread_create(&p[1], NULL, &actuatePanel, (void*)&panels[1]);
    struct YZ *hello = (struct YZ*)yz;

    int y = -((hello)->y); int z = -((hello)->z);
    // Set push point
    pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
    setAnglesDown(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
  int n = 10; int delay = 0; int distIndex = 0;
   
//	pthread_t p[1];
//	pthread_create(&p[1], NULL, &actuatePanel, (void*)&);
//	pthread_join(p[1],NULL);
    int servo = pushPoint.id; 
printf("serv %d\n", servo);
int random = rand() %10;
    for (int i = 0;  i<NUMPANELS; i++)
    {  delay = DELAY_BW_PANELZONES*exp(i); 
        // If Push origin on panel, ignore the actuation on the ith panel!
     // if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
        
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        //if (i == distIndices[distIndex]) { delay = DELAY_BW_PANELZONES*exp(distIndex+1) ; distIndex++; }
        panels[i].delay = delay;
	printf("pan %d has delay %d\n",i,delay);
	//cout <<"actuatttee \n";
       random = rand() %10;
	 actuatePanel( (void*)&panels[random]);
	 actuatePanel( (void*)&panels[random]);
 	 actuatePanel( (void*)&panels[random]);
 

    }

/*
 
	actuatePanel((void*)&panels);
		actuatePanel((void*)&panels[0]);
		actuatePanel((void*)&panels[0]);
		actuatePanel((void*)&panels[0]);

*/

}
//petal close
void* mode4(void *yz){
  	petal = 2;
	//pthread_t p[1];
	//pthread_create(&p[1], NULL, &actuatePanel, (void*)&panels[1]);
    struct YZ *hello = (struct YZ*)yz;

    int y = -((hello)->y); int z = -((hello)->z);
    // Set push point
    pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
    setAngles(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
  int i=0; 
 while( i<NUMPANELS)
 { printf ("pan %d dist from push %f\n",i, panels[i].distance(pushPoint)); 
	if (panels[i].distance(pushPoint) < RADIUS_PANEL) {printf("%d\n",i); break; }
       i++; 
}
printf("panell %d\n", i);
 int n = 10; int delay = 0; int distIndex = 0;
   
	pthread_t p[1];
	pthread_create(&p[1], NULL, &actuatePanel, (void*)&panels[i]);
	pthread_join(p[1],NULL);
    int servo = pushPoint.id; 
 
//	actuatePanel((void*)&panels[0]);
//setServoPulse(servo, 0); 
}


void *mode5(void *yz)
{

printf("HI MODE 5!!\n");
    petal = 0;
    //cout<<"MODE 2 TESTT \n";
    // struct for points y,z to be passed to MODE2
    struct YZ *hello = (struct YZ*)yz;

    int y = -((hello)->y); int z = -((hello)->z);
    // Set push point
    pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
    setAngles(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
    // Sort panels according to Push point. 
    merge_sort(panels, 0, NUMPANELS-1, pushPoint);
    // Create zones from sorted array. Get zone indices.
    getIndices(panels, distIndices, pushPoint, RADIUS_PANEL);
    
    //CHANGE:
    // Debugging - Sorted array
    int j = 0;
    stringstream ss;
    for (int i = 0; i < NUMPANELS; i++) 
    { 
        std::stringstream ss;
        ss<<panels[i].id<<" | ";
        cout << panels[i].id << "("<< panels[i].angle << "-"<< panels[i].distance(pushPoint)<<") | ";  
        sendPacket(sendto_ip,5000,ss.str().c_str());
        if (i == distIndices[j]-1) 
        {
            cout << " || ";
            ss<<"|| "; 
            sendPacket(sendto_ip,5000,ss.str().c_str());
            j++;
        }
    }
    cout << endl;
    
    int n = 10; int delay = 0; int distIndex = 0;
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    int threadInd = 0;
    for (int i = 0;  i<NUMPANELS; i++)
    {  //delay = DELAY_BW_PANELZONES*exp(i); 
        // If Push origin on panel, ignore the actuation on the ith panel!
     if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
    printf("dist ind %d\n",distIndex);    
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        if (i == distIndices[distIndex]) { delay = 10000 - exp((distIndex)) ; printf("dist!!!%d\n", distIndex); distIndex++; }
        panels[i].delay = delay;
	printf("pan %d has delay %d\n",i,delay);
	//cout <<"actuatttee \n";
        pthread_create(&p[i], NULL, &actuatePanel, (void*)&panels[i]);
    }
//actuatePanel((void*)&panels[7]);

    // Wait until all threads finish executing
    for (int i = 0; i<NUMPANELS; i++)
    {
        pthread_join(p[i], NULL);
    }
}


// Mode 2 (PUSH): Actuate panel according to (y,z) of push origin
//void mode2(int y, int z)
//vertical
void* mode6(void *yz) 
{
    petal = 1;
    //cout<<"MODE 2 TESTT \n";
    // struct for points y,z to be passed to MODE2
    struct YZ *hello = (struct YZ*)yz;

    int y = -((hello)->y); int z = -((hello)->z);
    // Set push point
   // pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
   setAnglesUp(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
    // Sort panels according to Push point. 
    //merge_sort(panels, 0, NUMPANELS-1, pushPoint);
    // Create zones from sorted array. Get zone indices.
   // getIndices(panels, distIndices, pushPoint, RADIUS_PANEL);
    
    //CHANGE:
    // Debugging - Sorted array
    int j = 0;
    stringstream ss;
    for (int i = 0; i < NUMPANELS; i++) 
    { 
        std::stringstream ss;
        ss<<panels[i].id<<" | ";
    //    cout << panels[i].id << "("<< panels[i].angle << "-"<< panels[i].distance(pushPoint)<<") | ";  
        sendPacket(sendto_ip,5000,ss.str().c_str());
        if (i == distIndices[j]-1) 
        {
            cout << " || ";
            ss<<"|| "; 
            sendPacket(sendto_ip,5000,ss.str().c_str());
            j++;
        }
    }
    cout << endl;
    
    int n = 10; int delay = 0; int distIndex = 0;
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    int threadInd = 0;
    for (int i = 0;  i<NUMPANELS; i++)
    {  delay = DELAY_BW_PANELZONES; 
        // If Push origin on panel, ignore the actuation on the ith panel!
     // if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
        
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        delay = DELAY_BW_PANELZONES*exp(i) ;
        panels[i].delay = delay;
	printf("pan %d has delay %d\n",i,delay);
	//cout <<"actuatttee \n";
//    actuatePanel((void*)&panels[i]); 
 //actuatePanel((void*)&panels[i]);   
//	actuatePanel((void*)&panels[i]);   
	 
	pthread_create(&p[i], NULL, &actuatePanel, (void*)&panels[i]);
    }


    // Wait until all threads finish executing
    for (int i = 0; i<NUMPANELS; i++)
    {
        pthread_join(p[i], NULL);
    }
}

// Mode 2 (PUSH): Actuate panel according to (y,z) of push origin
//void mode2(int y, int z)
//random
void* mode7(void *yz) 
{
    petal = 0;
    //cout<<"MODE 2 TESTT \n";
    // struct for points y,z to be passed to MODE2
    struct YZ *hello = (struct YZ*)yz;

    int y = +((hello)->y); int z = +((hello)->z);
    // Set push point
   // pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
    setAngles(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
    // Sort panels according to Push point. 
    //merge_sort(panels, 0, NUMPANELS-1, pushPoint);
    // Create zones from sorted array. Get zone indices.
   // getIndices(panels, distIndices, pushPoint, RADIUS_PANEL);
    
    //CHANGE:
    // Debugging - Sorted array
    int j = 0;
    stringstream ss;
    for (int i = 0; i < NUMPANELS; i++) 
    { 
        std::stringstream ss;
        ss<<panels[i].id<<" | ";
    //    cout << panels[i].id << "("<< panels[i].angle << "-"<< panels[i].distance(pushPoint)<<") | ";  
        sendPacket(sendto_ip,5000,ss.str().c_str());
        if (i == distIndices[j]-1) 
        {
            cout << " || ";
            ss<<"|| "; 
            sendPacket(sendto_ip,5000,ss.str().c_str());
            j++;
        }
    }
    cout << endl;
    
    int n = 10; int delay = 0; int distIndex = 0;
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    int threadInd = 0;
int random = rand() %10;
    for (int i = 0;  i<NUMPANELS; i++)
    {  delay = DELAY_BW_PANELZONES*exp(i); 
        // If Push origin on panel, ignore the actuation on the ith panel!
     // if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
        
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        //if (i == distIndices[distIndex]) { delay = DELAY_BW_PANELZONES*exp(distIndex+1) ; distIndex++; }
        panels[i].delay = delay;
	printf("pan %d has delay %d\n",i,delay);
	//cout <<"actuatttee \n";
       random = rand() %10;
	 actuatePanel( (void*)&panels[random]);
    }


    // Wait until all threads finish executing
    for (int i = 0; i<NUMPANELS; i++)
    {
        pthread_join(p[i], NULL);
    }
}



// Mode 2 (PUSH): Actuate panel according to (y,z) of push origin
//void mode2(int y, int z)
//random up
void* mode8(void *yz) 
{
    petal = 1;
    //cout<<"MODE 2 TESTT \n";
    // struct for points y,z to be passed to MODE2
    struct YZ *hello = (struct YZ*)yz;

    int y = +((hello)->y); int z = +((hello)->z);
    // Set push point
   // pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
    setAnglesDown(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
    // Sort panels according to Push point. 
    //merge_sort(panels, 0, NUMPANELS-1, pushPoint);
    // Create zones from sorted array. Get zone indices.
   // getIndices(panels, distIndices, pushPoint, RADIUS_PANEL);
    /*
    //CHANGE:
    // Debugging - Sorted array
    int j = 0;
    stringstream ss;
    for (int i = 0; i < NUMPANELS; i++) 
    { 
        std::stringstream ss;
        ss<<panels[i].id<<" | ";
    //    cout << panels[i].id << "("<< panels[i].angle << "-"<< panels[i].distance(pushPoint)<<") | ";  
        sendPacket(sendto_ip,5000,ss.str().c_str());
        if (i == distIndices[j]-1) 
        {
            cout << " || ";
            ss<<"|| "; 
            sendPacket(sendto_ip,5000,ss.str().c_str());
            j++;
        }
    }
    cout << endl;
    */
    int n = 10; int delay = 0; int distIndex = 0;
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    int threadInd = 0;
int random = rand() %10;
    for (int i = 0;  i<NUMPANELS; i++)
    {  delay = DELAY_BW_PANELZONES*exp(i); 
        // If Push origin on panel, ignore the actuation on the ith panel!
     // if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
        
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        //if (i == distIndices[distIndex]) { delay = DELAY_BW_PANELZONES*exp(distIndex+1) ; distIndex++; }
        panels[i].delay = delay;
	printf("pan %d has delay %d\n",i,delay);
	//cout <<"actuatttee \n";
       random = rand() %10;
	 actuatePanel( (void*)&panels[random]);
    }


    // Wait until all threads finish executing
    for (int i = 0; i<NUMPANELS; i++)
    {
        pthread_join(p[i], NULL);
    }
}

// Mode 9 (PUSH): Actuate panel according to (y,z) of push origin
//void mode2(int y, int z)
//random up
void* mode9(void *yz) 
{
    petal = 0;
    //cout<<"MODE 2 TESTT \n";
    // struct for points y,z to be passed to MODE2
    struct YZ *hello = (struct YZ*)yz;

    int y = +((hello)->y); int z = +((hello)->z);
    // Set push point
   // pushPoint.setY(y); pushPoint.setZ(z);
    // Set the angles of all panels radiating away from the push point
    setAnglesUp(panels, pushPoint, NUMPANELS);
    // Initialize all variables w.r.t panel actuation (Call after setting angle!)
    initPanels(panels);
    // Sort panels according to Push point. 
  // merge_sort(panels, 0, NUMPANELS-1, pushPoint);
    // Create zones from sorted array. Get zone indices.
   // getIndices(panels, distIndices, pushPoint, RADIUS_PANEL);
    
    //CHANGE:
    // Debugging - Sorted array
    int j = 0;
    stringstream ss;
    for (int i = 0; i < NUMPANELS; i++) 
    { 
        std::stringstream ss;
        ss<<panels[i].id<<" | ";
    //    cout << panels[i].id << "("<< panels[i].angle << "-"<< panels[i].distance(pushPoint)<<") | ";  
        sendPacket(sendto_ip,5000,ss.str().c_str());
        if (i == distIndices[j]-1) 
        {
            cout << " || ";
            ss<<"|| "; 
            sendPacket(sendto_ip,5000,ss.str().c_str());
            j++;
        }
    }
    cout << endl;
    
    int n = 10; int delay = 0; int distIndex = 0;
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    int threadInd = 0;
int random = rand() %10;
    for (int i = 0;  i<NUMPANELS; i++)
    {  delay += DELAY_BW_PANELZONES; 
        // If Push origin on panel, ignore the actuation on the ith panel!
     // if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
        
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        //if (i == distIndices[distIndex]) { delay = DELAY_BW_PANELZONES*exp(distIndex+1) ; distIndex++; }
        panels[i].delay = delay;
	printf("pan %d has delay %d\n",i,delay);
	//cout <<"actuatttee \n";
     //  random = rand() %10;
//	 actuatePanel( (void*)&panels[i]);
 pthread_create(&p[i], NULL, &actuatePanel, (void*)&panels[i]);
   
    }


    // Wait until all threads finish executing
    for (int i = 0; i<NUMPANELS; i++)
    {
        pthread_join(p[i], NULL);
    }
}




// Initializes the panels with ids, delay, Center (y,z) of each panel
void initPanels()
{
    for (int i = 0; i<NUMPANELS; i++) 
    { 
        panels[i].id = i; 
        panels[i].delay = 0;
    }
    
     // Hard code: Set y & z center point of each panel
    // Calibrate this when the position of the kinect is changed wrt to the board.
     panels[0].setY(-240); panels[0].setZ(-2250);
     panels[1].setY(-560); panels[1].setZ(-2250);
     panels[5].setY(-240); panels[5].setZ(-1850);
     panels[6].setY(-560); panels[6].setZ(-1850);
     panels[7].setY(-150); panels[7].setZ(-2050);
     panels[8].setY(-400); panels[8].setZ(-2050);
     panels[9].setY(-650); panels[9].setZ(-2050);
     // Don't use currently
     panels[2].setY(-150); panels[2].setZ(-2480);
     panels[3].setY(-400); panels[3].setZ(-2480);
     panels[4].setY(-650); panels[4].setZ(-2480);
}

int main() 
{
    init(); // Initializes the PWM servo drivers
    initPanels();  // Initializes the panels with ids & Set the (y,z) center point.
    
    // Porting serial comm thread to main thread:
    int i = 0;

    sendPacket(sendto_ip,5000,"Entered in main");
    YZ yz;

    while(1)
    {
	//printf("in\n");
     if (recievePacket(5000)>0 && stackFlag == 0)
        {
            stackFlag=1;
        }
     	
        // Once the packet is received, enter into Mode 1/Mode 2.
	// CODE HERE DELETED, SSH FRESH CODE ON
     if(stackFlag == 1) // when there's a packet
     {
	if(*data == '1') //mode 1
	{
		int col = *(data+1) - '0';
		int dir = *(data+2) - '0';
		initPanels();
		mode10(col,dir);
	}
     if (*data == '2') // Mode 2!
            {
                pthread_t call_mode2;
                int i = 0; int y = 0; int z = 0;
                while ((++i) <= 4) { y = y*10 + *(data+i) - '0'; } // Form int y coordinate from characters
                i--;
                while ((++i) <= 8) { z = z*10 + *(data+i) - '0'; } // Form int z coordinate from characters
                printf("Mode 2 | y: %d | z: %d\n", y, z);
                stringstream ss ;
                ss<<"Mode 2 | y:"<<y<<" | z: "<<z;
                sendPacket(sendto_ip, 5000, ss.str().c_str());
                yz.y = y; yz.z = z;
                pthread_create(&call_mode2, NULL, &mode2, (void*)&yz);
                pthread_join(call_mode2, NULL);
                sendPacket(sendto_ip,5000,"DONE");
            }
         
	}	
	stackFlag = 0;
	}



}

// Was used for testing 		
/*
//	int k;
//	for (k = 0; k < 30; k++) 
while(true){
 initPanels();  // Initializes the panels with ids & Set the (y,z) center point.
    
	int random = rand() % 10;


	printf("randd %d\n", random);
	
	printf("panel y %f, panel z %f\n", panels[random].y, panels[random].z);
        // hold for kinect reading
	// if longer than 10 seconds then move on
	//sleep(10);
	//else 
	
	//yz.y = panels[random].y;
        //yz.z = panels[random].z;
	//yz.y = kinect y; yz.z = kinect z;
	//no thread because its one at a time

	yz.y = -240; yz.z = -1850;//5

	mode5(&yz, random);

	sleep(3);

	pthread_t call_mode2;
	pthread_create(&call_mode2, NULL, &mod5,(void*)&yz);
	pthread_join(call_mode2,NULL);*/


//UNCOMMENT THIS THIS IF NOT CONNECTED TO KINECT, 
/*
//	yz.y = 150; yz.z = 2480;//2
//	yz.y = 400; yz.z = 2480;//3
//	yz.y = 650; yz.z = 2480;//4

//	yz.y = 240; yz.z = 2250;//0
//	yz.y = 560; yz.z = 2250;//1
//	yz.y = 240; yz.z = 1850;//5
//	yz.y = 560; yz.z = 1850;//6
//	yz.y = 150; yz.z = 2050;//7
//	yz.y = 400; yz.z = 2050;//8
//	yz.y = 650; yz.z = 2050;//9

	pthread_t call_mode2;
	pthread_create(&call_mode2, NULL, &mode11,(void*)&yz);
	pthread_join(call_mode2,NULL);
//}

}*/
