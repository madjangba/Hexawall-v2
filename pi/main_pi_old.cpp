/*
    AUTHORS:
    Dhruva Kumar, dhruva.kumar08@gmail.com
    Aditya GOurav, adityagoyrav@gmail.com
*/

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

#define MINPULSE 1.0
#define MAXPULSE 2.0
#define MID MINPULSE + (MAXPULSE - MINPULSE)/2.0

#define NO_OF_STEPS 10 // No. of steps from start (MINPULSE/MAX) to end (MAXPULSE/MIN) point
#define DELAY_BW_PULSES 0 // Delay in ms between each setServoPulse() call
// [Deprecated] #define INC_BEW_PULSES 0.02 // for loop increment from (min)MINPULSE - (max)MAXPULSE
#define DELAY_BW_MOTIONS 100 // Delay between Shoot & Recoil

#define DELAY_BW_PANELZONES 220 // For mode 2, delay of panel actuation between zones created
#define RADIUS_PANEL 130 // Radius of each panel (Used to ignore the panel actuation for which the push point lies on that panel)

#define NUMPANELS 10

using namespace std;

PCA9685 pwm1;    // nothing soldered
PCA9685 pwm2;  // A0 soldered
// CHANGE: Create instance of PwmServoDriver class.
//Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(p9, p10, 0x80);
//Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(p9, p10, 0x82);

// Global variables
Panel panels[NUMPANELS]; // Array of panel (named Panel) objects
Panel pushPoint(0,0); // Center push point
// Stack to store the commands from serial data
char *stack[10]; int stackPtr = 0; int stackFlag = 0;
int distIndices[5] = {0}; // Indices of start of panel zones. 


// for recieving udp packets
char buffer[100];
char *data = &buffer[0];
char sendto_ip[20] = "158.130.62.97";


int recievePacket(int port)
{
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

/*
-----------Summary of all the functions----------------

void init() : Initializes the PWM servo drivers (Prescale. Set I2C freq.)
void setServoPulse(uint8_t n, float pulse) : Maps panel no. to servos and feeds them PWM (ms -> 0-4094)
~void angleDisplay(char * buffer) : Displays the angle received from serial data [Deprectaed/Debugging] 
int quadrant(int angle) : Input: angle (0-360). Returns quadrant it lies in (1.2.3.4)

void xyShoot(void const *args) : Part 1 { panelActuate() } (Moves panel from center to desired point)
void waitBWMotions(void const *args) : Part 2 { panelActuate() } (Waits)
void xyRecoil(void const *args) : Part 3 { panelActuate() } (Moves panel from desired point to center)

void actuatePanel(void const *panel) : Given panel object (Panel). With specified delay (for zones) actuate panel (Shoot.wait.Recoil) 
~void push(char * buffer) : Pushes serial data onto global queue (TEMP)
~void serialCom() : Listener thread which keeps looking for oncoming serial data and enqueues into global data q. 
void actuatePanels(int n, ...) Input: n (no. of arguments), all the panel objects that need to be actuated | Function: For all the panel objects passed, it creates threads and actuates them. [Used for Mode 1]

void mode1(int col, int dir) : Operates Mode 1 (Walk). Input: Column. Direction| Function: Actuates corresponding panels in that column [Dependencies: actuatePanels()]
void mode2(int y, int z): Mode 2 (PUSH): Actuate panel according to (y,z) of push origin

void initPanels(): Initializes the panels with ids, delay, center (y,z) of each panel

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

//CHANGE:
// Initializes the PWM servo driver
void init()
{
    //pwm1.begin(); pwm2.begin();
    pwm1.init(1,0x40); pwm2.init(1,0x41); 
    pwm1.setPWMFreq(100); pwm2.setPWMFreq(100);     //This value is decided for 10ms interval - 100 Hz PWM frequency
    //pwm1.setI2Cfreq(400000); pwm2.setI2Cfreq(400000); //400kHz  
}

// Maps panel no. to servo no. and sets the servo pulse accordingly
// TEMP: Panel numbers from L:R: 2.3.4.0.1.7.8.9.5.6
void setServoPulse(uint8_t n, float pulse) // pulse in ms 
{
    
    pulse = 4094 * pulse / 10; // 10 ms ; period of pwm
    if (n < 10) { pwm1.setPWM(n, 0, pulse); }
    else {n = n - 10; pwm2.setPWM(n, 0, pulse); }
}

// [Deprecated/debugging] Displays the serial input buffer
int angleDisplay(char * buffer)
{
   int p = 0;
   //pc.printf("\r\nYou entered: "); 
   int angle = 0;
   while(*(buffer+p) != '\r') { angle = (angle * 10) + *(buffer+(p++)) - 48; }  
   //pc.printf("%d\r\n",angle);
   stringstream ss;
   ss<<angle;
   sendPacket(sendto_ip, 5000, ss.str().c_str());
   return angle;
}

// Abstract! -> Panel
/*
// Input: angle (0-360) | Returns the quadrant (1.2.3.4) in which it lies. 
int quadrant(int angle)
{
    if (angle >= 0 && angle <= 90) { return 1; }
    if (angle > 90 && angle <= 180) { return 2; }
    if (angle > 180 && angle <= 270) { return 3; }
    if (angle > 270 && angle <= 360) { return 4; }
    return 0;
}
*/

//  Part 1 of actuatePanel(): Shoot! (Moves panel from center to desired point)
void xyShoot(void const *args)
{
    // Extract id & angle
    //int servo = *((const int*)args); int angle = *(((const int*)args)+1);
    int servo = static_cast<const Panel*>(args)->id; 
    int angle = static_cast<const Panel*>(args)->angle;

    // Debugging
    //pc.printf("Reached xyShoot! Servo: %d\r\n", servo);
    
    // Abstract -> Panel!
    /*
    int quad = quadrant(angle);
    float start = MID;
    float xEnd = MID - cos(angle*PI/180)*fabs((MID - MINPULSE)); 
    float yEnd = MID - sin(angle*PI/180)*fabs((MID - MINPULSE)); 
    float xStep = fabs(start - xEnd)/NO_OF_STEPS; 
    float yStep = fabs(start - yEnd)/NO_OF_STEPS; 
    */
    float xStep = static_cast<const Panel*>(args)->xStepShoot;
    float yStep = static_cast<const Panel*>(args)->yStepShoot;

    float count = 0; float x = MID; float y = MID;
    while (count < NO_OF_STEPS)
    {
        setServoPulse(servo*2, x);
        setServoPulse(servo*2 + 1, y);

        // Abstract -> Panel!
        /*
        if (quad == 1) { x -= xStep; y -= yStep; } // xMin | yMin
        else if (quad == 2) { x += xStep; y -= yStep; } // xMax | yMin
        else if (quad == 3) { x += xStep; y += yStep; } // xMax | yMax
        else if (quad == 4) { x -= xStep; y += yStep; } // xMin | yMax
        */
        x += xStep; y += yStep;
        count++;
        //CHANGE::
        //Thread::wait(DELAY_BW_PULSES);
        usleep(DELAY_BW_PULSES*1000);
    }
    //pc.printf("xyShoot Done!\r\n");
}

//  Part 2 of actuatePanel: Wait!
void waitBWMotions(void const *args)
{
    //pc.printf("Reached waitBWMotions!\r\n");
    //CHANGE:
    //Thread::wait(DELAY_BW_MOTIONS);
    usleep(DELAY_BW_MOTIONS*1000);
}

//  Part 3 of actuatePanel: Recoil! (Moves panel from desired point to center)
void xyRecoil(void const *args)
{
    // Extract id & angle
    int servo = static_cast<const Panel*>(args)->id; 
    int angle = static_cast<const Panel*>(args)->angle;

    // Debugging
    //pc.printf("Reached xyRecoil! Servo: %d\r\n", servo);
    
    // Abstract!
    /*
    int quad = quadrant(angle);
    float xStart = MID - cos(angle*PI/180)*abs((MID - MINPULSE));
    float yStart = MID - sin(angle*PI/180)*abs((MID - MINPULSE));  
    float end = MID;
    float xStep = abs(xStart - end)/NO_OF_STEPS;
    float yStep = abs(yStart - end)/NO_OF_STEPS;
    */

    float xStart = static_cast<const Panel*>(args)->xStart; 
    float yStart = static_cast<const Panel*>(args)->yStart;  
    float xStep = static_cast<const Panel*>(args)->xStepRecoil; 
    float yStep= static_cast<const Panel*>(args)->yStepRecoil; 
    
    float count = 0; float x = xStart; float y = yStart;
    while (count < NO_OF_STEPS)
    {
        setServoPulse(servo*2, x);
        setServoPulse(servo*2 + 1, y);

        // Abstract!
        /*
        if (quad == 1) { x += xStep; y += yStep; } // xMin | yMin
        else if (quad == 2) { x -= xStep; y += yStep; } // xMax | yMin
        else if (quad == 3) { x -= xStep; y -= yStep; } // xMax | yMax
        else if (quad == 4) { x += xStep; y -= yStep; } // xMin | yMax
        */
        x += xStep; y += yStep;
        count++;
        //CHANGE:
        //Thread::wait(DELAY_BW_PULSES);
        usleep(DELAY_BW_PULSES*1000);
    }
    //CHANGE
    // Bring to rest/stationary position after a short delay
    //Thread::wait(200);
    usleep(400*1000);
    setServoPulse(servo*2, 0); setServoPulse(servo*2 + 1, 0);
    stringstream ss;
    ss<<servo<<" xyRecoil Done!";
    sendPacket(sendto_ip, 5000, ss.str().c_str());
    //pc.printf("xyRecoil Done!\r\n");
}

// Input: Panel object (which contains servo number and angle - dereferenced later) | Function: Actuates panel
void* actuatePanel(void *panel)
{
    // CHANGE:
    // Delay before the current panel should actuate.
    int delay = static_cast<Panel*>(panel)->delay; 
    //Thread::wait(delay);
    usleep(delay*1000);
    
    xyShoot(panel);
    waitBWMotions(panel);
    xyRecoil(panel);

}

// [Not Used!] Pushes the new command onto stack and updates pointer. Didn't work on it. Currently doing it one at a time.    
void push(char * buffer)
{
    //*(stack + stackPtr) = buffer;
    *stack = buffer;
    //stackPtr++;
    stackFlag = 1;
}

/*
// Listener thread for getting serial data continuously
void serialCom(void const *args)
{
    char buffer[50];
    int i = 0;
    pc.printf("Enter!\r\n");
    while (1)
    {
        if (pc.readable())
        {
            char input = pc.getc();
            *(buffer+(i++)) = input;
            pc.putc(input);
            if (input == 13 && stackFlag == 0) // When carriage return && panels are not being actuated, then execute actuation
            // Note: Will need to change if stack implemented to stack up multiple commands and pop them later.
            {
                *(buffer + (i-1)) = '\0'; // Null character
                //int angle = angleDisplay(buffer);
                push(buffer);
                i = 0;
                pc.printf("Enter!\r\n");
            }
        } 
    }
}
*/

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
    /*
    int over = 0;
    while (over < n)
    {
        // CHANGE:
        for (int i = 0; i <n; i++) { if (p[i].get_state() == 0 ) over++; else over--; }
    }
    //CHANGE:
    // Explicitly terminate the threads
    for(int i = 0; i<n; i++) { p[i].terminate(); };
    */
}

// Mode 1: Actuate panel according to column & direction

void mode1(int col, int dir)
{
    dir = (dir == 0) ? 90 : 270;
    setAllAngles(panels, NUMPANELS, dir); // Set angles of all panels as direction
    initPanels(panels); // Initialize all variables w.r.t panel actuation (Call after setting angle!)

    if (col == 1)
    {        
        actuatePanels(3, (void *)&panels[2], (void *)&panels[3], (void *)&panels[4]);
        //p[0].init(actuatePanel, (void *)&panels[2]);
        //p[1].init(actuatePanel, (void *)&panels[3]);
        //p[2].init(actuatePanel, (void *)&panels[4]);
        
        //while(p[0].get_state() != 0 && p[1].get_state() != 0 && p[2].get_state() != 0);
    }
    if (col == 2) { actuatePanels(2, (void *)&panels[0], (void *)&panels[1]); }
    if (col == 3) { actuatePanels(3, (void *)&panels[7], (void *)&panels[8], (void *)&panels[9]); }
    if (col == 4) { actuatePanels(2, (void *)&panels[5], (void *)&panels[6]); }
    if (col == 5) { actuatePanels(7, (void *)&panels[0], (void *)&panels[1], (void *)&panels[5], (void *)&panels[6], (void *)&panels[7], (void *)&panels[8], (void *)&panels[9]); }
}


// Mode 2 (PUSH): Actuate panel according to (y,z) of push origin
//void mode2(int y, int z)
void* mode2(void *yz)
{
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
        //pc.printf("%d | ", panels[i].id); 
        std::stringstream ss;
        ss<<panels[i].id<<"("<<panels[i].angle<<")"<<" | ";
        //printf("%d | ", panels[i].id); 
        sendPacket(sendto_ip,5000,ss.str().c_str());
        if (i == distIndices[j]-1) 
        {
            //pc.printf("|| "); j++; 
            //ss<<"|| ";
            sendPacket(sendto_ip,5000,"||");
            j++;
        }
    }
    //pc.printf("\r\n");
    //sendPacket("158.130.62.118",5000,"\r\n");
    
    int n = 7; int delay = 0; int distIndex = 0;
    // CHANGE:
    pthread_t p[n]; // Create n Threads
    int threadInd = 0;
    for (int i = 0;  i<NUMPANELS; i++)
    { 
        // If Push origin on panel, ignore the actuation!
        if (panels[i].distance(pushPoint) < RADIUS_PANEL) { continue; }
        
        // TEMP: Ignore Panel 2,3,4! (Since the mbed doesn't support so many threads!)
        if (panels[i].id == 5 || panels[i].id == 6 || panels[i].id == 9) { continue; }
        
        // Increment delay based on sorted indices (Zones)
        // Currently set for linearly increasing delay. Can change depending on testing. 
        if (i == distIndices[distIndex]) { delay += DELAY_BW_PANELZONES; distIndex++; }
        panels[i].delay = delay;
        
        // TEMP: Instead of threadInd use i
        // needs to be changed to ptherad_t API
        //p[threadInd++].init(actuatePanel, (void *)&panels[i]); // Create and initialize threads
        pthread_create(&p[threadInd++], NULL, &actuatePanel, (void*)&panels[i]);
        //pthread_join(p[i], NULL);
    }

    // Wait until all threads finish executing
    
    for (int i = 0; i<7; i++)
    {
        pthread_join(p[i], NULL);
    }
    
    /*
    int over = 0;
    while (over < n)
    {
        for (int i = 0; i <n; i++) {  if (p[i].get_state() == 0 ) over++; else over--; }
    }
    // Explicitly terminate the threads
    for(int i = 0; i<n; i++) { p[i].terminate(); }
    */
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
    //Thread input(serialCom);
    
    // Porting serial comm thread to main thread:
    //char buffer[50]; buffer defined as global
    int i = 0;
    //pc.printf("Enter!\r\n");
    sendPacket(sendto_ip,5000,"Entered in main");
    YZ yz;

    while(1)
    {
        if (recievePacket(5000)>0 && stackFlag == 0)
        {
            stackFlag=1;
        }

        // Once the packet is received, enter into Mode 1/Mode 2.
        if(stackFlag == 1) // When there's a packet!
        {
            //*stack = buffer;
           // char * data = *stack;
           
            //pc.printf("Reached main! %c", *data);
            if (*data == '1') // Mode 1!
            {
                int col = *(data+1) - '0';
                int dir = *(data+2) - '0';
                //pc.printf("Mode 1 | Col: %d | Dir: %d\r\n", col, dir);
                initPanels();
                mode1(col, dir);
            }
            if (*data == '2') // Mode 2!
            {
                pthread_t call_mode2;
                int i = 0; int y = 0; int z = 0;
                while ((++i) <= 4) { y = y*10 + *(data+i) - '0'; } // Form int y coordinate from characters
                i--;
                while ((++i) <= 8) { z = z*10 + *(data+i) - '0'; } // Form int z coordinate from characters
                //pc.printf("Mode 2 | y: %d | z: %d\r\n", y, z);
                stringstream ss ;
                ss<<"Mode 2 | y:"<<y<<" | z: "<<z;
                sendPacket(sendto_ip, 5000, ss.str().c_str());
                yz.y = y; yz.z = z;
                pthread_create(&call_mode2, NULL, &mode2, (void*)&yz);
                pthread_join(call_mode2, NULL);
                sendPacket(sendto_ip,5000,"DONE");
            }
            stackFlag = 0; // Reset packet flow
            
        }
            /*
        if
            int i = 0; int y = 0; int z = 0;
            while ((++i) <= 4) { y = y*10 + *(data+i) - '0'; } // Form int y coordinate from characters
                i--;
            while ((++i) <= 8) { z = z*10 + *(data+i) - '0'; } // Form int z coordinate from characters
            //std::cout<<"Y = "<<y<<": Z = "<<z<<"\n";
            mode2(y, z);
            sendPacket(sendto_ip,5000,"Enter");
        }
        stackFlag=0;     // Reset packet flow
        
        // Listener for TCP packet from Kinect. 
        if (pc.readable() && stackFlag == 0)
        {
            char input = pc.getc();
            *(buffer+(i++)) = input;
            pc.putc(input);
            if (input == 13 && stackFlag == 0) // When carriage return && panels are not being actuated, then execute actuation
            // Note: Will need to change if stack implemented to stack up multiple commands and pop them later.
            {
                *(buffer + (i-1)) = '\0'; // Null character
                //int angle = angleDisplay(buffer);
                //push(buffer);
                stackFlag = 1;
                i = 0;
                pc.printf("Enter!\r\n");
            }
        } 
        
        // Once the packet is received, enter into Mode 1/Mode 2.
        if(stackFlag == 1) // When there's a packet!
        {
            //*stack = buffer;
           // char * data = *stack;
           char * data = buffer;
            //pc.printf("Reached main! %c", *data);
            if (*data == '1') // Mode 1!
            {
                int col = *(data+1) - '0';
                int dir = *(data+2) - '0';
                //pc.printf("Mode 1 | Col: %d | Dir: %d\r\n", col, dir);
                initPanels();
                mode1(col, dir);
            }
            if (*data == '2') // Mode 2!
            {
                int i = 0; int y = 0; int z = 0;
                while ((++i) <= 4) { y = y*10 + *(data+i) - '0'; } // Form int y coordinate from characters
                i--;
                while ((++i) <= 8) { z = z*10 + *(data+i) - '0'; } // Form int z coordinate from characters
                pc.printf("Mode 2 | y: %d | z: %d\r\n", y, z);
                mode2(y, z);
            }
            stackFlag = 0; // Reset packet flow
            */
        //}
                        
    }       
}

