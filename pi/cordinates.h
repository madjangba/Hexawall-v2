
//#define CORDINATES_H
#define PI 3.14159265
#define NUMPANELS 10

#define MINPULSE 1.0
#define MAXPULSE 2.0
#define MID MINPULSE + (MAXPULSE - MINPULSE)/2.0
#define NO_OF_STEPS 10 // No. of steps from start (MINPULSE/MAX) to end (MAXPULSE/MIN) point

class Panel 
{
    public:
    float y;
    float z;
    int angle;
    int id;
    int delay; // Delay after which the panel thread should start

    //xyShoot
    float xEnd;
  float yEnd;
  float xStepShoot;
    float yStepShoot;

    //xyRecoil
    float xStart;
    float yStart;
    float xStepRecoil;
    float yStepRecoil;

    Panel ();
    Panel (int i, int angle);
    float distance (Panel pushPoint);
    void gotoAngle(Panel pushPoint);
    bool nearer(Panel p, Panel from);
    void setY(int yy);
    void setZ(int zz);
    void init(); // Initializes the variables needed for actuation(xStart.xEnd.yStart.xStep....). 
private:
    int quadrant(int angle); // Input: angle (0-360) | Returns the quadrant (1.2.3.4) in which it lies. 
    
};

void merge(Panel panels[], int low,int mid,int high, Panel pushPoint);
void merge_sort(Panel p[],int low,int high, Panel pushPoint);
void setAngles(Panel panels[], Panel pushPoint, int numPanels); // set gotoAngles of all panels
void setAnglesDown(Panel panels[], Panel pushPoint, int numPanels); // set gotoAngles of all panels
void setAnglesUp(Panel panels[], Panel pushPoint, int numPanels); // set gotoAngles of all panels
void setAnglesLeft(Panel panels[], Panel pushPoint, int numPanels); // set gotoAngles of all panels
void setAnglesRight(Panel panels[], Panel pushPoint, int numPanels); // set gotoAngles of all panels
void setAllAngles(Panel panels[], int numPanels, int angle); // Set angles of all panels in array uniformly as angle (For Mode 1)
void getIndices(Panel panels[], int distIndices[], Panel pushPoint, int radius);     // get indices of points where there is change in zone, based on radius of panels ; 'r' radius of panel
void initPanels(Panel panels[]); // Initializes variables needed for actuation for all the panels. Note: Call only after setting the angle of the Panel!




