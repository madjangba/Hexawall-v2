#include "cordinates.h"
#include "math.h"
#include <cmath>

using namespace std;

    Panel :: Panel () {y = 0; z = 0;}
    Panel :: Panel (int i, int angle)
    {
        // set panel number
        y = 0;
        z = 0;
        id = i;
        angle = angle;
    }
    
    float Panel:: distance (Panel pushPoint)
    {
        float res ;
        res = powf(y-pushPoint.y,2) + powf(z-pushPoint.z,2);
        return sqrt(res);
    }
    
    bool Panel :: nearer(Panel p, Panel from)
    {
        if (distance(from) <= p.distance(from)) return true;
        return false;
    }
    
    void Panel :: gotoAngle(Panel pushPoint)
    {
        angle = (int)(atan2((y-pushPoint.y),(z-pushPoint.z))*180.0/PI);
        if (angle<0) 
            {
                angle+=360;
            }
    }
    
    void Panel :: setY(int yy)
    {
        y = float(yy);///100.0;
    }
    
    void Panel :: setZ(int zz)
    {
        z = float(zz);///100.0;
    }

    // Input: angle (0-360) | Returns the quadrant (1.2.3.4) in which it lies. 
    int Panel :: quadrant(int angle)
    {
        if (angle >= 0 && angle <= 90) { return 1; }
        if (angle > 90 && angle <= 180) { return 2; }
        if (angle > 180 && angle <= 270) { return 3; }
        if (angle > 270 && angle <= 360) { return 4; }
        return 0;
    }

    // Initializes the variables needed for actuation(xStart.xEnd.yStart.xStepShoot....). Note: Call only after setting the angle of the Panel!
    void Panel :: init()
    {
        int quad = quadrant(angle);

        //xyShoot!
        float start = MID;
        float xEnd = MID - cos(angle*PI/180)*fabs((MID - MINPULSE)); 
        float yEnd = MID - sin(angle*PI/180)*fabs((MID - MINPULSE)); 
        xStepShoot = fabs(start - xEnd)/NO_OF_STEPS; 
        yStepShoot = fabs(start - yEnd)/NO_OF_STEPS; 

        // xyRecoil!
        xStart = MID - cos(angle*PI/180)*abs((MID - MINPULSE));
        yStart = MID - sin(angle*PI/180)*abs((MID - MINPULSE));  
        float end = MID;
        xStepRecoil = abs(xStart - end)/NO_OF_STEPS;
        yStepRecoil = abs(yStart - end)/NO_OF_STEPS;

        if (quad == 1) { xStepShoot = -xStepShoot; yStepShoot = -yStepShoot;  /*Recoil!*/ /*xStepRecoil += xStepRecoil; yStepRecoil += yStepRecoil; */  } // xMin | yMin
        else if (quad == 2) { yStepShoot = -yStepShoot; /*Recoil!*/ xStepRecoil = -xStepRecoil; /*yStepRecoil += yStepRecoil;*/ } // xMax | yMin
        else if (quad == 3) { xStepRecoil = -xStepRecoil; yStepRecoil = -yStepRecoil; } // xMax | yMax
        else if (quad == 4) { xStepShoot = -xStepShoot; /*y += yStepShoot;*/ /*Recoil!*/ /*xStepRecoil += xStepRecoil;*/ yStepRecoil = -yStepRecoil;} // xMin | yMax
    }

    // Initializes the variables needed for actuation for all the panels
    void initPanels(Panel panels[])
    {
        for (int i = 0; i<NUMPANELS; i++) { panels[i].init(); }
    }
    
    void setAngles(Panel panels[], Panel pushPoint, int numPanels)
    {
        for (int i = 0; i<numPanels; i++)
        {
            panels[i].gotoAngle(pushPoint);
        }
    }

    void setAnglesDown(Panel panels[], Panel pushPoint, int numPanels)
    {
        for (int i = 0; i<numPanels; i++)
        {
            panels[i].angle = 270;
        }
    }
  
     void setAnglesUp(Panel panels[], Panel pushPoint, int numPanels)
    {
        for (int i = 0; i<numPanels; i++)
        {
            panels[i].angle = 90;
        }
    }
   
     void setAnglesLeft(Panel panels[], Panel pushPoint, int numPanels)
    {
        for (int i = 0; i<numPanels; i++)
        {
            panels[i].angle = 180;
        }
    }
    
     void setAnglesRight(Panel panels[], Panel pushPoint, int numPanels)
    {
        for (int i = 0; i<numPanels; i++)
        {
            panels[i].angle = 360;
        }
    }
 
     // Set angles of all panels in array uniformly as angle (For Mode 1)
    void setAllAngles(Panel panels[], int numPanels, int angle)
    {
        for (int i = 0; i<numPanels; i++)
        {
            panels[i].angle = angle;
        }
    }

    void merge(Panel panels[], int low,int mid,int high, Panel pushPoint)
    {
         int h,i,j,k;
         Panel temp[10];
         h=low;
         i=low;
         j=mid+1;
        
         while((h<=mid)&&(j<=high))
         {
            if(panels[h].nearer(panels[j], pushPoint))
            {
                temp[i]=panels[h];
                h++;
            }
            else
            {
                temp[i]=panels[j];
                j++;
            }
            i++;
         }
            if(h>mid)
            {
            for(k=j;k<=high;k++)
            {
                temp[i]=panels[k];
                i++;
            }
         }
            else
            {
            for(k=h;k<=mid;k++)
            {
                temp[i]=panels[k];
                i++;
            }
         }
         for(k=low;k<=high;k++) {panels[k]=temp[k];}
    }

    void merge_sort(Panel p[],int low,int high, Panel pushPoint)
    {
         int mid;
         if(low<high)
         {
          mid=(low+high)/2;
          merge_sort(p,low,mid,pushPoint);
          merge_sort(p,mid+1,high,pushPoint);
          merge(p,low,mid,high,pushPoint);
         }
    }
    // TEMP: Need to generalize
    void getIndices(Panel p[],int distIndices[], Panel pushPoint, int radius)
    {
        int idx = 0;
        for (int i = 0; i<NUMPANELS-1; i++)
        {
            if (p[i].distance(pushPoint)>3*radius && p[i].distance(pushPoint)<5.5*radius) { distIndices[0] = i; }
            if (p[i].distance(pushPoint)>=5.5*radius && p[i].distance(pushPoint)<8*radius) { distIndices[1] = i; }
	    if (p[i].distance(pushPoint)>=8*radius) { distIndices[2] = i;} 
        }
    }
