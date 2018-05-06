// Programming exercise 3.1

// @author: Carlos Viescas Huerta

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
/*

Write  code  which  calculates  the 3×3 rotation matrix given:

A specification of which axis to rotate around.

A specification of whether to use fixed frame or Euler rotation.

The 3 parameters corresponding to the rotation around the individual axes.

*/

using namespace std;

float world[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
// int rot_axis[3];
float rool, pitch, yaw;


#define PI 3.14159256

int main()
{
	cout << endl;
	cout << "	===========================================" << endl;
	cout << "		     ROTATION MATRICES" << endl;
	cout << "	===========================================" << endl;
	
	// User Interface	

	cout << "Program to compute rotation matrices given RPY Euler angles." << endl;
	cout << "Axes and angles of rotation are specified as follow:" << endl;
	cout << "	>> Yaw (Ox), Pitch (Oy), Rool (Oz)." << endl;
	cout << "Introduce the value of the angles of rotation ---> 0° <= angle (deg) <= 360°" << endl;
	cout << "	>> Yaw (Ox) =";
	cin >> yaw;
	cout << "	>> Pitch (Oy) =";
	cin >> pitch;
	cout << "	>> Rool (Oz) = ";
	cin >> rool;
	cout << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << endl;
	// Calculating...

  	cout << "OUTPUT RESULTS.\n";
	cout << " " << endl;
	cout << "	>> World reference frame:\n";
	cout << " " << endl;
	for ( int i = 0; i < 3; i++ )
	{	
     		for ( int j = 0; j < 3; j++ ) 
		{      
        		
        	 	cout << world[i][j]<< "  ";
     		}
		cout << endl;
	}
	cout << " " << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << " " << endl;
	cout << "	>> Values introduced by the user:\n";
	cout << " " << endl;
	cout << "[ Ox, Oy, Oz ] = [ " << yaw << ", " << pitch << ", " << rool << " ]" << endl;
	
	// Calculation
	float sx = sin(yaw*PI/180);
	float cx = cos(yaw*PI/180);

	float sy = sin(pitch*PI/180);
	float cy = cos(pitch*PI/180);

	float sz = sin(rool*PI/180);
	float cz = cos(rool*PI/180);

	float RPY[3][3] = {{cy*cz, sx*sy*cz-cx*sz, cx*sy*cz+sx*sz},{cy*sz, sx*sy*sz+cx*cz, cx*sy*sz-sx*cz},{-sy, sx*cy, cx*cy}};

	cout << " " << endl;
	cout << "	>> R-P-Y MATRIX:" << endl;
	cout << " " << endl;
	for ( int a = 0; a < 3; a++ )
	{	
     		for ( int b = 0; b < 3; b++ ) 
		{      
        		
        	 	cout << RPY[a][b] << "  ";
     		}
		cout << endl;
	}
	cout << " " << endl;
	cout << "--------------------------------------------------------------------" << endl;
  	cout << endl;
	cout << "Done. Check out <<output.txt>>" << endl;

  	return 0;
}
