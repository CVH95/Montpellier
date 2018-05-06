#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>


using namespace std;

#define PI 3.14159256

float x, y, z;
float world[3][3] = {{1,0,0},{0,1,0},{0,0,1}};


int main()
{	
	// User interface

	cout << endl;
	cout << "	===========================================" << endl;
	cout << "		     EULER ANGLES" << endl;
	cout << "	===========================================" << endl;
	cout << endl;
	cout << "Introduce the value of the yaw angle (rotation around X) ---> 0° <= yaw angle (deg) <= 360°" << endl;
	cout << "	>> Yaw (Ox) = ";
	cin >> x;
	cout << "Introduce the value of the pitch angle (rotation around Y) ---> 0° <= pitch angle (deg) <= 360°" << endl;
	cout << "	>> Pitch (Oy) = ";
	cin >> y;
	cout << "Introduce the value of the rool angle (rotation around Z) ---> 0° <= rool angle (deg) <= 360°" << endl;
	cout << "	>> Rool (Oz) = ";
	cin >> z;
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
	cout << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << endl;
	cout << " " << endl;
	cout << "	>> Values introduced by the user:\n";
	cout << " " << endl;
	cout << "R-P-Y [ " << z << ", " << y << ", " << x << " ] (deg)" << endl;
	
	// calculation
	float sx = sin(x*PI/180);
	float cx = cos(x*PI/180);

	float sy = sin(y*PI/180);
	float cy = cos(y*PI/180);

	float sz = sin(z*PI/180);
	float cz = cos(z*PI/180);

	float RPY[3][3] = {{cy*cz, sx*sy*cz-cx*sz, cx*sy*cz+sx*sz},{cy*sz, sx*sy*sz+cx*cz, cx*sy*sz-sx*cz},{-sy, sx*cy, cx*cy}};
	/*
	Take into account that 3X3 matrix nomenclature in the memory is as follows:

		RPY[0][0] 	RPY[0][1] 	RPY[0][2]
	RPY = 	RPY[1][0] 	RPY[1][1] 	RPY[1][2]
		RPY[2][0] 	RPY[2][1] 	RPY[2][2]

	*/
	float r31 = RPY[2][0];
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
	cout << "COEFFICIENT TAKEN FROM THE MATRIX TO CALCULATE EULER ANGLES:" << endl;
	cout << " " << endl;
	cout << "	>> RPY[3][1] = " << r31 << endl;
	cout << " " << endl;

	if ( abs(RPY[2][0]) == 1)
	{
		cout << "Jodete, tienes que resolver el sistema de ecuaciones!!" << endl;
	}
	else
	{
		float OY = asin(-r31);
		float COSY = cos(OY);
		float OX = atan2(COSY*RPY[2][1], COSY*RPY[2][2]);
		float OZ = atan2(COSY*RPY[1][0], COSY*RPY[0][0]);
		cout << "R-P-Y recalculated:" << endl;
		cout << endl;
		cout << "	>> [Oz, Oy, Ox] = [ " << OZ*180/PI << ", " << OY*180/PI << ", " << OX*180/PI << " ] (deg)" << endl;
	}
	
	cout << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << endl;
	cout << "Done. Check -----> output.txt. " << endl;
	
  	return 0;
}
	
