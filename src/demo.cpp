/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libicp.
Authors: Andreas Geiger

libicp is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libicp is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libicp; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// Demo program showing how libicp can be used

#include <iostream>
#include "icpPointToPlane.h"
#include "icpPointToPoint.h"
#include "ReadCSV.h"


using namespace std;

double MeasurementError (double SpanPos) {


  vector< vector<double> > guess;
  vector< vector<double> > measured;
  FindAerofoilAt(SpanPos, guess);

  cout<<"Estimated Coordinates"<<endl<<endl;
  for(int i = 0; i<guess.size(); i++)
  {
  	cout<<guess[i][0]<<"    "<<guess[i][1]<<endl;
  }
	
  cout<<endl<<"Measured Coordinates"<<endl<<endl;
  getMeasuredData("ExampleDate.csv",measured);
  int closesti=0;
  double closest=sqrt(measured[0][0]*measured[0][0]+measured[0][1]*measured[0][1]);
  double dist;
  for(int i = 0; i<measured.size(); i++)
  {
  	cout<<measured[i][0]<<"    "<<measured[i][1]<<endl;
	if (i>0)
	{
		dist=sqrt(measured[i][0]*measured[i][0]+measured[i][1]*measured[i][1]);
		if (dist<closest)
		{
			closest = dist;
			closesti = i;
		}
	}
  }
  
  // define a 3 dim problem with 10000 model points
  // and 10000 template points:
  int32_t dim = 2; //Dimensionality of model points
  int32_t M_num = 10000; // number of points
  int32_t T_num = 100; // number of points

  /*
  Use real data 
  */
  T_num = guess.size(); // number of points
  M_num = measured.size(); // number of points
  dim=2;
  
  
  double* MM = (double*)calloc(dim*M_num,sizeof(double)); //Pointer to first model points
  double* TT = (double*)calloc(dim*T_num,sizeof(double)); 
  int k=0;
  double offset [dim] = { measured[closesti][0],measured[closesti][1]};// offset from first 
  for(int i =0; i<M_num;i++){
		
		MM[k*dim+0] = measured[i][0]- offset[0];
		MM[k*dim+1] = measured[i][1]- offset[1];
		/*
		MM[k*dim+0] = measured[i][0];
		MM[k*dim+1] = measured[i][1];
		*/
	k++;
  }
  k=0;
  for(int i =0; i<T_num;i++){
		TT[k*dim+0] = guess[i][0];
		TT[k*dim+1] = guess[i][1];
	k++;
  }
  
  // start with identity as initial transformation
  // in practice you might want to use some kind of prediction here
  Matrix R = Matrix::eye(dim); // Initial/final  rotation matrix
  Matrix t(dim,1); // initial/final translation vector

  // run point-to-plane ICP (-1 = no outlier threshold)
  cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
  
  IcpPointToPlane icp(TT,T_num,dim);//IcpPointToPlane icp(M,M_num,dim);
  //IcpPointToPoint icp(M,M_num,dim); // M_num not just num dont have to be the same length
  //icp.setMaxIterations(10000);
  //icp.setMinDeltaParam(10000);
  icp.fit(MM,M_num,R,t,-1);//icp.fit(T,T_num,R,t,-1); //inlier distance T_num not num domnt have to be the same length

  // results
  cout <<"R:"<<endl<<R<<endl<<"t:"<<endl<<t<<endl<<endl;
  double* trans= (double*)calloc(dim,sizeof(double));
  t.getData(trans);
  double* ang= (double*)calloc(dim*dim,sizeof(double));
  R.getData(ang);
  cout << "initial offset:" <<endl<<"x: "<<offset[0]<<" m"<<" z: "<<offset[1]<<" m"<<endl << endl;
  //offset[0]=offset[0] +trans[0];
  //offset[1]=offset[1] +trans[1];
  cout << "final total offset:" <<endl<<"x: "<<offset[0]<<" m"<<" z: "<<offset[1]<<" m"<<endl << endl;
  cout << "rotation:"<<endl<<asin(ang[1])*180/3.14159265<<" degrees"<<endl << endl;

  // success
  
  //Correct Measured
  vector< vector<double> > corrected;
  for(int i = 0; i<M_num; i++)
  {
	  vector<double>  temp;
  temp.push_back((measured[i][0]-offset[0])*ang[0]+(measured[i][1]-offset[1])*ang[1]-trans[0]);
	  temp.push_back((measured[i][0]-offset[0])*ang[2]+(measured[i][1]-offset[1])*ang[3]-trans[1]);
	  corrected.push_back(temp);
  }
    for(int i = 0; i<corrected.size(); i++)
  {
  	cout<<corrected[i][0]<<"    "<<corrected[i][1]<<endl;
  }
  // Find error
  double error;
  error = findError(guess,corrected);
  cout<<"error is : "<<error<<endl;
  // free memory
  free(MM);
  free(TT);
  free(ang);
  free(trans);
  return error;
  
}

int main (int argc, char** argv) {
	double error;
	error=MeasurementError(0.2);
	cout<<error<<endl;
}

