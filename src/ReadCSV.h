//Return Vector
#ifndef READCSV_H
#define READCSV_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <iomanip>

using namespace std;

/*
* It will iterate through a row  in file and
* call the given callback on each line
*/
/*
MODIFY TO HEADER!!!
*/


bool iterateFile(string fileName,int readRow, function<void (const double & )> callback)
{
// Open the File
ifstream in(fileName.c_str());
// Check if object is valid
if(!in)
{
cerr << "Cannot open the File : "<<fileName<<endl;
return false;
}
string str;
// Read the next line from File untill it reaches the end.
while (getline(in, str))
{
// Call the given callback
istringstream iss(str);
string lineStream;
string::size_type sz;

vector<double> row;

	while(getline(iss, lineStream, ','))	
	{
		row.push_back(stod(lineStream,&sz));
	}

callback(row[readRow]);
}
//Close The File
in.close();
return true;
}

void getMeasuredData(string FileName, vector< vector<double> >& Sol)
{
bool res;	
	vector< double> Measured_x;
	vector< double> Measured_z;
	
	res = iterateFile(FileName,0,[&](const double & temp){Measured_x.push_back(temp);});
	res = iterateFile(FileName,1,[&](const double & temp){Measured_z.push_back(temp);});
	
	for(int i = 0; i<Measured_x.size(); i++)
	{
		vector< double> temp2;
		temp2.push_back(Measured_x[i]);
		temp2.push_back(Measured_z[i]);
		Sol.push_back(temp2);
	}
}

double findError(vector< vector<double> > Known, vector< vector<double> > Measured)
{
	double minerror;
	double minerror2;
	int minerrori;
	int minerror2i;
	double mKnown;
	double mMeasured;
	double cKnown;
	double cMeasured;
	double xIntercept;
	double zIntercept;
	double errorout;
	double error;
	
	errorout = 0;
	for(int i = 0; i<Measured.size();i++){
		for(int j = 0; j<Known.size();j++){
			// calculate distance to point.
			error = sqrt( (Known[j][0]-Measured[i][0])*(Known[j][0]-Measured[i][0])+(Known[j][1]-Measured[i][1])*(Known[j][1]-Measured[i][1]) );
			
			// find two points on the guessed aerofoil to interpolate between.
			if(j==0){
				minerror = error;
				minerrori=j;
			}else if(j==1){
				if(error<minerror){
					minerror2 = minerror;
					minerror2i= minerrori;
					minerror= error;
					minerrori= j;
				}else{
					minerror2= error;
					minerror2i= j;
				}
			}else if(j>1){
				if(error<minerror){
					minerror2 = minerror;
					minerror2i= minerrori;
					minerror= error;
					minerrori= j;
				}else if(error<minerror2){
					minerror2 = error;
					minerror2i= j;
				}
			}
		}
		// work out distance to line imbetween closest two guessed points
		mKnown=((Known[minerror2i][1]-Known[minerrori][1])/(Known[minerror2i][0]-Known[minerrori][0]));
		cKnown=Known[minerror2i][1]-mKnown*Known[minerror2i][0];
		mMeasured=-(1/mKnown);
		cMeasured=Measured[i][1]-mMeasured*Measured[i][0];
		xIntercept=(cMeasured-cKnown)/(mKnown-mMeasured);
		zIntercept=mKnown*xIntercept+cKnown;
		errorout = errorout + sqrt( (Measured[i][0]-xIntercept)*(Measured[i][0]-xIntercept)+(Measured[i][1]-zIntercept)*(Measured[i][1]-zIntercept) );
		
	}
	return errorout;
}

void FindAerofoilAt( double SpanPos, vector< vector<double> >& Sol)
{

vector<double> Root_x;
vector<double> Root_z;
vector<double> Tip_x;
vector<double> Tip_z;
bool res;

//Call given lambda function for each line in file
//Get Columns
res = iterateFile("fx61184-il.csv",0,[&](const double & temp){Root_x.push_back(temp);});
res = iterateFile("fx61184-il.csv",1,[&](const double & temp){Root_z.push_back(temp);});
res = iterateFile("fx60126-il.csv",0,[&](const double & temp){Tip_x.push_back(temp);});
res = iterateFile("fx60126-il.csv",1,[&](const double & temp){Tip_z.push_back(temp);});

//Wing parameters.
double Span = 7.5937;
double Tip_Chord = 0.5625;
double Root_Chord = 1.21875;

//Multiply the chords by the right amount.These start at 0.1m long and in mm
double temp1{0.01*Root_Chord};
transform(Root_x.begin(),Root_x.end(),Root_x.begin(), [&temp1](auto& c){return c*temp1;});
transform(Root_z.begin(),Root_z.end(),Root_z.begin(), [&temp1](auto& c){return c*temp1;});
double temp2{0.01*Tip_Chord};
transform(Tip_x.begin(),Tip_x.end(),Tip_x.begin(), [&temp2](auto& c){return c*temp2;});
transform(Tip_z.begin(),Tip_z.end(),Tip_z.begin(), [&temp2](auto& c){return c*temp2;});

//adding  in y direction
vector<double> Root_y (Root_x.size()); 
vector<double> Tip_y (Tip_x.size());
fill(Root_y.begin(),Root_y.end(),0);
fill(Tip_y.begin(),Tip_y.end(), Span);

//double SpanPos = Span/2; now called by function
vector<double> Sol_x (Root_x.size());
vector<double> Sol_y (Root_y.size());
vector<double> Sol_z (Root_z.size());

if( SpanPos > Root_y[0] && SpanPos < Tip_y[0] ){

for(int i = 0; i<Root_x.size(); i++){
	Sol_x[i] = Root_x[i] + SpanPos*(Tip_x[i]-Root_x[i])/Span;
	Sol_y[i] = Root_y[i] + SpanPos*(Tip_y[i]-Root_y[i])/Span;
	Sol_z[i] = Root_z[i] + SpanPos*(Tip_z[i]-Root_z[i])/Span;
}

int p=5;//precision
for(int i = 0; i<Root_x.size(); i++){
        if (i==0){
                //cout<<"Root Aerofoil @: "<<setprecision(p)<<Root_y[i]<<"\t\t Tip Aerofoil @: "<<setprecision(p)<<Tip_y[i]<<"\t\t Interpolated Aerofoil @: "<<setprecision(p)<<Sol_y[i]<<endl;
                //cout<<"X\tZ\t\tX\tZ\t\tX\tZ"<<endl;
        }
        //cout<<setprecision(p)<<Root_x[i]<<"\t"<<setprecision(p)<<Root_z[i]<<"\t\t"<<setprecision(p)<<Tip_x[i]<<"\t"<<setprecision(p)<<Tip_z[i]<<"\t\t"<<setprecision(p)<<Sol_x[i]<<"\t"<<setprecision(p)<<Sol_z[i]<<endl;

	vector< double> temp2;
	temp2.push_back(Sol_x[i]);
	temp2.push_back(Sol_z[i]);
	Sol.push_back(temp2);

}

}else{
	cout<< "Spanwise Position Must be on the wing"<<endl;
	vector< double> temp2;
	temp2.push_back(-1);
	temp2.push_back(-1);
	Sol.push_back(temp2);
}

//cout<< " "<<endl;
//cout<< " "<<endl;
//cout<< " "<<endl;

}

#endif
