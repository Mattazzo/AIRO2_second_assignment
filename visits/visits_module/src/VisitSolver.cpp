/*
<one line to give the program's name and a brief idea of what it does.>
Copyright (C) 2015  <copyright holder> <email>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>

#include "armadillo"
#include <initializer_list>

using namespace std;
using namespace arma;



//map <string, vector<double> > region_mapping;
mat Pinit, Qgamma, Qalpha, A;

extern "C" ExternalSolver* create_object(){
    return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver){
    delete externalSolver;
}

VisitSolver::VisitSolver(){}

VisitSolver::~VisitSolver(){}

void VisitSolver::loadSolver(string *parameters, int n){
    starting_position = "r0";
    string Paramers = parameters[0];

    char const *x[]={"dummy"};
    char const *y[]={"act-cost","triggered"};
    parseParameters(Paramers);
    affected = list<string>(x,x+1);
    dependencies = list<string>(y,y+2);

    string waypoint_file = "/home/enrico/Desktop/visits/visits_domain/waypoint.txt";
    parseWaypoint(waypoint_file);

    string landmark_file = "/home/enrico/Desktop/visits/visits_domain/landmark.txt";
    parseLandmark(landmark_file);

    startEKF();
}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic){

    map<string, double> toReturn;
    map<string, double>::iterator iSIt = initialState.begin();
    map<string, double>::iterator isEnd = initialState.end();
    double dummy;
    double act_cost;

    map<string, double> trigger;

    for(;iSIt!=isEnd;++iSIt){
        string parameter = iSIt->first;
        string function = iSIt->first;
        double value = iSIt->second;

        function.erase(0,1);
        function.erase(function.length()-1,function.length());
        int n=function.find(" ");
        if(n!=-1){
            string arg=function;
            string tmp = function.substr(n+1,5);

            function.erase(n,function.length()-1);
            arg.erase(0,n+1);
            if(function=="triggered"){
                trigger[arg] = value>0?1:0;
                if (value>0){

                    string from = tmp.substr(0,2);   // from and to are regions, need to extract wps (poses)
                    string to = tmp.substr(3,2);
                    from.replace(0,1,"wp");
                    to.replace(0,1,"wp");

                    toReturn["(dummy)"] = localize(from, to);
                }
            }
        }else{
            if(function=="dummy"){
                dummy = value;

            }else if(function=="act-cost"){
                act_cost = value;
                //cout << "COST " << act_cost << " VALUE " << value << endl;
            } //else if(function=="dummy1"){
                //duy = value;
                ////cout << parameter << " " << value << endl;
            //}
        }
    }


    double results = calculateExtern(dummy, act_cost);

    if (ExternalSolver::verbose){
        cout << "(dummy) " << results << endl;
    }

    //toReturn["(dummy)"] = results;

    return toReturn;
}

list<string> VisitSolver::getParameters(){
    return affected;
}

list<string> VisitSolver::getDependencies(){
    return dependencies;
}


void VisitSolver::parseParameters(string parameters){

    int curr, next;
    string line;
    ifstream parametersFile(parameters.c_str());

    if (parametersFile.is_open()){
        while (getline(parametersFile,line)){
            curr=line.find(" ");
            string region_name = line.substr(0,curr).c_str();
            curr=curr+1;
            while(true ){
                next=line.find(" ",curr);
                region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
                if (next ==-1)
                break;
                curr=next+1;
            }
        }
    }
}

double VisitSolver::calculateExtern(double external, double total_cost){
//float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
    double cost = 2;//random1;
    return cost;
}

void VisitSolver::parseWaypoint(string waypoint_file){

    int curr, next;
    string line;
    double pose1, pose2, pose3;
    ifstream parametersFile(waypoint_file);

    if (parametersFile.is_open()){
        while (getline(parametersFile,line)){
            curr=line.find("[");
            string waypoint_name = line.substr(0,curr).c_str();

            curr=curr+1;
            next=line.find(",",curr);

            pose1 = (double)atof(line.substr(curr,next-curr).c_str());
            curr=next+1; next=line.find(",",curr);

            pose2 = (double)atof(line.substr(curr,next-curr).c_str());
            curr=next+1; next=line.find("]",curr);

            pose3 = (double)atof(line.substr(curr,next-curr).c_str());

            waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};
        }
    }
}

void VisitSolver::parseLandmark(string landmark_file){

    int curr, next;
    string line;
    double pose1, pose2, pose3;
    ifstream parametersFile(landmark_file);

    if (parametersFile.is_open()){
        while (getline(parametersFile,line)){
            curr=line.find("[");
            string landmark_name = line.substr(0,curr).c_str();

            curr=curr+1;
            next=line.find(",",curr);

            pose1 = (double)atof(line.substr(curr,next-curr).c_str());
            curr=next+1; next=line.find(",",curr);

            pose2 = (double)atof(line.substr(curr,next-curr).c_str());
            curr=next+1; next=line.find("]",curr);

            pose3 = (double)atof(line.substr(curr,next-curr).c_str());

            landmark[landmark_name] = vector<double> {pose1, pose2, pose3};
        }
    }
}

double VisitSolver::localize(string from, string to){

    // from coordinates
    double x1 = waypoint[from][0];
    double y1 = waypoint[from][1];
    double z1 = waypoint[from][2];

    // to coordinates
    double x2 = waypoint[to][0];
    double y2 = waypoint[to][1];
    double z2 = waypoint[to][2];

    // travelled distance
    double distance = sqrt(pow(x1-x2,2) + pow(y1-y2,2));

    // pre-orientation
    double dth = atan((y2-y1)/(x2-x1));

    // odometry equations
    double xodom = x1, yodom = y1, thodom = z1, steps = 0.05;
    mat P;

    // Prediction phase
    for(int i = 0; i < distance/steps; i++){
        thodom = dth;
        xodom += steps*cos(thodom);
        yodom += steps*sin(thodom);

        // matrix A
        A = { {1, 0, 0},
              {0, 1, 0},
              {-distance*sin(thodom), distance*cos(thodom), 1}
            };

        P = A*Pinit*A.t() + Qalpha;
    }

    // measurement matrix, 3 measurements are enough to determine the robot position
    mat Y = { pow(xodom-landmark["l1"][0],2) + pow(yodom-landmark["l1"][1],2),
              pow(xodom-landmark["l2"][0],2) + pow(yodom-landmark["l2"][1],2),
              pow(xodom-landmark["l3"][0],2) + pow(yodom-landmark["l3"][1],2),
            };

    // state derivative
    mat C = { {2*(xodom-landmark["l1"][0]), 2*(yodom-landmark["l1"][1]), -2*steps*sin(thodom)*(xodom-landmark["l1"][0])+2*steps*cos(thodom)*(yodom-landmark["l1"][1])},
              {2*(xodom-landmark["l2"][0]), 2*(yodom-landmark["l2"][1]), -2*steps*sin(thodom)*(xodom-landmark["l2"][0])+2*steps*cos(thodom)*(yodom-landmark["l2"][1])},
              {2*(xodom-landmark["l3"][0]), 2*(yodom-landmark["l3"][1]), -2*steps*sin(thodom)*(xodom-landmark["l3"][0])+2*steps*cos(thodom)*(yodom-landmark["l3"][1])}
            };

    // gain matrix
    mat K = P*C.t()*(C*P*C.t() + Qgamma).i();

    // final P after measurement
    mat Pmeas = (eye(3,3) - K*C)*P;

    cout << endl << "Euclidian distance = " << distance << endl;
    cout << "Trace of covariance matrix = " << trace(Pmeas) << endl;

    return distance + trace(Pmeas);
}

void VisitSolver::startEKF(){
    Pinit = { {0.02,0,0}, // first row
              {0,0.02,0}, // second row
              {0,0,0.02}  // third row
            };

    Qgamma = { {0.02,0,0}, // first row
               {0,0.02,0}, // second row
               {0,0,0.02}, // third row
             };

    Qalpha = { {0.4,0,0}, // first row
               {0,0.4,0}, // second row
               {0,0,0.4}  // third row
             };
}
