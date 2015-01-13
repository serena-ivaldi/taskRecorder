/*
 * Copyright (C) 2012 MACSi Project
 * Author: Serena Ivaldi
 * email:  serena.ivaldi@isir.upmc.fr
 * website: www.macsi.isir.upmc.fr
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include <iostream>
#include <sstream>

#include "taskRecorder.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

// utils for printing parameters
#define DSCP(V,oss) V; oss<< #V <<endl;
#define DSCPI(V,oss,i) V; oss<< #V <<"  i="<<i<<endl;
#define DSCPS(V,oss) oss<< #V <<endl;



////////////////////////////////////////
////////////////////////////////////////

//          ARMS RECORDER

////////////////////////////////////////
////////////////////////////////////////

//-----------------------------------------------------------
ArmsRecorder::ArmsRecorder(string moduleName, string robotName):RateThread(20)
{
    counter = 0;
    ddTO=ddRA=ddLA=0; ddCartRA=ddCartLA=0;
    robot=robotName;
    //part=armPart;

    name=moduleName;

    status = IDLE;
    startTime=Time::now();
    fileName="object.txt";

    // useful stuff for recording
    cur_x_RA.resize(3,0.0); cur_x_LA=cur_x_RA;
    cur_o_RA.resize(4,0.0); cur_o_LA=cur_x_LA;
    
    cur_q_RA.resize(15,0.0);
    cur_q_LA.resize(15,0.0);
    cur_q_TO.resize(3,0.0);

    recordedTraj_RA.clear();
    recordedTraj_LA.clear();
    recordedTraj_TO.clear();
    tmp.clear();

    descriptor.clear();

}

//-----------------------------------------------------------
ArmsRecorder::~ArmsRecorder(){}

//-----------------------------------------------------------
thread_status ArmsRecorder::getThreadStatus()
{
    return status;
}

//-----------------------------------------------------------
bool ArmsRecorder::threadInit()
{
    //now connect to the robot
    bool dderror = false;


    // RIGHT ARM: joint
    optionsRA.put("device","remote_controlboard");
    optionsRA.put("local",string("/"+name+"/right_arm").c_str());
    optionsRA.put("remote",string("/"+robot+"/right_arm").c_str());
    ddRA = new PolyDriver;


    // LEFT ARM: joint + cartesian
    optionsLA.put("device","remote_controlboard");
    optionsLA.put("local",string("/"+name+"/left_arm").c_str());
    optionsLA.put("remote",string("/"+robot+"/left_arm").c_str());
    ddLA = new PolyDriver;

    // TORSO: joint only
    optionsTO.put("device","remote_controlboard");
    optionsTO.put("local",string("/"+name+"/torso").c_str());
    optionsTO.put("remote",string("/"+robot+"/torso").c_str());
    ddTO = new PolyDriver;

        // CHECK DD
    if(!ddRA->open(optionsRA))
    {
        cout<<"TaskRecorder: problems connecting to the remote driver of RIGHT ARM" << endl;
        dderror = true;
    }
    if(!ddLA->open(optionsLA))
    {
        cout<<"TaskRecorder: problems connecting to the remote driver of LEFT ARM" << endl;
        dderror = true;
    }
    if(!ddTO->open(optionsTO))
    {
        cout<<"TaskRecorder: problems connecting to the remote driver of TORSO" << endl;
        dderror = true;
    }
    if(!ddRA->view(iencRA) )
    {
        cout<<"TaskRecorder: problem in acquiring interfaces, aborting thread for RIGTH ARM"<<endl;
        dderror = true;
    }
    if(!ddLA->view(iencLA) )
    {
        cout<<"TaskRecorder: problem in acquiring interfaces, aborting thread for LETF ARM"<<endl;
        dderror = true;
    }
    if(!ddTO->view(iencTO) )
    {
        cout<<"TaskRecorder: problem in acquiring interfaces, aborting thread for TORSO"<<endl;
        dderror = true;
    }


    if (dderror)
    {
        delete ddTO;
        delete ddRA;
        delete ddLA;
        return false;
    }


    // RA now connect to cartesian interfaces
    optionsCartRA.put("device","cartesiancontrollerclient");
    optionsCartRA.put("remote",string("/"+robot+"/cartesianController/right_arm").c_str());
    optionsCartRA.put("local",string("/"+name+"/right_arm/cartesian").c_str());
    ddCartRA = new PolyDriver;


    // LA now connect to cartesian interfaces
    optionsCartLA.put("device","cartesiancontrollerclient");
    optionsCartLA.put("remote",string("/"+robot+"/cartesianController/left_arm").c_str());
    optionsCartLA.put("local",string("/"+name+"/left_arm/cartesian").c_str());
    ddCartLA = new PolyDriver;




    if (!ddCartRA->open(optionsCartRA))
    {
        cout<<"TaskRecorder: problems connecting to the Cartesian interface of RIGTH ARM"<<endl;
        dderror = true;
    }
    if (!ddCartLA->open(optionsCartLA))
    {
        cout<<"TaskRecorder: problems connecting to the Cartesian interface of LEFT ARM"<<endl;
        dderror = true;
    }
    if(!ddCartRA->isValid())
    {
        cout<<"TaskRecorder: invalid Cartesian interface for RIGTH ARM" <<endl;
        dderror = true;
    }
    if(!ddCartLA->isValid())
    {
        cout<<"TaskRecorder: invalid Cartesian interface for LEFT ARM" <<endl;
        dderror = true;
    }
    if(!ddCartRA->view(icrtRA))
    {
        cout<<"TaskRecorder: problem in acquiring Cartesian interface, aborting thread"<<endl;
        dderror = true;
    }
    if(!ddCartLA->view(icrtLA))
    {
        cout<<"TaskRecorder: problem in acquiring Cartesian interface, aborting thread"<<endl;
        dderror = true;
    }
    if (dderror)
    {
        delete ddCartRA;
        delete ddCartLA;
        return false;
    }

    // initializing filters
    // left arm
    filter_xd_L = new AWLinEstimator(16,1.0);
    filter_od_L = new AWLinEstimator(16,1.0);
    filter_qd_L = new AWLinEstimator(16,1.0);

    // right arm
    filter_xd_R = new AWLinEstimator(16,1.0);
    filter_od_R = new AWLinEstimator(16,1.0);
    filter_qd_R = new AWLinEstimator(16,1.0);

    // torso
    filter_qd_T = new AWLinEstimator(16,1.0);

    //
    status = IDLE;
    startTime=Time::now();
    cout<<"ArmsRecorder is ready"<<endl;
    return true;
}

//-----------------------------------------------------------
void ArmsRecorder::threadRelease()
{
    cout<<"Closing drivers"<<endl;

    if(filter_xd_L){ delete filter_xd_L;filter_xd_L = 0;}
    if(filter_od_L){ delete filter_od_L;filter_od_L = 0;}
    if(filter_qd_L){ delete filter_qd_L;filter_qd_L = 0;}
    if(filter_xd_R){ delete filter_xd_R;filter_xd_R = 0;}
    if(filter_od_R){ delete filter_od_R;filter_od_R = 0;}
    if(filter_qd_R){ delete filter_qd_R;filter_qd_R = 0;}
    if(filter_qd_T){ delete filter_qd_T;filter_qd_T = 0;}

    delete ddRA;
    delete ddLA;
    delete ddTO;
    delete ddCartRA;
    delete ddCartLA;
    ddRA = NULL;
    ddLA = NULL;
    ddTO = NULL;
    ddCartRA = NULL;
    ddCartLA = NULL;
    cout<<"Closing ArmsRecorder"<<endl;
}

//-----------------------------------------------------------
void ArmsRecorder::run()
{
    //cout<<curTime<<endl;

    // in any case, update the filters for estimating velocities 
    //
    icrtRA->getPose(cur_x_RA, cur_o_RA);
    iencRA->getEncoders(cur_q_RA.data());
    //
    icrtLA->getPose(cur_x_LA,cur_o_LA);
    iencLA->getEncoders(cur_q_LA.data());
    //
    iencTO->getEncoders(cur_q_TO.data());

    filterEstimation();


    if(status==RECORDING)
    {
        //collect datas from ports
        if (counter == 0)
        {
            startTime = Time::now ();
        }

        curTime = Time::now();
        actualTime = curTime - startTime;
       
        // save them somewhere (they'll be written when we stop recording)
        int c=0;
        descriptor.clear();
        descriptor.str("");

        // saving data: time endeff=(xyz o1234) fingers=(q7..15)

        recordedTraj_timestamp.push_back (actualTime);
        descriptor << "actualTime" <<endl;
        
        printf("\nRecording time (ms)        \r%f \n",actualTime);

        //cout << "  Recording  " << actualTime << endl;
 
        //=================== RA
        c = 0;
        tmp.clear();
        tmp.resize(7+16+7+16);
        descriptor<< "Rigth Arm" <<endl;

        tmp[c++] = DSCP(cur_x_RA[0],descriptor);
        tmp[c++] = DSCP(cur_x_RA[1],descriptor);
        tmp[c++] = DSCP(cur_x_RA[2],descriptor);
        tmp[c++] = DSCP(cur_o_RA[0],descriptor);
        tmp[c++] = DSCP(cur_o_RA[1],descriptor);
        tmp[c++] = DSCP(cur_o_RA[2],descriptor);
        tmp[c++] = DSCP(cur_o_RA[3],descriptor);
        for(int i=0; i<=15;i++)
        {
            tmp[c++] = DSCPI(cur_q_RA[i],descriptor,i);
        }
        tmp[c++] = DSCP(cur_xd_RA[0],descriptor);
        tmp[c++] = DSCP(cur_xd_RA[1],descriptor);
        tmp[c++] = DSCP(cur_xd_RA[2],descriptor);
        tmp[c++] = DSCP(cur_od_RA[0],descriptor);
        tmp[c++] = DSCP(cur_od_RA[1],descriptor);
        tmp[c++] = DSCP(cur_od_RA[2],descriptor);
        tmp[c++] = DSCP(cur_od_RA[3],descriptor);
        for(int i=0; i<=15;i++)
        {
            tmp[c++] = DSCPI(cur_qd_RA[i],descriptor,i);
        }
        recordedTraj_RA.push_back(tmp);
        

        //=================== LA
        c = 0;
        tmp.clear();
        tmp.resize(7+16+7+16);
        descriptor<< "Left Arm" <<endl;

        tmp[c++] = DSCP(cur_x_LA[0],descriptor);
        tmp[c++] = DSCP(cur_x_LA[1],descriptor);
        tmp[c++] = DSCP(cur_x_LA[2],descriptor);
        tmp[c++] = DSCP(cur_o_LA[0],descriptor);
        tmp[c++] = DSCP(cur_o_LA[1],descriptor);
        tmp[c++] = DSCP(cur_o_LA[2],descriptor);
        tmp[c++] = DSCP(cur_o_LA[3],descriptor);
        for(int i=0; i<=15;i++)
        {
            tmp[c++] = DSCPI(cur_q_LA[i],descriptor,i);
        }
        tmp[c++] = DSCP(cur_xd_LA[0],descriptor);
        tmp[c++] = DSCP(cur_xd_LA[1],descriptor);
        tmp[c++] = DSCP(cur_xd_LA[2],descriptor);
        tmp[c++] = DSCP(cur_od_LA[0],descriptor);
        tmp[c++] = DSCP(cur_od_LA[1],descriptor);
        tmp[c++] = DSCP(cur_od_LA[2],descriptor);
        tmp[c++] = DSCP(cur_od_LA[3],descriptor);
        for(int i=0; i<=15;i++)
        {
            tmp[c++] = DSCPI(cur_qd_LA[i],descriptor,i);
        }
        recordedTraj_LA.push_back(tmp);


        //=================== TO
        c = 0;
        tmp.clear();
        tmp.resize(3+3);
        descriptor<< "Torso" <<endl;
        for(int i=0; i<3;i++)
        {
            tmp[c++] = DSCPI(cur_q_TO[i],descriptor,i);
        }
        for(int i=0; i<3;i++)
        {
            tmp[c++] = DSCPI(cur_qd_TO[i],descriptor,i);
        }
        recordedTraj_TO.push_back(tmp);

        counter++;
    }
    else
    {
        /*
        if(curTime-startTime) > 60)
        {
            cout<<"ArmsRecorder: waiting alive "<<endl;
        }
        */
        Time::delay(1.0);
    }
}

//-----------------------------------------------------------
bool ArmsRecorder::startRecording()
{
    if(status==RECORDING)
    {
        cout<<"ArmsRecorder is already recording. Error."<<endl;
        return false;
    }
    counter=0;
    startTime = Time::now();
    recordedTraj_timestamp.clear ();
    status = RECORDING;  
    return true;
}

//-----------------------------------------------------------
double ArmsRecorder::stopRecording()
{
    if(status==IDLE)
    {
        cout<<"ArmsRecorder is not recording, why do you want to stop it? Error."<<endl;
        return -1.0;
    }
    status = IDLE;
    return (Time::now() - startTime);
}

//-----------------------------------------------------------
bool ArmsRecorder::flushRecording()
{
    if(status==RECORDING)
    {
        cout<<"ArmsRecorder will only write to file when the recording is over. Error."<<endl;
        return false;
    }

    int totalRec = recordedTraj_RA.size();

    ofstream out;
    out.open(fileName.c_str());
    if(!out)
    {
        cout<<"TaskRecorder can't open in write mode the trajectory file: "<<fileName<<endl
            <<"Aborting."<<endl;
        return false;
    }
    else
        cout<<"ArmsRecorder: saving "<<totalRec<<" items on "<<fileName<<endl;

    tmp.clear();

    for(int i=0; i<totalRec;i++)
    {
        out << recordedTraj_timestamp.front () << " ";
        out << recordedTraj_RA.front().toString () << " ";
        out << recordedTraj_LA.front ().toString () << " ";
        out << recordedTraj_TO.front ().toString () << " ";
        out << endl;

        recordedTraj_timestamp.pop_front ();
        recordedTraj_RA.pop_front();
        recordedTraj_LA.pop_front();
        recordedTraj_TO.pop_front();
    }

    recordedTraj_RA.clear();
    recordedTraj_LA.clear();
    recordedTraj_TO.clear();
    
    tmp.clear();
    out.close();

    out.open(string(fileName+".info").c_str());
    if(!out)
    {
        cout<<"ArmsRecorder: there were errors saving the information file: "<<string(fileName+".info")<<endl
            <<"Aborting."<<endl;
        return false;
    }
    else
        cout<<"ArmsRecorder: saving info file on "<<string(fileName+".info")<<endl;

    out<<descriptor.str()<<endl;
    out.close();

    return true;

}

//----------------------------------------------------------
void ArmsRecorder::setRecording(string outFile)
{
    fileName = outFile;
    cout << "ArmsRecorder will save object in file=" << fileName << endl;
} 


//-----------------------------------------------------------
void ArmsRecorder::filterEstimation()
{

    AWPolyElement el;
    el.time=Time::now();

    // LA
    el.data = cur_x_LA;
    cur_xd_LA = filter_xd_L->estimate(el);

    el.data = cur_o_LA;
    cur_od_LA = filter_od_L->estimate(el);

    el.data = cur_q_LA;
    cur_qd_LA = filter_qd_L->estimate(el);

    // RA
    el.data = cur_x_RA;
    cur_xd_RA = filter_xd_R->estimate(el);

    el.data = cur_o_RA;
    cur_od_RA = filter_od_R->estimate(el);

    el.data = cur_q_RA;
    cur_qd_RA = filter_qd_R->estimate(el);

    //TO
    el.data = cur_q_TO;
    cur_qd_TO = filter_qd_T->estimate(el);

}



