/* 
 * Copyright (C) 2015 CODYCO Project
 * Author: Serena Ivaldi <serena.ivaldi@inria.fr>
 * website: www.codyco.eu
 *
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
//
//#include <gtk/gtk.h>
//
//#include <macsi/modHelp/modHelp.h>
//#include <macsi/objects/objects.h>
//
#include <string>
#include <iostream>
//
#include "taskRecorder.h"

// necessary for cartesian interfaces
YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
//using namespace macsi::modHelp;
//using namespace macsi::objects;
using namespace std;


// recording status
#define STATUS_IDLE         0
#define STATUS_RECORDING    1
#define VALIDATION_REQUIRED 2



//==> from modHelp

#define displayValue(V) cout<<" "<< #V <<" : "<<V<<endl;
#define displayNameValue(S,V) cout<<" "<< S <<" : "<<V<<endl;
#define displayVector(V) cout<<" "<< #V <<" : "<<V.toString()<<endl;
#define displayNameVector(S,V) cout<<" "<< S <<" : "<<V.toString()<<endl;

 void readString(ResourceFinder &rf, string name, string &v, string vdefault)
{
if(rf.check(name.c_str()))
{
v = rf.find(name.c_str()).asString();
}
else
{
v = vdefault;
cout<<"Could not find value for "<<name<<". "
<<"Setting default "<<vdefault<<endl;
}
displayNameValue(name,v);
}


void readInt(ResourceFinder &rf, string name, int &v, int vdefault)
{
if(rf.check(name.c_str()))
{
v = rf.find(name.c_str()).asInt();
}
else
{
v = vdefault;
cout<<"Could not find value for "<<name<<". "
<<"Setting default "<<vdefault<<endl;
}
displayNameValue(name,v);
}




//==>end






// the status of the process: recording or idle
int status;
// thread rate
int rate;
// the name of the module (for ports)
string moduleName;
// the name of the file used to save the datas (without extension)
string fileName;
// the path where to save files
string pathFiles;
// the robot name
string robotName;
// the arm type
//string armType;

// the number of trials to do for the current session
int nbFiles;
// the number of trials done so far in the current session
int currentFile;
// the file where to record
string fileTrajectory;


// UTILS
//==================================================================
string i2s(int n)
{
    char buff [10];
    sprintf(buff,"%d",n);
    return string(buff);
}





// basic class module, just to open the quit port
class ManagerModule: public RFModule
{
protected:

    ArmsRecorder *taskrecorder;
    Port           rpcPort;
    int count;

public:
    ManagerModule() { }

    virtual bool configure(ResourceFinder &finder)
    {
        Time::turboBoost();
        count=0;

        pathFiles=finder.getHomeContextPath();
        cout<<"Writing to path = "<<pathFiles<<endl;

        readString(finder,"file",fileName,"recordTask.txt");
        readString(finder,"robot",robotName,"icubSim");
        readInt(finder,"rate",rate,10);
        readString(finder,"name",moduleName,"recordArms");


        // create the recorder
        taskrecorder = new ArmsRecorder(moduleName,robotName);
        if (!taskrecorder->start())
        {
            cout<<"recordArms: could not start the thread... aborting"<<endl;
            delete taskrecorder;
            return false;
        }

        rpcPort.open(string("/"+moduleName+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        taskrecorder->stop();
        delete taskrecorder;

        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }
    virtual bool updateModule()
    {
        if(count==0)
            cout<<"To send commands, open a rpc port: \n"
                <<"      yarp rpc --client /myPort \n"
                <<"Then connect with: \n"
                <<"      yarp connect /myPort /recordArms/rpc"<<endl;

        if(count%60==0)
            cout<<" recordArms module alive since "<<(count/60)<<" mins ... "<<endl;

        count++;
        return true;
    }

    // message handler
    bool respond(const Bottle& command, Bottle& reply)
    {
        ConstString cmd = command.get(0).asString();
        cout<<"first command = "<<cmd<<endl;

        if (cmd=="quit")
            return false;

        if(cmd=="STOP_REC")
        {
            double time_rec;
            // stop recording, and save to file
            time_rec = taskrecorder->stopRecording();
            taskrecorder->flushRecording();

            reply.clear();
            reply.addString("ACK");
            reply.addString("STOP_REC");
            reply.addDouble(time_rec);
            return true;
        }

        if (cmd=="START_REC")
        {
            ConstString fileName = command.get(1).asString();

            // set the parameters for saving the trajectory , then start recording
            taskrecorder->setRecording(fileName.c_str());
            taskrecorder->startRecording();

            reply.clear();
            reply.addString("ACK");
            reply.addString("START_REC");
            return true;
        }

        // if we're here, nothing is known
        reply.clear();
        reply.addString("ERR");
        return true;

     }
};



//=================================================================
//                     MAIN
//=================================================================

int main(int argc, char * argv[])
{
    YARP_REGISTER_DEVICES(icubmod)

    //initialize yarp network
    Network yarp;

    // create device drivers and connect to icub
    if (!yarp.checkNetwork())
    {
        cout<<"YARP network not available. Aborting."<<endl;
        return -1;
    }
	
    ResourceFinder finder;
    //retrieve information for the list of parts
    finder.setVerbose(true);
    finder.setDefaultContext("taskRecorder");
    finder.setDefaultConfigFile("taskRecorder.ini");
    finder.configure(argc,argv);

    if (finder.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--from   fileName: input configuration file" << endl;
        cout << "\t--context dirName: resource finder context"  << endl;

        return 0;
    }

    ManagerModule module;
    module.runModule(finder);


    return 0;

}


