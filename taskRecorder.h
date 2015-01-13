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

#ifndef RECORD_ARMS_H
#define RECORD_ARMS_H

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <vector>
#include <deque>
#include <sstream>
#include <string>
#include <stdio.h>
#include <fstream>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

 using namespace iCub::ctrl;

//
//#define PORT_BOTTLE 0
//#define PORT_VECTOR 1

enum thread_status {RECORDING = 0, IDLE=1 };




class ArmsRecorder: public yarp::os::RateThread
{
private:

    // interface with icub
    yarp::dev::IEncoders            *iencTO, *iencRA, *iencLA;
    yarp::dev::ICartesianControl    *icrtRA, *icrtLA;
    yarp::dev::PolyDriver           *ddTO, *ddRA, *ddLA;
    yarp::dev::PolyDriver           *ddCartRA, *ddCartLA;
    yarp::os::Property              optionsTO, optionsLA, optionsRA, optionsCartRA, optionsCartLA;
    std::string robot;
    //std::string part;
    std::string name;

    // for recording
    // to tqke the current values
    yarp::sig::Vector               cur_x_RA, cur_x_LA, 
                                    cur_o_RA,cur_o_LA,
                                    cur_q_TO, cur_q_RA, cur_q_LA,
                                    tmp;
    // to take the new values via filtering
    yarp::sig::Vector               cur_xd_RA, cur_xd_LA, 
                                    cur_od_RA, cur_od_LA,
                                    cur_qd_TO, cur_qd_RA, cur_qd_LA;

    //std::string                     object;
    std::deque<yarp::sig::Vector>   recordedTraj_RA, recordedTraj_LA, recordedTraj_TO;
    std::stringstream descriptor;
    std::deque<double>              recordedTraj_timestamp;       
    bool firstRec;
    std::ofstream fout;
    std::string fileName;

    thread_status status;
    double startTime;
    double oldTime;
    int counter;
    double curTime;
    double actualTime;

    // filters
    iCub::ctrl::AWLinEstimator      *filter_xd_L, *filter_od_L, *filter_qd_L,
                                    *filter_xd_R, *filter_od_R, *filter_qd_R,
                                    *filter_qd_T;

public:

    ArmsRecorder(std::string moduleName, std::string robotName);
    ~ArmsRecorder();

    bool threadInit();
    inline thread_status getThreadStatus();

    void run();
    void threadRelease();

    // set the file and the object
    void setRecording(std::string outFile);
    // writes on the file: this must be called after stopRecording
    bool flushRecording();

    // the main functions used to regulate recording phase
    // just start recording (enables in the run)
    bool startRecording();
    // stop recording and retrieve the elapsed time for the last recording
    double stopRecording();

    // estimate velocities
    void filterEstimation();

};



#endif
