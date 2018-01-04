/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */
#ifndef _HALO_VOXEL_H_
#define _HALO_VOXEL_H_

#include "CameraSystem.h"

//#include "SimpleOpt.h"
#include "Common.h"
//#include "Logger.h"
#include <iomanip>
#include <fstream>
#include <ostream>

#include <thread>
#include <thread>                // std::thread
#include <mutex>                // std::mutex, std::unique_lock
#include <condition_variable>    // std::condition_variable
#include <queue>
#include <unistd.h>

using namespace Voxel;
using namespace std;


int cameraInit();
//james
void setCallback();
int printOutFrameInfo(float *xyziPointFrame, short *phaseFrame);
int cameraStop();


#endif	/* _KDTREE_H_ */
