/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */
#include "DepthEyeH1Sample.h"
using namespace Voxel;
using namespace std;

std::mutex img_mtx;
std::condition_variable img_cond_var;
bool stopStreaming = false;
char rawFrameQueue[42496 * 2];
char depthFrameQueue[4800 * sizeof(short)];
char XYZpointCloutQueue[4800 * sizeof(float) * 4];

int cameraInit(){
	logger.setDefaultLogLevel(LOG_INFO);
	CameraSystem sys;

	// Get all valid detected devices
	const Vector<DevicePtr> &devices = sys.scan();
    
	DevicePtr toConnect;

	std::cout << "Detected devices: " << std::endl;
	for (auto &d: devices){
		std::cout << d->id() << std::endl;
		toConnect = d;
	}

	if (!toConnect) {
		logger(LOG_ERROR)<< "No valid device found for the specified"<< std::endl;
		return -1;
	}

	DepthCameraPtr depthCamera = sys.connect(toConnect);

	if (!depthCamera) {
		logger(LOG_ERROR) << "Could not load depth camera for device "<< toConnect->id() << std::endl;
		return -1;
	}

	if (!depthCamera->isInitialized()) {
		logger(LOG_ERROR) << "Depth camera not initialized for device "<< toConnect->id() << std::endl;
		return -1;
	}
	unsigned int scratch1 = 0;
	bool scratch1_value = depthCamera->get("scratch1", scratch1, true);
	depthCamera->set("stanby", true);
	if (scratch1_value &&scratch1>0)
	{
		std::cout << "reseting camera"<< std::endl;
		depthCamera->stop();
		depthCamera->reset();
		sys.disconnect(depthCamera);
		depthCamera = nullptr;
		depthCamera = sys.connect(toConnect);
		depthCamera->refreshParams();
	}

	std::cout << "Successfully loaded depth camera for device "<< toConnect->id() << std::endl;

	depthCamera->registerCallback(DepthCamera::FRAME_RAW_FRAME_PROCESSED,
			[&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {

			const ToFRawFrame *d = dynamic_cast<const ToFRawFrame *>(&frame);

			if(!d)
			{
				std::cout << "Null frame captured? or not of type ToFRawFrame" << std::endl;
				return;
			}
			//pushing file to queue
		     std::lock_guard<std::mutex> guard(img_mtx);
		     memcpy((char *)depthFrameQueue, (char *)d->phase(), sizeof(short) * d->size.width * d->size.height);
		     //************data format****************
		     //phase data:     (char *)d->phase(), sizeof(short)*d->size.width*d->size.height
		     //amplitude data: (char *)d->phase(), sizeof(short)*d->size.width*d->size.height
		     //ambient data:   (char *)d->ambient(), sizeof(char)*d->size.width*d->size.height
		     //flags data:     (char *)d->flags(), sizeof(char)*d->size.width*d->size.height)
		});

	depthCamera->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, 
			[&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
	 		const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
	 		if(!d){
	 			std::cout << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
	 			return;
	 		}
	 		//pushing file to queue
	 		std::lock_guard<std::mutex> guard(img_mtx);
	 		memcpy((char *)XYZpointCloutQueue, (char *)d->points.data(), sizeof(IntensityPoint)*d->points.size());
	 		img_cond_var.notify_all();
	 		if(stopStreaming == true){
	 			dc.stop();
	 		}
	 	});
	//***************end of call back functions.....

	FrameRate r;

	if (depthCamera->start()) {
		if (depthCamera->getFrameRate(r)){
			logger(LOG_INFO) << "Capturing at a frame rate of "<< r.getFrameRate() << " fps" << std::endl;
		}

		depthCamera->wait();
	} else{
		logger(LOG_ERROR) << "Could not start the depth camera "<< depthCamera->id() << std::endl;
	}
	std::cout << "Capturing stop..." << std::endl;
	depthCamera->reset();
	sys.disconnect(depthCamera);
	return 0;
}

int printOutFrameInfo(float *xyziPointFrame, short *phaseFrame){
	std::unique_lock<std::mutex> unlock(img_mtx);

	while((img_cond_var.wait_for(unlock, std::chrono::seconds(1))) == std::cv_status::timeout){
		std::cout << "Halo H1 Depth API: waiting for data..." << std::endl;
	}
	char* depthOutput = depthFrameQueue;
	memcpy(phaseFrame, depthOutput, sizeof(short) * 4800);


	char* xyzOutput = XYZpointCloutQueue;
	memcpy((char*)xyziPointFrame, (char*)xyzOutput, sizeof(float)*4800*4);
	printf("[x=%f, y=%f, z=%f, i=%f]\n", xyziPointFrame[2440], 
				xyziPointFrame[2441], xyziPointFrame[2442], xyziPointFrame[2443]);
	
	return 0;
}

int cameraStop(){
	stopStreaming = true;
	return 0;
}

int main(int argc, char const *argv[])
{
	float xyziPointFrame[60*80*4];
	short phaseFrame[60*80];
	std::cout << "Starting Halo H1 Depth Camera" << std::endl;
	std::thread CameraThread(cameraInit);

	CameraThread.detach();
	int total_capture_frames = 500;
	for (int i = 0; i < total_capture_frames; ++i)
	{
		printf("getting frame[%d]\n", i);
		printOutFrameInfo(xyziPointFrame, phaseFrame);
		usleep(30000);//capturing at about 30fps
	}
	cameraStop();
	for (int i = 0; i < 3; ++i)
	{
		sleep(1);//waiting background thread to stop
	}
	std::cout << "camera thread finished" << std::endl;

	return 0;
}

