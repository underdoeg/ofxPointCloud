#ifndef OFXPOINTCLOUDGRABBER_H
#define OFXPOINTCLOUDGRABBER_H

#include <libfreenect2/logger.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/common/io.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <OpenNI.h>
#include <ofMain.h>

namespace ofxPointCloud{

template<typename Type>
class Grabber: public ofThread{
public:
	Grabber(std::string path):
		cloud(new typename pcl::PointCloud<Type>()),
		cloudThread(new typename pcl::PointCloud<Type>()),
		devicePath(path),
		bFrameNew(false),
		isColor(typeid(Type) == typeid(pcl::PointXYZRGBA)){

	}

	virtual void start(){
		startThread();
	}

	//virtual void threadedFunction() override = 0 ;

	virtual void stop(){
		stopThread();
	}

	typename pcl::PointCloud<Type>::Ptr getCloud(){
		typename pcl::PointCloud<Type>::Ptr ret(new typename pcl::PointCloud<Type>());
		lock();
		pcl::copyPointCloud(*cloud, *ret);
		unlock();
		return ret;
	}

	ofPixels getPixels(){
		ofPixels ret;
		lock();
		ret = pixels;
		unlock();
		return ret;
	}

	bool isFrameNew(){
		bool ret;
		lock();
		ret = bFrameNew;
		unlock();
		return ret;
	}

	void frameHandled(){
		lock();
		bFrameNew = false;
		unlock();
	}

protected:
	typename pcl::PointCloud<Type>::Ptr cloud, cloudThread;
	ofPixels pixels, pixelsThread;
	std::string devicePath;
	bool isColor;
	bool bFrameNew;
};


///////////////////////////////////////////////////////////////////////////
///////////////////////// pcl openni2 grabber

template<typename Type>
class GrabberPclOpenNI2: public Grabber<Type>{
public:
	GrabberPclOpenNI2(std::string address):Grabber<Type>(address){};

	void threadedFunction() override{
		pcl::io::OpenNI2Grabber grabber;

		boost::function<void (const typename pcl::PointCloud<Type>::ConstPtr&)> callback = boost::bind(&GrabberPclOpenNI2<Type>::fromGrabber, this, _1);
		grabber.registerCallback(callback);

		grabber.start();

		while(this->isThreadRunning() && grabber.isRunning()){
			this->sleep(1000);
		}

		grabber.stop();
	}

private:
    void fromGrabber(const typename pcl::PointCloud<Type>::ConstPtr& other){
        this->lock();
        pcl::copyPointCloud(*other, *this->cloud);
        this->bFrameNew = true;
        this->unlock();
    }
};

///////////////////////////////////////////////////////////////////////////
///////////////////////// openni2 grabber (incomplete)

template<typename Type>
class GrabberOpenNI2: public Grabber<Type>{
public:
	GrabberOpenNI2(std::string address):Grabber<Type>(address){};

	void threadedFunction() override{
		initialize();

		openni::Device device;

		Status rc = device.open(openni::ANY_DEVICE);
		if (rc != openni::STATUS_OK){
			ofLogError("GrabberOpenNI2") << "Couldn't open device "<< openni::OpenNI::getExtendedError();
			return;
		}

		openni::VideoStream depth;

		if (device.getSensorInfo(openni::SENSOR_DEPTH) != NULL){
			rc = depth.create(device, openni::SENSOR_DEPTH);
			if (rc != openni::STATUS_OK){
				ofLogError("GrabberOpenNI2") << "Couldn't create depth stream " << openni::OpenNI::getExtendedError();
				return;
			}
		}

		openni::VideoFrameRef frame;

		while(this->isThreadRunning()){
			int changedStreamDummy;
			openni::VideoStream* streams = &depth;
			rc = openni::OpenNI::waitForAnyStream(&streams, 1, &changedStreamDummy);

			//
			if (rc != openni::STATUS_OK){
				ofLogError("GrabberOpenNI2") << "Wait failed!" << openni::OpenNI::getExtendedError();
				continue;
			}

			//
			rc = depth.readFrame(&frame);
			if (rc != openni::STATUS_OK){
				ofLogError("GrabberOpenNI2") << "Wait failed!" << openni::OpenNI::getExtendedError();
				continue;
			}

			if (frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_100_UM){
				ofLogError("GrabberOpenNI2") << "Unexpected frame format";
				continue;
			}
		}

		depth.stop();
		depth.destroy();
		device.close();
	}

private:
	static void initialize(){
		if(bInitialized)
			return;

		Status rc = openni::OpenNI::initialize();
		if (rc != openni::STATUS_OK){
			ofLogError("GrabberOpenNI2") << openni::OpenNI::getExtendedError();
			return;
		}

		bInitialized = true;
	}

	static bool bInitialized;
};

template<typename Type>
bool GrabberOpenNI2<Type>::bInitialized = false;


///////////////////////////////////////////////////////////////////////////
///////////////////////// freenect2 (colors do not work correctly)
static libfreenect2::Freenect2 freenect2;


template<typename Type>
class GrabberFreenect2: public Grabber<Type>{

public:
	GrabberFreenect2(std::string serial=""):Grabber<Type>(serial){
	}

	~GrabberFreenect2(){
	}

	void threadedFunction() override{

		//libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));

		ofLogNotice("ofxKinect2Grabber") << "Starting to grab";
		if(freenect2.enumerateDevices() == 0){
			ofLogError("ofxKinect2Grabber") << "no devices connected";
			return;
		}


		libfreenect2::PacketPipeline* pipeline;

#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
		pipeline = new libfreenect2::CudaPacketPipeline();
#else
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
		pipeline = new libfreenect2::CpuPacketPipeline();
#endif
#endif
#endif

		libfreenect2::Freenect2Device* device;

		if(Grabber<Type>::devicePath.size()>0)
			device = freenect2.openDevice(Grabber<Type>::devicePath, pipeline);
		else
			device = freenect2.openDefaultDevice(pipeline);

		if(!device){
			ofLogError("ofxKinect2Grabber") << "could not open device";
			return;
		}


		//set config
		libfreenect2::Freenect2Device::Config config;
		config.EnableBilateralFilter = true;
		config.EnableEdgeAwareFilter = true;
		config.MaxDepth = 5;
		config.MinDepth = .3;

		device->setConfiguration(config);

		libfreenect2::Freenect2Device::IrCameraParams irParams;

		//set ir parameters manually. Don't know why this is necessary. Should work without

		//#define OTHER_CALIBRATION
#ifdef OTHER_CALIBRATION
		irParams.fx = 364.7546f;
		irParams.fy = 365.5064f;
		irParams.cx = 254.0044f;
		irParams.cy = 200.9755f;
		irParams.k1 = 0.0900f;
		irParams.k2 = -0.2460f;
		irParams.k3 = 0.0566f;
		irParams.p1 = 0.0018f;
		irParams.p2 = 0.0017f;
#else
		irParams.fx = 365.456f;
		irParams.fy = 365.456f;
		irParams.cx = 254.878f;
		irParams.cy = 205.395f;
		irParams.k1 = 0.0905474;
		irParams.k2 = -0.26819;
		irParams.k3 = 0.0950862;
		irParams.p1 = 0.0f;
		irParams.p2 = 0.0f;
#endif

		device->setIrCameraParams(irParams);

		libfreenect2::Freenect2Device::ColorCameraParams colorParams;
		colorParams.fx = 1081.37;
		colorParams.fy = 1081.37;
		colorParams.cx = 959.5;
		colorParams.cy = 539.5;

		device->setColorCameraParams(colorParams);

		ofLogNotice("ofxKinect2Grabber") << "Opened device with serial number " << device->getSerialNumber();

		//REGISTRATION
		prepareMatrices(device->getIrCameraParams());
		libfreenect2::Registration registration(device->getIrCameraParams(), device->getColorCameraParams());
		libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

		int frameTypes = 0;

		frameTypes |= libfreenect2::Frame::Ir | libfreenect2::Frame::Type::Depth;

		if(this->isColor){
			frameTypes |= libfreenect2::Frame::Type::Color;
		}

		//add listener and start stream
		libfreenect2::FrameMap frames;
		libfreenect2::SyncMultiFrameListener listener(frameTypes);
		device->setIrAndDepthFrameListener(&listener);

		if(this->isColor)
			device->setColorFrameListener(&listener);

		if(!device->startStreams(this->isColor, true)){
			ofLogError("ofxKinect2Grabber") << "could not start stream";
			return;
		}

		//run thread
		while(Grabber<Type>::isThreadRunning()){
			//hold thread until new frames come in
			listener.waitForNewFrame(frames);

			//retreive frames
			libfreenect2::Frame* rgb;
			if(this->isColor)
				rgb = frames[libfreenect2::Frame::Color];
			libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

			//allocate and undistort
			if(this->isColor){
				registration.apply(rgb, depth, &undistorted, &registered);
			}else{
				registration.undistortDepth(depth, &undistorted);
			}

			//copy image pixels
			if(this->isColor){
				libfreenect2::Frame* imgFrame = &registered;
				int w = imgFrame->width;
				int h = imgFrame->height;
				ofPixelFormat format = OF_PIXELS_BGRA;
				if(imgFrame->format == libfreenect2::Frame::RGBX)
					format = OF_PIXELS_RGBA;
				this->pixelsThread.setFromPixels(imgFrame->data, imgFrame->width, imgFrame->height, format);
			}

			//release the frame
			listener.release(frames);


			unsigned totalSize = undistorted.width * undistorted.height;

			if(this->cloudThread->size() != totalSize){
				this->cloudThread->resize(totalSize);
				this->cloudThread->width = undistorted.width;
				this->cloudThread->height = undistorted.height;
			}

			//convert
			static const float badPoint = std::numeric_limits<float>::quiet_NaN();
			bool isDense = true;
			Type* p = &this->cloudThread->points[0];
			float px, py, pz, color;
			for(unsigned y=0; y<undistorted.height; y++){
				for(unsigned x=0; x<undistorted.width; x++){

					if(!this->isColor)
						registration.getPointXYZ(&undistorted, y, x, px, py, pz);
					else
						registration.getPointXYZRGB(&undistorted, &registered, y, x, px, py, pz, color);

					Type& p = this->cloudThread->at(x, y);

					//check if point is valid

					if(std::isnan(pz) || pz <= 0.001){
						p.x = p.y = p.z = badPoint;
						isDense = false;
						continue;
					}

					p.x = px;
					p.y = py;
					p.z = pz;

					//cast so we dont need another function for this
					if(this->isColor){
						pcl::PointXYZRGBA& pc = (pcl::PointXYZRGBA&)p;
						char data[4];
						data[0] = (int) color >> 24;
						data[1] = (int) color >> 16;
						data[2] = (int) color >> 8;
						data[3] = (int) color;
						//std::cout << (int)data[2] << " - ";
						//pc.a = 255;
					}
				}
			}

			//ofLog() << "ASD";

			this->cloudThread->is_dense = isDense;


			//
			//now copy into threaded
			this->lock();
			pcl::copyPointCloud(*this->cloudThread, *this->cloud);
			if(this->isColor){
				this->pixels.setFromPixels(this->pixelsThread.getData(), this->pixelsThread.getWidth(), this->pixelsThread.getHeight(), this->pixelsThread.getPixelFormat());
			}
			this->bFrameNew = true;
			this->unlock();
		}

		device->stop();
		device->close();
		delete device;
	}

private:
	void prepareMatrices(const libfreenect2::Freenect2Device::IrCameraParams & depth_p){
		const int w = 512;
		const int h = 424;
		float * pm1 = colmap.data();
		float * pm2 = rowmap.data();
		for(int i = 0; i < w; i++){
			*pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
		}for (int i = 0; i < h; i++){
			*pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
		}
	}

	Eigen::Matrix<float,512,1> colmap;
	Eigen::Matrix<float,424,1> rowmap;
public:

};

}

#endif // OFXPOINTCLOUDGRABBER_H
