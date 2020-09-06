#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#define WIN "diff"

using namespace std;
using namespace cv;

enum Processor { cl, gl, cpu };

void onMouse(int event, int x, int y, int flags, void* param);

//2.定义回调函数
//event是时间
void onMouse(int event, int x, int y, int flags, void* param)
{
	Mat *im = reinterpret_cast<Mat*>(param);  //指针类型的强制转换
	//Mat im = *(Mat*)param; 强转加解引用
	Mat dstImg;
	switch (event)
	{
	case CV_EVENT_LBUTTONDOWN:
		std::cout << "at(" << x << "," << y << ")value is:"
			<< static_cast<int>(im->at<uchar>(cv::Point(x, y))) << std::endl;
		break;
	}
}


int main(int argc, char** argv)
{
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = nullptr;
	libfreenect2::PacketPipeline *pipeline = nullptr;
	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}
	string serial = freenect2.getDefaultDeviceSerialNumber();
	if (serial == "")  return -1;
	cout << "The serial number is :" << serial << endl;

	int depthProcessor = Processor::cl;
	if (depthProcessor == Processor::cpu)
	{
		if (!pipeline)
		{
			pipeline = new libfreenect2::CpuPacketPipeline();
		}
	}
	else
		if (depthProcessor == Processor::gl)
		{
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		if (!pipeline)
			pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
		std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
		}
		else
			if (depthProcessor == Processor::cl)
			{
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		if (!pipeline)
			pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
		std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
			}

	if (pipeline)
	{
		dev = freenect2.openDevice(serial, pipeline);
	}
	else
	{
		dev = freenect2.openDevice(serial);
	}
	if (dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	//! [listeners]
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
	libfreenect2::FrameMap frames;
	dev->setIrAndDepthFrameListener(&listener);

	dev->start();
	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	//! [start]
	Mat depthmat, depthmatUndistorted, rgbd, rgbd2;

	//----------------视觉检测部分---------------------
	Mat diffresult; //定义去除背景后的结果图
	Mat bgsrc; //定义背景图

	Mat gray;

	//-----跟踪参数-----

	for (int p = 0;; p++)
	{
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		//! [loop start]
		Mat depthmat = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
		Mat a = depthmat / 4500.0f;

		a.convertTo(a, CV_8UC1, 255.0);

		imshow("srcvideo", a);//相机原图像
		
		setMouseCallback("srcvideo", onMouse, &a);  //第三个参数void* 用户数据

		listener.release(frames);

		if (waitKey(1) == 27)
			break;

	}
	dev->stop();
	dev->close();

	return 0;
}
