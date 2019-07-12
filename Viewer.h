/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#ifndef VIEWER_H
#define VIEWER_H

#include <OpenNI.h>
#include <vector>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/video/background_segm.hpp"
#include "usbController.h"
#include <fstream>

#define MAX_DEPTH 10000

enum DisplayModes
{
	DISPLAY_MODE_OVERLAY,
	DISPLAY_MODE_DEPTH,
	DISPLAY_MODE_IMAGE
};

class SampleViewer
{
public:
	SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color);
	virtual ~SampleViewer();

	virtual openni::Status init(int argc, char **argv);
	virtual openni::Status run();	//Does not return
    double getVolume(const openni::DepthPixel *pixel);

    void getAvgVolume();

protected:
	virtual void display();
	virtual void displayPostDraw(){};	// Overload to draw over the screen image

	virtual void onKey(unsigned char key, int x, int y);

	virtual openni::Status initOpenGL(int argc, char **argv);
	void initOpenGLHooks();

	openni::VideoFrameRef		m_depthFrame;
	openni::VideoFrameRef		m_colorFrame;

	openni::Device&			m_device;
	openni::VideoStream&			m_depthStream;
	openni::VideoStream&			m_colorStream;
	openni::VideoStream**		m_streams;

private:
	SampleViewer(const SampleViewer&);
	SampleViewer& operator=(SampleViewer&);

    openni::Status initPort();

    void getsubDepths(const openni::DepthPixel *pixel);

    void convertDepthPixelToMat(const openni::DepthPixel *pixel, cv::Mat &src);

    void initBgModel();

	static SampleViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();
	static void glutKeyboard(unsigned char key, int x, int y);

    static void glutTimer(int i);

	float			m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;
	DisplayModes		m_eViewState;
    openni::RGB888Pixel *m_pTexMap;
	int			m_width;
	int			m_height;
    cv::Ptr<cv::BackgroundSubtractorMOG2> bgsubtractor;
    cv::Mat frame, foreground, background, realFg;
    usbRelayController usbController;
    std::vector<double> volumes;
};


#endif // VIEWER_H
