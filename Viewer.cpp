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
// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE 
	#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "Viewer.h"

#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif

#include "OniSampleUtilities.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <opencv2/highgui.hpp>

using namespace std;

#define GL_WIN_SIZE_X	1280
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

SampleViewer* SampleViewer::ms_self = NULL;
/***
 * new
 */
const int cropOriginX=232;
const int cropOriginY=100;
const int cropWidth=208;
const int cropHeight=128;
const int ERROR=3;
int messageCode=0;
bool flag= false;
double avgDepth=0;


void SampleViewer::glutIdle()
{
	glutPostRedisplay();
}
void SampleViewer::glutDisplay()
{
	SampleViewer::ms_self->display();
}
void SampleViewer::glutKeyboard(unsigned char key, int x, int y)
{
	SampleViewer::ms_self->onKey(key, x, y);
}

SampleViewer::SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color) :
	m_device(device), m_depthStream(depth), m_colorStream(color), m_streams(NULL), m_eViewState(DEFAULT_DISPLAY_MODE), m_pTexMap(NULL)

{
	ms_self = this;
	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
}
SampleViewer::~SampleViewer()
{
	delete[] m_pTexMap;

	ms_self = NULL;

	if (m_streams != NULL)
	{
		delete []m_streams;
		delete[] bgDepths;
        delete[] subDepths;
	}
}

openni::Status SampleViewer::init(int argc, char **argv)
{
	openni::VideoMode depthVideoMode;
	openni::VideoMode colorVideoMode;

	if (m_depthStream.isValid() && m_colorStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
		colorVideoMode = m_colorStream.getVideoMode();

		int depthWidth = depthVideoMode.getResolutionX();
		int depthHeight = depthVideoMode.getResolutionY();
		int colorWidth = colorVideoMode.getResolutionX();
		int colorHeight = colorVideoMode.getResolutionY();

		if (depthWidth == colorWidth &&
			depthHeight == colorHeight)
		{
			m_width = depthWidth;
			m_height = depthHeight;
		}
		else
		{
			printf("Error - expect color and depth to be in same resolution: D: %dx%d, C: %dx%d\n",
				depthWidth, depthHeight,
				colorWidth, colorHeight);
			return openni::STATUS_ERROR;
		}
	}
	else if (m_depthStream.isValid())
	{
		depthVideoMode = m_depthStream.getVideoMode();
		m_width = depthVideoMode.getResolutionX();
		m_height = depthVideoMode.getResolutionY();
	}
	else if (m_colorStream.isValid())
	{
		colorVideoMode = m_colorStream.getVideoMode();
		m_width = colorVideoMode.getResolutionX();
		m_height = colorVideoMode.getResolutionY();
	}
	else
	{
		printf("Error - expects at least one of the streams to be valid...\n");
		return openni::STATUS_ERROR;
	}

	m_streams = new openni::VideoStream*[2];
	m_streams[0] = &m_depthStream;
	m_streams[1] = &m_colorStream;

	// Texture map init
	m_nTexMapX = MIN_CHUNKS_SIZE(m_width, TEXTURE_SIZE);
	m_nTexMapY = MIN_CHUNKS_SIZE(m_height, TEXTURE_SIZE);
	m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];

    bgDepths = new openni::DepthPixel[m_width * m_height];
    subDepths = new openni::DepthPixel[m_width * m_height];

	return initOpenGL(argc, argv);

}
openni::Status SampleViewer::run()	//Does not return
{
	glutMainLoop();

	return openni::STATUS_OK;
}

/***
 * look
 */
void SampleViewer::display()
{
    int changedIndex;
    openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
    if (rc != openni::STATUS_OK) {
        printf("Wait failed\n");
        return;
    }

    switch (changedIndex) {
        case 0:
            m_depthStream.readFrame(&m_depthFrame); break;
        case 1:
            m_colorStream.readFrame(&m_colorFrame); break;
        default:
            printf("Error in wait\n");
    }

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);

    if (m_depthFrame.isValid()) {
        calculateHistogram(m_pDepthHist, MAX_DEPTH, m_depthFrame);
    }

    memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

    // check if we need to draw image frame to texture
    if ((m_eViewState == DISPLAY_MODE_OVERLAY ||
            m_eViewState == DISPLAY_MODE_IMAGE) && m_colorFrame.isValid()) {
        const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
        openni::RGB888Pixel* pTexRow = m_pTexMap + m_colorFrame.getCropOriginY() * m_nTexMapX;
        int rowSize = m_colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);

        for (int y = 0; y < m_colorFrame.getHeight(); ++y) {
            const openni::RGB888Pixel* pImage = pImageRow;
            openni::RGB888Pixel* pTex = pTexRow + m_colorFrame.getCropOriginX();

            for (int x = 0; x < m_colorFrame.getWidth(); ++x, ++pImage, ++pTex) {
                *pTex = *pImage;
            }

            pImageRow += rowSize;
            pTexRow += m_nTexMapX;
        }
    }


    // check if we need to draw depth frame to texture
    if ((m_eViewState == DISPLAY_MODE_OVERLAY ||
         m_eViewState == DISPLAY_MODE_DEPTH) && m_depthFrame.isValid()) {
        const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
        openni::RGB888Pixel* pTexRow = m_pTexMap + m_depthFrame.getCropOriginY() * m_nTexMapX;
        int rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

        switch (messageCode){
            case 1:
                std::cout << "depth " << pDepthRow[250 + 112 * m_width] << std::endl;
                break;
            case 2:
                getVolume(pDepthRow);
                break;
        }

        for (int y = 0; y < m_height; ++y) {
            const openni::DepthPixel* pDepth = pDepthRow;
            openni::RGB888Pixel* pTex = pTexRow + m_depthFrame.getCropOriginX();

            for (int x = 0; x < m_width; ++x, ++pDepth, ++pTex) {
                if (*pDepth != 0) {
                    int nHistValue = m_pDepthHist[*pDepth];
                    pTex->r = nHistValue;
                    pTex->g = nHistValue;
                    pTex->b = 0;
                }

                if(((y == cropOriginY) || (y == cropOriginY + cropHeight)) && (x > cropOriginX && x < cropOriginX + cropWidth)){
                    pTex->r = 255;
                    pTex->g = 0;
                    pTex->b = 0;
                }

                if(((x == cropOriginX || x == cropOriginX + cropWidth)) && (y > cropOriginY && y < cropOriginY + cropHeight)){
                    pTex->r = 255;
                    pTex->g = 0;
                    pTex->b = 0;
                }


            }

            pDepthRow += rowSize;
            pTexRow += m_nTexMapX;
        }

    }


    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

    // Display the OpenGL texture map
    glColor4f(1,1,1,1);

    glBegin(GL_QUADS);

    int nXRes = m_width;
    int nYRes = m_height;

    // upper left
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    // upper right
    glTexCoord2f((float)nXRes / (float)m_nTexMapX, 0);
    glVertex2f(GL_WIN_SIZE_X, 0);
    // bottom right
    glTexCoord2f((float)nXRes / (float)m_nTexMapX, (float)nYRes / (float)m_nTexMapY);
    glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    // bottom left
    glTexCoord2f(0, (float)nYRes / (float)m_nTexMapY);
    glVertex2f(0, GL_WIN_SIZE_Y);

    glEnd();

    // Swap the OpenGL display buffers
    glutSwapBuffers();

}

/***
 * look
 * @param key
 */
void SampleViewer::onKey(unsigned char key, int /*x*/, int /*y*/)
{
    switch (key) {
        case 27:
            m_depthStream.stop();
            m_colorStream.stop();
            m_depthStream.destroy();
            m_colorStream.destroy();
            m_device.close();
            openni::OpenNI::shutdown();

            exit (1);
        case '1':
            m_eViewState = DISPLAY_MODE_OVERLAY;
            m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
            break;
        case '2':
            m_eViewState = DISPLAY_MODE_DEPTH;
            m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
            break;
        case '3':
            m_eViewState = DISPLAY_MODE_IMAGE;
            m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
            break;
        case 'm':
            m_depthStream.setMirroringEnabled(!m_depthStream.getMirroringEnabled());
            m_colorStream.setMirroringEnabled(!m_colorStream.getMirroringEnabled());
            break;
        case 'i':
            flag=!flag;
            if(flag){
                messageCode=1;
            } else{
                memcpy(bgDepths, m_depthFrame.getData(), m_width * m_height);
                messageCode=0;
            }
            break;
        case 'p':
            messageCode=2;
            break;
        case 'u':
            display(bgDepths);
            break;
        case 't':
            display((const openni::DepthPixel *) m_depthFrame.getData());
            break;
        case 'r':
            messageCode=0;
            getsubDepths((const openni::DepthPixel *) m_depthFrame.getData());
            display(subDepths);
            /*for(int i=100;i<130;++i){
                float cropOriginWX=0.0,cropOriginWY=0.0,cropOriginWZ=0.0;
                openni::CoordinateConverter::convertDepthToWorld(m_depthStream,i,240,960,&cropOriginWX,&cropOriginWY,&cropOriginWZ);
                std::cout<<"cropOriginWX:"<<cropOriginWX<<" cropOriginWY:"<<cropOriginWY<<" "<<std::endl;
            }*/

            /*getsubDepths((const openni::DepthPixel*)m_depthFrame.getData());
            for (int y = 0; y < m_height; ++y) {
                for (int x = 0; x < m_width; ++x) {
                    if(subDepths[x+y*m_width]!=0){
                        std::cout<<subDepths[x+y*m_width]<<" ";
                    } else{
                        std::cout<<" ";
                    }


                }
                std::cout<<std::endl;
            }
            break;*/


    }

}

openni::Status SampleViewer::initOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (m_strSampleName);
	// 	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	initOpenGLHooks();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	return openni::STATUS_OK;

}
void SampleViewer::initOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
}

/*void SampleViewer::smoothImage(const openni::DepthPixel* pixel){
    for (int y = 0; y < 480; ++y) {
        float * data = src.ptr<float>(y);
        for (int x = 0; x < 640; ++x) {
            data[x]=pixel[x+y*m_depthFrame.getWidth()];
        }
    }
    cv::bilateralFilter(src,dst,5,30,100);
}*/
/*void convertDepthPixelToMat(const openni::DepthPixel* pixel,cv::Mat& src){
    for (int y = 0; y < src.rows; ++y) {
        float * data = src.ptr<float>(y);
        for (int x = 0; x < src.cols; ++x) {
            data[x]=pixel[x+y*src.cols];
        }
    }
}*/

void SampleViewer::display(const openni::DepthPixel *pixel) {
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            if (pixel[x + y * m_width] != 0) {
                std::cout << pixel[x + y * m_width] << " ";
            } else {
                std::cout << " ";
            }


        }
        std::cout << std::endl;
    }
}

int SampleViewer::getMaxDepth(const openni::DepthPixel *pixel) {
    int temp = pixel[0];
    int pos = 0;
    for (int i = 1; i < m_width * m_height; ++i) {
        if (pixel[i] > temp) {
            temp = pixel[i];
            pos = i;
        }
    }
    return pos;
}

void SampleViewer::getsubDepths(const openni::DepthPixel *pixel) {
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            int actual = pixel[x + y * m_width];
            int bgDepth = bgDepths[x + y * m_width];
            int sub = bgDepth - actual;
            /*subDepths[x+y*m_width]=sub;*/
            if (actual != 0 && bgDepth != 0 && sub > ERROR) {
                subDepths[x + y * m_width] = sub;
            } else {
                subDepths[x + y * m_width] = 0;
            }

        }
    }
}

double SampleViewer::getVolume(const openni::DepthPixel *pixel) {
    double sum = 0.0, pixelArea = 0.0;
    //smoothImage(pixel);
    getsubDepths(pixel);
    int maxDepthPos = getMaxDepth(subDepths);
    if (maxDepthPos == 0) {
        std::cout << "volume= " << 0 << std::endl;
        return 0;
    }
    float cropOriginWX = 0.0, cropOriginWY = 0.0, cropOriginWZ = 0.0;
    openni::CoordinateConverter::convertDepthToWorld(m_depthStream, 0, 0, bgDepths[maxDepthPos], &cropOriginWX,
                                                     &cropOriginWY, &cropOriginWZ);

    float cropOriginWX1 = 0.0, cropOriginWY1 = 0.0;
    openni::CoordinateConverter::convertDepthToWorld(m_depthStream, m_width, m_height, bgDepths[maxDepthPos],
                                                     &cropOriginWX1, &cropOriginWY1, &cropOriginWZ);

    float maxAvgWidth = (cropOriginWX1 - cropOriginWX) / m_width;
    float maxAvgHeight = (cropOriginWY - cropOriginWY1) / m_height;

    openni::CoordinateConverter::convertDepthToWorld(m_depthStream, 0, 0, pixel[maxDepthPos], &cropOriginWX,
                                                     &cropOriginWY, &cropOriginWZ);

    openni::CoordinateConverter::convertDepthToWorld(m_depthStream, m_width, m_height, pixel[maxDepthPos],
                                                     &cropOriginWX1, &cropOriginWY1, &cropOriginWZ);

    float minAvgWidth = (cropOriginWX1 - cropOriginWX) / m_width;
    float minAvgHeight = (cropOriginWY - cropOriginWY1) / m_height;

    float avgWidth = (maxAvgWidth + minAvgWidth) / 2;
    float avgHeight = (maxAvgHeight + minAvgHeight) / 2;
    pixelArea = avgWidth * avgHeight / 100;
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            int subdepth = subDepths[x + y * m_width];
            sum += subdepth * pixelArea;
        }
    }

    std::cout << "volume= " << sum << std::endl;

}
