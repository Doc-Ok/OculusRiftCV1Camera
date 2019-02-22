/***********************************************************************
OculusRiftCV1CameraViewer - Vrui application to view a live video stream
from an Oculus Rift CV1 "tracking sensor."
Copyright (c) 2019 Oliver Kreylos

This file is part of the optical/inertial sensor fusion tracking
package.

The optical/inertial sensor fusion tracking package is free software;
you can redistribute it and/or modify it under the terms of the GNU
General Public License as published by the Free Software Foundation;
either version 2 of the License, or (at your option) any later version.

The optical/inertial sensor fusion tracking package is distributed in
the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the optical/inertial sensor fusion tracking package; if not, write
to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,
Boston, MA 02111-1307 USA
***********************************************************************/

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <Misc/FunctionCalls.h>
#include <Threads/TripleBuffer.h>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Math/Math.h>
#include <GL/gl.h>
#include <GL/GLObject.h>
#include <GL/GLContextData.h>
#include <GL/Extensions/GLARBTextureRectangle.h>
#include <GL/GLGeometryWrappers.h>
#include <Vrui/Vrui.h>
#include <Vrui/Application.h>

#include "OculusRiftCV1Camera.h"

class OculusRiftCV1CameraViewer:public Vrui::Application,public GLObject
	{
	/* Embedded classes: */
	private:
	struct DataItem:public GLObject::DataItem // Structure containing per-OpenGL context state
		{
		/* Elements: */
		public:
		GLuint textureObjectId; // ID of texture object holding the most recently received video frame
		unsigned int version; // Version number of video frame currently in texture object
		
		/* Constructors and destructors: */
		DataItem(void);
		virtual ~DataItem(void);
		};
	
	/* Elements: */
	OculusRiftCV1Camera* camera; // Pointer to a camera object
	Threads::TripleBuffer<unsigned char*> frames; // Triple buffer of received video frames, which are new-allocated arrays of 8-bit greyscale pixels
	unsigned int version; // Version number of currently locked video frame
	std::string saveFrameNameTemplate; // Printf-style file name template to save video frames
	unsigned int saveFrameIndex; // Index for next video frame to be saved
	volatile bool saveNextFrame; // Flag to save the next incoming video frame
	
	/* Private methods: */
	void streamingCallback(unsigned char* newFramebuffer); // Callback function receiving frame buffers from the camera object
	
	/* Constructors and destructors: */
	public:
	OculusRiftCV1CameraViewer(int& argc,char**& argv);
	virtual ~OculusRiftCV1CameraViewer(void);
	
	/* Methods from class Vrui::Application: */
	virtual void frame(void);
	virtual void display(GLContextData& contextData) const;
	virtual void resetNavigation(void);
	virtual void eventCallback(EventID eventId,Vrui::InputDevice::ButtonCallbackData* cbData);
	
	/* Methods from class GLObject: */
	virtual void initContext(GLContextData& contextData) const;
	};

/****************************************************
Methods of class OculusRiftCV1CameraViewer::DataItem:
****************************************************/

OculusRiftCV1CameraViewer::DataItem::DataItem(void)
	:textureObjectId(0),
	 version(0)
	{
	/* Initialize required OpenGL extensions: */
	GLARBTextureRectangle::initExtension();
	
	/* Create the texture object: */
	glGenTextures(1,&textureObjectId);
	}

OculusRiftCV1CameraViewer::DataItem::~DataItem(void)
	{
	/* Destroy the texture object: */
	glDeleteTextures(1,&textureObjectId);
	}

/******************************************
Methods of class OculusRiftCV1CameraViewer:
******************************************/

void OculusRiftCV1CameraViewer::streamingCallback(unsigned char* newFramebuffer)
	{
	/* Post the new video frame into the triple buffer: */
	unsigned char*& framebuffer=frames.startNewValue();
	
	/* Delete the old frame buffer and store the new one: */
	delete[] framebuffer;
	framebuffer=newFramebuffer;
	
	frames.postNewValue();
	
	/* Wake up the main thread; */
	Vrui::requestUpdate();
	
	/* Check if the frame is supposed to be saved: */
	if(saveNextFrame)
		{
		try
			{
			/* Save the new frame as a greyscale PPM file: */
			char saveFrameName[1024];
			snprintf(saveFrameName,sizeof(saveFrameName),saveFrameNameTemplate.c_str(),saveFrameIndex);
			
			/* Create a new PPM file: */
			IO::FilePtr ppm=IO::openFile(saveFrameName,IO::File::WriteOnly);
			
			/* Write the PPM header: */
			char headerBuffer[256];
			unsigned int w=camera->getFrameSize(0);
			unsigned int h=camera->getFrameSize(1);
			size_t headerSize=snprintf(headerBuffer,sizeof(headerBuffer),"P5\n%u %u\n255\n",w,h);
			ppm->write(headerBuffer,headerSize);
			
			/* Write the video frame's pixel data: */
			ppm->write(newFramebuffer,size_t(h)*size_t(w));
			
			/* Increment the frame index for the next saved frame: */
			++saveFrameIndex;
			}
		catch(const std::runtime_error& err)
			{
			/* Print an error message and carry on: */
			std::cerr<<"Caught exception "<<err.what()<<" while saving video frame "<<saveFrameIndex<<std::endl;
			}
		
		/* Reset the frame saving flag: */
		saveNextFrame=false;
		}
	}

OculusRiftCV1CameraViewer::OculusRiftCV1CameraViewer(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 camera(0),
	 version(0),
	 saveFrameNameTemplate("SavedVideoFrame%04u.ppm"),
	 saveFrameIndex(0),
	 saveNextFrame(false)
	{
	/* Parse the command line: */
	unsigned int deviceIndex=0;
	bool autoExposure=false;
	unsigned int gain=~0x0U;
	unsigned int exposure=~0x0U;
	for(int argi=1;argi<argc;++argi)
		{
		if(argv[argi][0]=='-')
			if(strcasecmp(argv[argi]+1,"auto")==0)
				autoExposure=true;
			else if(strcasecmp(argv[argi]+1,"gain")==0)
				{
				++argi;
				if(argi<argc)
					gain=(unsigned int)(atoi(argv[argi]));
				else
					std::cerr<<"Ignoring dangling "<<argv[argi-1]<<" parameter"<<std::endl;
				}
			else if(strcasecmp(argv[argi]+1,"exposure")==0)
				{
				++argi;
				if(argi<argc)
					exposure=(unsigned int)(atoi(argv[argi]));
				else
					std::cerr<<"Ignoring dangling "<<argv[argi-1]<<" parameter"<<std::endl;
				}
			else
				std::cerr<<"Ignoring command line parameter "<<argv[argi]<<std::endl;
		else
			deviceIndex=(unsigned int)(atoi(argv[argi]));
		}
	
	/* Create a tool to save video frames: */
	addEventTool("Save Video Frame",0,0);
	
	/* Open the requested camera device: */
	camera=new OculusRiftCV1Camera(deviceIndex);
	
	/* Print current exposure and gain controls: */
	std::cout<<"Automatic exposure control "<<(camera->getAutoExposure()?"enabled":"disabled")<<std::endl;
	std::cout<<"Global gain value        : "<<camera->getGain()<<std::endl;
	std::cout<<"Exposure time            : "<<camera->getExposureTime()<<std::endl;
	
	/* Initialize the incoming video frame triple buffer: */
	for(int i=0;i<3;++i)
		frames.getBuffer(i)=0;
	
	/* Start streaming: */
	camera->startStreaming(Misc::createFunctionCall(this,&OculusRiftCV1CameraViewer::streamingCallback));
	
	/* Set camera's imaging parameters: */
	camera->setFlip(false,true); // Raw camera image is upside down
	if(autoExposure)
		camera->setAutoExposure(true,true,true);
	if(gain!=~0x0U)
		camera->setGain(gain);
	if(exposure!=~0x0U)
		camera->setExposureTime(exposure);
	}

OculusRiftCV1CameraViewer::~OculusRiftCV1CameraViewer(void)
	{
	/* Stop streaming: */
	camera->stopStreaming();
	delete camera;
	
	/* Clear the incoming video frame triple buffer: */
	for(int i=0;i<3;++i)
		delete[] frames.getBuffer(i);
	}

void OculusRiftCV1CameraViewer::frame(void)
	{
	/* Check if there is a new video frame: */
	if(frames.lockNewValue())
		{
		/* Invalidate the texture object holding the currently displayed video frame: */
		++version;
		}
	}

void OculusRiftCV1CameraViewer::display(GLContextData& contextData) const
	{
	/* Set up OpenGL state: */
	glPushAttrib(GL_ENABLE_BIT);
	
	/* Bind the video texture: */
	DataItem* dataItem=contextData.retrieveDataItem<DataItem>(this);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->textureObjectId);
	
	/* Check if the texture image is outdated: */
	unsigned int w=camera->getFrameSize(0);
	unsigned int h=camera->getFrameSize(1);
	if(dataItem->version!=version)
		{
		/* Upload the most recently received video frame into the texture image: */
		glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGBA8,w,h,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,frames.getLockedValue());
		
		/* Mark the texture object as up-to-date: */
		dataItem->version=version;
		}
	
	/* Display the video frame: */
	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_RECTANGLE_ARB);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_REPLACE);
	
	/* Map the video frame onto a grid of (distorted) tiles to correct the camera's lens distortion: */
	for(int y=1;y<=64;++y)
		{
		/* Draw a row of tiles as a quad strip: */
		glBegin(GL_QUAD_STRIP);
		for(int x=0;x<=96;++x)
			{
			/* Calculate the original positions of the strip's next top and bottom vertices: */
			OculusRiftCV1Camera::Point dp1(double(x)*double(w)/96.0,double(y)*double(h)/64.0);
			OculusRiftCV1Camera::Point dp0(double(x)*double(w)/96.0,double(y-1)*double(h)/64.0);
			
			/* Check if the two vertices can be undistorted: */
			// if(camera->canUndistort(dp1)&&camera->canUndistort(dp0)) // Actually, don't check
				{
				/* Use the original positions as texture coordinate, and the undistorted positions as vertex positions: */
				glTexCoord(dp1);
				glVertex(camera->undistort(dp1));
				glTexCoord(dp0);
				glVertex(camera->undistort(dp0));
				}
			}
		glEnd();
		}
	
	/* Protect the video texture: */
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	
	/* Restore OpenGL state: */
	glPopAttrib();
	}

void OculusRiftCV1CameraViewer::resetNavigation(void)
	{
	/* Center the display on the video frame: */
	Vrui::Scalar w(camera->getFrameSize(0));
	Vrui::Scalar h(camera->getFrameSize(1));
	Vrui::Point center(Math::div2(w),Math::div2(h),0);
	Vrui::Scalar size(Math::div2(Math::sqrt(w*w+h*h)));
	
	/* Make the size a bit larger to account for lens distortion correction: */
	size*=Vrui::Scalar(1.5);
	
	/* Arrange the video texture to fit into the display, with y pointing up: */
	Vrui::setNavigationTransformation(center,size,Vrui::Vector(0,1,0));
	}

void OculusRiftCV1CameraViewer::eventCallback(EventID eventId,Vrui::InputDevice::ButtonCallbackData* cbData)
	{
	/* Was the button just pressed? */
	if(cbData->newButtonState)
		{
		/* Is this the frame saver tool? */
		if(eventId==0)
			{
			/* Save the next incoming video frame from the background thread: */
			saveNextFrame=true;
			}
		}
	}

void OculusRiftCV1CameraViewer::initContext(GLContextData& contextData) const
	{
	/* Create a data item and associate it with this application object: */
	DataItem* dataItem=new DataItem;
	contextData.addDataItem(this,dataItem);
	
	/* Initialize the texture object: */
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,dataItem->textureObjectId);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_MAX_LEVEL,0);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_RECTANGLE_ARB,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_RECTANGLE_ARB,0);
	}

/* Run the application: */
VRUI_APPLICATION_RUN(OculusRiftCV1CameraViewer)
