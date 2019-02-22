/***********************************************************************
OculusRiftCV1Camera - Class to receive images from the Oculus Rift CV1
"tracking sensor" directly via USB, circumventing the (intentionally?)
broken UVC interface.
Copyright (c) 2018-2019 Oliver Kreylos

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

#ifndef OCULUSRIFTCV1CAMERA_INCLUDED
#define OCULUSRIFTCV1CAMERA_INCLUDED

#include <USB/Device.h>
#include <USB/TransferPool.h>
#include <Geometry/Point.h>

namespace Misc {
template <class ParameterParam>
class FunctionCall;
}

class OculusRiftCV1Camera:public USB::Device
	{
	/* Embedded classes: */
	public:
	typedef Misc::FunctionCall<unsigned char*> StreamingCallback; // Type for functions called when a complete frame has been received; callee takes ownership of provided frame
	typedef Geometry::Point<double,2> Point; // Type for points in the camera's imaging plane
	
	/* Elements: */
	private:
	unsigned int frameSize[2]; // Camera's frame size (width and height in pixels)
	double fx,fy; // Camera's focal length in x and y
	double cx,cy; // Camera's center of projection
	double k[4]; // Camera's lens distortion correction coefficients
	double maxR2; // Maximum distance from distortion center where the undistortion formula can be evaluated
	StreamingCallback* streamingCallback; // Function called when a complete frame has been received
	USB::TransferPool* transferPool; // A pool of isochronous USB transfer buffers to receive image data from the camera
	unsigned char* frameBuffer; // Buffer receiving the current frame via isochronous USB transfers
	unsigned char frameId; // The ID bit of the current frame
	unsigned int framePresentationTime; // The current frame's presentation time as provided by UVC
	unsigned char* framePtr; // Write position into the current frame
	size_t frameRemainder; // Number of bytes still missing from the current frame
	
	/* Private methods: */
	void transferCallback(USB::TransferPool::Transfer* transfer);
	
	/* Constructors and destructors: */
	public:
	OculusRiftCV1Camera(unsigned int deviceIndex); // Connects to the CV1 camera of the given zero-based device index on the local USB bus
	~OculusRiftCV1Camera(void);
	
	/* Methods: */
	const unsigned int* getFrameSize(void) const // Returns the camera's frame size
		{
		return frameSize;
		}
	unsigned int getFrameSize(int dimension) const // Returns camera's frame width or height
		{
		return frameSize[dimension];
		}
	bool canUndistort(const Point& pixel) const; // Returns true if the given pixel can be undistorted
	Point undistort(const Point& pixel) const; // Returns the distortion-corrected position of the given pixel
	void startStreaming(StreamingCallback* newStreamingCallback); // Starts streaming video frames to the given callback function
	void stopStreaming(void); // Stops streaming
	
	/* Imaging control methods: */
	bool getHorizontalFlip(void); // Returns true if the image is flipped horizontally
	bool getVerticalFlip(void); // Returns true if the image is flipped vertically
	void setFlip(bool horizontalFlip,bool verticalFlip); // Sets the horizontal and vertical flipping flags
	bool getAutoExposure(void); // Returns true if auto exposure control is enabled
	void setAutoExposure(bool enable,bool adjustAnalogGain =false,bool adjustDigitalGain =false); // Enables or disables auto exposure control; optionally also enables analog and/or digital gain control
	unsigned int getGain(void); // Returns global gain value
	void setGain(unsigned int newGain); // Sets global gain value
	unsigned int getExposureTime(void); // Returns exposure time in pixel clock units
	void setExposureTime(unsigned int newExposureTime); // Sets exposure time in pixel clock units
	};

#endif
