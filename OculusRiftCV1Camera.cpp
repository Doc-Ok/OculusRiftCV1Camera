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

#include "OculusRiftCV1Camera.h"

#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <stdexcept>
#include <Misc/SizedTypes.h>
#include <Misc/ThrowStdErr.h>
#include <Misc/FunctionCalls.h>
#include <Realtime/Time.h>
#include <IO/FixedMemoryFile.h>
#include <USB/DeviceList.h>
#include <Geometry/Vector.h>

// DEBUGGING
#include <iostream>
#include <iomanip>

namespace {

/**************
Helper classes:
**************/

class UVC // Class to execute UVC commands on a USB device
	{
	/* Methods: */
	public:
	static void setCur(USB::Device& device,Misc::UInt8 interface,Misc::UInt8 entity,Misc::UInt8 selector,const Misc::UInt8* data,Misc::UInt16 length);
	static void getCur(USB::Device& device,Misc::UInt8 interface,Misc::UInt8 entity,Misc::UInt8 selector,Misc::UInt8* data,Misc::UInt16 length);
	static Misc::UInt16 getLen(USB::Device& device,Misc::UInt8 interface,Misc::UInt8 entity,Misc::UInt8 selector);
	};

class ESP770U // Class to communicate with the Oculus Rift CV1 camera's camera controller
	{
	/* Embedded classes: */
	private:
	enum Entities // Enumerated type for UVC entities used by this controller
		{
		ExtensionUnit=4
		};
	enum Selectors // Enumerated type for UVC selectors used by this controller
		{
		I2C=2,
		Register=3,
		Counter=10,
		Control=11,
		Data=12
		};
	
	/* Elements: */
	USB::Device& device; // Reference to the USB device representing the camera controller
	
	/* Private methods: */
	void setGetCur(int selector,Misc::UInt8* buffer,Misc::UInt16 length);
	Misc::UInt8 readRegister(Misc::UInt16 registerIndex);
	void writeRegister(Misc::UInt16 registerIndex,Misc::UInt8 value);
	Misc::UInt8 getCounter(void); // Returns the current value of the self-incrementing command counter
	void setCounter(Misc::UInt8 newCounter); // Sets the self-incrementing command counter
	void spiSetControl(Misc::UInt8 handle,Misc::UInt8 length);
	void spiSetData(const Misc::UInt8* data,Misc::UInt16 length);
	void spiGetData(Misc::UInt8* data,Misc::UInt16 length);
	void writeRadio(const Misc::UInt8* data,Misc::UInt16 length); // Writes a data buffer to the controller's nRF51288 radio component
	
	/* Constructors and destructors: */
	public:
	ESP770U(USB::Device& sDevice)
		:device(sDevice)
		{
		}
	
	/* Methods: */
	Misc::UInt8 queryFirmwareVersion(void); // Returns the controller's firmware version
	void readMemory(Misc::UInt32 address,Misc::UInt8* buffer,Misc::UInt16 length); // Reads a block from the controller's non-volatile memory
	void initController(void); // Initializes the controller for operation
	void initRadio(void); // Initializes the controller's radio component
	void setupRadio(Misc::UInt32 radioId); // Connects the controller's radio component to the radio of the given ID on the Rift headset
	Misc::UInt16 readI2C(Misc::UInt8 address,Misc::UInt16 registerIndex); // Reads a 16-bit value via I2C
	void writeI2C(Misc::UInt8 address,Misc::UInt16 registerIndex,Misc::UInt16 value); // Writes a 16-bit value via I2C
	};

class AR0134 // Class to communicate with the Oculus Rift CV1 camera's imaging controller
	{
	/* Embedded classes: */
	private:
	enum Addresses // Enumerated type for chip addresses
		{
		I2CAddress=0x20 // AR0134 chip's address on the ESP770U camera controller's I2C bus
		};
	
	enum Registers // Enumerated type for chip register addresses
		{
		ChipVersionReg=          0x3000,
		YAddrStart=              0x3002,
		XAddrStart=              0x3004,
		YAddrEnd=                0x3006,
		XAddrEnd=                0x3008,
		FrameLengthLines=        0x300a,
		LineLengthPck=           0x300c,
		RevisionNumber=          0x300e,
		CoarseIntegrationTime=   0x3012,
		FineIntegrationTime=     0x3014,
		CoarseIntegrationTimeCb= 0x3016,
		FineIntegrationTimeCb=   0x3018,
		ResetRegister=           0x301a,
		DataPedestal=            0x301e,
		GpiStatus=               0x3026,
		RowSpeed=                0x3028,
		VtPixClkDiv=             0x302a,
		VtSysClkDiv=             0x302c,
		PrePllClkDiv=            0x302e,
		PllMultiplier=           0x3030,
		DigitalBinning=          0x3032,
		FrameCount=              0x303a,
		FrameStatus=             0x303c,
		ReadMode=                0x3040,
		DarkControl=             0x3044,
		Flash=                   0x3046,
		Green1Gain=              0x3056,
		BlueGain=                0x3058,
		RedGain=                 0x305a,
		Green2Gain=              0x305c,
		GlobalGain=              0x305e,
		EmbeddedDataControl=     0x3064,
		DatapathSelect=          0x306e,
		TestPatternMode=         0x3070,
		TestDataRed=             0x3072,
		TestDataGreenr=          0x3074,
		TestDataBlue=            0x3076,
		TestDataGreenb=          0x3078,
		TestRawMode=             0x307a,
		SeqDataPort=             0x3086,
		SeqCtrlPort=             0x3088,
		XAddrStartCb=            0x308a,
		YAddrStartCb=            0x308c,
		XAddrEndCb=              0x308e,
		YAddrEndCb=              0x3090,
		XEvenInc=                0x30a0,
		XOddInc=                 0x30a2,
		YEvenInc=                0x30a4,
		YOddInc=                 0x30a6,
		YOddIncCb=               0x30a8,
		FrameLengthLinesCb=      0x30aa,
		FrameExposure=           0x30ac,
		DigitalTest=             0x30b0,
		TempsensData=            0x30b2,
		TempsensCtrl=            0x30b4,
		Green1GainCb=            0x30bc,
		BlueGainCb=              0x30be,
		RedGainCb=               0x30c0,
		Green2GainCb=            0x30c2,
		GlobalGainCb=            0x30c4,
		TempsensCalib1=          0x30c6,
		TempsensCalib2=          0x30c8,
		TempsensCalib3=          0x30ca,
		TempsensCalib4=          0x30cc,
		ColumnCorrection=        0x30d4,
		AeCtrlReg=               0x3100,
		AeLumaTargetReg=         0x3102,
		AeMinEvStepReg=          0x3108,
		AeMaxEvStepReg=          0x310a,
		AeDampOffsetReg=         0x310c,
		AeDampGainReg=           0x310e,
		AeDampMaxReg=            0x3110,
		AeMaxExposureReg=        0x311c,
		AeMinExposureReg=        0x311e,
		AeDarkCurThreshReg=      0x3124,
		AeCurrentGains=          0x312a,
		AeRoiXStartOffset=       0x3140,
		AeRoiYStartOffset=       0x3142,
		AeRoiXSize=              0x3144,
		AeRoiYSize=              0x3146,
		AeMeanL=                 0x3152,
		AeCoarseIntegrationTime= 0x3164,
		AeAgExposureHi=          0x3166,
		AeAgExposureLo=          0x3168,
		DeltaDkLevel=            0x3188,
		HispiTiming=             0x31c0,
		HispiControlStatus=      0x31c6,
		HispiCrc0=               0x31c8,
		HispiCrc1=               0x31ca,
		HispiCrc2=               0x31cc,
		HispiCrc3=               0x31ce,
		StatFrameId=             0x31d2,
		I2cWrtChecksum=          0x31d6,
		HorizontalCursorPosition=0x31e8,
		VerticalCursorPosition=  0x31ea,
		HorizontalCursorWidth=   0x31ec,
		VerticalCursorWidth=     0x31ee,
		I2cIds=                  0x31fc
		};
	
	enum ResetRegisterFlags // Flag values for the reset register (0x301a)
		{
		Reset=               0x0001,
		Restart=             0x0002,
		Stream=              0x0004,
		LockReg=             0x0008,
		StdbyEof=            0x0010,
		DrivePins=           0x0040,
		ParallelEn=          0x0080,
		GpiEn=               0x0100,
		MaskBad=             0x0200,
		RestartBad=          0x0400,
		ForcedPllOn=         0x0800,
		SmiaSerialiserDis=   0x1000,
		GroupedParameterHold=0x8000
		};
	
	enum GpiStatusFlags // Flag values for the GpiStatus register (0x3026)
		{
		Saddr=  0x0001,
		OeN=    0x0002,
		Trigger=0x0004,
		Standby=0x0008
		};
	
	enum DigitalBinningFlags // Flag values for the digital binning register (0x3032)
		{
		DigitalBinningMask=            0x0003,
		HorizontalOnlyBinning=         0x0001,
		HorizontalAndVerticalBinning=  0x0002,
		DigitalBinningCbMask=          0x0030,
		HorizontalOnlyBinningCb=       0x0010,
		HorizontalAndVerticalBinningCb=0x0020,
		};
	
	enum FrameStatusFlags // Flag values for the frame status register (0x303c)
		{
		Framesync=    0x0001,
		StandbyStatus=0x0002
		};
	
	enum ReadModeFlags // Flag values for the read mode register (0x3040)
		{
		HorizMirror=0x4000,
		VertFlip=   0x8000
		};
	
	enum DarkControlFlags // Flag values for the dark control register (0x3044)
		{
		ShowDarkCols=        0x0200,
		RowNoiseCorrectionEn=0x0400,
		ShowDarkExtraRows=   0x0800,
		ShowColcorrRows=     0x1000
		};
	
	enum FlashFlags // Flag values for the flash register (0x3046)
		{
		InvertFlash=0x0080,
		EnFlash=    0x0100,
		Triggered=  0x4000,
		Strobe=     0x8000
		};
	
	enum EmbeddedDataFlags // Flag values for the embedded data control register (0x3064)
		{
		EmbeddedStatsEn=0x0080,
		EmbeddedData=0x0100
		};
	
	enum DatapathSelectFlags // Flag values for the datapath select register (0x306e)
		{
		SpecialLineValidMask=    0x0003,
		LineValid=               0x0001,
		LineValidXorFrameValid=  0x0002,
		TrueBayer=               0x0010,
		PostscalerDataSel=       0x0100,
		SlewRateCtrlPixclkMask=  0x1c00,
		SlewRateCtrlParallelMask=0xe000
		};
	
	enum TestPatternModes // Modes for the test pattern mode register (0x3070)
		{
		NormalOperation=0,
		SolidColor=1,
		ColorBar=2,
		FadeToGray=3,
		Walking1s=256
		};
	
	enum DigitalTestFlags // Flag values for the digital test register (0x30b0)
		{
		ColGainMask=      0x0030,
		MonoChrome=       0x0080,
		ColGainCbMask=    0x0300,
		EnableShortLlpck= 0x0400,
		ContextB=         0x2000,
		PllCompleteBypass=0x4000
		};
	
	enum ColumnCorrectionFlags // Flag values for the column correction register (0x30d4)
		{
		ColcorrRowsMask=0x000f,
		DoubleSamples=  0x2000,
		DoubleRange=    0x4000,
		Enable=         0x8000
		};
	
	enum AeCtrlRegFlags // Flag values for the auto exposure control register (0x3100)
		{
		AeEnable=0x0001,
		AutoAgEn=0x0002,
		AutoDgEn=0x0010,
		MinAnaGainMask=0x0060
		};
	
	enum AeCurrentGainsFlags // Flag values for the current auto exposure gain values register (0x312a)
		{
		AeDigGainMask=0x00ff,
		AeAnaGainMask=0x0300
		};
	
	/* Elements: */
	private:
	ESP770U& esp770u; // Reference to the camera controller providing access to the imaging controller
	
	/* Private methods: */
	Misc::UInt16 readRegister(Misc::UInt16 registerIndex);
	void writeRegister(Misc::UInt16 registerIndex,Misc::UInt16 value);
	
	/* Constructors and destructors: */
	public:
	AR0134(ESP770U& sEsp770u)
		:esp770u(sEsp770u)
		{
		}
	
	/* Methods: */
	void dumpRegisters(void);
	void init(void); // Initializes the chip for camera operation
	bool getHorizontalFlip(void); // Returns true if the image is flipped horizontally
	bool getVerticalFlip(void); // Returns true if the image is flipped vertically
	void setFlip(bool horizontalFlip,bool verticalFlip); // Sets the horizontal and vertical flipping flags
	bool getAutoExposure(void); // Returns true if automatic exposure control is enabled
	void setAutoExposure(bool enable,bool adjustAnalogGain,bool adjustDigitalGain); // Enables or disables automatic exposure control
	Misc::UInt16 getGain(void); // Returns the current digital gain value
	void setGain(Misc::UInt16 gain); // Sets digital gain
	void setWindow(Misc::UInt16 x,Misc::UInt16 y,Misc::UInt16 width,Misc::UInt16 height); // Sets the sensor's image capture window
	Misc::UInt16 getTotalWidth(void); // Returns the total width of a frame including horizontal blanking
	Misc::UInt16 getTotalHeight(void); // Returns the total height of a frame including vertical blanking
	void setFrameTimings(bool minBlank); // Sets loose or tight frame timings (horizontal and vertical blanking periods) for assumed full frame
	Misc::UInt16 getCoarseExposureTime(void); // Returns current sensor integration time in line clock units
	void setCoarseExposureTime(Misc::UInt16 coarseExposure); // Sets sensor integration time in line clock units
	Misc::UInt16 getFineExposureTime(void); // Returns current sensor integration time in pixel clock units
	void setFineExposureTime(Misc::UInt16 fineExposure); // Sets sensor integration time in pixel clock units
	Misc::UInt32 getExposureTime(void); // Returns sensor integration time in pixel clock units
	void setExposureTime(Misc::UInt32 exposure); // Sets sensor integration time in pixel clock units
	void setSync(bool enable); // Enables or disables synchronized exposure mode
	};

/********************
Methods of class UVC:
********************/

void UVC::setCur(USB::Device& device,Misc::UInt8 interface,Misc::UInt8 entity,Misc::UInt8 selector,const Misc::UInt8* data,Misc::UInt16 length)
	{
	/* Write a control transfer: */
	unsigned int requestType=LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE;
	unsigned int request=0x01; // UVC request for SET_CUR
	device.writeControl(requestType,request,Misc::UInt16(selector)<<8,(Misc::UInt16(entity)<<8)|Misc::UInt16(interface),data,length,1000);
	}

void UVC::getCur(USB::Device& device,Misc::UInt8 interface,Misc::UInt8 entity,Misc::UInt8 selector,Misc::UInt8* data,Misc::UInt16 length)
	{
	/* Read a control transfer: */
	unsigned int requestType=LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE;
	unsigned int request=0x81; // UVC request for GET_CUR
	size_t readLen=device.readControl(requestType,request,Misc::UInt16(selector)<<8,(Misc::UInt16(entity)<<8)|Misc::UInt16(interface),data,length,1000);
	if(readLen!=length)
		Misc::throwStdErr("UVC::getCur: Expected %u bytes, received %u",length,readLen);
	}

Misc::UInt16 UVC::getLen(USB::Device& device,Misc::UInt8 interface,Misc::UInt8 entity,Misc::UInt8 selector)
	{
	/* Read a control transfer: */
	unsigned int requestType=LIBUSB_REQUEST_TYPE_CLASS|LIBUSB_RECIPIENT_INTERFACE;
	unsigned int request=0x85; // UVC request for GET_LEN
	Misc::UInt8 result[2];
	size_t readLen=device.readControl(requestType,request,Misc::UInt16(selector)<<8,(Misc::UInt16(entity)<<8)|Misc::UInt16(interface),result,sizeof(result),1000);
	if(readLen!=sizeof(result))
		Misc::throwStdErr("UVC::getLen: Expected %u bytes, received %u",sizeof(result),readLen);
	
	return (Misc::UInt16(result[1])<<8)|Misc::UInt16(result[0]);
	}

/************************
Methods of class ESP770U:
************************/

void ESP770U::setGetCur(int selector,Misc::UInt8* buffer,Misc::UInt16 length)
	{
	/* Execute a UVC SET_CUR followed by a UVC GET_CUR: */
	UVC::setCur(device,0,ExtensionUnit,selector,buffer,length);
	UVC::getCur(device,0,ExtensionUnit,selector,buffer,length);
	}

Misc::UInt8 ESP770U::readRegister(Misc::UInt16 registerIndex)
	{
	/* Write a command to the register selector: */
	Misc::UInt8 command[4];
	command[0]=0x82U; // Opcode for read register
	command[1]=Misc::UInt8(registerIndex>>8);
	command[2]=Misc::UInt8(registerIndex&0xffU);
	command[3]=0x00U;
	setGetCur(Register,command,sizeof(command));
	
	/* Check the result: */
	if(command[0]!=0x82U||command[2]!=0x00U)
		throw std::runtime_error("ESP770U::readRegister: Invalid command buffer");
	
	return command[1];
	}

void ESP770U::writeRegister(Misc::UInt16 registerIndex,Misc::UInt8 value)
	{
	/* Write a command to the register selector: */
	Misc::UInt8 command[4];
	command[0]=0x02U; // Opcode for write register
	command[1]=Misc::UInt8(registerIndex>>8);
	command[2]=Misc::UInt8(registerIndex&0xffU);
	command[3]=value;
	setGetCur(Register,command,sizeof(command));
	
	/* Check the result: */
	if(command[0]!=0x02U||command[1]!=Misc::UInt8(registerIndex>>8)||command[2]!=Misc::UInt8(registerIndex&0xffU)||command[3]!=value)
		throw std::runtime_error("ESP770U::writeRegister: Invalid command buffer");
	}

Misc::UInt8 ESP770U::getCounter(void)
	{
	/* Read a UVC control: */
	Misc::UInt8 result;
	UVC::getCur(device,0,ExtensionUnit,Counter,&result,sizeof(result));
	
	return result;
	}

void ESP770U::setCounter(Misc::UInt8 newCounter)
	{
	/* Write a UVC control: */
	UVC::setCur(device,0,ExtensionUnit,Counter,&newCounter,sizeof(newCounter));
	}

void ESP770U::spiSetControl(Misc::UInt8 handle,Misc::UInt8 length)
	{
	/* Write a command to the control selector: */
	Misc::UInt8 command[16];
	memset(command,0,sizeof(command));
	command[0]=0x00U;
	command[1]=handle;
	command[2]=0x80U;
	command[3]=0x01U;
	command[9]=length;
	UVC::setCur(device,0,ExtensionUnit,Control,command,sizeof(command));
	}

void ESP770U::spiSetData(const Misc::UInt8* data,Misc::UInt16 length)
	{
	/* Write the data buffer to the data selector: */
	UVC::setCur(device,0,ExtensionUnit,Data,data,length);
	}

void ESP770U::spiGetData(Misc::UInt8* data,Misc::UInt16 length)
	{
	/* Read the data buffer from the data selector: */
	UVC::getCur(device,0,ExtensionUnit,Data,data,length);
	}

void ESP770U::writeRadio(const Misc::UInt8* data,Misc::UInt16 length)
	{
	/* Check if the data fits into the maximum buffer: */
	if(length>126)
		throw std::runtime_error("ESP770U::writeRadio: Data block too large");
	
	/* Copy given data block into a fixed-size buffer and calculate a checksum in the last byte: */
	Misc::UInt8 buffer[127];
	memset(buffer,0,sizeof(buffer));
	for(Misc::UInt16 i=0;i<length;++i)
		{
		buffer[i]=data[i];
		buffer[126]-=buffer[i];
		}
	
	/* Write the buffer: */
	spiSetControl(0x81,sizeof(buffer));
	spiSetData(buffer,sizeof(buffer));
	
	/* Read the buffer back: */
	spiSetControl(0x41,sizeof(buffer));
	spiGetData(buffer,sizeof(buffer));
	
	/* Clear the buffer and write it a second time: */
	memset(buffer,0,sizeof(buffer));
	spiSetControl(0x81,sizeof(buffer));
	spiSetData(buffer,sizeof(buffer));
	
	/* ... and read it back yet again: */
	spiSetControl(0x41,sizeof(buffer));
	spiGetData(buffer,sizeof(buffer));
	
	/* Calculate the returned buffer's checksum: */
	Misc::UInt8 checkSum=0x00U;
	for(int i=0;i<127;++i)
		checkSum+=buffer[i];
	
	/* Check the check sum and compare the second result to the original buffer: */
	if(checkSum!=0x00U||buffer[0]!=data[0]||buffer[1]!=data[1])
		throw std::runtime_error("ESP770U::writeRadio: Invalid return buffer");
	}

Misc::UInt8 ESP770U::queryFirmwareVersion(void)
	{
	/* Write a command to the register selector: */
	Misc::UInt8 command[4];
	command[0]=0xa0U; // Opcode for get firmware version
	command[1]=0x03U;
	command[2]=0x00U;
	command[3]=0x00U;
	setGetCur(Register,command,sizeof(command));
	
	/* Check the result: */
	if(command[0]!=0xa0U||command[2]!=0x00U||command[3]!=0x00U)
		throw std::runtime_error("ESP770U::queryFirmwareVersion: Invalid command buffer");
	
	return command[1];
	}

void ESP770U::readMemory(Misc::UInt32 address,Misc::UInt8* buffer,Misc::UInt16 length)
	{
	/* Get the current value of the command counter: */
	Misc::UInt8 counter=getCounter();
	
	/* Write a command to the control selector: */
	Misc::UInt8 command[16];
	memset(command,0,sizeof(command));
	command[0]=counter;
	command[1]=0x41U;
	command[2]=0x03;
	command[3]=0x01;
	command[5]=Misc::UInt8((address>>16)&0xffU);
	command[6]=Misc::UInt8((address>>8)&0xffU);
	command[7]=Misc::UInt8(address&0xffU);
	command[8]=Misc::UInt8(length>>8);
	command[9]=Misc::UInt8(length&0xffU);
	UVC::setCur(device,0,ExtensionUnit,Control,command,sizeof(command));
	
	/* Read the memory block from the data selector: */
	memset(buffer,0,length);
	UVC::getCur(device,0,ExtensionUnit,Data,buffer,length);
	
	/* Reset the command counter: */
	setCounter(counter);
	}

void ESP770U::initController(void)
	{
	/* Read and write a bunch of registers to do something important to the controller: */
	Misc::UInt8 value=readRegister(0xf05aU);
	if(value!=0x01U&&value!=0x03U)
		{
		// DEBUGGING
		std::cout<<"ESP770U::initController: Wrong value 0x"<<std::hex<<value<<std::dec<<" in register 0xf05a; continuing"<<std::endl;
		// Misc::throwStdErr("ESP770U::initController: Wrong value %u in register 0xf05a",value);
		}
	writeRegister(0xf05aU,0x01U);
	
	value=readRegister(0xf018U);
	if(value!=0x0eU)
		{
		// DEBUGGING
		std::cout<<"ESP770U::initController: Wrong value 0x"<<std::hex<<value<<std::dec<<" in register 0xf018; continuing"<<std::endl;
		// Misc::throwStdErr("ESP770U::initController: Wrong value %u in register 0xf018",value);
		}
	writeRegister(0xf018U,0x0fU);
	
	value=readRegister(0xf017U);
	if(value!=0xecU&&value!=0xedU)
		{
		// DEBUGGING
		std::cout<<"ESP770U::initController: Wrong value 0x"<<std::hex<<value<<std::dec<<" in register 0xf017; continuing"<<std::endl;
		// Misc::throwStdErr("ESP770U::initController: Wrong value %u in register 0xf017",value);
		}
	writeRegister(0xf017U,value|0x01U);
	writeRegister(0xf017U,value&~0x01U);
	
	writeRegister(0xf018U,0x0eU);
	}

void ESP770U::initRadio(void)
	{
	/* Wait for the radio to initialize: */
	usleep(50000);
	
	/* Write a bunch of buffers to the radio component to ... do stuff: */
	Misc::UInt8 command0[2]={0x01U,0x01U};
	writeRadio(command0,sizeof(command0));
	
	Misc::UInt8 command1[2]={0x11U,0x01U};
	writeRadio(command1,sizeof(command1));
	
	Misc::UInt8 value=readRegister(0xf014U);
	if(value!=0x1aU&&value!=0x1bU)
		{
		// DEBUGGING
		std::cout<<"ESP770U::initRadio: Wrong value 0x"<<std::hex<<value<<std::dec<<" in register 0xf014; continuing"<<std::endl;
		// Misc::throwStdErr("ESP770U::initRadio: Wrong value %u in register 0xf014",value);
		}
	
	Misc::UInt8 command2[2]={0x21U,0x01U};
	writeRadio(command2,sizeof(command2));
	
	Misc::UInt8 command3[3]={0x31U,0x01U};
	writeRadio(command3,sizeof(command3));
	}

void ESP770U::setupRadio(Misc::UInt32 radioId)
	{
	/* Send a sequence of commands to the radio component: */
	Misc::UInt8 command0[7];
	command0[0]=0x40U;
	command0[1]=0x10U;
	command0[2]=Misc::UInt8(radioId&0xffU);
	command0[3]=Misc::UInt8((radioId>>8)&0xffU);
	command0[4]=Misc::UInt8((radioId>>16)&0xffU);
	command0[5]=Misc::UInt8((radioId>>24)&0xffU);
	command0[6]=0x8cU;
	writeRadio(command0,sizeof(command0));
	
	Misc::UInt8 command1[10]={0x50U,0x11U,0xf4U,0x01U,0x00U,0x00U,0x67U,0xffU,0xffU,0xffU};
	writeRadio(command1,sizeof(command1));
	
	Misc::UInt8 command2[2]={0x61U,0x12U};
	writeRadio(command2,sizeof(command2));
	
	Misc::UInt8 command3[2]={0x71U,0x85U};
	writeRadio(command3,sizeof(command3));
	
	Misc::UInt8 command4[2]={0x81U,0x86U};
	writeRadio(command4,sizeof(command4));
	}

Misc::UInt16 ESP770U::readI2C(Misc::UInt8 address,Misc::UInt16 registerIndex)
	{
	/* Write a command to the I2C selector: */
	Misc::UInt8 command[6];
	command[0]=0x86U;
	command[1]=address;
	command[2]=Misc::UInt8(registerIndex>>8);
	command[3]=Misc::UInt8(registerIndex&0xffU);
	command[4]=0x00U;
	command[5]=0x00U;
	setGetCur(I2C,command,sizeof(command));
	
	/* Check the return buffer: */
	if(command[0]!=0x86U||command[4]!=0x00U||command[5]!=0x00U)
		throw std::runtime_error("ESP770U::readI2C: Invalid return buffer");
	
	return (Misc::UInt16(command[2])<<8)|Misc::UInt16(command[1]);
	}

void ESP770U::writeI2C(Misc::UInt8 address,Misc::UInt16 registerIndex,Misc::UInt16 value)
	{
	/* Write a command to the I2C selector: */
	Misc::UInt8 command[6];
	command[0]=0x06U;
	command[1]=address;
	command[2]=Misc::UInt8(registerIndex>>8);
	command[3]=Misc::UInt8(registerIndex&0xffU);
	command[4]=Misc::UInt8(value>>8);
	command[5]=Misc::UInt8(value&0xffU);
	setGetCur(I2C,command,sizeof(command));
	
	/* Check the return buffer: */
	if(command[0]!=0x06U||command[1]!=address||command[2]!=Misc::UInt8(registerIndex>>8)||command[3]!=Misc::UInt8(registerIndex&0xffU)||command[4]!=Misc::UInt8(value>>8)||command[5]!=Misc::UInt8(value&0xffU))
		throw std::runtime_error("ESP770U::writeI2C: Invalid return buffer");
	}

/***********************
Methods of class AR0134:
***********************/

Misc::UInt16 AR0134::readRegister(Misc::UInt16 registerIndex)
	{
	return esp770u.readI2C(I2CAddress,registerIndex);
	}

void AR0134::writeRegister(Misc::UInt16 registerIndex,Misc::UInt16 value)
	{
	esp770u.writeI2C(I2CAddress,registerIndex,value);
	}

void AR0134::dumpRegisters(void)
	{
	/* Dump all interesting registers: */
	std::cout<<"AR0134 register contents:"<<std::endl;
	std::cout<<"  Capture window        "<<readRegister(XAddrStart)<<", "<<readRegister(YAddrStart)<<", "<<readRegister(XAddrEnd)<<", "<<readRegister(YAddrEnd)<<std::endl;
	std::cout<<"  FrameLengthLines      "<<readRegister(FrameLengthLines)<<std::endl;
	std::cout<<"  LineLengthPck         "<<readRegister(LineLengthPck)<<std::endl;
	std::cout<<"  CoarseIntegrationTime "<<readRegister(CoarseIntegrationTime)<<std::endl;
	std::cout<<"  FineIntegrationTime   "<<readRegister(FineIntegrationTime)<<std::endl;
	Misc::UInt16 resetRegister=readRegister(ResetRegister);
	std::cout<<"  ResetRegister         "<<std::setfill('0')<<std::hex<<std::setw(4)<<resetRegister<<std::dec<<std::endl;
	std::cout<<"    Stream                "<<((resetRegister&Stream)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    LockReg               "<<((resetRegister&LockReg)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    StdbyEof              "<<((resetRegister&StdbyEof)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    DrivePins             "<<((resetRegister&DrivePins)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    ParallelEn            "<<((resetRegister&ParallelEn)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    GpiEn                 "<<((resetRegister&GpiEn)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    MaskBad               "<<((resetRegister&MaskBad)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    RestartBad            "<<((resetRegister&RestartBad)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    ForcedPllOn           "<<((resetRegister&ForcedPllOn)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    SmiaSerialiserDis     "<<((resetRegister&SmiaSerialiserDis)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    GroupedParameterHold  "<<((resetRegister&GroupedParameterHold)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"  DataPedestal          "<<readRegister(DataPedestal)<<std::endl;
	Misc::UInt16 gpiStatus=readRegister(GpiStatus);
	std::cout<<"  GpiStatus             "<<std::setfill('0')<<std::hex<<std::setw(4)<<gpiStatus<<std::dec<<std::endl;
	std::cout<<"    Saddr                 "<<((gpiStatus&Saddr)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    OeN                   "<<((gpiStatus&OeN)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    Trigger               "<<((gpiStatus&Trigger)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    Standby               "<<((gpiStatus&Standby)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"  RowSpeed              "<<std::setfill('0')<<std::hex<<std::setw(4)<<readRegister(RowSpeed)<<std::dec<<std::endl;
	std::cout<<"  DigitalBinning        "<<std::setfill('0')<<std::hex<<std::setw(4)<<readRegister(DigitalBinning)<<std::dec<<std::endl;
	std::cout<<"  FrameCount            "<<readRegister(FrameCount)<<std::endl;
	std::cout<<"  FrameStatus           "<<std::setfill('0')<<std::hex<<std::setw(4)<<readRegister(FrameStatus)<<std::dec<<std::endl;
	Misc::UInt16 readMode=readRegister(ReadMode);
	std::cout<<"  ReadMode              "<<std::setfill('0')<<std::hex<<std::setw(4)<<readMode<<std::dec<<std::endl;
	std::cout<<"    Horizontal mirroring  "<<((readMode&HorizMirror)?"on":"off")<<std::endl;
	std::cout<<"    Vertical flipping     "<<((readMode&VertFlip)?"on":"off")<<std::endl;
	std::cout<<"  DarkControl           "<<std::setfill('0')<<std::hex<<std::setw(4)<<readRegister(DarkControl)<<std::dec<<std::endl;
	std::cout<<"  Flash                 "<<std::setfill('0')<<std::hex<<std::setw(4)<<readRegister(Flash)<<std::dec<<std::endl;
	std::cout<<"  GlobalGain            "<<readRegister(GlobalGain)<<std::endl;
	Misc::UInt16 embeddedDataControl=readRegister(EmbeddedDataControl);
	std::cout<<"  EmbeddedDataControl   "<<std::setfill('0')<<std::hex<<std::setw(4)<<embeddedDataControl<<std::dec<<std::endl;
	std::cout<<"    EmbeddedStatsEn       "<<((embeddedDataControl&EmbeddedStatsEn)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    EmbeddedData          "<<((embeddedDataControl&EmbeddedData)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"  DatapathSelect        "<<std::setfill('0')<<std::hex<<std::setw(4)<<readRegister(DatapathSelect)<<std::dec<<std::endl;
	Misc::UInt16 digitalTest=readRegister(DigitalTest);
	std::cout<<"  DigitalTest           "<<std::setfill('0')<<std::hex<<std::setw(4)<<digitalTest<<std::dec<<std::endl;
	std::cout<<"    ColGain               "<<((digitalTest&ColGainMask)>>4)<<std::endl;
	std::cout<<"    MonoChrome            "<<((digitalTest&MonoChrome)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    ColGainCb             "<<((digitalTest&ColGainCbMask)>>8)<<std::endl;
	std::cout<<"    EnableShortLlpck      "<<((digitalTest&EnableShortLlpck)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    ContextB              "<<((digitalTest&ContextB)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    PllCompleteBypass     "<<((digitalTest&PllCompleteBypass)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"  TempsensData          "<<readRegister(TempsensData)<<std::endl;
	std::cout<<"  ColumnCorrection      "<<std::setfill('0')<<std::hex<<std::setw(4)<<readRegister(ColumnCorrection)<<std::dec<<std::endl;
	Misc::UInt16 aeCtrlReg=readRegister(AeCtrlReg);
	std::cout<<"  AeCtrlReg             "<<std::setfill('0')<<std::hex<<std::setw(4)<<aeCtrlReg<<std::dec<<std::endl;
	std::cout<<"    AeEnable              "<<((aeCtrlReg&AeEnable)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    AutoAgEn              "<<((aeCtrlReg&AutoAgEn)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    AutoDgEn              "<<((aeCtrlReg&AutoDgEn)!=0x0000?"on":"off")<<std::endl;
	std::cout<<"    MinAnaGain            "<<((aeCtrlReg&MinAnaGainMask)>>5)<<std::endl;
	std::cout<<"  AeLumaTargetReg       "<<readRegister(AeLumaTargetReg)<<std::endl;
	std::cout<<"  AeMinEvStepReg        "<<readRegister(AeMinEvStepReg)<<std::endl;
	std::cout<<"  AeMaxEvStepReg        "<<readRegister(AeMaxEvStepReg)<<std::endl;
	std::cout<<"  AeMaxExposureReg      "<<readRegister(AeMaxExposureReg)<<std::endl;
	std::cout<<"  AeMinExposureReg      "<<readRegister(AeMinExposureReg)<<std::endl;
	}

void AR0134::init(void)
	{
	usleep(100000);
	
	/* Read chip version and revision number: */
	unsigned int version=readRegister(ChipVersionReg);
	unsigned int revision=readRegister(RevisionNumber);
	if(version!=0x2406U||revision!=0x1300U)
		Misc::throwStdErr("AR0134::init: Unsupported chip version %u.%u",version,revision);
	
	/* Check the chip's digital test mode: */
	unsigned int testMode=readRegister(DigitalTest);
	if(testMode!=MonoChrome)
		throw std::runtime_error("AR0134: Unexpected camera mode");
	
	/* Enable embedded data and statistics: */
	Misc::UInt16 edc=readRegister(EmbeddedDataControl);
	writeRegister(EmbeddedDataControl,edc|EmbeddedStatsEn|EmbeddedData);
	}

bool AR0134::getHorizontalFlip(void)
	{
	return (readRegister(ReadMode)&HorizMirror)!=0x0U;
	}

bool AR0134::getVerticalFlip(void)
	{
	return (readRegister(ReadMode)&VertFlip)!=0x0U;
	}

void AR0134::setFlip(bool horizontalFlip,bool verticalFlip)
	{
	Misc::UInt16 readMode=readRegister(ReadMode);
	if(horizontalFlip)
		readMode|=HorizMirror;
	else
		readMode&=~HorizMirror;
	if(verticalFlip)
		readMode|=VertFlip;
	else
		readMode&=~VertFlip;
	writeRegister(ReadMode,readMode);
	}

bool AR0134::getAutoExposure(void)
	{
	return (readRegister(AeCtrlReg)&AeEnable)!=0x0000U;
	}

void AR0134::setAutoExposure(bool enable,bool adjustAnalogGain,bool adjustDigitalGain)
	{
	/* Set or clear the enable flag in the automatic exposure control register: */
	Misc::UInt16 ae=readRegister(AeCtrlReg);
	ae=ae&~(AeEnable|AutoAgEn|AutoDgEn);
	if(enable)
		ae=ae|AeEnable;
	if(adjustAnalogGain)
		ae=ae|AutoAgEn;
	if(adjustDigitalGain)
		ae=ae|AutoDgEn;
	writeRegister(AeCtrlReg,ae);
	}

Misc::UInt16 AR0134::getGain(void)
	{
	return readRegister(GlobalGain);
	}

void AR0134::setGain(Misc::UInt16 gain)
	{
	writeRegister(GlobalGain,gain);
	}

void AR0134::setWindow(Misc::UInt16 x,Misc::UInt16 y,Misc::UInt16 width,Misc::UInt16 height)
	{
	/* Set the window's top-left corner: */
	writeRegister(YAddrStart,y);
	writeRegister(XAddrStart,x);
	
	/* Set the window's bottom-right corner: */
	writeRegister(YAddrEnd,y+height-1);
	writeRegister(XAddrEnd,x+width-1);
	}

Misc::UInt16 AR0134::getTotalWidth(void)
	{
	return readRegister(LineLengthPck);
	}

Misc::UInt16 AR0134::getTotalHeight(void)
	{
	return readRegister(FrameLengthLines);
	}

void AR0134::setFrameTimings(bool minBlank)
	{
	/* Read the current total line length in pixels: */
	Misc::UInt16 lineLength=readRegister(LineLengthPck);
	
	/* Set the capture window to the entire sensor: */
	setWindow(0,0,1280,960);
	// setWindow(128,96,1024,768);
	
	/* Set tight or loose total line length: */
	writeRegister(LineLengthPck,1280+(minBlank?108:218));
	
	/* Enable or disable tight line timing in the digital test register: */
	Misc::UInt16 dt=readRegister(DigitalTest);
	if(minBlank)
		dt=dt|EnableShortLlpck;
	else
		dt=dt&(~EnableShortLlpck);
	writeRegister(DigitalTest,dt);
	
	/* Set tight or loose total frame height: */
	writeRegister(FrameLengthLines,960+(minBlank?23:37));
	}

Misc::UInt16 AR0134::getCoarseExposureTime(void)
	{
	return readRegister(CoarseIntegrationTime);
	}

void AR0134::setCoarseExposureTime(Misc::UInt16 coarseExposure)
	{
	writeRegister(CoarseIntegrationTime,coarseExposure);
	}

Misc::UInt16 AR0134::getFineExposureTime(void)
	{
	return readRegister(FineIntegrationTime);
	}

void AR0134::setFineExposureTime(Misc::UInt16 fineExposure)
	{
	writeRegister(FineIntegrationTime,fineExposure);
	}

Misc::UInt32 AR0134::getExposureTime(void)
	{
	/* Retrieve total frame width and coarse and fine exposure times: */
	Misc::UInt32 frameWidth=readRegister(LineLengthPck);
	Misc::UInt32 coarseExposure=readRegister(CoarseIntegrationTime);
	Misc::UInt32 fineExposure=readRegister(FineIntegrationTime);
	
	/* Calculate total exposure time: */
	return coarseExposure*frameWidth+fineExposure;
	}

void AR0134::setExposureTime(Misc::UInt32 exposureTime)
	{
	/* Retrieve total frame width: */
	Misc::UInt32 frameWidth=readRegister(LineLengthPck);
	
	/* Break total exposure time into coarse and fine components: */
	writeRegister(CoarseIntegrationTime,Misc::UInt16(exposureTime/frameWidth));
	writeRegister(FineIntegrationTime,Misc::UInt16(exposureTime%frameWidth));
	}

void AR0134::setSync(bool enable)
	{
	/* Enable or disable synchronization via the reset register: */
	Misc::UInt16 r=readRegister(ResetRegister);
	r=r&~(Stream|GpiEn|ForcedPllOn);
	if(enable)
		r|=GpiEn|ForcedPllOn;
	else
		r|=Stream;
	writeRegister(ResetRegister,r);
	}

}

/************************************
Methods of class OculusRiftCV1Camera:
************************************/

void OculusRiftCV1Camera::transferCallback(USB::TransferPool::Transfer* transfer)
	{
	/* Extract data from the transfer buffer: */
	const libusb_iso_packet_descriptor* packetDescriptorPtr=&transfer->getPacketDescriptor(0);
	const Misc::UInt8* packetDataPtr=transfer->getData();
	for(int packetIndex=0;packetIndex<transfer->getNumPackets();++packetIndex)
		{
		/* Check if the packet contains valid data: */
		size_t packetSize=packetDescriptorPtr->actual_length;
		size_t headerSize=packetDataPtr[0];
		if(packetDescriptorPtr->status==LIBUSB_TRANSFER_COMPLETED&&packetSize>=12&&headerSize==12&&(packetDataPtr[1]&0x40U)==0x00U)
			{
			/* Check if this packet belongs to a new frame: */
			if(frameId!=(packetDataPtr[1]&0x01U))
				{
				/* Start a new frame: */
				frameId=packetDataPtr[1]&0x01U;
				framePtr=frameBuffer;
				frameRemainder=size_t(frameSize[1])*size_t(frameSize[0]);
				
				/* Extract the new frame's presentation time: */
				Misc::UInt32 pt=Misc::UInt32(packetDataPtr[5]);
				pt=(pt<<8)|Misc::UInt32(packetDataPtr[4]);
				pt=(pt<<8)|Misc::UInt32(packetDataPtr[3]);
				pt=(pt<<8)|Misc::UInt32(packetDataPtr[2]);
				framePresentationTime=pt;
				}
			
			/* Check if the current frame is incomplete: */
			if(frameRemainder>0)
				{
				/* Copy the packet's payload into the current frame: */
				packetSize-=headerSize;
				if(packetSize<=frameRemainder)
					{
					/* Copy the packet's data into the frame: */
					memcpy(framePtr,packetDataPtr+headerSize,packetSize);
					framePtr+=packetSize;
					frameRemainder-=packetSize;
					
					/* Check if the frame is complete: */
					if(frameRemainder==0&&streamingCallback!=0)
						{
						/* Unlink the finished frame buffer and allocate a new one: */
						unsigned char* finishedFrame=frameBuffer;
						frameBuffer=new unsigned char[size_t(frameSize[1])*size_t(frameSize[0])];
						
						/* Hand the finished frame to the streaming client: */
						(*streamingCallback)(finishedFrame);
						}
					}
				else
					{
					/* Frame is overflowing; mark it as full and ignore it: */
					frameRemainder=0;
					}
				}
			}
		
		/* Go to the next isochronous transfer packet: */
		packetDataPtr+=packetDescriptorPtr->length;
		++packetDescriptorPtr;
		}
	
	/* Put the just-processed transfer back into the pool: */
	transferPool->release(transfer);
	}

namespace {

/**************
Helper classes:
**************/

class OculusRiftCV1CameraMatcher // Class to match an Oculus Rift CV1 camera device
	{
	/* Methods: */
	public:
	bool operator()(const libusb_device_descriptor& dd) const
		{
		/* Check for Oculus Rift CV1 camera devices: */
		return dd.idVendor==0x2833U&&dd.idProduct==0x0211U;
		}
	};

}

namespace {

/* Lens distortion correction parameters from some camera: */
OculusRiftCV1Camera::Point center(655.052,475.083);
double kappas[3]={5.16403e-07,2.44492e-13,6.881e-19};
double rhos[2]={-8.66716e-07, 8.37108e-07};

}

OculusRiftCV1Camera::OculusRiftCV1Camera(unsigned int deviceIndex)
	:streamingCallback(0),
	 transferPool(0),
	 frameBuffer(0)
	{
	/* Get the index-th Oculus Rift CV1 camera device from the current USB context: */
	USB::DeviceList deviceList;
	USB::Device::operator=(deviceList.getDevice(OculusRiftCV1CameraMatcher(),deviceIndex));
	if(!isValid())
		Misc::throwStdErr("OculusRiftCV1Camera::OculusRiftCV1Camera: Fewer than %u Oculus Rift CV1 camera devices detected",deviceIndex+1);
	
	/* Open the USB device: */
	open();
	
	/* Claim the UVC control interface, detaching any kernel drivers that already had it: */
	claimInterface(0,true);
	
	/* Query the camera controller's firmware version: */
	ESP770U controller(*this);
	unsigned int firmwareVersion=controller.queryFirmwareVersion();
	
	// DEBUGGING
	std::cout<<"OculusRiftCV1Camera: ESP770U camera controller's firmware version: "<<firmwareVersion<<std::endl;
	
	/* Initialize the camera controller: */
	controller.initController();
	
	/* Initialize the camera controller's radio component: */
	controller.initRadio();
	
	/* Set the expected frame size: */
	frameSize[0]=1280;
	frameSize[1]=960;
	
	/* Retrieve the camera's calibration data from the controller's non-volatile memory: */
	Misc::UInt8 calData[128];
	controller.readMemory(0x1d000U,calData,sizeof(calData));
	
	/* Extract calibration parameters from the buffer: */
	fy=fx=*(float*)(calData+0x30);
	cx=*(float*)(calData+0x34);
	cy=*(float*)(calData+0x38);
	k[0]=*(float*)(calData+0x48);
	k[1]=*(float*)(calData+0x4c);
	k[2]=*(float*)(calData+0x50);
	k[3]=*(float*)(calData+0x54);
	
	// DEBUGGING
	std::cout<<"OculusRiftCV1Camera: Calibration parameters:"<<std::endl;
	std::cout<<"  Focal length: "<<fx<<", "<<fy<<std::endl;
	std::cout<<"  Focus point: "<<cx<<", "<<cy<<std::endl;
	std::cout<<"  Lens distortion: "<<k[0]<<", "<<k[1]<<", "<<k[2]<<", "<<k[3]<<std::endl;
	
	/* Calculate the maximum undistortion radius: */
	double maxR=Math::sqrt(Math::sqr(center[0])+Math::sqr(center[1]));
	double lastCorrectedR(0);
	for(double r=1.0;r<maxR;r+=1.0)
		{
		/* Calculate the radial correction coefficient: */
		double r2=r*r;
		double radial(0);
		for(int i=2;i>=0;--i)
			radial=(radial+kappas[i])*r2;
		radial+=1.0;
		
		/* Calculate the corrected radius and bail out if it got smaller than the last corrected radius: */
		double correctedR=r*radial;
		if(correctedR<=lastCorrectedR)
			maxR=r-1.0;
		lastCorrectedR=correctedR;
		}
	maxR2=Math::sqr(maxR);
	}

OculusRiftCV1Camera::~OculusRiftCV1Camera(void)
	{
	/* Stop streaming if still active: */
	if(transferPool!=0)
		stopStreaming();
	
	/* Release the UVC control interface: */
	releaseInterface(0);
	}

bool OculusRiftCV1Camera::canUndistort(const OculusRiftCV1Camera::Point& pixel) const
	{
	return Geometry::sqr(pixel-center)<maxR2;
	}

OculusRiftCV1Camera::Point OculusRiftCV1Camera::undistort(const OculusRiftCV1Camera::Point& pixel) const
	{
	/*********************************************************************
	I have no idea what distortion correction formula they're using, so
	this is pure guesswork, and obviously wrong.
	For the time being, use distortion correction formula and parameters
	extracted from a random Oculus Rift CV1 camera that I had lying
	around.
	*********************************************************************/
	
	#if 0 // Don't use correction parameters from firmware
	
	/* Convert the given point from pixel to tangent space: */
	double tx=(pixel[0]-cx)/fx;
	double ty=(pixel[1]-cy)/fy;
	
	/* Apply the undistortion formula and return the point back in pixel space: */
	double tx2=tx*tx;
	double ty2=ty*ty;
	double txty=tx*ty;
	double r2=tx2+ty2;
	#if 0 // Radial and tangential correction
	double radial=1.0+(k[0]+k[1]*r2)*r2;
	return Point((tx*radial+2.0*k[2]*txty+k[3]*(r2+2.0*tx2))*fx+cx,
	             (ty*radial+k[2]*(r2+2.0*ty2)+2.0*k[3]*txty)*fy+cy);
	#elif 0 // Radial-only correction with rational radial term
	double radial=(1.0+(k[0]+k[2]*r2)*r2)/(1.0+(k[1]+k[3]*r2)*r2);
	return Point(tx*radial*fx+cx,
	             ty*radial*fy+cy);
	#else // Radial-only correction
	double radial=1.0+(k[0]+(k[1]+(k[2]+k[3]*r2)*r2)*r2)*r2;
	return Point(tx*radial*fx+cx,
	             ty*radial*fy+cy);
	#endif
	
	#else // Use correction parameters reconstructed from some camera
	
	/* Calculate radial correction factor: */
	Point::Vector d=pixel-center;
	double r2=d.sqr();
	double radial(0);
	for(int i=2;i>=0;--i)
		radial=(radial+kappas[i])*r2;
	radial+=1.0;
	
	/* Apply radial and tangential distortion correction: */
	return Point(center[0]+d[0]*radial+2.0*rhos[0]*d[0]*d[1]+rhos[1]*(r2+2.0*d[0]*d[0]), // Tangential distortion formula in x
	             center[1]+d[1]*radial+rhos[0]*(r2+2.0*d[1]*d[1])+2.0*rhos[1]*d[0]*d[1]); // Tangential distortion formula in y
	
	#endif
	}

void OculusRiftCV1Camera::startStreaming(OculusRiftCV1Camera::StreamingCallback* newStreamingCallback)
	{
	/* Throw an exception if already streaming: */
	if(transferPool!=0)
		throw std::runtime_error("OculusRiftCV1Camera::startStreaming: Already streaming");
	
	/* Store the streaming callback: */
	streamingCallback=newStreamingCallback;
	
	/* Set up the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	sensor.init();
	
	// sensor.setWindow(0,0,1280,960);
	sensor.setFrameTimings(true);
	// sensor.setCoarseExposureTime(26);
	// sensor.setFineExposureTime(646);
	sensor.setGain(128);
	sensor.setCoarseExposureTime(400);
	sensor.setFineExposureTime(15);
	sensor.setSync(false);
	// sensor.setAutoExposure(true);
	
	/* Claim the UVC data interface: */
	claimInterface(1,true);
	
	/* Assemble a UVC probe control request buffer to set and check the camera's current video format: */
	IO::FixedMemoryFile uvcProbe(26);
	uvcProbe.setEndianness(Misc::LittleEndian);
	uvcProbe.write<Misc::UInt16>(0x0000U); // bmHint
	uvcProbe.write<Misc::UInt8>(1); // bFormatIndex
	uvcProbe.write<Misc::UInt8>(4); // bFrameIndex
	uvcProbe.write<Misc::UInt32>(192000U); // dwFrameInterval
	uvcProbe.write<Misc::UInt16>(0); // wKeyFrameRate
	uvcProbe.write<Misc::UInt16>(0); // wPFrameRate
	uvcProbe.write<Misc::UInt16>(0); // wCompQuality
	uvcProbe.write<Misc::UInt16>(0); // wCompWindowSize
	uvcProbe.write<Misc::UInt16>(0); // wDelay
	uvcProbe.write<Misc::UInt32>(frameSize[1]*frameSize[0]); // dwMaxVideoFrameSize
	uvcProbe.write<Misc::UInt32>(3072); // dwMaxPayloadTransferSize
	#if 0 // Omit unnecessary fields at the end
	uvcProbe.write<Misc::UInt32>(0); // dwClockFrequency
	uvcProbe.write<Misc::UInt8>(0); // bmFramingInfo
	uvcProbe.write<Misc::UInt8>(0); // bPreferredVersion
	uvcProbe.write<Misc::UInt8>(0); // bMinVersion
	uvcProbe.write<Misc::UInt8>(0); // bMaxVersion
	#endif
	
	/* Probe the control: */
	UVC::setCur(*this,1,0,1,static_cast<const Misc::UInt8*>(uvcProbe.getMemory()),uvcProbe.getSize());
	
	/* Retrieve the probe result: */
	IO::FixedMemoryFile uvcProbeResult(26);
	uvcProbeResult.setEndianness(Misc::LittleEndian);
	UVC::getCur(*this,1,0,1,static_cast<Misc::UInt8*>(uvcProbeResult.getMemory()),uvcProbeResult.getSize());
	
	/* Check if the probe result matches expectations: */
	if(uvcProbeResult.read<Misc::UInt16>()!=0x0000U||
	   uvcProbeResult.read<Misc::UInt8>()!=1||
	   uvcProbeResult.read<Misc::UInt8>()!=4||
	   uvcProbeResult.read<Misc::UInt32>()!=200000||
	   uvcProbeResult.read<Misc::UInt16>()!=0||
	   uvcProbeResult.read<Misc::UInt16>()!=0||
	   uvcProbeResult.read<Misc::UInt16>()!=0||
	   uvcProbeResult.read<Misc::UInt16>()!=0||
	   uvcProbeResult.read<Misc::UInt16>()!=0||
	   uvcProbeResult.read<Misc::UInt32>()!=frameSize[1]*frameSize[0]||
	   uvcProbeResult.read<Misc::UInt32>()!=8192)
		{
		// DEBUGGING: */
		std::cout<<"OculusRiftCV1Camera::startStreaming: Mismatching expected video format; continuing"<<std::endl;
		}
	
	/* Commit the probe result: */
	UVC::setCur(*this,1,0,2,static_cast<const Misc::UInt8*>(uvcProbeResult.getMemory()),uvcProbeResult.getSize());
	
	/* Request the camera device's alternate setting for video streaming: */
	setAlternateSetting(1,2);
	
	/* Allocate a frame buffer: */
	size_t frameBufferSize=size_t(frameSize[1])*size_t(frameSize[0]);
	frameBuffer=new unsigned char[frameBufferSize];
	frameId=0; // Just a guess; this will be set to the correct value when the first frame packet arrives
	framePtr=frameBuffer;
	frameRemainder=frameBufferSize;
	
	/* Set up an isochronous transfer pool to receive image data: */
	transferPool=new USB::TransferPool(7,24,16384);
	
	/* Submit the transfer pool to start streaming: */
	transferPool->submit(*this,0x81,7,Misc::createFunctionCall(this,&OculusRiftCV1Camera::transferCallback));
	
	usleep(1000000);
	sensor.setCoarseExposureTime(800);
	sensor.setFineExposureTime(0);
	sensor.setGain(128);
	
	// DEBUGGING
	// sensor.dumpRegisters();
	}

void OculusRiftCV1Camera::stopStreaming(void)
	{
	/* Cancel all pending isochronous transfers: */
	transferPool->cancel();
	
	/* Delete the isochronous transfer pool: */
	delete transferPool;
	transferPool=0;
	
	/* Delete the current frame buffer: */
	delete[] frameBuffer;
	frameBuffer=0;
	
	/* Release the UVC data interface: */
	releaseInterface(1);
	
	/* Delete the streaming callback: */
	delete streamingCallback;
	streamingCallback=0;
	}

bool OculusRiftCV1Camera::getHorizontalFlip(void)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Return the horizontal flip flag: */
	return sensor.getHorizontalFlip();
	}

bool OculusRiftCV1Camera::getVerticalFlip(void)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Return the vertical flip flag: */
	return sensor.getVerticalFlip();
	}

void OculusRiftCV1Camera::setFlip(bool horizontalFlip,bool verticalFlip)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Set the horizontal and vertical flip flags: */
	sensor.setFlip(horizontalFlip,verticalFlip);
	}

bool OculusRiftCV1Camera::getAutoExposure(void)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Return the auto exposure flag: */
	return sensor.getAutoExposure();
	}

void OculusRiftCV1Camera::setAutoExposure(bool enable,bool adjustAnalogGain,bool adjustDigitalGain)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Set the auto exposure flags: */
	sensor.setAutoExposure(enable,adjustAnalogGain,adjustDigitalGain);
	}

unsigned int OculusRiftCV1Camera::getGain(void)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Return the global gain value: */
	return sensor.getGain();
	}

void OculusRiftCV1Camera::setGain(unsigned int newGain)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Set the global gain value: */
	sensor.setGain(newGain);
	}

unsigned int OculusRiftCV1Camera::getExposureTime(void)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Return the total exposure time: */
	return sensor.getExposureTime();
	}

void OculusRiftCV1Camera::setExposureTime(unsigned int newExposureTime)
	{
	/* Access the camera's imaging controller: */
	ESP770U controller(*this);
	AR0134 sensor(controller);
	
	/* Set the coarse exposure time: */
	sensor.setCoarseExposureTime(newExposureTime);
	}
