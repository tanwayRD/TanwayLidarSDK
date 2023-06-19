/*
* Software License Agreement (BSD License)
*
*  Copyright (c) Tanway science and technology co., LTD.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without modification,
*  are permitted provided  that the following conditions are met:
*
*   1.Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*   2.Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*   3.Neither the name of the copyright holder(s) nor the names of its  contributors
*     may be used to endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
#pragma once
#include "CommomHeader.h"

#define USE_EXCEPTION_ERROR(e, str)\
TWException exception;\
exception.SetErrorCode(e, str);\
std::lock_guard<std::mutex> lock(*m_mutex);\
if (m_funcException) m_funcException(exception);

#define USE_EXCEPTION_TIPS(e, str)\
TWException exception;\
exception.SetTipsCode(e, str);\
std::lock_guard<std::mutex> lock(*m_mutex);\
if (m_funcException) m_funcException(exception);

class TWException
{
public:
	TWException();
	~TWException();

	enum ErrorCode
	{
		TWEC_ERROR_NONE = 0x00,						///NONE
		TWEC_ERROR_INIT_SOCKET = (0x01 << 6),				///Init socket error!
		TWEC_ERROR_CREATE_SOCKET_POINT = (0x01 << 7),		///Create point cloud socket error!
		TWEC_ERROR_CREATE_SOCKET_GPS = (0x01 << 8),		///Create gps socket error!
		TWEC_ERROR_BIND_POINT = (0x01 << 9),				///Bind port for point cloud socket error!
		TWEC_ERROR_BIND_GPS = (0x01 << 10),					///Bind port for gps socket error!
		TWEC_ERROR_SETOPT_TIMEOUT_POINT = (0x01 << 11),		///Failed to set point cloud socket timeout!
		TWEC_ERROR_SETOPT_TIMEOUT_GPS = (0x01 << 12),		///Failed to set gps socket timeout!
		TWEC_ERROR_SOCKET_RECV_POINT = (0x01 << 13),		///The point cloud socket received failed and will exit!
		TWEC_ERROR_SOCKET_RECV_GPS = (0x01 << 14),			///The gps socket received failed and will exit!

		TWEC_ERROR_CREATE_SOCKET_DIF = (0x01 << 15),		///Create dif socket error!
		TWEC_ERROR_BIND_DIF = (0x01 << 16),					///Bind port for dif socket error!
		TWEC_ERROR_SETOPT_TIMEOUT_DIF = (0x01 << 17),		///Failed to set dif socket timeout!
		TWEC_ERROR_SOCKET_RECV_DIF = (0x01 << 18),			///The dif socket received failed and will exit!

		TWEC_ERROR_CREATE_SOCKET_SEND = (0x01 << 19),		///Create send socket error!
		TWEC_ERROR_SOCKET_SEND_FAILED = (0x01 << 20),		///Send data error!

		TWEC_ERROR_OPEN_PCAP_FAILED = 0x00,			///Open pcap file failed!
		TWEC_ERROR_PCAP_FILE_INVALID = 0x00,		///The pcap file is invalid!
	};
	enum TipsCode
	{
		TWEC_TIPS_NONE = 0x00,				///NONE
		LIDAR_ERROR_TIMESTAMP = (0x01 << 0),	//Lidar not synchronized in time!
		LIDAR_WARNING_HARDWARE = (0x01 << 1),	//LiDAR hardware is in a degraded state!
		LIDAR_ERROR_HARDWARE = (0x01 << 2),		//LiDAR hardware is in a faulty state!
		LIDAR_ERROR_UNCALIBRATED = (0x01 << 3),	 //Lidar has not been EOL calibrated!
		LIDAR_ERROR_SHELTERED = (0x01 << 4),	 //Lidar is obstructed!
		LIDAR_ERROR_DUST = (0x01 << 5),	 //Lidar is dirty!

		TWEC_TIPS_TIMEOUT_POINT = (0x01 << 23),		///Receive point data time out!
		TWEC_TIPS_TIMEOUT_GPS = (0x01 << 24),		///Receive gps data time out!
		TWEC_TIPS_EXIT_POINT = 0x00,		///The point cloud data receiver thread has exited!
		TWEC_TIPS_EXIT_GPS = 0x00,			///The gps data receiver thread has exited!
		TWEC_TIPS_EXIT_DECODE = 0x00,		///The decode package thread has exited!
		TWEC_TIPS_TIMEOUT_DIF = (0x01 << 25),		///Receive dif data time out!
		TWEC_TIPS_EXIT_DIF = 0x00,			///The dif data receiver thread has exited!

		TWEC_TIPS_OPEN_PCAP_SUCCESS = 0x00,	///Open pcap file successed!
		TWEC_TIPS_PCAP_EXIT = 0x00,			///Exit reading the PCAP file!
		TWEC_TIPS_REPEAT_PLAY = 0x00,		///Repeat to read!
		TWEC_TIPS_NOMATCH_DEVICE = (0x01 << 21),	///Lidar type and protocol data do not match!
		TWEC_TIPS_INVALID_DEVICE = (0x01 << 22),	///Invalid device type!
	};


	std::string ToString() const;
	ErrorCode GetErrorCode() const;
	TipsCode GetTipsCode() const;

	void SetErrorCode(ErrorCode code, std::string msg);
	void SetTipsCode(TipsCode code, std::string msg);

private:
	ErrorCode m_exceptionErrorCode;
	TipsCode m_exceptionTipsCode;
	std::string m_msg;
};

