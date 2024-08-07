/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file je_helper.h
 * @author Kevin Chen <kevindsp@qq.com>
 */

#pragma once

#include <cstdint>

#ifndef JE_READ_BUFFER_SIZE
#define JE_READ_BUFFER_SIZE 512 ///< buffer size for the read() call. Messages can be longer than that.
#endif

enum class JECallbackType {
	/**
	 * Read data from device.
	 * data1: points to a buffer to be written to. The first sizeof(int) bytes contain the
	 *        timeout in ms when calling the method.
	 * data2: buffer length in bytes. Less bytes than this can be read.
	 * return: num read bytes, 0 on timeout (the method can actually also return 0 before
	 *         the timeout happens).
	 */
	readDeviceData = 0,

	/**
	 * Write data to device
	 * data1: data to be written
	 * data2: number of bytes to write
	 * return: num written bytes
	 */
	writeDeviceData,

	/**
	 * set Baudrate
	 * data1: ignored
	 * data2: baudrate
	 * return: 1 on success
	 */
	setBaudrate,
	/**
	 * set PWM value for throttle
	 * data1: thr_pwm
	 * data2: str_pwm
	 * return: 1 on success
	 */
	setPWM,

	publish,

	getARM,

	getThr,

};

enum JESTATUS
{
 OFF = 0,
 WAITFORRPM = 1,
 IGNITE = 2,
 ACCELERATE = 3,
 STABILIZE = 4,
 LEARNLO = 6,
 SLOWDOWN = 8,
 RUN = 11,
 ACCDELAY = 12,
 SPEEDREG = 13,
 TWOSHAFTREG = 14,
 PREHEAT1 = 15,
 PREHEAT2 = 16,
 AUTOBLEED = 17,
 KEROSFULLON = 19,
};

enum FAULTSTATUS
{
	NODEFINE = 0,
	PWMSHUTDOWN = 1,
	OVERTEMP = 2,
	INGFAIL = 3,
	ACCTIMEOUT = 4,
	ACCSLOW = 5,
	OVERRPM = 6,
	LOWRPM = 7,
	LOWBAT = 8,
	AUTOOFF = 9,
	LOWTEMP = 10,
	HITEMP = 11,
	GLOWFAIL = 12,
	WTD = 13,
	FSOFF = 14,
	MANUOFF = 15,
	POWERFAIL = 16,
	TEMPSENFAIL = 17,
	FUELFAIL = 18,
	PROPFAIL = 19,
	SECENGFAIL = 20,
	SECENGDIFF = 21,
	NOOIL = 23,
	OVERCURR = 24,
	NOPUMP = 25,
	WRONGPUMP = 26,
	PUMPNOCOM = 27,
	NOFUEL = 28,
	LOWPUMPRPM = 29,
	CLUTCHFAIL = 31,
	ECUREBOOT = 32,
	NOCAN = 33,
	NORC = 34,
	ROTORBLOCK = 35,
};


struct je_para{
	uint64_t rpm_min;
	uint64_t rpm_max;
	float pwm_min;
	float pwm_max;
	float pwm_stop;
	float pwm_start;
	float max_start;
	float min_start;
	float consum_rate;
	float tank_cap;
	float fuel_low;
	float pul_n;
};

/** Callback function for platform-specific stuff.
 * data1 and data2 depend on type and user is the custom user-supplied argument.
 * @return <0 on error, >=0 on success (depending on type)
 */
typedef int (*JECallbackPtr)(JECallbackType type, void *data1, int data2, void *user,int data3,float *data4);


class JEHelper
{
public:

	enum class Interface : uint8_t {
		UART = 0,
		SPI
	};

	typedef enum
	 {
		 uninit =0,
		 standby=1,
		 starting=2,
		 running=3,
		 stopping=4,
		 fault=-1
	 }engine_status_t;

JEHelper(JECallbackPtr callback, void *callback_user)
{
_callback=callback;
 _callback_user=callback_user;
};

virtual ~JEHelper() = default;

	virtual int waiting();

	virtual int start();

	virtual int runn();

	virtual int stop();

	virtual int test(uint8_t);

	virtual int collect();

	virtual int handlefault(); /* continuously return false while everything is normal.return -1
					when the engine becomes abnormal.
				   	And it returns the _estatus number according to the internal _estatus
				   	while the engine resumes from abnormal to normal
					*/
	virtual int getJEpara();

	virtual int setJEpara();


protected:
	actuator_armed_s 				_armed{};

	//actuator_controls_s 				_actuators{};

	et_command_s					_et{};

	engine_status_s					*_engine_status{nullptr};


	bool _engine_started = false;

	bool _engine_starting = false;

	uint32_t _armed_time = 0;

	bool _throttle_armed = false;


	const char *device_name = "/dev/ttyS3"; /* default UART4 */



	int _Baudrate = 0;

	int _fd;

	int _ec_m = 0;

	uint8_t _e_n=0;

	je_para _para_set;

	je_para _para_got;


	const uint64_t _delaytime = 100000;





	float _engine_load(uint64_t rpm)
	{
		float engine_load=0;
		if(rpm>_para_set.rpm_min)
		{
		float	engine_out = (float)(rpm-_para_set.rpm_min)/(float)(_para_set.rpm_max-_para_set.rpm_min); // since the jet air flow's velocity is proportioanl to RPM,and the velocity's square is proportional to the thrust
			engine_load = engine_out * engine_out; //so RPM's square should be proportional to the engine load
		}
		if(rpm>_para_set.rpm_max)engine_load = 1;
		return engine_load;
	}

	double _fuelcomsued(double pump)
	{
		static hrt_abstime _last_read;
		uint64_t read_elapsed = hrt_elapsed_time(&_last_read);
		pump /= 100; // pump rate is readed in percentage
		static double _fuelcomsuption;
		_fuelcomsuption += (double)_para_set.consum_rate*pump*(double)read_elapsed/1000000;
		_last_read= hrt_absolute_time();
		return _fuelcomsuption;
	}


	JECallbackPtr 	 _callback{nullptr};
	void		 *_callback_user{};
	/**
	 * read from device
	 * @param buf: pointer to read buffer
	 * @param buf_length: size of read buffer
	 * @param timeout: timeout in ms
	 * @return: 0 for nothing read, or poll timed out
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int read(uint8_t *buf, int buf_length, int timeout)
	{
		memcpy(buf, &timeout, sizeof(timeout));
		return _callback(JECallbackType::readDeviceData, buf, buf_length, _callback_user,NULL,NULL);
	}

	/**
	 * write to the device
	 * @param buf
	 * @param buf_length
	 * @return num written bytes, -1 on error
	 */
	int write(const void *buf, int buf_length)
	{
		return _callback(JECallbackType::writeDeviceData, (void *)buf, buf_length, _callback_user,NULL,NULL);
	}

	/**
	 * set the Baudrate
	 * @param baudrate
	 * @return 0 on success, <0 otherwise
	 */
	int setBaudrate(int baudrate)
	{
		return _callback(JECallbackType::setBaudrate, nullptr, baudrate, _callback_user,NULL,NULL);
	}

	/**
	 * set the throttle and the starter's pwm value
	 * It can only set the thr_pwm,str_pwm when the input value >500, or the actural pwm output will switch back to the mixer's output.
	 * @return 0 on success, <0 otherwise
	 */
	int setThr(int thr_pwm,int str_pwm)
	{
		return _callback(JECallbackType::setPWM, nullptr, thr_pwm,_callback_user,str_pwm,NULL);
	}

	/**
	 * publish the orb message
	 * @param engine_status_s
	 * @return num written bytes, -1 on error
	 */
	int publish()
	{
		PX4_INFO("EGT:%f fuel remain:%f RPM:%d",(double)_engine_status->egt[0],(double)_engine_status->fuel_remain,(int)_engine_status->rpm[0]);//test
		return _callback(JECallbackType::publish, NULL, NULL, _callback_user,NULL,NULL);
	}

	bool getarm()
	{
		return (bool)_callback(JECallbackType::getARM, NULL, NULL, _callback_user,NULL,NULL);
	}

	float getthr()
	{
		float thr=0.0f;
		_callback(JECallbackType::getThr, NULL, NULL, _callback_user,NULL, &thr);
		return thr;
	}

};



