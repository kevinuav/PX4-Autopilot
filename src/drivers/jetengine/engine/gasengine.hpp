/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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
 * @file gasengine.hpp
 * @author Kevin Chen <kevindsp@qq.com>
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
//#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/engine_status.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/et_command.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#include <drivers/drv_hrt.h>
//#include "sPort_data.h"
#include <sys/ioctl.h>
#include "je_helper.h"





class GasEngine :public JEHelper,public ModuleParams
{
public:
	GasEngine(JECallbackPtr callback, void *callback_user,struct engine_status_s *engine_status);
	virtual ~GasEngine();

	int waiting();

	 int start();

	 int runn();

	 int stop();

	 int test(uint8_t);

	 int collect();

	 int handlefault();

	 int setbaud();

	 int getJEpara(); //send a query message for the engine parameter everytime. return false if there is para unsend.else it returns true while it finishes.

	 int setJEpara();// set the different parameter between FC and the engine. It returns false while there is different parameter unset.And it returns true while the job has done.



private:

	uint8_t _buf[JE_READ_BUFFER_SIZE];

	uint8_t _rx_buffer[512];
	uint16_t _rx_buffer_bytes{};

	pwm_input_s	_pwm_in;

	rpm_s 		_rpm;

	uint8_t _status;

	uint8_t _fault;

	uint8_t _last_fault;

	hrt_abstime _engine_stop_time = 0;

	hrt_abstime _now = 0;

	bool _restarting = false;

	bool _restarted = false;

	double _rpm1=0;

	double _rpm2=0;

	 int parseChar(uint8_t);

	 int handle(int);

	 int uart_init();

	 int set_opt(int,int,int,char,int);

	 double period2rpm(double);

	 uORB::Subscription		 			_pwm_in_sub{ORB_ID(pwm_input)};

	 uORB::Subscription				        _rpm_sub{ORB_ID(rpm)};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::R_N>) _r_n,
		(ParamInt<px4::params::PWM_START>) _pwm_start,
		(ParamInt<px4::params::MAX_STARTER>) _max_start,
		(ParamInt<px4::params::MIN_STARTER>) _min_start,
		(ParamInt<px4::params::JE_N>)          _E_N
	)


	 typedef enum
	 {
		standby=0,
		starting=1,
		idle=2,
		running=3,
		stopping=4,
		restarting=5,
		maximum_power=6
	 }engine_state;

	 typedef enum
	 {
		normal=0,
		lowbat=1,
		declutchfail=2,
		flameout=3,
		fuelfail=4,
		starterfail=5
	 }engine_fault;
};
