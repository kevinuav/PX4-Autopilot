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
 * @file jetcat.hpp
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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/et_command.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#include <drivers/drv_hrt.h>
//#include "sPort_data.h"
#include <sys/ioctl.h>
#include "je_helper.h"





class Jetcat :public JEHelper,public ModuleParams
{
public:
	Jetcat(JECallbackPtr callback, void *callback_user,struct engine_status_s *engine_status,int uart4);
	virtual ~Jetcat();

	int waiting();

	 int start();

	 int runn();

	 int stop();

	 int test(uint8_t);

	 int collect();

	 int handlefault();

	 int getJEpara(); //send a query message for the engine parameter everytime. return false if there is para unsent. else it returns true while it finishes.

	 int setJEpara(); // set the different parameter between FC and the engine. It returns false while there is different parameter unset.And it returns true while the job has done.

	 int pwm_start();


private:

	uint8_t _buf[JE_READ_BUFFER_SIZE];

	uint8_t _rx_buffer[512];
	uint16_t _rx_buffer_bytes{};

	char i2char(int);

	int _e_id=1;

	uint8_t _sent_cmd;

	uint8_t _status[4];

	bool flame_out = 0;

	bool starting_fault;

	uint8_t _fault[4];

	uint8_t _last_fault=0;

	hrt_abstime _engine_stop_time = 0;

	hrt_abstime _now = 0;

	bool _restarting = false;

	bool _restarted = false;

	 int parseChar(uint8_t);

	 int handle(int);

	 int cmdrac(uint8_t en);

	 int cmdra1(uint8_t en);

	 int cmdrfi(uint8_t en);

	 int cmdtco(uint8_t en, bool on);

	 int cmdwpe(uint8_t en,float thr);

	 int send_cmd(uint8_t);

	 int setbaud();

	// uORB::Subscription	_actuator_sub{ORB_ID(actuator_controls_0)};

	 DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FUEL_TANK_CAP>) _fuel_tank_cap,
		(ParamFloat<px4::params::FUEL_LOW>) _fuel_low,
		(ParamFloat<px4::params::FUEL_CONSUM_RATE>) _fuel_consum_rate,
		(ParamFloat<px4::params::RPM_MIN>) _rpm_min,
		(ParamFloat<px4::params::RPM_MAX>) _rpm_max,
		(ParamInt<px4::params::PWM_MAIN_MIN3>) _pwm3_min,
		(ParamInt<px4::params::PWM_MAIN_MAX3>) _pwm3_max,
		(ParamInt<px4::params::PWM_MAIN_DIS3>) _pwm3_stop,
		(ParamInt<px4::params::JE_M>)          _EC_M,
		(ParamInt<px4::params::JE_N>)          _E_N
	)

	#define JETCAT_BAUDRATE 115200

	 /*
	 typedef enum
	 {
		standby1=0,
		starting=1,
		igniting=2,
		ignited=3,
		warming1=4,
		warming2=5,
		warming3=6,
		warming4=7,
		warming5=8,
		declutch=9,
		idle1=10,
		accelerating=11,
		cooling1=17,
		cooling2=18,
		cooling3=19,
		cooling4=20,
		idle2=21,
		stoping=22,
		NoRC=23,
		standby2=24,
		restarting=29
	 }engine_state;
	*/

	 typedef enum
	 {
		normal=0,
		cooling=1,
		lowbat=2,
		overbat=3,
		lowrpm=4,
		ignfail=5,
		tempfail=7,
		warmingfail=9,
		declutchfail=18,
		flameout=22,
		fuelfail=31,
		overheat=32,
		starterfail=50
	 }Engine_fault;

	 typedef enum
	 {
		 RAC =0,  // ask for the engine's status
		 RGV =1,  // ask for the engine's generater's status
		 RA1 =2,  // ask for the engine's fault
		 RFI =3,  // ask for the engine's fuel's message
		 TCO =4,  // start the engine
		 tco =5,  // start the engine with asking for the engine status
		 WRP =6,  // setting the needed rpm
		 wrp =7,  // setting the needed rpm with asking for the real rpm
		 WPE =8,  // setting the needed thrust
		 wpe =9,  // setting the needed thrust with asking for the real thrust
		 XLO =10, // download the running datalog
		 SLO =11   // download the engine's  data
	 }jetcat_cmd_t;
};
