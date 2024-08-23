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
 * @file JetEngine.hpp
 * @author Kevin Chen <kevindsp@qq.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/engine_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/et_command.h>
#include <uORB/topics/jet_engine_ctl.h>

#include <drivers/drv_hrt.h>
//#include "sPort_data.h"
#include <sys/ioctl.h>

#include "engine/linton.hpp"
#include "engine/jetcat.hpp"


using namespace device;
using namespace time_literals;

static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(2040);

typedef enum {
	JE_NONE = 0,
	JE_LINTON,
	JE_JETCAT,
	JE_AMT,
	GE,
} je_mode_t;
class JetEngine : public ModuleBase<JetEngine>, public ModuleParams,public device::Device
{
public:
	JetEngine(const char *device,unsigned baudrate);
	~JetEngine() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static JetEngine *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void run() override;

	bool init1();

	int print_status() override;

private:


	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

//	uORB::Subscription	_actuator{ORB_ID(actuator_controls)};


	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};

	uORB::Subscription _motors_sub{ORB_ID(actuator_motors)};

	uORB::Subscription _et_command_sub{ORB_ID(et_command)};

	uORB::Publication<engine_status_s>		_engine_status_pub{ORB_ID(engine_status)};

	uORB::Publication<jet_engine_ctl_s>		_je_ctl_pub{ORB_ID(jet_engine_ctl)};

	actuator_armed_s 				_armed{};

	actuator_motors_s				_motors{};

	et_command_s					_et{};

	engine_status_s					_engine_status{};

	jet_engine_ctl_s				_jec{};

	bool _engine_started = false;

	uint32_t 					_armed_time = 0;

	uint32_t 					_armed_time_ms;

	uint32_t					_pwm_unavailable_time = 0;

	bool _throttle_armed = false;

	JetEngine						*_je{nullptr};



	Serial 				_uart {};					///< UART interface to jet engine

	int				_serial_fd{-1};

	int				_e_type={1};

	double				_trigger_n;					//trigger times every round of the propeller

	unsigned			_baudrate{0};					///< current baudrate
	const unsigned			_configured_baudrate{0};			///< configured baudrate (0=auto-detect)
	char				_port[20] {};

	JEHelper			*_helper{nullptr};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FUEL_TANK_CAP>) _fuel_tank_cap,
		(ParamFloat<px4::params::FUEL_CONSUM_RATE>) _fuel_consum_rate,
		(ParamFloat<px4::params::FUEL_LOW>) _fuel_low,
		(ParamFloat<px4::params::RPM_MIN>) _rpm_min,
		(ParamFloat<px4::params::RPM_MAX>) _rpm_max,
		(ParamFloat<px4::params::RC3_MIN>) _rc3_min,
		(ParamFloat<px4::params::RC3_MAX>) _rc3_max,
		(ParamFloat<px4::params::R_N>) _r_n,
		(ParamInt<px4::params::PWM_MAIN_DIS3>) _rc3_stop,
		(ParamInt<px4::params::ENGINE_TYPE>) _engine_type,
		(ParamInt<px4::params::JE_BR>) _je_br,
		(ParamInt<px4::params::JE_N>) _E_N
	)
/*
	float _fuel_tank_cap=1000,_fuel_consum_rate=10,_fuel_low=0.8,_rpm_min=1000,_rpm_max=2000,_rc3_min=1000,_rc3_max=2000,_tri_n=3;
	int _rc3_stop=900,_pwm_start=1200,_max_start=2000,_min_start=1000,_engine_type=4;
*/

	#define JETENGINE_DEFAULT_PORT	"/dev/ttyS3"


	const uint64_t _delaytime = 100000;

	uint8_t _buf[512];

	uint8_t _rx_buffer[512];

	uint16_t _rx_buffer_bytes{};

	//hrt_abstime _last_read{0};

	uint8_t _status;

	uint8_t _fault;

	uint8_t _last_fault;

	hrt_abstime _engine_stop_time = 0;

	hrt_abstime _now = 0;

	bool _restarting = false;

	bool _restarted = false;

	static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

	int setBaudrate(unsigned);

	int pollOrRead(uint8_t *buf, size_t buf_length, int timeout);

	 int waiting();

	 int start();

	 int runn();

	 int stop();

	 int test(uint8_t);

	 int collect();

	 int handlefault();

	 int parseChar(uint8_t);

	 int handle(int);

	 int uart_init();

	 int setPWM(uint16_t,uint16_t);

	 int publish();

	 int getArm();

	 int getThr(float *thr);

	 static int callback(JECallbackType type, void *data1, int data2, void *user,int data3,float *thr);
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
	 }engine_fault;
*/
};

