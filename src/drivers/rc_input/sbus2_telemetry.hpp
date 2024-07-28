/****************************************************************************
 *
 *	Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file sbus2_telemetry.hpp
 *
 * @author Kevindsp <kevindsp@qq.com> //Thanks a lot for Daniel Agar. He gives me lots of help for this sbus2 telemetry's code. He helps me get rid of my orignial register operating code to do this. Thank you! Daniel.
 */

#pragma once

#include <uORB/Subscription.hpp>
#include <drivers/drv_hrt.h>
#include <lib/rc/sbus.h>
#include <termios.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/module_params.h>
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/engine_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/home_position.h>

using matrix::Eulerf;
using matrix::Quatf;
using matrix::wrap_pi;

/*
#define USE_UART2 true  //V5+'s tele2 using USE_UART3

#ifdef USE_UART2
# define STM32_USARTx_BASE	STM32_USART2_BASE //Holybro's tele2 port<->ttyS3
# define STM32_IRQ_UARTx	STM32_IRQ_USART2
#endif
#ifdef USE_UART3
# define STM32_USARTx_BASE	STM32_USART3_BASE //Holybro's tele2 port<->ttyS3
# define STM32_IRQ_UARTx	STM32_IRQ_USART3
#endif
#ifdef USE_UART4
# define STM32_USARTx_BASE	STM32_UART4_BASE //CUAV_X7pro'gps2(UART4) ttyS2
# define STM32_IRQ_UARTx	STM32_IRQ_UART4
#endif
#ifdef USE_UART5
# define STM32_USARTx_BASE	STM32_UART5_BASE //
# define STM32_IRQ_UARTx	STM32_IRQ_UART5
#endif
#ifdef USE_UART6
# define STM32_USARTx_BASE	STM32_USART6_BASE //CUAV_V5+'s RC port<->ttyS4
# define STM32_IRQ_UARTx	STM32_IRQ_USART6
#endif
#ifdef USE_UART7
# define STM32_USARTx_BASE	STM32_UART7_BASE //
# define STM32_IRQ_UARTx	STM32_IRQ_UART7
#endif
#ifdef USE_UART8
# define STM32_USARTx_BASE	STM32_UART8_BASE //v6x'GPS2 port<->tty5  /cubepilot
# define STM32_IRQ_UARTx	STM32_IRQ_UART8
#endif
*/

//#define USART_CR1_IDLEIE          (1 << 4)  /* Bit 4:  IDLE Interrupt Enable */
//#define USART_CR1_UE              (1 << 0)  /* Bit 0:  USART enable */
//#define USART_ISR_IDLE            (1 << 4)  /* Bit 4:  Idle line detected */
//#define USART_ICR_IDLECF          (1 << 4)  /* Bit 4:  Idle line detected clear flag */
/*
#define USARTx_CR1 (*(volatile uint32_t *)(STM32_USARTx_BASE+0x0000))
#define USARTx_CR2 (*(volatile uint32_t *)(STM32_USARTx_BASE+0x0004))
#define USARTx_CR3 (*(volatile uint32_t *)(STM32_USARTx_BASE+0x0008))
#define USARTx_ISR (*(volatile uint32_t *)(STM32_USARTx_BASE+0x001c))
#define USARTx_ICR (*(volatile uint32_t *)(STM32_USARTx_BASE+0x0020))
#define USARTx_RDR (*(volatile uint32_t *)(STM32_USARTx_BASE+0x0024))
#define USARTx_TDR (*(volatile uint32_t *)(STM32_USARTx_BASE+0x0028))


#define USART2_TDR  USARTx_TDR

*/
/*
#define BATT_SLOT	1	//battery's voltage, current and amount used
#define AS_SLOT		4	//airspeed
#define RPM_SLOT	5	//rpm
#define JC_SLOT		8 	//send jet engine status as jet cat
#define TEMP_SLOT	22	//temperature
#define GPS_SLOT	24	//GPS
*/


/**
 * High-level class that handles sending of sbus2 telemetry data
 */
class Sbus2Telemetry : public ModuleParams
{
public:
	/**
	 *
	 */
	Sbus2Telemetry(int uart_fd);

	~Sbus2Telemetry() = default;

	/**
	 * update the frame number
	 */
	bool update(uint8_t);

	static void rcin_tim_i_isr(void *arg);

	static void rcin_tim_j_isr(void *arg);

	int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

private:

	inline int32_t f_toint32(float x);

	int send_batt(float current,float voltage,float capacity, uint8_t slot_num);

	int send_as(float as,uint8_t slot_num);

	int send_atemp(float atemp,uint8_t slot_num); //air temperature mesuring

	int send_rpm(uint64_t rpm,uint8_t slot_num);
	/*lat,lon(in 1E-7 degrees),alt(in 1E-3 meters above MSL, (millimetres)(the same with px4's vehicle_gps_position)),vel km/h,vel_d m/s */
	int send_gps_raw_int(int lat,int lon,int alt,float vel,uint64_t utc,float vel_d,uint8_t slot_num);
	/*lat,lon(in degrees),alt(in meters (the same with px4's vehicle_global_position))*/
	int send_gps_deg(double lat,double lon,float alt,float vel,uint64_t utc,float vel_d,uint8_t slot_num);

	int send_jetcat(
  	uint64_t rpm,
  	float egt,
  	float pump_volt,
  	float setrpm,
  	float thrust,
  	float fuel,
  	float fuelflow,
  	int altitude,
  	uint16_t quality,
  	float volt,
  	float current,
  	float speed,
  	uint8_t status,
	uint8_t slot_num);

	int send_sbs2(uint8_t slot_num,uint8_t bytes[2]);

	uORB::Subscription	_batt_sub{ORB_ID(battery_status)};
	uORB::Subscription	_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription	_global_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription	_es_sub{ORB_ID(engine_status)};
	uORB::Subscription	_as_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription	_atemp_sub{ORB_ID(airspeed)};
	uORB::Subscription	_att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription	_home_sub{ORB_ID(home_position)};


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SBS2_BATT_SLOT>) _batt_slot,
		(ParamInt<px4::params::SBS2_AS_SLOT>)   _as_slot,
		(ParamInt<px4::params::SBS2_RPM_SLOT>)  _rpm_slot,
		(ParamInt<px4::params::SBS2_TEMP_SLOT>) _atemp_slot,
		(ParamInt<px4::params::SBS2_GPS_SLOT>)  _gps_slot,
		(ParamInt<px4::params::SBS2_YAW_SLOT>)  _yaw_slot,
	//	(ParamInt<px4::params::SBS2_TT>)  _tt,
		(ParamInt<px4::params::SBS2_JC_SLOT>)   _jc_slot
	)

	uint8_t _batt_slot_num =0;
	uint8_t _as_slot_num   =0;
	uint8_t _yaw_slot_num   =0;
	uint8_t _rpm_slot_num  =0;
	uint8_t _atemp_slot_num =0;
	uint8_t _gps_slot_num  =0;
	uint8_t _jc_slot_num   =0;

	engine_status_s			_es{};
	battery_status_s		_batt{};
	sensor_gps_s			_gps{};
	vehicle_global_position_s	_global{};
	airspeed_validated_s		_as{};
	airspeed_s			_atemp{};
	vehicle_attitude_s 		_att{};
	home_position_s			_home{};

	matrix::Dcmf _R{matrix::eye<float, 3>()};

	hrt_abstime _last_update{0};

	void set_uart_single_wire(int uart, bool single_wire);


	static constexpr int num_data_types{4}; ///< number of different telemetry data types

	int _next_type{0};

	int _uart_fd;

	pthread_attr_t  attr;

	struct sched_param param;

	int ret=0;

	pthread_t pth;

};
