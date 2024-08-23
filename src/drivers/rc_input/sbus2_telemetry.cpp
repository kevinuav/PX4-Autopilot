/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file sbus2_telemetry.cpp
 *
 * @author Kevin Chen <kevindsp@qq.com>
 */

#include "sbus2_telemetry.hpp"

/*
//slot[frame][slot_sn][byte in slot]
static uint8_t send_buffer[4][8][3]={{{0x03,0x40,0x00},{0x83,0x40,0x00},{0x43,0x40,0x00},{0xC3,0x40,0x00},{0x23,0x40,0x00},{0xa3,0x40,0x00},{0x63,0x40,0x00},{0xe3,0x40,0x00}},
			      {{0x13,0x40,0x00},{0x93,0x40,0x00},{0x53,0x40,0x00},{0xD3,0x40,0x00},{0x33,0x40,0x00},{0xB3,0x40,0x00},{0x73,0x40,0x00},{0xF3,0x40,0x00}},
			      {{0x0B,0x40,0x00},{0x8B,0x40,0x00},{0x4B,0x40,0x00},{0xCB,0x40,0x00},{0x2B,0x40,0x00},{0xAB,0x40,0x00},{0x6B,0x40,0x00},{0xEB,0x40,0x00}},
			      {{0x1B,0x40,0x00},{0x9B,0x40,0x00},{0x5B,0x40,0x00},{0xDB,0x40,0x00},{0x3B,0x40,0x00},{0xBB,0x40,0x00},{0x7B,0x40,0x00},{0xFB,0x40,0x00}}};

uint8_t   Slot_ID[32] = {   0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
                            0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
                            0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
                            0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB
                        };
*/
//slot[frame][slot_sn]
static uint8_t slot[4][45]={{			    //frame 0
				    0xff,0xff,0xff,//padding for the FC's serial port which can't be configured as inverted
				    0x83,0x40,0x00, //slot 1
				    0xff,0xff,0xff, //padding for the 360us gap
				    0x43,0x40,0x00, //slot 2
				    0xff,0xff,0xff,
				    0xC3,0x40,0x00,
				    0xff,0xff,0xff,
				    0x23,0x40,0x00,
				    0xff,0xff,0xff,
				    0xa3,0x40,0x00,
				    0xff,0xff,0xff,
				    0x63,0x40,0x00,
				    0xff,0xff,0xff,
				    0xe3,0x40,0x00 },
				    {		    //frame 1
				    0x13,0x40,0x00, //slot 0
				    0xff,0xff,0xff, //padding for the 360us gap
				    0x93,0x40,0x00, //slot 1
				    0xff,0xff,0xff,
				    0x53,0x40,0x00,
				    0xff,0xff,0xff,
				    0xD3,0x40,0x00,
				    0xff,0xff,0xff,
				    0x33,0x40,0x00,
				    0xff,0xff,0xff,
				    0xB3,0x40,0x00,
				    0xff,0xff,0xff,
				    0x73,0x40,0x00,
				    0xff,0xff,0xff,
				    0xF3,0x40,0x00 },
				    {
				    0x0B,0x40,0x00,
				    0xff,0xff,0xff,
				    0x8B,0x40,0x00,
				    0xff,0xff,0xff,
				    0x4B,0x40,0x00,
				    0xff,0xff,0xff,
				    0xCB,0x40,0x00,
				    0xff,0xff,0xff,
				    0x2B,0x40,0x00,
				    0xff,0xff,0xff,
				    0xAB,0x40,0x00,
				    0xff,0xff,0xff,
				    0x6B,0x40,0x00,
				    0xff,0xff,0xff,
				    0xEB,0x40,0x00 },
				    {
				    0x1B,0x40,0x00,
				    0xff,0xff,0xff,
				    0x9B,0x40,0x00,
				    0xff,0xff,0xff,
				    0x5B,0x40,0x00,
				    0xff,0xff,0xff,
				    0xDB,0x40,0x00,
				    0xff,0xff,0xff,
				    0x3B,0x40,0x00,
				    0xff,0xff,0xff,
				    0xBB,0x40,0x00,
				    0xff,0xff,0xff,
				    0x7B,0x40,0x00,
				    0xff,0xff,0xff,
				    0xFB,0x40,0x00
				    }
				    };

static bool tx_busy=0;

static uint8_t _frame_num=0;

uint8_t _inner_frame_num = 0; //在触发时赋值，免得在发送中间被修改

struct pollfd fds;

static bool send_pause = 0;

static hrt_abstime now = 0;

hrt_abstime elapsed_time = 0;

int _usart=0;

//int _usart2;

//uint32_t _test_time =0;

void *rcin_usart_thread(void *arg);

Sbus2Telemetry::Sbus2Telemetry(int uart_fd) :
	ModuleParams(nullptr), _uart_fd(uart_fd)
{
	set_uart_single_wire(_uart_fd,true);

      	fds.fd = _uart_fd;
	_usart = _uart_fd;
    	fds.events = POLLIN;
     	pthread_attr_init(&attr);
     	param.sched_priority = SCHED_PRIORITY_MAX; //I have tried the default priority. But it just can't fit the timing for the sbus2.That may cause the RC-receiving issue.
    	pthread_attr_setschedparam(&attr, &param);
    	pthread_attr_setstacksize(&attr, 1000);

      	ret = pthread_create(&pth, &attr, rcin_usart_thread, NULL );
      if (ret != 0)
      {
          printf("USART_initialize: pthread_create failed: %d\n", ret);
      }
/*
    _usart2 = ::open("/dev/ttyS1",  O_RDWR | O_NONBLOCK | O_NOCTTY); //
    set_opt(_usart2, 57600, 8, 'N', 1);
    */

}

bool
Sbus2Telemetry::update(uint8_t frame_num)
{
	_frame_num=frame_num; //update the frame number at every loop of the data update

	_batt_slot_num = _batt_slot.get();
	_as_slot_num   = _as_slot.get();
	_yaw_slot_num   = _yaw_slot.get();
	_rpm_slot_num  = _rpm_slot.get();
	_gps_slot_num  = _gps_slot.get();
	_jc_slot_num   = _jc_slot.get();
	_atemp_slot_num = _atemp_slot.get();

  		if(_batt_sub.update(&_batt)&&(_batt_slot_num>0)) // update battery's voltage and current
		{
			send_batt(_batt.current_a,_batt.voltage_v,_batt.discharged_mah,_batt_slot_num);
		}

		if(_as_sub.update(&_as)&&(_as_slot_num>0)) //update airspeed
		{
			send_as(_as.indicated_airspeed_m_s*3.6f,_as_slot_num);
		}

		if(_atemp_sub.update(&_atemp)&&(_atemp_slot_num>0)) //update air temperature
		{
			send_atemp(_atemp.air_temperature_celsius,_atemp_slot_num);
		}

/*
		if(_gps_sub.update(&_gps)&&(_gps_slot_num>0)) //update gps with raw gps position
		{
			send_gps_raw_int(_gps.lat,_gps.lon,_gps.alt,_gps.vel_m_s*3.6f,_gps.time_utc_usec,-_gps.vel_d_m_s,_gps_slot_num);
		}
*/
		_gps_sub.update(&_gps);
		if(_global_sub.update(&_global)&&(_gps_slot_num>0)) //update gps with global position
		{
			send_gps_deg(_global.lat,_global.lon,_global.alt,_gps.vel_m_s*3.6f,_gps.time_utc_usec,-_gps.vel_d_m_s,_gps_slot_num);
		}

		if(_att_sub.update(&_att)&&(_yaw_slot_num>0)) //update yaw as air temperature
		{
			float home_bearing=0;
			_home_sub.update(&_home);
			if(_home.valid_hpos)
			{
				home_bearing = get_bearing_to_next_waypoint(_global.lat,_global.lon,_home.lat,_home.lon);
			}
			else home_bearing = 0;
			_R = matrix::Quatf(_att.q);
			matrix::Eulerf euler_angles(_R);
			float yaw = wrap_pi(home_bearing - euler_angles.psi())*57.3f;
			send_atemp(yaw,_yaw_slot_num);
			//PX4_INFO("yaw update:%f",(double)yaw);
		}

		if(_rpm_sub.update(&_rpm)&&_rpm_slot_num>0)
		{
		send_rpm((uint64_t)_rpm.indicated_frequency_rpm,_rpm_slot_num);
		}

		if(_es_sub.update(&_es)&&(_jc_slot_num>0)) //update engine status: 1 update jetengine's status as jetcat. The other engines status update as the propeller(not jet)
		{
			send_jetcat(_es.rpm[0],_es.egt[0],_es.vol[0],_es.thr_sp,_es.engine_load[0],_es.fuel_remain,1.5,(uint32_t)f_toint32(_global.alt),86,_batt.voltage_v,1.85,_as.indicated_airspeed_m_s*3.6f,_es.status[0],_jc_slot_num);
		}

/*		send_batt(20,14.56,2100,_batt_slot_num);
		send_gps_deg(33.12345,123.12345,_global.alt,0,0,0,_gps_slot_num);
		send_jetcat(123,456,5.6,.9,0,1200,1.5,79,86,3.8,1.85,68,3,_jc_slot_num);
*/
/*	static hrt_abstime blink_time=0;
	static uint8_t i=0;
		if(hrt_elapsed_time(&blink_time)>1000000){blink_time=hrt_absolute_time();if(i<50)i++;else i=0;}
		send_atemp(i,1);
		send_atemp(i,2);
		send_atemp(i,3);
		send_atemp(i,4);
		send_atemp(i,5);
		send_atemp(i,6);
		send_atemp(i,7);
		send_atemp(i,8);
		send_atemp(i,9);
		send_atemp(i,10);
		send_atemp(i,11);
		send_atemp(i,12);
		send_atemp(i,13);
		send_atemp(i,14);
		send_atemp(i,15);
	//	_test_time= _tt.get();
*/
	return PX4_OK;
}

int
Sbus2Telemetry::send_batt(float current,float voltage,float capacity,uint8_t slot_num)
{
	uint8_t bytes[2];
	uint16_t value=0;

//current
	current *= 100;
	value = (uint16_t)current;

   	if ( value > 0x3FFF )
  	 {
      // max current is 163.83
	value = 0x3FFF;
 	  }
 	bytes[0] = value >> 8;
 	bytes[0] = bytes[0] | 0x40;
  	bytes[0] = bytes[0] & 0x7F;
  	bytes[1] = value;
  	send_sbs2(slot_num,bytes);

//VOLTAGE
	voltage*=100;
	value = (uint16_t)voltage;
	bytes[0] = value >> 8;
	bytes[1] = value;
	send_sbs2(slot_num+1,bytes);

// CAPACITY
	value = (uint16_t)capacity;
	bytes[0] = value >> 8;
	bytes[1] = value;
	send_sbs2(slot_num+2,bytes);
   return PX4_OK;
}

int
Sbus2Telemetry:: send_as(float as,uint8_t slot_num)
{
	uint8_t bytes[2];
	uint16_t value = (uint16_t)f_toint32((float)as);
	value = value | 0x4000;
  	 bytes[0] = value >> 8;
  	 bytes[1] = value;
	send_sbs2(slot_num,bytes);
	   return PX4_OK;
}

int
Sbus2Telemetry::send_atemp(float atemp,uint8_t slot_num)
{
	uint8_t bytes[2];
	int16_t value = (int16_t)f_toint32(atemp);
	value = value | 0x4000;
  	 bytes[0] = value >> 8;
  	 bytes[1] = value;
	send_sbs2(slot_num,bytes);
	   return PX4_OK;
}

int
Sbus2Telemetry::send_rpm(uint64_t rpm,uint8_t slot_num)
{
	uint8_t bytes[2];
	uint32_t value = 0;
	value =  rpm / 6;
   if(value > 0xffff){
    value = 0xffff;
   }
  	bytes[0] = value;
	bytes[1] = value >> 8; //这个坑爹，别人都是高位在前，低位在后，他可好，把其反过来，big ending 而且遥控里还要设置齿轮比
	send_sbs2(slot_num,bytes);
	return PX4_OK;
}

int
Sbus2Telemetry::send_gps_raw_int(int lat,int lon,int alt,float vel,uint64_t utc,float vel_d, uint8_t slot_num)
{
   uint8_t bytes[2];
   bool latitude_pos = false;
   bool longitude_pos = false;
   alt/=1000;
   int16_t alt16 = (int16_t)alt;
   uint16_t speed16 = (uint16_t)abs(vel);
   int16_t vario= (int16_t)vel_d;
   uint16_t value1=0;
   int16_t value2=0;
   uint32_t value3=0;

   int32_t _latitude = lat/10; //在PX4的数据中raw int的经纬信息中，lat,lon都是7位，所以要先除以10
  int32_t _longitude = lon/10;
  int32_t _latitude_deg = _latitude/1000000;
  int32_t _longitude_deg = _longitude/1000000;
  int32_t _latitude_min = _latitude%1000000;
  int32_t _longitude_min = _longitude%1000000;
  _latitude = _latitude_deg * 1000000;
  _longitude = _longitude_deg * 1000000;
  _latitude = _latitude + ((_latitude_min * 60)/100);
  _longitude = _longitude + ((_longitude_min * 60)/100);
// SPEED -> Bit 14(bytes[1] bit7) -> GPS Valid or not
   value1 = speed16 | 0x4000;
   if (value1 > 0x43E7 ){
      value1 = 0x43E7;  // max Speed is 999 km/h
   }
   else if( value1 < 0x4000){
     value1 = 0x4000;
   }
   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num,bytes);

//HIGHT
   value2 = alt16 | 0x4000;
   /*if(value2 > 0x7FFF ){		// max = +16383
     value2 = 0x7FFF;
   }
   else if( value2 < 0xC000){	// min = -16384
     value2 = 0xC000;
   }*/
   bytes[0] = value2 >> 8;
   bytes[1] = value2;
   send_sbs2(slot_num+1,bytes);

   //TIME -> 12:34:56 Uhr = 12*60*60 + 34*60 + 56 = 45296 = 0xB0F0
   utc/=1000000;
   uint16_t utc_16 = (uint16_t)utc;
   bytes[0] = utc_16>>8;
   bytes[1] = utc_16;
   send_sbs2(slot_num+2,bytes);

   // VARIO
   value2 = vario * 10;
   bytes[0] = value2 >> 8;
   bytes[1] = value2;
   send_sbs2(slot_num+3,bytes);

// LATITUDE
   if(_latitude >= 0){
    latitude_pos = true;
   }
   else{
    latitude_pos = false;
    _latitude = _latitude * -1;
   }
   bytes[0] = (uint8_t)(_latitude/1000000);
   value3 = (uint32_t)(_latitude%1000000);
   if(latitude_pos){
    bytes[1] = ((value3 >> 16) & 0x0f);    // North
   }
   else{
    bytes[1] = ((value3 >> 16) | 0x1f);    // South
   }
   send_sbs2(slot_num+4,bytes);

   bytes[0] = ((value3 >> 8) & 0xff);
   bytes[1] = value3 & 0xff;
   send_sbs2(slot_num+5,bytes);

// LONGITUDE
   if(_longitude >= 0){
    longitude_pos = true;
   }
   else{
    longitude_pos = false;
    _longitude = _longitude * -1;
   }
   bytes[0] = (uint8_t)(_longitude/1000000);
   value3 = (uint32_t)(_longitude%1000000);
   if(longitude_pos){
     bytes[1] = ((value3 >> 16) & 0x0f);    // Eath
   }
   else{
     bytes[1] = ((value3 >> 16) | 0x1f);    // West
   }
   send_sbs2(slot_num+6,bytes);
   bytes[0] = ((value3 >> 8) & 0xff);
   bytes[1] = value3 & 0xff;
   send_sbs2(slot_num+7,bytes);
   return PX4_OK;
}

int
Sbus2Telemetry::send_gps_deg(double lat,double lon,float alt,float vel,uint64_t utc,float vel_d,uint8_t slot_num)
{
  int32_t _latitude = lat * 10000000; //因为raw_int 那边lat已经除以10了，所以这边要乘回去，PX4的狗屎！
  int32_t _longitude = lon * 10000000;
  int32_t _latitude_deg = _latitude/1000000;
  int32_t _longitude_deg = _longitude/1000000;
  int32_t _latitude_min = _latitude%1000000;
  int32_t _longitude_min = _longitude%1000000;
  _latitude = _latitude_deg * 1000000;
  _longitude = _longitude_deg * 1000000;
  _latitude = _latitude + _latitude_min ;
  _longitude = _longitude + _longitude_min;
  send_gps_raw_int( _latitude, _longitude,(uint32_t)f_toint32(alt*1000), vel, utc, vel_d, slot_num);
  return PX4_OK;
}

int
Sbus2Telemetry:: send_jetcat(
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
			uint8_t slot_num)
{
	uint8_t bytes[2];
	uint32_t value = 0;
	uint16_t value1 = 0;
	// Actual RPM with 0x4000 Offset -> why?
   	value = (uint32_t)rpm / 100;
  	 value = value | 0x4000;
  	 if(value > 0xffff){
    	value = 0xffff;
   		}
  	 bytes[0] = value >> 8;
	 bytes[1] = value;
	 send_sbs2(slot_num,bytes);

	 // EGT Abgastemperatur in °C
   value1 = (uint16_t)f_toint32(egt);
   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num+1,bytes);

   // Pump Voltage 12.34V = 1234
   value1 = (uint16_t)f_toint32(pump_volt);
   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num+2,bytes);

   // Setpoint RPM without Offset
   value1 = (uint16_t)f_toint32(setrpm) / 100;

   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num+3,bytes);

   // Thrust 123.4N = 1234
   value1 = (uint16_t)f_toint32(thrust);
   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num+4,bytes);

   // Fuel (remain) in ml
   value1 = (uint16_t)f_toint32(fuel);
   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num+5,bytes);

   // Fuel Flow in ml/min
   value1 = (uint16_t)f_toint32(fuelflow);
   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num+6,bytes);

   // Altitude -> without offset?
   value = altitude;
   bytes[0] = value >> 8;
   bytes[1] = value;
   send_sbs2(slot_num+7,bytes);

   // Fuel Quality in %
   value = quality;
   bytes[0] = value >> 8;
   bytes[1] = value;
   send_sbs2(slot_num+8,bytes);

   // Voltage 12.34V = 1234
   value1 = (uint16_t)f_toint32(volt);
   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num+9,bytes);

   // Current 123.4A = 1234
   value1 = (uint16_t)f_toint32(current);
   bytes[0] = value1 >> 8;
   bytes[1] = value1;
   send_sbs2(slot_num+10,bytes);

   // Speed in km/h
   value1 = (uint16_t)f_toint32((float)abs(speed*100));
   value1/=100;
   bytes[0] = value1 >> 8;
   bytes[1] = (0x00ff&value1);
   send_sbs2(slot_num+11,bytes);


   // Second RPM without Offset

   // Status and Error Code
   value = status;
   bytes[0] = value>>8;
   bytes[1] = value;
   send_sbs2(slot_num+12,bytes); // 经实验，第13个slot的确是第二轴的数据，故推测第12个slot应该就是状态slot


return PX4_OK;
}

int
Sbus2Telemetry::send_sbs2(uint8_t slot_num,uint8_t datas[2])
{
	uint8_t fram_num=0,slot_sn=0;
	fram_num = slot_num/(uint8_t)8;
	slot_sn = slot_num % (uint8_t)8;
	if(fram_num==0)
	{
	slot_sn--;
	if(datas[0]!=0x0f)slot[fram_num][slot_sn*6+4] = datas[0];
	if(datas[1]!=0x0f)slot[fram_num][slot_sn*6+5] = datas[1];
	}else
	{
	if(datas[0]!=0x0f)slot[fram_num][slot_sn*6+1] = datas[0];
	if(datas[1]!=0x0f)slot[fram_num][slot_sn*6+2] = datas[1];
	}
	//PX4_INFO("SLOT ID:%d",slot[fram_num][slot_sn*6]);
	return PX4_OK;
}




inline int32_t
Sbus2Telemetry::f_toint32(float x)
{
//取得符号位，设置掩码
uint32_t n = ((*(uint32_t*)&x) & 0x80000000) ? 0xFFC00000 : 0;//一个三元操作符，直接储存掩码
x += 12582912.0f;//魔法数字加法
return ((*(uint32_t*)&x) & 0x3FFFFF) | n;//直接or运算
}



void
Sbus2Telemetry::set_uart_single_wire(int uart, bool single_wire)
{
	int result=0;
	result = ::ioctl(uart, TIOCSSINGLEWIRE, single_wire ? (SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PUSHPULL |
			SER_SINGLEWIRE_PULLDOWN) : 0);
 //  USARTx_CR1 &= ~(1<<5);
	if (result < 0) {
		PX4_WARN("setting TIOCSSINGLEWIRE failed");
	}
	else
   {
      PX4_WARN("setting TIOCSSINGLEWIRE successful code:%d",result);
   }
}


void *
rcin_usart_thread(void *arg)
{
   int ret=0;
   while(1)
   {
	//USART2_TDR= 0x00;
	ret=0;
   if(!tx_busy)ret = poll(&fds,1,-1);
   if(ret>0) //mark the tx_busy flag as true and save the hrt time stamp when it is triggered
   {
	elapsed_time = hrt_elapsed_time(&now);
	if(elapsed_time<1500){
		send_pause=1;
	}
	else{
		send_pause=0;
	}
	tx_busy = 1;
	now = hrt_absolute_time();
	//USART2_TDR= _frame_num;
   }
	if(tx_busy)
	{
		//USART2_TDR= _frame_num;
		if(_frame_num<3)_inner_frame_num = _frame_num+1; //_frame_num can be changed in the phase of the tramsmiting. So we need to determin it to the _inner_frame_num before the sending
			else _inner_frame_num = 0; //can't be removed
		tx_busy=0;

		usleep(4000);//while it is wating for the hrt we can return the resource to the system first. And come back when it almost need to write() since it needs the CPU for high accuracy timing.

		while(1)
		{

			elapsed_time = hrt_elapsed_time(&now); //CPU operates for the high accuracy timing
			if(_inner_frame_num==0) // the first frame's slot #0 is occupied by the receiver's voltage
			{
			      if(elapsed_time>5278) //5278 only effective for H7 not F7 (5300)the time needs to be accuracy,or it will make the next of frame un acceptable to the fasstest receiver. 这个时间不准居然不会影响到本帧反而会映响到下一帧，坑爹！！
			{
			//	USART2_TDR= _inner_frame_num;
	 			if(!send_pause)write(_usart,slot[0],sizeof(slot[0]));
				break;
			}
			}else if(elapsed_time>4920)
			{
			//	USART2_TDR= _inner_frame_num;
	 			if(!send_pause)write(_usart,slot[_inner_frame_num],sizeof(slot[_inner_frame_num]));
				break;
			}
		}

		usleep(8000); //after write(). We returns the CPU resource to the system until it almost receives next frame of data
		tcflush(_usart, TCIOFLUSH);// for avoiding the contamination of the sending data.We should clean the usart before setting the tx_busy as false, or the poll() could return immediately.
		//USART2_TDR= _frame_num;
	}
   }
 return NULL;
}

/*
int
Sbus2Telemetry::set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	//保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	memset(&newtio, 0, sizeof(newtio));
	//步骤一，设置字符大小
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//设置停止位
	switch (nBits)
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}
	//设置奇偶校验位
	switch (nEvent)
	{
		case 'O': //奇数
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E': //偶数
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N': //无奇偶校验位
			newtio.c_cflag &= ~PARENB;
			break;
	}
	switch (nSpeed)
	{
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;

		case 19200:
			cfsetispeed(&newtio, B19200);
			cfsetospeed(&newtio, B19200);
			break;

		case 38400:
			cfsetispeed(&newtio, B38400);
			cfsetospeed(&newtio, B38400);
			break;

		case 57600:
			cfsetispeed(&newtio, B57600);
			cfsetospeed(&newtio, B57600);
			break;

		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;

		case 230400:
			cfsetispeed(&newtio, B230400);
			cfsetospeed(&newtio, B230400);
			break;

		default:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
	}
	//设置停止位
	if (nStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (nStop == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	//设置等待时间和最小接收字符
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	//处理未接收字符
	tcflush(fd, TCIFLUSH);
	//激活新配置
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}
	printf("set done!\n");
	return 0;
}
*/

