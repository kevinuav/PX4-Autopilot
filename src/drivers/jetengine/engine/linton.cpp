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
 * @file linton.cpp
 * @author Kevin Chen <kevindsp@qq.com>
 */


#include "linton.hpp"
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <ctime>
#include <stdlib.h>
#include <stdio.h>
#include <cstdint>


#include <uORB/topics/mavlink_log.h>
#include <systemlib/mavlink_log.h>

extern orb_advert_t mavlink_log_pub;


Linton::Linton( JECallbackPtr  callback, void *callback_user,struct engine_status_s *engine_status,int uart4)
:JEHelper(callback,callback_user),ModuleParams(nullptr)
{
	_para_set.pwm_min=_pwm3_min.get();
	_para_set.pwm_max=_pwm3_max.get();
	_para_set.pwm_stop=_pwm3_stop.get();
	_para_set.rpm_min=(uint64_t)_rpm_min.get();
	_para_set.rpm_max=(uint64_t)_rpm_max.get();
	_para_set.consum_rate=_fuel_consum_rate.get();
	_para_set.tank_cap=_fuel_tank_cap.get();
	_para_set.fuel_low=_fuel_low.get();
	_engine_status=engine_status;
	_ec_m=_EC_M.get();
	_e_n=_E_N.get();
	uart_init();
}

Linton::~Linton()
{

}


int
Linton::uart_init()
{
	//setBaudrate(115200);
	//set_opt(_uart4, _Baudrate, 8, 'N', 1);
	return 1;
}



int
Linton::waiting()
{
	const char jet_status[] = {"@HMI=0,0\r\n"};

	_engine_status->e_n=_e_n;

		write(jet_status, sizeof(jet_status));

	return true;
}

int
Linton::start()
{
	static int last_fault=0;

	bool start_fault=(_fault==5||_fault==7||_fault==9||_fault==18||_fault==50);

	if(last_fault!=_fault&&start_fault)
	{
		PX4_INFO("starting fail");
		starting_fault=1;
		last_fault=_fault;
		return engine_status_t::fault;
	}
	if(_status==0||_status==24)
	{
	const char jet_start[] ={"@C.HMI=1,0\r\n"};
	write(jet_start, sizeof(jet_start));
	return false;
	}else
	{
	       if(_status==9
		||_status==10
		||_status==11
		||_status==12
		||_status==13
		||_status==14
		||_status==15
		||_status==16)
		{last_fault=_fault;
		return true;
		}

		else
		{
			last_fault=_fault;
			return false;
		}
	}
}

int
Linton::runn()
{
	static int last_fault=0;

	if(last_fault!=_fault&&_fault==22)
	{
		mavlink_log_critical(&mavlink_log_pub, "engine flame out!");
		flame_out=1;
	}
	PX4_INFO("flame out%d _fault%d last_fault%d",(int)flame_out,_fault,last_fault);
	if(flame_out&&_status!=29)
	{
		last_fault=_fault;
		return -1;
	}
	else
	{
		last_fault=_fault;
		return 0;
	}
}

int
Linton::stop()
{
	PX4_INFO("_status%d",(int)_status);
	if(
	   _status!=0
	 &&_status!=24
	 &&_status!=17
	 &&_status!=18
	 &&_status!=19
	 &&_status!=20
	 &&_status!=22)
	 {
	const char jet_stop[] = {"@C.HMI=0,0\r\n"};
	write(jet_stop, sizeof(jet_stop));
//	PX4_INFO("sent stop");
		return false;
	 }
	if(_status==0||_status==24)return true;
	else
	{
//	PX4_INFO("not yet");
	return false;
	}
}

int
Linton::test(uint8_t cmd)
{
	const char mot_stop[]={"@C.HMI=34.0"};
	const char mot_start[]={"@C.HMI=34.1"};
	const char pump_stop[]={"@C.HMI=35.0"};
	const char pump_start[]={"@C.HMI=35.1"};
	const char glowor_stop[]={"@C.HMI=36.0"};
	const char glowor_start[]={"@C.HMI=36.1"};

	switch(cmd)
	{
	case 0: write(mot_start, sizeof(mot_start));break;
	case 1: write(pump_start, sizeof(pump_start));break;
	case 2: write(glowor_start, sizeof(glowor_start));break;
	case 3: write(mot_stop, sizeof(mot_stop));break;
	case 4: write(pump_stop, sizeof(pump_stop));break;
	case 5: write(glowor_stop, sizeof(glowor_stop));break;
	default : write(mot_stop, sizeof(mot_stop));
		  write(pump_stop, sizeof(pump_stop));
		  write(glowor_stop, sizeof(glowor_stop));
	}
	return true;
}

int
Linton::handlefault()
{
	static uint8_t last_status=0;
	bool run=(_status==10||_status==11||_status==12||_status==13||_status==14||_status==15||_status==16);
	PX4_INFO("status%d fault%d last status%d last fault",(int)_status,(int)_fault,(int)last_status);
	if(last_status==29)
	{
	if(run)
	{
		flame_out=0;
		last_status=_status;
		return engine_status_t::running;
	}
	else
	{
		last_status=_status;
		return engine_status_t::fault;
	}
	}

	if(_status==0||_status==24)
	{
		starting_fault=0;
		last_status=_status;
		return engine_status_t::standby;
	}
	else
	{
		last_status=_status;
		return engine_status_t::fault;
	}
}





int
Linton::collect()
{
			int j = 0;
			int bytes_count = 0;
			/* then poll or read for new data */
		//	warnx("before read");
			int ret=read(_buf, sizeof(_buf),1000);
			//warnx("%d",ret);
		//	warnx("after read");
			if (ret < 0) {
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout while polling or just nothing read if reading, let's
				 * stay here, and use timeout below. */
				return 0;
			} else if (ret > 0) {
				/* if we have new data from ECU, go handle it */
				bytes_count = ret;
			}
		/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;

				if ((l = parseChar(_buf[j])) > 0) {
					/* return to configure during configuration or to the ECU driver during normal work
					 * if a packet has arrived */
					//warnx("parchar %d",l);
					handle(l);

				}

				j++;
			}
			/* everything is read */
			j = bytes_count = 0;


		return ret;


}

int Linton::parseChar(uint8_t b)
{
	int iRet = 0;
	static uint8_t a0,a1,a2;
	a2 = a1;
	a1 = a0;
	a0 = b;
	if(a0==0xff&&a1==0xff&&a2==0xff)
	{
		iRet = _rx_buffer_bytes;
		_rx_buffer_bytes = 0;

	}
	else
	{
		_rx_buffer[_rx_buffer_bytes++] = b;
		return 0;
	}

	return iRet;
}

int Linton::handle(int len)
{
	char *endp;

	bool val=0;

	char *bufptr = (char *)_rx_buffer;
	char *dptr = (char *)_rx_buffer;
	char *bod=(char *)_rx_buffer; //对指针操作一定要小心，由于之前初始化没做好，飞控经常在启动阶段或者读串口字符串时死机。
	char *eod=(char *)(_rx_buffer+3);
	uint dlen=0;
/*	for(int i=0; i<(len-7);i++)
	{
		if(*(bufptr+i)=='=')
		{
			if(*(bufptr+i+1)=='"')
			{
				bufptr =bufptr+i+2;
				val=0;
			}
		else
		{
		bufptr = bufptr+i+1;
		val=1;
		}
		}
	}
*/
	bool fir=0;
	for(int i=3;i<len;i++) //message begin with "page",so i begins as 4,in order to make sure avoid the incomplete data begins from 3
	{
		if(!fir && *(dptr+i)=='.'){bod=dptr+i;fir=1;}
		else if(*(dptr+i)=='.')eod=dptr+i;

		if(*(bufptr+i)=='=')
		{
			if(*(bufptr+i+1)=='"')
			{
				bufptr =bufptr+i+2;
				val=0;
			}
		else
		{
		bufptr = bufptr+i+1;
		val=1;
		}
		}
	}
	dlen= eod-bod-1;

	if(memcmp(dptr+4,"51",2)==0)
	{

		if (memcmp(bod+1, "n1", dlen) == 0)
		{
			double egt=0;
			if(val)
			egt = strtod(bufptr, &endp);
			_engine_status->egt[0]=egt;
			//warnx("egt=%f",egt);
		}else
		if (memcmp(bod+1, "t11", dlen) == 0)
		{
			double vol=0;
			vol = strtod(bufptr, &endp);
			_engine_status->vol[0]=vol;
			//warnx("pump vol=%f",vol);
		}else
		if (memcmp(bod+1, "va1", dlen) == 0)
		{
			uint64_t rpm;
			rpm = strtol(bufptr, &endp,10);
			_engine_status->rpm[0]=rpm;
			_engine_status->engine_load[0] = _engine_load(rpm);
		//	warnx("rpm=%d",(int)rpm);
		//	warnx("engine load=%f",(double)engine_load);
		}else
		if (memcmp(bod+1, "va4", dlen) == 0)
		{
			static uint8_t last_status;
			_status = strtol(bufptr, &endp,10);

			_engine_status->status[0]=_status;
		//	warnx("status=%d",_status);
			if(last_status!=_status)
			{
			switch(_status)
			{
			case 0	:mavlink_log_critical(&mavlink_log_pub, "engine stand by");break;
		//	case engine_state::starting	:mavlink_log_critical(&mavlink_log_pub, "engine starting....");break;
		//	case engine_state::igniting	:mavlink_log_critical(&mavlink_log_pub, "engine igniting...");break;
		//	case engine_state::ignited	:mavlink_log_critical(&mavlink_log_pub, "engine ignited!");break;
		//	case engine_state::warming1	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 1");break;
		//	case engine_state::warming2	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 2");break;
		//	case engine_state::warming3	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 3");break;
		//	case engine_state::warming4	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 4");break;
		//	case engine_state::warming5	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 5");break;
		//	case engine_state::declutch	:mavlink_log_critical(&mavlink_log_pub, "engine declutch");break;
		//	case engine_state::idle1	:mavlink_log_critical(&mavlink_log_pub, "engine idle");break;
		//	case engine_state::accelerating	:mavlink_log_critical(&mavlink_log_pub, "engine accelerating");break;
		//	case 12	:mavlink_log_critical(&mavlink_log_pub, "engine ");break;
		//	case 13	:mavlink_log_critical(&mavlink_log_pub, "engine ");break;
		//	case 14	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
		//	case 15	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
		//	case 16	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
		//	case engine_state::cooling1	:mavlink_log_critical(&mavlink_log_pub, "engine cooling down");break;
		//	case engine_state::cooling2	:mavlink_log_critical(&mavlink_log_pub, "engine cooling down2");break;
		//	case engine_state::cooling3	:mavlink_log_critical(&mavlink_log_pub, "engine cooling down3");break;
		//	case engine_state::cooling4	:mavlink_log_critical(&mavlink_log_pub, "engine cooling down");break;
		//	case engine_state::idle2	:mavlink_log_critical(&mavlink_log_pub, "engine idle");break;
			case 22	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
		//	case engine_state::NoRC		:mavlink_log_critical(&mavlink_log_pub, "engine no RC input");break;
		//	case engine_state::standby2	:mavlink_log_critical(&mavlink_log_pub, "engine stand by");break;
		//	case 25	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
		//	case 26	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
		//	case 27	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
		//	case 28	:mavlink_log_critical(&mavlink_log_pub, "engine maximum power");break;
			case 29	:mavlink_log_critical(&mavlink_log_pub, "engine restarting...");break;
		//	case 30	:mavlink_log_critical(&mavlink_log_pub, "throttle curve learning");break;


			default: if(_status>30)mavlink_log_critical(&mavlink_log_pub, "engine error");
			}
			}
			last_status= _status;

		}else
		if (memcmp(bod+1, "va5", dlen) == 0)
		{

			_fault = strtol(bufptr, &endp,10);
			_engine_status->fault[0]=_fault;
			_now = hrt_absolute_time();
			if(!_last_fault&&_fault==engine_fault::flameout)
			{
				_engine_stop_time=hrt_absolute_time();
			}
			if(_now-_engine_stop_time>200000)   //waiting for the ECU's responding 0.2s
			{
				if(_status==29) _restarting = true;    //restarting coz it response "restarting"
			}
			if(_restarting)
			{
				if(_now-_engine_stop_time>10000000)  //waiting for 10 seconds coz the restarting procedure take about 2.5s
				{
				if(_fault==engine_fault::flameout)        //make sure if the engine has been restarted successfully?
				{
					_restarted = false;
				}
				else
				{
					_restarted = true;
				}
				_restarting = false;	//no matter if is the restarting success the restarting procedure has timeout and finish
				}
			}
			if(_fault==engine_fault::ignfail||(_fault==engine_fault::flameout&&!_restarted&&!_restarting))_engine_status->engine_failure=true; //so if the engine has flameout it should be in the restarting procedure or already has been restarted successfully
			else _engine_status->engine_failure = false;
		//	warnx("fault=%d",_fault);
			_last_fault = _fault;
		}else
		if (memcmp(bod+1, "t7", dlen) == 0)
		{
			uint8_t min,sec;
			uint16_t running_time;
			min = strtol(bufptr, &endp,10);
			bufptr = endp + 1;
			sec = strtol(bufptr,&endp,10);
			running_time=60*min + sec;
			_engine_status->running_time[0]=running_time;
			//warnx("running time=%d:%d",min,sec);
			//warnx("running time in seconds=%d",running_time);
		}else
		if (memcmp(bod+1, "j0", dlen) == 0)
		{
			static bool warned = 0;
			double pump;
		/*	float fuel_consum_rate=_fuel_consum_rate.get();
			float fuel_tank_cap=_fuel_tank_cap.get();
			float fuel_low = _fuel_low.get();

			double fuel_tank_cap=1500.0f;
			double fuel_low = 0.6f;
		*/	pump = strtod(bufptr, &endp);
			double fuel_comsuption=_fuelcomsued(pump);

			double fuel_remain = (double)_para_set.tank_cap - fuel_comsuption;

			_engine_status->fuel_remain = fuel_remain;

		//	warnx("comdumed=%f comsumrate=%f pump=%f read_elapes=%d\n",_fuelcomsuption,fuel_consum_rate,pump,(int)read_elapsed);
			if(((double)_para_set.fuel_low>fuel_remain)&&!warned)
			{
				mavlink_log_critical(&mavlink_log_pub, "remaining fuel has reached the low point!");
				warned = 1;
			}

			_engine_status->pump[0]=pump;
			_engine_status->fuel_consumed = (float)fuel_comsuption;
		}

	}else if(memcmp(dptr+4,"10",2)==0) //got rpm setting
	{
		if (memcmp(bod+1, "n1", dlen) == 0)
		{
			_para_got.rpm_min = strtol(bufptr, &endp,10);
		}else
		if (memcmp(bod+1, "n2", dlen) == 0)
		{
			_para_got.rpm_max = strtol(bufptr, &endp,10);
		}
	}else if(memcmp(dptr+4,"13",2)==0) //got pwm setting
	{
		if (memcmp(bod+1, "n0", dlen) == 0)
		{
			_para_got.pwm_stop = (float)strtod(bufptr, &endp);

		}else
		if (memcmp(bod+1, "n1", dlen) == 0)
		{
			_para_got.pwm_min = (float)strtod(bufptr, &endp);


		}else
		if (memcmp(bod+1, "n2", dlen) == 0)
		{
			_para_got.pwm_max = (float)strtod(bufptr, &endp);

		}
	}
	/*_actuator.update(&_actuators);
	if(_armed.armed)
	_engine_status->thr_sp= _actuators.control[actuator_controls_s::INDEX_THROTTLE];
	else _engine_status->thr_sp=0;
	_engine_status_pub.publish(_engine_status); 不应该在这里发布ORB消息，否则每一小段消息，每一个数据就发布一次
*/
return 1;
}

int
Linton::setJEpara() // Although linton engine only has 2 groups of parameters. But we need to compare them one by one. So it needs to do 5 times
{
//	int je_br=_je_br.get();
//	setBaudrate(je_br);  //no need to set baudrate dynamically while the engine's running.
	char para[30]={};

	static int i=0;
/*	PX4_INFO("set");
	PX4_INFO("para got: pwm max%f min%f close%f rpm_max:%d rpm_min:%d",(double)_para_got.pwm_max,(double)_para_got.pwm_min,(double)_para_got.pwm_stop,(int)_para_got.rpm_max,(int)_para_got.rpm_min);
*/
	while(1)
	{
		if(i==0){ i++;		if(_para_set.rpm_min!=_para_got.rpm_min){sprintf(para,"@SetData=71,%d;\r\n",(int)_para_set.rpm_min);break;} }
		if(i==1){ i++;		if(_para_set.rpm_max!=_para_got.rpm_max){sprintf(para,"@SetData=70,%d;\r\n",(int)_para_set.rpm_max);break;} }
		if(i==2){ i++;		if(fabs(_para_set.pwm_stop-_para_got.pwm_stop)>.001){sprintf(para,"@SetData=12,%d;\r\n",(int)_para_set.pwm_stop);break;} }
		if(i==3){ i++;		if(fabs(_para_set.pwm_min-_para_got.pwm_min)>.001){sprintf(para,"@SetData=13,%d;\r\n",(int)_para_set.pwm_min);break;} }
		if(i==4){ i++;		if(fabs(_para_set.pwm_max-_para_got.pwm_max)>.001){sprintf(para,"@SetData=14,%d;\r\n",(int)_para_set.pwm_max);break;} }
		if(i>=5){ i=0;		return 1;}
	}
/*	PX4_INFO("i= %d",i);
	PX4_INFO("para set: pwm max%f min%f close%f rpm_max:%d rpm_min:%d ",(double)_para_set.pwm_max,(double)_para_set.pwm_min,(double)_para_set.pwm_stop,(int)_para_set.rpm_max,(int)_para_set.rpm_min);
	PX4_INFO("para got: pwm max%f min%f close%f rpm_max:%d rpm_min:%d",(double)_para_got.pwm_max,(double)_para_got.pwm_min,(double)_para_got.pwm_stop,(int)_para_got.rpm_max,(int)_para_got.rpm_min);

			warnx("got pwm max:%f,setted pwm max:%f",(double)_para_got.pwm_max,(double)_para_set.pwm_max);
			warnx("got pwm min:%f,setted pwm min:%f",(double)_para_got.pwm_min,(double)_para_set.pwm_min);
			warnx("got pwm stop:%f, setted pwm stop:%f",(double)_para_got.pwm_stop,(double)_para_set.pwm_stop);
			warnx("got rpm max:%f, setted rpm max:%f",(double)_para_got.rpm_max,(double)_para_set.rpm_max);
			warnx("got rpm min:%f, setted rpm min:%f",(double)_para_got.rpm_min,(double)_para_set.rpm_min);
		warnx("i=%d",i);
*/
		write(para,sizeof(para));
	return 0;
}

int
Linton::getJEpara() //linton engine only need to send 2 groups of parameters request
{
	static uint8_t para_num=0;
	int para_got=0;
	const char get_pwm[]={"@HMI=13,0;\r\n"};
	const char get_rpm[]={"@HMI=10,0;\r\n"};
	if(para_num==0){write(get_pwm,sizeof(get_pwm));}
	if(para_num==1){write(get_rpm,sizeof(get_rpm));}
	if(para_num<2)para_num++;
	else
	{para_num = 0; para_got = 1;}

	return para_got;
}
