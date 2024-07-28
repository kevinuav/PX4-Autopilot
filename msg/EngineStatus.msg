uint64 timestamp	# time since system start (microseconds)

uint8 ETYPE_NONE = 0
uint8 ETYPE_LINTON = 1
uint8 ETYPE_JETCAT = 3
uint8 ETYPE_GE = 4

int8 UNINIT = 0
int8 STANDBY = 1
int8 STARTING = 2
int8 RUNNING = 3
int8 STOPPING = 4
int8 FAULT = -1

uint8 e_n		# how many engines the plane has
uint8 e_id		# engine's id
float32[4] egt		# temperature
float32[4] vol		# pump's voltage
uint64[4] rpm		# RPM
uint8[4] status		# ECU's status index
uint8[4] fault		# ECU's fault index
uint8 engine_type	# enginetype
uint16[4] running_time	# engine's running time
float32[4] pump		# pump's power
float32 fuel_consumed	# the fuel has been comsumed
float32 fuel_remain	# fuel remain in the fuel tank
float32[4] engine_load	# engine's load(output)
float32 thr_sp		# throttle setpoint comes from the flight controler
bool engine_failure
