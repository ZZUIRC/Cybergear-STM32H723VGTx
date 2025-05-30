#ifndef CYBER_GEAR_DRIVER_DEFS_H
#define CYBER_GEAR_DRIVER_DEFS_H

#define CMD_CONTROL 0
#define CMD_POSITION 1
#define CMD_RESPONSE 2
#define CMD_ENABLE 3
#define CMD_RESET 4
#define CMD_SET_MECH_POSITION_TO_ZERO 6
#define CMD_CHANGE_CAN_ID 7
#define CMD_RAM_READ 17
#define CMD_RAM_WRITE 18
#define CMD_GET_MOTOR_FAIL 21

#define AD_AA 0X00

#define ADDR_RUN_MODE 0x7005
#define ADDR_IQ_REF 0x7006
#define ADDR_SPEED_REF 0x700A
#define ADDR_LIMIT_TORQUE 0x700B
#define ADDR_CURRENT_KP 0x7010
#define ADDR_CURRENT_KI 0x7011
#define ADDR_CURRENT_FILTER_GAIN 0x7014
#define ADDR_LOC_REF 0x7016
#define ADDR_LIMIT_SPEED 0x7017
#define ADDR_LIMIT_CURRENT 0x7018
#define ADDR_MECH_POS 0x7019
#define ADDR_IQF 0x701A
#define ADDR_MECH_VEL 0x701B
#define ADDR_VBUS 0x701C
#define ADDR_ROTATION 0x701D
#define ADDR_LOC_KP 0x701E
#define ADDR_SPD_KP 0x701F
#define ADDR_SPD_KI 0x7020

#define MODE_MOTION 0x00
#define MODE_POSITION 0x01
#define MODE_SPEED 0x02
#define MODE_CURRENT 0x03

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -12.0f
#define T_MAX 12.0f
#define IQ_MIN -27.0f
#define IQ_MAX 27.0f
#define CURRENT_FILTER_GAIN_MIN 0.0f
#define CURRENT_FILTER_GAIN_MAX 1.0f

#define IQ_REF_MAX 23.0f
#define IQ_REF_MIN -23.0f
#define SPD_REF_MAX 30.0f
#define SPD_REF_MIN -30.0f
#define LIMIT_TORQUE_MAX 12.0f
#define LIMIT_TORQUE_MIN 0.0f
#define CUR_KP_MAX 200.0f
#define CUR_KP_MIN 0.0f
#define CUR_KI_MAX 200.0f
#define CUR_KI_MIN 0.0f
#define LOC_KP_MAX 200.0f
#define LOC_KP_MIN 0.0f
#define SPD_KP_MAX 200.0f
#define SPD_KP_MIN 0.0f
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_SPD_MIN 0.0f
#define LIMIT_CURRENT_MAX 27.0f
#define LIMIT_CURRENT_MIN 0.0f

#define DEFAULT_CURRENT_KP 0.125f
#define DEFAULT_CURRENT_KI 0.0158f
#define DEFAULT_CURRENT_FINTER_GAIN 0.1f
#define DEFAULT_POSITION_KP 30.0f
#define DEFAULT_VELOCITY_KP 2.0f
#define DEFAULT_VELOCITY_KI 0.002f
#define DEFAULT_VELOCITY_LIMIT 2.0f
#define DEFAULT_CURRENT_LIMIT 27.0f
#define DEFAULT_TORQUE_LIMIT 12.0f

#define RET_CYBERGEAR_OK 0x00
#define RET_CYBERGEAR_MSG_NOT_AVAIL 0x01
#define RET_CYBERGEAR_INVALID_CAN_ID 0x02
#define RET_CYBERGEAR_INVALID_PACKET 0x03

#define CYBERGEAR_RESPONSE_TIME_USEC 250

#define CW 1
#define CCW -1

#endif  // !CYBER_GEAR_DRIVER_DEFS_H
