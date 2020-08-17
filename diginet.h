/*
 * app_commands.h
 *
 *  Created on: 10-Jan-2020
 *      Author: adee9
 */

#ifndef APP_COMMANDS_H_
#define APP_COMMANDS_H_


#define MILLISECONDS			33ul
#define SECONDS					32768ul
#define MINUTES					(SECONDS*60ul)

#define PRIMARY_ELEMENT		0
#define MY_VENDOR_ID			0x02FF
#define MY_MODEL_ID			0xABCD
#define MESH_BROADCAST		0xC003
#define APP_BROADCAST			0xFFFF
#define PUBLISH_TYPE			1
#define NO_PUBLISH_TYPE		0

#define RS485_BROADCAST 		0xFF

#define DIGINET_CTRLR_ADDR	0x00
#define DIGINET_UID_LEN		12
#define DIGINET_ADDR_EXISTS	0xAA
#define DIGINET_NO_ADDR		0x55

typedef enum {
	FT_WHOIS=0,
	FT_IAM,
	FT_DATAREQ,
	FT_DATARES,
	FT_STATUSCHANGE,
	FT_COMMISSIONING,
	FT_COMMAND,
	FT_ACK,
	FT_NACK,
	FT_SWITCH_CMD,
	FT_EVENT,
	FT_CUSTOM_CMD = 63
} Frame_Type_t;


typedef enum {
	WHOIS,
	BATTERY_STATUS,
	PIR,
	ULTRASOUND,
	ALS,
	BAY_LIGHTING,
	TEMP_HUMIDITY,
	CO2,
	tVOC,
	DUST_SENSOR ,
	OZONE,
	PRESSURE,
	MIC,
	JUNCTIONAL_TEMP,
	SENSOR_PKT_STATS,
	SENSOR_RSSI,
	LCD_CONFIG						= 0x0B,
	ENV_ADDR 							= 0x0D,
	CONFIG_REQ_LCD 					= 0X1C,
	CONFIG_REQ_ENV 					= 0X1D,
	CONFIG_REQ_SCENE 				= 0X1E,
	CONFIG_REQ_SW 					= 0X1F,
	SW_SCENE 							= 0x2A,
	DIM_PERCENT 						= 0x2B,
	TEMP_UP 							= 0x2C,
	TEMP_DOWN 						= 0x2D,
	PAIRING_REQ 						= 0x30,
	PAIRING_ACCPT 					= 0x31,
	PAIRING_CMPLT 					= 0x32,
	PAIRING_STOP 					= 0x33,
	GOTO_OTA							= 0x3E,
	TIME_SYNC 						= 0x40,
	ENV_POLL 							= 0x71,
	COMMISSIONED						= 0xC0,
	STOP_TXN 							= 0xC1,
	SYSTEM_RESET 					= 0xC2,
	DEF_CONFIG 						= 0xD0,
	CUSTM_SENSOR_CONFIG 			= 0xD1,
	CONFIG_REQUEST					= 0xD2,
	FACTORY_RESET					= 0xD4,
	PGA_CONFIG 						= 0xD5,
	SENSOR_INDENTIFY 				= 0xD6,
	PIR_RETRANSMISSION_EN 		= 0xD7,
	PIR_RETRANSMISSION_DIS 		= 0xD8,
	SENSOR_MODE 						= 0xD9,
	CRONTAB							= 0xDA,
	CONFIG_ENABLE					= 0xDB,
	SHID_ASSIGNEMT 					= 0xE0,
	ULTRA_SOUND_CONFIGPARAMS		= 0xF0,
	ULTRA_SOUND_TEST_EN 			= 0xF1,
	ULTRA_SOUND_TEST_DIS	 		= 0xF2,
	US_REG_WRITE		 				= 0xF3,
	PGA_REG_READ						= 0xF4,
	US_DIS_MEAS_CONSTRAINT		= 0xF5,
	CMD_ACK							= 0xF6,
	PROV_MODE_ENABLE				= 0xFE
} Data_Cmd_t;

typedef enum
{
	APP_SCENE_CMD_ID = 0x1A,//sending
	APP_BALLAST_CMD_ID = 0x06,//send
	APP_TEMP_CMD_ID = 0x1D,//send
	SCENE_DELETION = 0x36,
	TEMP_UPDATE_GATEWAY = 0x34,
	SCENE_CONFIG = 0x35,
	ENV_CONFIG = 0x32,
	MAPPING_INFO = 0x31,
	SW_CONFIG = 0x30,
} Extended_Cmds;

// Packet helpers

#define PREAMBLEH		0x03
#define PREAMBLEL		0x56
#define MAX_PACKET_LENGTH	64
#define CRCLENGTH				0x02
#define RS485_HEADER_LEN			5
#define HEADER_SIZE			7

typedef struct {
	uint16_t preamble;
	Frame_Type_t frameType;
	uint8_t sourceAddress;
	uint8_t destinationAddress;
	uint16_t HeaderCRC;
	uint8_t PacketData[64]; // Byte 0 is data length and last 2 bytes are CRC
} __attribute__((__packed__)) RS485_Frame_t;



typedef enum {
	UNOCCUPIED	= 0x00,
	OCCUPIED		= 0xFF
} Pir_Status_t;

typedef struct {
	uint16_t lux;
}__attribute__((__packed__)) Als_Status_t;

typedef struct{
	uint16_t temperature;
	uint16_t humidity;
}__attribute__((__packed__)) HT_Status_t;

typedef struct {
	union{
		Pir_Status_t pir_status;
		Als_Status_t als_status;
		HT_Status_t ht_status;
	};
}__attribute__((__packed__)) sensor_data_t;

typedef struct {
	uint16_t dist_threshold;
	uint16_t sampling_time_sec;
//	uint16_t us_reTransmissionTimeout;
}__attribute__((__packed__)) US_Sensor_Cfg_t;

typedef struct {
	uint16_t pir_Sleep_time_sec;
	uint16_t pir_waitWatch_time_sec;
	uint16_t pir_reTransmissionTimeout;
}__attribute__((__packed__)) PIR_Sensor_Cfg_t;

typedef struct {
	uint16_t freq_LUXmeasure_sec;
	uint16_t luxThreshold;
	uint16_t calibration_factor;
}__attribute__((__packed__)) ALS_Sensor_Cfg_t;

typedef struct {
	uint16_t freq_THmeasure_sec;
	uint16_t tempThreshold;
	uint16_t humidityThreshold;
}__attribute__((__packed__)) HT_Sensor_Cfg_t;

typedef struct {
	uint32_t 				deviceType;
	US_Sensor_Cfg_t 		sUSconfig;
	PIR_Sensor_Cfg_t 	sPIR_configartions;
	ALS_Sensor_Cfg_t 	sALSconfig;
	HT_Sensor_Cfg_t 		sTHconfig;
	uint16_t 				configVersion;
	uint16_t 				firmwareVersion;
}__attribute__((__packed__)) Sensor_Cfg_t;

typedef struct {
	uint32_t 		deviceType;
	uint16_t 		configVersion;
	uint16_t 		firmwareVersion;
	uint8_t 		paired_router;
}__attribute__((__packed__)) Sensor_Cfg_Min_t;

typedef struct sensor_db_t {
	uint8_t			ext_id[12];
	uint16_t			mesh_short_id;
	uint8_t			master_short_id;
	uint8_t			cfg[sizeof(Sensor_Cfg_Min_t)];
	uint32_t			num_packets_rx;
	uint8_t			rssi;
	uint8_t			seq_no;
}__attribute__((__packed__)) sensor_db_t;

typedef struct pairing_resp_t {
	Frame_Type_t 			ftype;
	Data_Cmd_t			cmd_id;
	uint32_t				UTC;
} __attribute__((__packed__)) pairing_resp_t;


//CMD_IAM len is 0x29 for als_calibration and us retransmission timeout
//CMD_IAM len is 0x25 without als_calibration and us retransmission timeout
//CMD_IAM len is 0x27 with als_calibration

typedef enum {
	CMD_WHOIS,
	CMD_IAM  = 22,
	CMD_DATAREQ_AP  = 0x01,
	CMD_DATAREQ_BS = 0x01,
	CMD_DATAREQ_PIR = 0x01,
	CMD_DATAREQ_US = 0x01,
	CMD_DATAREQ_ALS = 0x01,
	CMD_DATAREQ_BL = 0x01,
	CMD_DATAREQ_TH = 0x01,
	CMD_DATARES_AP = 0x06,
	CMD_DATARES_BS  = 0x02,
	CMD_DATARES_PIR = 0x02,
	CMD_DATARES_US = 0x02,
	CMD_DATARES_ALS = 0x03,
	CMD_DATARES_BL = 0x02,
	CMD_DATARES_TH = 0x05,
	CMD_STATCH_BS  = 0x02,
	CMD_STATCH_US  = 0x02,
	CMD_STATCH_PIR  = 0x02,
	CMD_STATCH_ALS = 0x03,
	CMD_STATCH_BL  = 0x02,
	CMD_STATCH_TH  = 0x05,
	CMD_cmdSHID  = 14,
	CMD_COMMISSION  = 0x00,
	CMD_FACTORY_RESET = 0x01,
	CMD_SYSTEM_RESET = 0x01,
	CMD_SNESOR_CONFIG = 26,
	CMD_PGA_CONFIG,
	CMD_SYS_IDENTIFY = 0x01 ,
	CMD_PIR_RETRANS_EN_DIS = 0x01,
	CMD_SENSOR_MODE = 0x03 ,
	CMD_DATARES_PGA_READ = 0x03,
	CMD_DATALEN_MAX = 40
} DataCmd_length;

typedef struct vm_payload_t {
	Frame_Type_t	f_type;
	Data_Cmd_t			Cmd;
	union{
		Sensor_Cfg_t cfg_pkt;
		sensor_data_t sensor_data;
	};
}__attribute__((__packed__)) vm_payload_t;

#endif /* APP_COMMANDS_H_ */
