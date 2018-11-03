#include <includes.h>            
#include "nmeaParse.h"
#include "nmeaParse2.h"
#include "gmath.h"

#define UBX_CFG_ENABLE    0
#define UBX_CFG_NAV2      1
#define UBX_CFG_NAV5      2
#define UBX_CFG_NAVX5     3
#define UBX_CFG_RATE      4

static void gpsTask (void);
void initUbxCfg(u8 cfg);
void handerGpsData(const char* gpsBuffer,int size);
void sendNmeaToUart();
static NMEA_BASE_MSG ValidLastPos;
extern NMEA_BASE_MSG nmea_base_msg;

char g_Mode = 0;
char g_Even = 1;
nmeaPOS lastPos;
nmeaPOS nowPos;
double dst;
struct_gpsDataMQ  gpsDataMQ;
nmea_msg gpsx;
u8 hDOP_Diff = 0xff;   //上次的水平因子减去当前的水平因子
u8 hDOP_Last = 0; 
u8 validDirPosCnt = 0;
u8 validPosCnt= 0;
int hAcc = 0;

void GPRMC_CallBack(struct_GPSRMC GPS_RMC_Data) {
    //printf("=========GPRMC_CallBack=====\n");
    gpsx.state = GPS_RMC_Data.Status;
}
void GPGGA_CallBack(struct_GPSGGA GPS_GGA_Data) {
	nmea_base_msg.hdop = atof(GPS_GGA_Data.HDOP);
    if(hDOP_Last != 0) {
        if(hDOP_Last > nmea_base_msg.hdop) {
            hDOP_Diff = hDOP_Last - nmea_base_msg.hdop;
        } else {
            hDOP_Diff = nmea_base_msg.hdop - hDOP_Last;
        }
        hDOP_Last = nmea_base_msg.hdop;
    } else {
        hDOP_Last = nmea_base_msg.hdop;
        hDOP_Diff = 0xff;
    }
	//printf("dif_hdop=%.1f\n",hDOP_Diff);
}
void GPGSA_CallBack(struct_GPSGSA GPS_GSA_Data) {
    gpsx.fixmode = GPS_GSA_Data.Mode2;
	return;
   	printf("=========GPGSA_CallBack=====\n");
    printf("Mode1 = %c\n",GPS_GSA_Data.Mode);
    printf("Mode2 = %c\n",GPS_GSA_Data.Mode2);
    printf("Mode2 = %c\n",GPS_GSA_Data.Mode2);
    printf("Mode2 = %c\n",GPS_GSA_Data.Mode2);
    printf("Mode2 = %c\n",GPS_GSA_Data.Mode2);
	
}
void GPGSV_CallBack(struct_GPSGSV GPS_GSV_Data) {
	int i;
	return;

   	printf("=========GPGSV_CallBack=====\n");
    printf("SatInView = %d\n",GPS_GSV_Data.SatInView);
    for(i=0;i < 12;i++) {
       printf("SatID=%02d,Elevation=%02d,Azimuth=%03d,SNR=%02d\n",GPS_GSV_Data.SatInfo[i].SatID,GPS_GSV_Data.SatInfo[i].Elevation,
                                                                  GPS_GSV_Data.SatInfo[i].Azimuth,GPS_GSV_Data.SatInfo[i].SNR);
    }
}
void GPVTG_CallBack(nmeaGPVTG vtgPack) {
    gpsx.speed = vtgPack.spk;
    gpsx.dir = vtgPack.dir;
    //printf("spk=%.3f\n",vtgPack.spk);
	return;

    printf("dir=%f,dir_t=%c dec=%3.2f,dec_m=%c spn=%f,spn_n=%c spk=%f,spk_k=%c\n",
										 vtgPack.dir,vtgPack.dir_t,
										 vtgPack.dec,vtgPack.dec_m,
										 vtgPack.spn,vtgPack.spn_n,
										 vtgPack.spk,vtgPack.spk_k);
}
void GPGLL_CallBack(struct_GPGLL GLLPack) {
    //printf("%d\n",g_Even);
    if(g_Even == 1) {
		g_Even = 2;
	    ledTurnOn(0);
	} else {
	    g_Even = 1;
	    ledTurnOn(1);
	}
    sendNmeaToUart();
}

void initNmeaParserCallBack() {
    initParserCallBack(GPRMC_CallBack,GPGGA_CallBack,
	                   GPGSA_CallBack,GPGSV_CallBack,
	                   GPVTG_CallBack,GPGLL_CallBack);
}
void debugMode(char mode) {
  switch(mode) {
  	case '1':
	  break;
    case '2': //show RTC
	  break;
	case '3': //show ubx
	  break;
  }
}

enum CAR_MODE
{
	CAR_MODE_invalid    = 0x00, //未定位
	CAR_MODE_standstill = 0x01, //固定模式
	CAR_MODE_startUp    = 0x02, //启动模式
    CAR_MODE_park       = 0x03, //停车模式
    CAR_MODE_travel     = 0x04, //行驶模式
};

#define TYPE_NO_POS             1
#define TYPE_BYPASS             2
#define TYPE_LAST_VAILID_POS    3
u8 transportMode = TYPE_LAST_VAILID_POS;
void _sendNmeaProtocol(int type) {
    transportMode = type;
    switch(type) {
	    case TYPE_NO_POS:

			break;
		case TYPE_BYPASS:
            memcpy(&ValidLastPos.latitude[0], &nmea_base_msg.latitude[0],10);
            memcpy(&ValidLastPos.longitude[0],&nmea_base_msg.longitude[0],11);
            break;
        case TYPE_LAST_VAILID_POS:

			break;
	}
}

void sendNmeaToUart() {
   	//printf("sntu:%d,%d,%d,%c,%f \n",hAcc,hDOP_Diff,gpsx.speed,gpsx.state,gpsx.dir);
	if(hAcc < 80) {
		if(hDOP_Diff < 2) {
            if(gpsx.speed < 180) {
                if(gpsx.state=='A') {   //有效定位
                    if(gpsx.dir>=0 && gpsx.dir <=360) {
                        validDirPosCnt++;
                        if(validDirPosCnt > 4) {
                            validPosCnt = 0;
                            _sendNmeaProtocol(TYPE_BYPASS);
                        } else {
                            _sendNmeaProtocol(TYPE_NO_POS);
                        }
                    } else {  //定位无航向
                        if(gpsx.fixmode == 2 || gpsx.fixmode == 3) {
                            validPosCnt++;
                            if(validPosCnt > 4) {
                                validDirPosCnt = 0;
                                if(ValidLastPos.latitude[0] != '\0') {
                                    _sendNmeaProtocol(TYPE_LAST_VAILID_POS);
                                } else {
                                    _sendNmeaProtocol(TYPE_BYPASS);
                                }
                            } else {
                                _sendNmeaProtocol(TYPE_NO_POS);
                            }
                        } else { //1=未定位
                            _sendNmeaProtocol(TYPE_NO_POS);
                        }
                    }
                } else {  //无效定位
                    _sendNmeaProtocol(TYPE_NO_POS);
                    initUbxCfg(UBX_CFG_NAV2);  //pAcc设置为500米
                    validPosCnt = 0;
                    validPosCnt = 0;
                }
            }else {
                _sendNmeaProtocol(TYPE_NO_POS);
            }
		}else {
			_sendNmeaProtocol(TYPE_NO_POS);
		}
	} else {
	    _sendNmeaProtocol(TYPE_NO_POS);
	}
}
void initUbxCfg(u8 cfg){
	u8 i = 0,length =0 ;
	u8 *p;
	u8 ubxCfgNavPosllh[] = {  //enable NAV-POSLLH (0x01 0x02)
		0xB5,0x62,0x06,0x01,0x06,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x11,0x88,
	};
	char ubxCfgNav2[] = {  //pAcc = 600m
	    0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x00,0x03,0x00,0x00,0x00,0x00,
        0x10,0x27,0x00,0x00,0x05,0x00,0xFA,0x00,0xFA,0x00,0x58,0x02,0x5E,0x01,
        0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x74,0x86,
	};
    char ubxCfgNav5[] = {  //Min SV Elevation = 0 deg
        0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x00,0x03,0x00,0x00,0x00,0x00,
        0x10,0x27,0x00,0x00,0x00,0x00,0xFA,0x00,0xFA,0x00,0x64,0x00,0x2C,0x01,
        0x00,0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x42,0x97,
	};
    char ubxCfgNAVX5[] = {  //Version0 and  use AssisNow Autonomous
        0xB5,0x62,0x06,0x23,0x28,0x00,0x00,0x00,0x4C,0x66,0x00,0x00,0x00,0x00,0x00,0x00,
        0x03,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x01,0x00,0x00,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7B,0x9E,
	};
	char ubxCfgRate[] = {  // 500ms = 5HZ
	     0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77,
	};

	if(cfg == UBX_CFG_ENABLE) {
		p = ubxCfgNavPosllh;
		length = sizeof(ubxCfgNavPosllh);
	} else if(cfg == UBX_CFG_NAV2){
		p = ubxCfgNav2;
		length = sizeof(ubxCfgNav2);
    } else if(cfg == UBX_CFG_NAV5){
		p = ubxCfgNav5;
		length = sizeof(ubxCfgNav5);
    } else if(cfg == UBX_CFG_NAVX5){
		p = ubxCfgNAVX5;
		length = sizeof(ubxCfgNAVX5);
	} else if(cfg == UBX_CFG_RATE) {
		p = ubxCfgRate;
		length = sizeof(ubxCfgRate);
	}
	//printf("initUbxCfg cfg = %d,length = %d",cfg,length);
	for(i = 0;i < length;i++) {
        Uart2_PutChar(p[i]);
		//printf("%02x",p[i]);
	}
    //printf("\n");
}
void printHexToString(const char* buffer,u8 buffer_sz) {
	u8 i = 0;
    for(i = 0;i < buffer_sz;i++) {
		if(g_Mode == '3')  printf("%x ",buffer[i]);
		else               Uart1_PutChar(buffer[i]);
	}
	printf("\n");
}
int valid = 0;
void handerUbxData(const char* ubxBuffer,int size) {
	
	printHexToString(ubxBuffer,size);
	if(ubxBuffer[2]==0x01 && ubxBuffer[3]==0x02 ) {//0xB5,0x62,0x01,0x02
		//hAcc = ubxhAcc(ubxBuffer,size);
		//printf("hAcc = %.2f m\n",(double)hAcc/1000);
		#if 0
		if(hAcc < 100*1000 && hAcc > 0) { //hAcc < 100m
		    if(valid == 0) {
				valid = 1;
				enableUbxCfg(1);
			}
		} else {
			valid = 0;
		}
		#endif
	}
}
/*判断是否为有效定位
**1.HDOP不跳动,
**2.有航向
**3.有效定位的经纬度
**
*/
u8 isEffectivePositioning() {
   u8 isEffective = 0;
   
   return isEffective;
}
void initGpsModule() {
	initNmeaParserCallBack();
	USART2_Configuration();
	initUbxCfg(UBX_CFG_ENABLE);	
	initUbxCfg(UBX_CFG_NAV5);
	initUbxCfg(UBX_CFG_NAVX5);
	//initUbxCfg(UBX_CFG_RATE);
	
    memcpy(&ValidLastPos.latitude[0], "2039.76711",10);
    memcpy(&ValidLastPos.longitude[0], "10401.32691",11);
}
static u8 crcResult=0;
void doCRC(char c,u8 flag) {
    flag = flag & 0x70;
    if(flag == GPS_PARSE_STATUS_CRC_START) {
        crcResult ^= c;
    } else if(flag == GPS_PARSE_STATUS_CRC_END) {

    }
    //printf(" c=%c 0x%x,crc=0x%x   ",c,c,result);
}

char sended = 0;
void outputNmea(char c,char flag) {
    char i = 0;
    if((flag & 0x03) == GPS_PARSE_STATUS_LATITUDE) {
        if(transportMode == TYPE_BYPASS) {
            doCRC(c,flag);
            Uart1_PutChar(c);
        } else if(transportMode == TYPE_NO_POS){

        } else if(transportMode == TYPE_LAST_VAILID_POS){
            if(sended == 0) {
                Uart1_PutChar(',');
                doCRC(c,flag);
                for(i = 0;i < 10;i++) {
                    c = ValidLastPos.latitude[i];
                    doCRC(c,flag);
                    Uart1_PutChar(c);
                }
                sended = 1;
             }
        }
    } else if((flag & 0x03) == GPS_PARSE_STATUS_LONGTITUDE) {
        if(transportMode == TYPE_BYPASS) {
            doCRC(c,flag);
            Uart1_PutChar(c);
        } else if(transportMode == TYPE_NO_POS){

        } else if(transportMode == TYPE_LAST_VAILID_POS){
            if(sended == 0) {
                Uart1_PutChar(',');
                doCRC(c,flag);
                for(i = 0;i < 11;i++) {
                    c = ValidLastPos.longitude[i];
                    doCRC(c,flag);
                    Uart1_PutChar(c);
                }
                sended = 1;
            }
        }
    } else {
           sended = 0;
           doCRC(c,flag);
           if((flag&0xf0) == GPS_PARSE_STATUS_CRC_CRC1) {
              printf("%x",crcResult);
              crcResult = 0;
           } else if((flag&0xf0) == GPS_PARSE_STATUS_CRC_CRC2) {

           } else {
              Uart1_PutChar(c);
           }
      }
}

char ubxStart = 0;
static char lastChar = 0;
void gpsTaskMain() {	
	char bufferSize = 0;	
    int i = 0,c = 0,flag;
	
	bufferSize = getQueueSize();
	//printf("s=%d\n",bufferSize);
	if(bufferSize >= 1) {
		for(i = 0;i < bufferSize;i++) {
			c = PopQueue();
            //printf("%x",c);
            //return;
            if(ubxStart) {
                Uart1_PutChar(c);
                if(ubx_Parser(c)==1) 
                    ubxStart = 0;
            } else if(lastChar == 0xB5 && c == 0x62) {
                ubxStart = 1;
                lastChar = 0;
                Uart1_PutChar(0xB5);
                Uart1_PutChar(0x62);
                ubx_Parser(0xB5);
                ubx_Parser(0x62);
            } else if(c == 0xB5) { //0xB5 0x62
                lastChar = 0xB5;
            } else {
                lastChar = 0;
                flag = NMEA_Parser(c);
                //outputNmea(c,flag);
                Uart1_PutChar(c);
            }
		}
    }
}
