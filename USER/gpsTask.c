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

char g_Mode = 0;
char g_Even = 1;
nmeaPOS lastPos;
nmeaPOS nowPos;
double dst;
u8 crcFlag = 0;
struct_gpsDataMQ  gpsDataMQ;

void GPRMC_CallBack(struct_GPSRMC GPS_RMC_Data) {
    crcFlag = 3;
    //printf("=========GPRMC_CallBack=====\n");
}
void GPGGA_CallBack(struct_GPSGGA GPS_GGA_Data) {
    crcFlag = 3;
	//printf("Altitude:%.1fmeter\n",atof(GPS_GGA_Data.Altitude));
}
void GPGSA_CallBack(struct_GPSGSA GPS_GSA_Data) {
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
	return;

    printf("dir=%f,dir_t=%c dec=%3.2f,dec_m=%c spn=%f,spn_n=%c spk=%f,spk_k=%c\n",
										 vtgPack.dir,vtgPack.dir_t,
										 vtgPack.dec,vtgPack.dec_m,
										 vtgPack.spn,vtgPack.spn_n,
										 vtgPack.spk,vtgPack.spk_k);
}
void GPGLL_CallBack(struct_GPGLL GLLPack) {
    crcFlag = 3;
    //printf("%d\n",g_Even);
    if(g_Even == 1) {
		g_Even = 2;
	    ledTurnOn(0);
	} else {
	    g_Even = 1;
	    ledTurnOn(1);
	}
}

void initNmeaParserCallBack() {
    initParserCallBack(GPRMC_CallBack,GPGGA_CallBack,
	                   GPGSA_CallBack,GPGSV_CallBack,
	                   GPVTG_CallBack,GPGLL_CallBack);
}
nmea_msg gpsx;
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
char validLocation_has_dir_count = 0;  //有效定位，且有航向计数,
extern char* g_gpsData_ptr;
GpsFrame gpsFrame;
u8 gpsFrameData[9][100];
u8 hDOP_Diff = 0;
u8 hDOP_Last = 0; 
u8 validDirPosCnt = 0;
u8 validPosCnt= 0;
int hAcc = 0;
#define TYPE_NO_POS             1
#define TYPE_BYPASS             2
#define TYPE_LAST_VAILID_POS    3

void _sendNmeaProtocol(int type) {
    switch(type) {
        case 4:
             if(gpsFrame.nmeaPtrRMC != 0) {
				 //NMEA_StrReplace(gpsFrame.nmeaPtrRMC,3,NULL); //latitude
				 //NMEA_StrReplace(gpsFrame.nmeaPtrRMC,5,NULL); //longitude
			 }
			 if(gpsFrame.nmeaPtrGGA != 0) {
				 //NMEA_StrReplace(gpsFrame.nmeaPtrGGA,3,NULL); //latitude
				 //NMEA_StrReplace(gpsFrame.nmeaPtrGGA,5,NULL); //longitude

			 }
			 if(gpsFrame.nmeaPtrGLL != 0) {
				 //NMEA_StrReplace(gpsFrame.nmeaPtrGLL,3,NULL); //latitude
				 //NMEA_StrReplace(gpsFrame.nmeaPtrGLL,5,NULL); //longitude
	         }
			 break;
	    case TYPE_BYPASS:

			 break;
		case TYPE_LAST_VAILID_POS:

			 break;
	}
	if(gpsFrame.nmeaPtrRMC != 0) {
	   printf("%s\n",gpsFrame.nmeaPtrRMC);
	   gpsFrame.nmeaPtrRMC = 0;
	}
	if(gpsFrame.nmeaPtrGGA != 0) {
		printf("%s\n",gpsFrame.nmeaPtrGGA);
		gpsFrame.nmeaPtrGGA = 0;
	}
	if(gpsFrame.nmeaPtrGLL != 0) {
		printf("%s\n",gpsFrame.nmeaPtrGLL);
		gpsFrame.nmeaPtrGLL = 0;
	}

}
void sendNmeaToUart() {
	if(hDOP_Diff < 2) {
		if(hAcc < 80) {
			if(gpsx.state=='A') {   //有效定位
				if(gpsx.dir>=0 && gpsx.dir <=360) {
					validDirPosCnt++;
					if(validDirPosCnt > 4) {
	                    validPosCnt = 0;
						_sendNmeaProtocol(TYPE_BYPASS);
					} else {
						_sendNmeaProtocol(TYPE_LAST_VAILID_POS);
					}
				} else {
					validPosCnt++;
					if(validPosCnt > 3) {
						validDirPosCnt = 0;
						if(gpsx.hdop < 80) {
	                        _sendNmeaProtocol(TYPE_BYPASS);
						} else {
	                        _sendNmeaProtocol(TYPE_NO_POS);
						}
					} else {
						_sendNmeaProtocol(TYPE_NO_POS);
					}
				}
			} else {  //无效定位
			    _sendNmeaProtocol(TYPE_NO_POS);
				initUbxCfg(UBX_CFG_NAV2);
				validPosCnt = 0;
				validPosCnt = 0;
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
        0xB5,0x62,0x06,0x23,0x28,0x00,0x00,0x00,0x4C,0x66,0x00,0x00,0x00,0x00,
        0x00,0x00,0x03,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x64,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x7B,0x9E,
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
		//Uart1_PutChar(enableCFG[i]);
	}
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
}
u8 result;

char ubxStart = 0;
void gpsTaskMain() {	
	char bufferSize = 0;	
    int i = 0,c = 0,flag;
	
	bufferSize = getQueueSize();
	//printf("s=%d\n",bufferSize);
	if(bufferSize >= 1) {
		for(i = 0;i < bufferSize;i++) {
			c = PopQueue();
            if(c == 0xff) {

            } else if(ubxStart) {
                Uart1_PutChar(c);
                if(ubx_Parser(c)==1) 
                    ubxStart = 0;
            } else if(c == 0xB5) { //0xB5 0x62
                ubxStart = 1;
                Uart1_PutChar(c);
                ubx_Parser(c);
            } else {
                flag = NMEA_Parser(c);
                //printf("flag=0x%x,c=%c\n",flag,c);
                if(c == ',') {
                    Uart1_PutChar(',');
                } else if((flag&0X03) != 0x00) {
                    if((flag&0x03) == GPS_PARSE_STATUS_LATITUDE)
                        Uart1_PutChar('<');
                    else if((flag&0x03) == GPS_PARSE_STATUS_LONGTITUDE)
                        Uart1_PutChar('>');
                } else if(crcFlag > 0){
                    if(crcFlag == 3)
                        Uart1_PutChar('*');
                    else if(crcFlag == 2) {
                        Uart1_PutChar('a');
                    } else if(crcFlag == 1) {
                        Uart1_PutChar('b');
                    }
                    crcFlag--;
                } else {
                    Uart1_PutChar(c);
                }
                if((flag&0x30) == GPS_PARSE_STATUS_CRC_START) {
                    result ^= c;
                } 
            }
		}
    }
}
