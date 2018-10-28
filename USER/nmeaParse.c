#include <stdio.h>
#include <stdlib.h>
#include "nmeaParse.h"

static u8   NMEA_Start = 0;                 // NMEA ��俪ʼ. ��⵽ $ ʱ�� 1
static u8   NMEA_TypeParsed = 0;            // NMEA ���ͽ������
static u8   NMEA_MsgType = NMEA_NULL;       // NMEA �������
static char NMEA_MsgTypeBuff[] = "GPxxx,";  // NMEA �������ʶ�𻺴�
static u8   NMEA_MsgTypeIndex = 0;          // ��ȡ NMEA ����ʶ���ַ��ĸ���
static u8   NMEA_MsgBlock = 0;              // NMEA �����ֶκ� ��0��ʼ
static u8   NMEA_MsgBlockDatIndex = 0;      // NMEA ����ÿ���ֶ����ַ����� ��0��ʼ

static u8   GPS_Parse_Status = 0;           // ��ǰ����״̬.
static u8   SateInfoIndex = 0;              //

static struct_GPSRMC GPS_RMC_Data;
static struct_GPSGGA GPS_GGA_Data;
static struct_GPSGSA GPS_GSA_Data;
static struct_GPSGSV GPS_GSV_Data;
static nmeaGPVTG     GPS_VTG_Data;
static struct_GPGLL  GPS_GLL_Data;

static struct_parser_callback Parser_CallBack;

static void ParserGPGGA(char SBuf) {
    switch (SBuf) {
        case '*':   //������
            NMEA_Start=0;
			if(Parser_CallBack.gpggaCallback != NULL)
		        Parser_CallBack.gpggaCallback(GPS_GGA_Data);
            break;
        case ',':   //���ֶν���
            NMEA_MsgBlock++;
            NMEA_MsgBlockDatIndex=0;
            break;
        default:    //�ֶ��ַ�
            switch (NMEA_MsgBlock) { // �жϵ�ǰ�����ĸ��ֶ�
			            /*
			            case 0:             // <1> UTCʱ��,hhmmss
			                break;
			            case GPGGA_LATITUDE:             // <2> γ�� ddmm.mmmm
			                break;
			            case 2:             // <3> γ�Ȱ��� N/S
			                break;
			            case GPGGA_LONGTITUDE:             // <4> ���� dddmm.mmmm
			                break;
			            case 4:             // <5> ���Ȱ��� E/W
			                break;
			            */
                    case 5:      // <6> GPS״̬ 0=δ��λ, 1=�ǲ�ֶ�λ, 2=��ֶ�λ, 6=���ڹ���
                           GPS_GGA_Data.PositionFix=SBuf;
                           break;
                    case 6:      // <7> ����ʹ�õ��������� 00~12
                          switch (NMEA_MsgBlockDatIndex)
                          {
                              case 0:
                                     GPS_GGA_Data.SatUsed=(SBuf-'0')*10;
                                     break;
                               case 1:
                                     GPS_GGA_Data.SatUsed+=(SBuf-'0');
                                     break;
                          }
                          break;
                     /*
                                    case 7:             //<8> HDOPˮƽ�������� 0.5~99.9
                                            GPS_GGA_Data.HDOP[GPS_GGA_Data.BlockIndex]=SBuf;
                                            break;
                                   */
                     case 8:         //<9> ���θ߶� -9999.9~99999.9
                           GPS_GGA_Data.Altitude[NMEA_MsgBlockDatIndex]=SBuf;
                           break;
                  }
        NMEA_MsgBlockDatIndex++;     //�ֶ��ַ�����++, ָ����һ���ַ�
    }
}
//$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh
static int ParserGPRMC(char SBuf) {
    switch (SBuf) {
        case '*':
            NMEA_Start=0;
            //GPS_Parse_Status=GPS_PARSE_OK;       //�������, ���Դ���
            if(Parser_CallBack.gprmcCallback != NULL)
                Parser_CallBack.gprmcCallback(GPS_RMC_Data);
            break;
        case ',':
            NMEA_MsgBlock++;
            NMEA_MsgBlockDatIndex=0;
            break;
        default:
            switch (NMEA_MsgBlock){
                case 0:         // <1> UTCʱ�� hhmmss.mmm
                    switch (NMEA_MsgBlockDatIndex) {
                        case 0: // hh
                            GPS_RMC_Data.UTCDateTime[3]=(SBuf-'0')*10;
                            break;
                        case 1:
                            GPS_RMC_Data.UTCDateTime[3]+=(SBuf-'0');
                            GPS_RMC_Data.UTCDateTime[3]+=8;  // Դʱ���� UTC, ת���ɱ���ʱ�� +8, ����Ҫ�ж��Ƿ񳬹�23Сʱ
                            break;
                        case 2: // mm
                            GPS_RMC_Data.UTCDateTime[4]=(SBuf-'0')*10;
                            break;
                        case 3:
                            GPS_RMC_Data.UTCDateTime[4]+=(SBuf-'0');
                            break;
                        case 4: // ss
                            GPS_RMC_Data.UTCDateTime[5]=(SBuf-'0')*10;
                            break;
                        case 5:
                            GPS_RMC_Data.UTCDateTime[5]+=(SBuf-'0');
                            break;
                    }
                    break;
                case 1:         // <2> ��λ״̬ A=��Ч��λ, V=��Ч��λ
                    GPS_RMC_Data.Status=SBuf;
                    break;
                case GPRMC_LATITUDE:         // <3> γ�� ddmm.mmmm
                    //GPS_RMC_Data.Latitude[NMEA_MsgBlockDatIndex]=SBuf;    //DEBUG
                    switch (NMEA_MsgBlockDatIndex) {           // ǰ��0Ҳ�����, �ֱ�ת������ֵ��
                        case 0:     // dd
                            GPS_RMC_Data.LatitudeD=(SBuf-'0')*10;
                            break;
                        case 1:
                            GPS_RMC_Data.LatitudeD+=(SBuf-'0');
                            break;
                        case 2:     // mm
                            GPS_RMC_Data.LatitudeM=(SBuf-'0')*10;
                            break;
                        case 3:
                            GPS_RMC_Data.LatitudeM+=(SBuf-'0');
                            break;
                        case 4:     // С����
                            break;
                        case 5:     // mmmm
                            GPS_RMC_Data.LatitudeS=(SBuf-'0')*1000;
                            break;
                        case 6:
                            GPS_RMC_Data.LatitudeS+=(SBuf-'0')*100;
                            break;
                        case 7:
                            GPS_RMC_Data.LatitudeS+=(SBuf-'0')*10;
                            break;
                        case 8:
                            GPS_RMC_Data.LatitudeS+=(SBuf-'0');
                            break;
                     }
                     break;
                case 3:         //<4> γ�Ȱ��� N/S
                    GPS_RMC_Data.NS=SBuf;
                    break;
                case GPRMC_LONGTITUDE:         //<5> ���� dddmm.mmmm
                    //GPS_RMC_Data.Longitude[NMEA_MsgBlockDatIndex]=SBuf;   //DEBUG
                    switch (NMEA_MsgBlockDatIndex) {           // ǰ��0Ҳ�����, �ֱ�ת������ֵ��
                        case 0:     // ddd
                            GPS_RMC_Data.LongitudeD=(SBuf-'0')*100;
                            break;
                        case 1:
                            GPS_RMC_Data.LongitudeD+=(SBuf-'0')*10;
                            break;
                        case 2:
                            GPS_RMC_Data.LongitudeD+=(SBuf-'0');
                            break;
                        case 3:     // mm
                            GPS_RMC_Data.LongitudeM=(SBuf-'0')*10;
                            break;
                        case 4:
                            GPS_RMC_Data.LongitudeM+=(SBuf-'0');
                            break;
                        case 5:     // С����
                            break;
                        case 6:     // mmmm
                            GPS_RMC_Data.LongitudeS=(SBuf-'0')*1000;
                            break;
                        case 7:
                            GPS_RMC_Data.LongitudeS+=(SBuf-'0')*100;
                            break;
                        case 8:
                            GPS_RMC_Data.LongitudeS+=(SBuf-'0')*10;
                            break;
                        case 9:
                            GPS_RMC_Data.LongitudeS+=(SBuf-'0');
                            break;
                    }
                    break;
                case 5:         // <6> ���Ȱ��� E/W
                    GPS_RMC_Data.EW=SBuf;
                    break;
                case 6:         // <7> �������� 000.0~999.9 ��
                    //GPS_RMC_Data.sSpeed[NMEA_MsgBlockDatIndex]=SBuf;  //DEBUG
                    switch (NMEA_MsgBlockDatIndex) {    // ǰ��0Ҳ�����, ת������ֵ��, ��ֵx10
                    case 0:
                        GPS_RMC_Data.Speed=(SBuf-'0')*1000;
                        break;
                    case 1:
                        GPS_RMC_Data.Speed+=(SBuf-'0')*100;
                        break;
                    case 2:
                        GPS_RMC_Data.Speed+=(SBuf-'0')*10;
                        break;
                    case 3:
                        break;
                    case 4:
                        GPS_RMC_Data.Speed+=(SBuf-'0');
                        break;
                    }
                    break;
                case 7:         // <8> ���溽�� 000.0~359.9 ��, ���汱Ϊ�ο���׼
                    //GPS_RMC_Data.sCourse[NMEA_MsgBlockDatIndex]=SBuf; //DEBUG
                    switch (NMEA_MsgBlockDatIndex){    // ǰ��0Ҳ�����, ת������ֵ��, ��ֵx10
                    case 0:
                        GPS_RMC_Data.Course=(SBuf-'0')*1000;
                        break;
                    case 1:
                        GPS_RMC_Data.Course+=(SBuf-'0')*100;
                        break;
                    case 2:
                        GPS_RMC_Data.Course+=(SBuf-'0')*10;
                        break;
                    case 3: // С����, ����
                        break;
                    case 4:
                        GPS_RMC_Data.Course+=(SBuf-'0');
                        break;
                    }
                    break;
        case 8:         // <9> UTC���� ddmmyy
            switch (NMEA_MsgBlockDatIndex)
            {
            case 0: // dd
                GPS_RMC_Data.UTCDateTime[2]=(SBuf-'0')*10;
                break;
            case 1:
                GPS_RMC_Data.UTCDateTime[2]+=(SBuf-'0');
                if (GPS_RMC_Data.UTCDateTime[3]>23)     // ���Сʱ����23, ���������Ҫ+1
                {
                    GPS_RMC_Data.UTCDateTime[3]-=24;    // Hour
                    GPS_RMC_Data.UTCDateTime[2]++;      // Day
                }
                break;
            case 2: // mm
                GPS_RMC_Data.UTCDateTime[1]=(SBuf-'0')*10;
                break;
            case 3:
                GPS_RMC_Data.UTCDateTime[1]+=(SBuf-'0');
                switch (GPS_RMC_Data.UTCDateTime[1])    // ���ݴ�С�����ж������Ƿ����, ������·�++
                {
                case 2:                             // �˴�δ��������29������
                    if (GPS_RMC_Data.UTCDateTime[2]>28)
                    {
                        GPS_RMC_Data.UTCDateTime[2]-=28;
                        GPS_RMC_Data.UTCDateTime[1]++;
                    }
                    break;
                case 1:                             // ���� 31 ��
                case 3:
                case 5:
                case 7:
                case 8:
                case 10:
                case 12:
                    if (GPS_RMC_Data.UTCDateTime[2]>31)
                    {
                        GPS_RMC_Data.UTCDateTime[2]-=31;
                        GPS_RMC_Data.UTCDateTime[1]++;
                    }
                    break;
                case 4:                             // С�� 30 ��
                case 6:
                case 9:
                case 11:
                    if (GPS_RMC_Data.UTCDateTime[2]>30)
                    {
                        GPS_RMC_Data.UTCDateTime[2]-=30;
                        GPS_RMC_Data.UTCDateTime[1]++;
                    }
                    break;
                }
                break;
            case 4:
                GPS_RMC_Data.UTCDateTime[0]=(SBuf-'0')*10;
                break;
            case 5:
                GPS_RMC_Data.UTCDateTime[0]+=(SBuf-'0');
                if (GPS_RMC_Data.UTCDateTime[1]>12)     // ����·ݳ���, ���� ++
                {
                    GPS_RMC_Data.UTCDateTime[1]=1;
                    GPS_RMC_Data.UTCDateTime[0]++;
                }
                break;
            }
            break;
        }
        NMEA_MsgBlockDatIndex++;  // ���ֶ��ַ���� +1
    }
}

static void ParserGPGSA(char SBuf)
{
    switch (SBuf)
    {
    case '*':               // ������ݽ���, ����2λУ��ֵ
        NMEA_Start=0;
		if(Parser_CallBack.gpgsaCallback != NULL)
		Parser_CallBack.gpgsaCallback(GPS_GSA_Data);
        break;
    case ',':               // ,�ָ���, �ֶ� +1, �ֶ��ڲ��ַ���Ź���
        NMEA_MsgBlock++;
        NMEA_MsgBlockDatIndex=0;
        //�˴�����ȷ�� GPS_GSA_Data.SatUsedList[] ��ʼ��.
        break;
    default:
        switch (NMEA_MsgBlock)
        {
        case 0:         // <1>ģʽ M=�ֶ�, A=�Զ�
            GPS_GSA_Data.Mode=SBuf;
            break;
        case 1:         // <2>��λ��ʽ 1=δ��λ, 2=��ά��λ, 3=��ά��λ
            GPS_GSA_Data.Mode2=SBuf;
            break;
        case 2:         // <3> PRN 01~32 ʹ���е����Ǳ��
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            switch (NMEA_MsgBlockDatIndex)
            {           // ǰ��0Ҳ�����, ת������ֵ��
            case 0:
                GPS_GSA_Data.SatUsedList[NMEA_MsgBlock-2]=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSA_Data.SatUsedList[NMEA_MsgBlock-2]+=(SBuf-'0');
                break;
            }
            break;
        case 14:        // <4> PDOP λ�þ������� 0.5~99.9
            GPS_GSA_Data.PDOP[NMEA_MsgBlockDatIndex]=SBuf;
            break;
        case 15:        // <5> HDOP ˮƽ�������� 0.5~99.9
            GPS_GSA_Data.HDOP[NMEA_MsgBlockDatIndex]=SBuf;
            break;
        case 16:        // <6> VDOP ��ֱ�������� 0.5~99.9
            GPS_GSA_Data.VDOP[NMEA_MsgBlockDatIndex]=SBuf;
            break;
        }
        NMEA_MsgBlockDatIndex++;  // ���ֶ��ַ���� +1
    }
}

static void ParserGPGSV(char SBuf)
{
    switch (SBuf)
    {
    case '*':               // ������ݽ���, ����2λУ��ֵ
        NMEA_Start=0;
		SateInfoIndex=0;
		if(Parser_CallBack.gpgsvCallback != NULL)
		Parser_CallBack.gpgsvCallback(GPS_GSV_Data);
        break;
    case ',':               // ,�ָ���, �ֶ� +1, �ֶ��ڲ��ַ���Ź���
        NMEA_MsgBlock++;
        NMEA_MsgBlockDatIndex=0;
        break;
    default:
        switch (NMEA_MsgBlock)
        {
            /*
            case 0:         // <1> GSV��������
                break;
                */
        case 1:         // <2> ����GSV�ı��
            if (SBuf=='1') SateInfoIndex=0;
            //��������һ�� GSV ��� ���ж�������Ϣ���¿�ʼ
            break;
        case 2:         // <3> �ɼ����ǵ����� 00~12
            switch (NMEA_MsgBlockDatIndex)
            {           // ǰ��0Ҳ�����, ת������ֵ��
            case 0:
                GPS_GSV_Data.SatInView=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSV_Data.SatInView+=(SBuf-'0');
                break;
            }
            break;
        case 3:         // <4> ���Ǳ�� 01~32
        case 7:
        case 11:
        case 15:
            switch (NMEA_MsgBlockDatIndex)
            {           // ǰ��0Ҳ�����, ת������ֵ��
            case 0:
                GPS_GSV_Data.SatInfo[SateInfoIndex].SatID=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSV_Data.SatInfo[SateInfoIndex].SatID+=(SBuf-'0');
                SateInfoIndex++;
                break;
            }
            break;
        case 4:         // <5>�������� 00~90 ��
        case 8:
        case 12:
        case 16:
            switch (NMEA_MsgBlockDatIndex)
            {           // ǰ��0Ҳ�����, ת������ֵ��
            case 0:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].Elevation=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].Elevation+=(SBuf-'0');
                break;
            }
            break;
        case 5:         // <6>���Ƿ�λ�� 000~359 ��
        case 9:
        case 13:
        case 17:
            switch (NMEA_MsgBlockDatIndex)
            {           // ǰ��0Ҳ�����, ת������ֵ��
            case 0:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].Azimuth=(SBuf-'0')*100;
                break;
            case 1:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].Azimuth+=(SBuf-'0')*10;
                break;
            case 2:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].Azimuth+=(SBuf-'0');
                break;
            }
            break;
        case 6:         // <7>Ѷ�������� C/No 00~99
        case 10:
        case 14:
        case 18:
            switch (NMEA_MsgBlockDatIndex)
            {           // ǰ��0Ҳ�����, ת������ֵ��
            case 0:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].SNR=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].SNR+=(SBuf-'0');
                break;
            }
            break;
        }
        NMEA_MsgBlockDatIndex++;  // ���ֶ��ַ���� +1
    }
}
//char dec[] ="326.22"; //326.22
static void ParserGPVTG(char SBuf) {
    switch (SBuf)
    {
        case '*':   //������
            NMEA_Start=0;
			//GPS_VTG_Data.dec = strtof(dec,NULL);
			if(Parser_CallBack.gpvtgCallback != NULL)
		    Parser_CallBack.gpvtgCallback(GPS_VTG_Data);
            break;
        case ',':   //���ֶν���
            NMEA_MsgBlock++;
            NMEA_MsgBlockDatIndex=0;
            break;
        default:    //�ֶ��ַ�
            switch (NMEA_MsgBlock) { // �жϵ�ǰ�����ĸ��ֶ�
                case 0:              // <1> �˶��Ƕȣ�000 - 359����ǰ��λ��������0��
                     switch (NMEA_MsgBlockDatIndex) {
				        case 0:
					          break;
				        case 1:
							  break;
			        }
				    break;
            }
		    NMEA_MsgBlockDatIndex++;	 //�ֶ��ַ�����++, ָ����һ���ַ�
            break;
    }
}
u8 waitCrcFlag = 0;
u8 crcCnt = 0;
//Geographic Position��GLL��������λ��Ϣ
static void ParserGPGLL(char SBuf) {
    if(waitCrcFlag == 1) {
        crcCnt++;
        if(crcCnt == 2) {
            crcCnt = 0;
            waitCrcFlag = 0;
            NMEA_Start=0;
            if(Parser_CallBack.gpgllCallback != NULL)
                Parser_CallBack.gpgllCallback(GPS_GLL_Data);
        }
        return;
    }
    switch (SBuf) {
        case '*':   //������
            waitCrcFlag = 1;
            break;
        case ',':   //���ֶν���
            NMEA_MsgBlock++;
            NMEA_MsgBlockDatIndex=0;
            break;
        default:    //�ֶ��ַ�
            switch (NMEA_MsgBlock) { // �жϵ�ǰ�����ĸ��ֶ�
                case GPGLL_LATITUDE:  //γ��ddmm.mmmm���ȷָ�ʽ��ǰ��λ��������0��
				    break;
                case GPGLL_LONGTITUDE:
                    break;
            }
		    NMEA_MsgBlockDatIndex++;	 //�ֶ��ַ�����++, ָ����һ���ַ�
            break;
    }
}

u8 NMEA_Parser(char SBuf) {
    u8 i;

    GPS_Parse_Status=0;
    if (NMEA_Start) {  // ��������$��ʼ�� NMEA ���, ����NMEA ��������:
        if (NMEA_TypeParsed) { // NMEA ������ͽ������, �������͵��ý�������
            switch (NMEA_MsgType) {
                case NMEA_GPGGA:
                    ParserGPGGA(SBuf);
                    if(NMEA_MsgBlock == GPGGA_LATITUDE || NMEA_MsgBlock == GPGGA_LONGTITUDE) {
                        GPS_Parse_Status = 1;
                    }
                    break;
                case NMEA_GPGSA:
                    ParserGPGSA(SBuf);
                    break;
                case NMEA_GPGSV:
                    ParserGPGSV(SBuf);
                    break;
                case NMEA_GPRMC:
                    ParserGPRMC(SBuf);
                    if(NMEA_MsgBlock == GPRMC_LATITUDE || NMEA_MsgBlock == GPRMC_LONGTITUDE) {
                        GPS_Parse_Status = 1;
                    }
                    break;
                case NMEA_GPVTG:
				    ParserGPVTG(SBuf);
				    break;
                case NMEA_GPGLL:
				    ParserGPGLL(SBuf);
                    if(NMEA_MsgBlock == GPGLL_LATITUDE || NMEA_MsgBlock == GPGLL_LONGTITUDE) {
                        GPS_Parse_Status = 1;
                    }
                    break;
                default:    //�޷�ʶ��ĸ�ʽ, ��λ
                    NMEA_Start=0;
                    NMEA_TypeParsed=0;
                    NMEA_MsgType=NMEA_NULL;
                    NMEA_MsgTypeIndex=1;
                    break;
            }
        } else {  // NMEA �������δ����, �������͵��ý�������
            switch (SBuf) {
                case ',': // NMEA ��������ֶν���,��ʼ�ж�
                    // GPS ���˳�� - 0
                    if(NMEA_MsgTypeBuff[3]=='G'&&NMEA_MsgTypeBuff[4]=='A') {
                        //��ʼ����������
                        //��ʼ����λ��Ϣ����
                        //GPS_GGA_Data.PositionFix=0x00;
                        GPS_GGA_Data.SatUsed=0x00;
                        NMEA_MsgType=NMEA_GPGGA;
                    }
                    // GPS ���˳�� - 1
                    if(NMEA_MsgTypeBuff[3]=='S'&&NMEA_MsgTypeBuff[4]=='A') {
                        //��ʼ��ʹ���е������б�
                        for (i=0;i<12;i++) {
                            GPS_GSA_Data.SatUsedList[i]=0x00;
                        }
                        //��ʼ����������
                        for (i=0;i<5;i++){
                            GPS_GSA_Data.HDOP[i]=0x00;
                            GPS_GSA_Data.VDOP[i]=0x00;
                            GPS_GSA_Data.PDOP[i]=0x00;
                        }
                        //GPS_GSA_Data.Mode=0x00;
                        //GPS_GSA_Data.Mode2=0x00;
                        //����GSV���������, ���г�ʼ�����ܷ��� GSV��,
                        //����ᵼ��ǰ��������������ݱ���ʼ����
                        //��ʼ�������źŷ�λ������
                        for (i=0;i<12;i++){
                            GPS_GSV_Data.SatInfo[i].SatID=0x00;
                            GPS_GSV_Data.SatInfo[i].SNR=0x00;
                            GPS_GSV_Data.SatInfo[i].Elevation=0x00;
                            GPS_GSV_Data.SatInfo[i].Azimuth=0x0000;
                        }
                        //GPS_GSV_Data.SatInView=0x00;
                        NMEA_MsgType=NMEA_GPGSA;
                    }
                    // GPS ���˳�� - 2,3,4
                    if (NMEA_MsgTypeBuff[4]=='V'){
                        NMEA_MsgType=NMEA_GPGSV;
                    }
                    if(NMEA_MsgTypeBuff[3] == 'T' && NMEA_MsgTypeBuff[4] == 'G') {
                        NMEA_MsgType = NMEA_GPVTG;
				    }
                    if(NMEA_MsgTypeBuff[3] == 'L' && NMEA_MsgTypeBuff[4] == 'L') {
                        NMEA_MsgType = NMEA_GPGLL;
				    }
                    // GPS ���˳�� - 5
                    if (NMEA_MsgTypeBuff[4]=='C'){
                        //GPS_RMC_Data.Status='-';
                        //��ʼ����γ�����ݺ��ٶ�,����
                        GPS_RMC_Data.LatitudeD=0x00;
                        GPS_RMC_Data.LatitudeM=0x00;
                        GPS_RMC_Data.LatitudeS=0x0000;
                        GPS_RMC_Data.NS='-';
                        
                        GPS_RMC_Data.LongitudeD=0x00;
                        GPS_RMC_Data.LongitudeM=0x00;
                        GPS_RMC_Data.LongitudeS=0x0000;
                        GPS_RMC_Data.EW='-';
                        
                        GPS_RMC_Data.Speed=0x0000;
                        GPS_RMC_Data.Course=0x0000;
                        
                        NMEA_MsgType=NMEA_GPRMC;
                    }
                    //�˴������������, ����䲻��ʶ��, ��NMEA_MsgTypeΪNULL������,
                    //��תΪ�������ͽ���ʱ����ת���޷�ʶ��ĸ�ʽ, ����λ
                    NMEA_TypeParsed=1;
                    NMEA_MsgTypeIndex=1;
                    NMEA_MsgBlock=0;
                    NMEA_MsgBlockDatIndex=0;
                    break;
                case '*':
                    NMEA_Start=0;
                    //GPSģ���ϵ�ʱ���
                    //$PSRF Model Name : J3S31_DGCB1_496 *45
                    //$PSRF *321.3*30
                    //$PSRF*17
                    //$PSRF*17
                    //$PSRF Product by J communications Co., Ltd *4C
                    //$PSRF Revision by Young Wook *69
                    //$PSRF www.jcomco.com *06
                    //�����ж�ʧЧ,
                    break;
                default:  //���ڵ�һ���ֶ���, ��������
                    NMEA_MsgTypeBuff[NMEA_MsgTypeIndex]=SBuf;
                    NMEA_MsgTypeIndex++;
                    if (NMEA_MsgTypeIndex>5) NMEA_Start=0;
                    // NMEA ���ͳ��� 5 ���ַ�, (����Խ��, ��������)
                    // ���жϲ��������� NMEA ���, ���Թ��˾�.
                    break;
            }
        }
    } else { //δ������$, ѭ�����ղ��ж� ֱ�� $
        if (SBuf=='$') {           //���յ�$, ��һ���ַ���Ϊ�����ж��ַ�, �Ƚ�����ر�����ʼ��
            NMEA_Start = 1;         //�´ε��������NMEA ��������:
            NMEA_MsgTypeIndex = 0;  //��ͷ���GPS�����ַ�������
            NMEA_TypeParsed = 0;
            NMEA_MsgType = NMEA_NULL;
            NMEA_MsgBlock = 0;
            NMEA_MsgBlockDatIndex = 0;
        }
    }

    return GPS_Parse_Status;
}
void initParserCallBack(GPRMC_CALLBACK gprmcCallback,GPGGA_CALLBACK gpggaCallback,
	                       GPGSA_CALLBACK gpgsaCallback,GPGSV_CALLBACK gpgsvCallback,
	                       GPVTG_CALLBACK gpVTGCallback,GPGLL_CALLBACK gpgllCallback){
	Parser_CallBack.gprmcCallback = gprmcCallback;
	Parser_CallBack.gpggaCallback = gpggaCallback;
	Parser_CallBack.gpgsaCallback = gpgsaCallback;
	Parser_CallBack.gpgsvCallback = gpgsvCallback;
	Parser_CallBack.gpvtgCallback = gpVTGCallback;
	Parser_CallBack.gpgllCallback = gpgllCallback;
}

/*   Header             ID          Length      Payload    Checksum
 *  0xB5 0x62    0x01 0x02       28                         CK_A CK_B
*/
int ParserUbxNavPOSLLH(const char *buff, int buff_sz) {
   int hAcc = 0;  //unit mm
   /*
     u8 buf[36] = {
         0xB5,0x62,0x01,0x02,0x1C,0x00,0xB8,0x14,0x60,0x0F,0x35,0x44,0xE7,0x43,0x76,0x5E,
         0x80,0x0D,0xF1,0xE8,0x00,0x00,0x3C,0xF4,0x00,0x00,0x91,0x73,0x00,0x00,0xF1,0xE6,
         0x03,0x00,0x45,0xB2};
     */
   hAcc = buff[26]|buff[27]<<8|buff[28]<<16|buff[29]<<24;
   
   //printf("hAcc = %.1f\n",(double)hAcc/1000);
   return hAcc;
}
#define HEADER_B5   0XB5
#define HEADER_62   0X62
enum UBX_TYPE
{
	UBX_TYPE_UNKOWN   = 0x00,
	UBX_TYPE_CLASS    = 0x01,
	UBX_TYPE_ID       = 0x02,
	UBX_TYPE_LENGTH1  = 0x03,
	UBX_TYPE_LENGTH2  = 0x04,
    CAR_MODE_PAYLOAD  = 0x05,
    CAR_MODE_CK_A     = 0x06,
    CAR_MODE_CK_B     = 0x07,
};

static u8 ubx_Start = 0;  
static u8 ubxType = UBX_TYPE_UNKOWN;
static u8 ubxClass = 0;
static u8 ubxID = 0;
static u8 ubxLength = 0;
static u8 ubxPayLoadIndex = 0;
int mhAcc = 0;

u8 ubx_Parser(char SBuf) {    
    
    switch(SBuf) {
        case HEADER_B5:
            ubxType = HEADER_62;
            break;
        case HEADER_62:
            if(ubxType == HEADER_62) {
                ubxType = UBX_TYPE_CLASS;
            } else {
                ubxType = UBX_TYPE_UNKOWN;
            }
            break;
        default:
            switch(ubxType) {
                case UBX_TYPE_CLASS:
                    ubxType = UBX_TYPE_ID;
                    ubxClass = SBuf;
                    break;
                case UBX_TYPE_ID:
                    ubxType = UBX_TYPE_LENGTH1;
                    ubxID = SBuf;
                    break;
                case UBX_TYPE_LENGTH1:
                    ubxType = UBX_TYPE_LENGTH2;
                    ubxLength = SBuf;
                    break;
                case UBX_TYPE_LENGTH2:
                    ubxType = CAR_MODE_PAYLOAD;
                    ubxLength += SBuf<<8;
                    ubxPayLoadIndex = 0;
                    //printf("leng=%d\n",ubxLength);
                    break;
                case CAR_MODE_PAYLOAD:
                    //printf("Index=%d\n",ubxPayLoadIndex);
                    if(ubxClass == 0x01 && ubxID == 0x02) {
                        if(ubxPayLoadIndex == 20) {
                            mhAcc = SBuf;   //2D Acc
                        } else if(ubxPayLoadIndex == 21) {
                            mhAcc |= SBuf<<8;
                        } else if(ubxPayLoadIndex == 22) {
                            mhAcc |= SBuf<<16;
                        } else if(ubxPayLoadIndex == 23) {
                            mhAcc |= SBuf<<24;
                        }
                    }
                    ubxPayLoadIndex++;
                    if(ubxPayLoadIndex == ubxLength) {
                        ubxType = CAR_MODE_CK_A;
                    }
                    break;
                case CAR_MODE_CK_A:
                    ubxType = CAR_MODE_CK_B;
                    break;
                case CAR_MODE_CK_B:
                    ubxType = UBX_TYPE_UNKOWN;
                    ubxPayLoadIndex = 0;
                    //printf("hAcc = %.1f\n",(double)mhAcc/1000);
                    return 1;
                    break;
                default:
                    break;
            }
            break;
    }
    return 0;
}
