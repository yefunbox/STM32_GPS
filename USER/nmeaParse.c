#include <stdio.h>
#include <stdlib.h>
#include "nmeaParse.h"

static u8   NMEA_Start = 0;                 // NMEA 语句开始. 检测到 $ 时置 1
static u8   NMEA_TypeParsed = 0;            // NMEA 类型解析完毕
static u8   NMEA_MsgType = NMEA_NULL;       // NMEA 语句类型
static char NMEA_MsgTypeBuff[] = "GPxxx,";  // NMEA 语句类型识别缓存
static u8   NMEA_MsgTypeIndex = 0;          // 读取 NMEA 类型识别字符的个数
static u8   NMEA_MsgBlock = 0;              // NMEA 数据字段号 从0开始
static u8   NMEA_MsgBlockDatIndex = 0;      // NMEA 数据每个字段内字符索引 从0开始

static u8   GPS_Parse_Status = 0;           // 当前解析状态.
static u8   SateInfoIndex = 0;              //
static u8   CRC_CNT = 0;

static struct_GPSRMC GPS_RMC_Data;
static struct_GPSGGA GPS_GGA_Data;
static struct_GPSGSA GPS_GSA_Data;
static struct_GPSGSV GPS_GSV_Data;
static nmeaGPVTG     GPS_VTG_Data;
static struct_GPGLL  GPS_GLL_Data;
NMEA_BASE_MSG nmea_base_msg;

static struct_parser_callback Parser_CallBack;

static void ParserGPGGA(char SBuf) {
    switch (SBuf) {
        case '*':   //语句结束
            //NMEA_Start=0;
			if(Parser_CallBack.gpggaCallback != NULL)
		        Parser_CallBack.gpggaCallback(GPS_GGA_Data);
            break;
        case ',':   //该字段结束
            NMEA_MsgBlock++;
            NMEA_MsgBlockDatIndex=0;
            break;
        default:    //字段字符
            switch (NMEA_MsgBlock) { // 判断当前处于哪个字段
			            /*
			            case 0:             // <1> UTC时间,hhmmss
			                break;
			            case GPGGA_LATITUDE:             // <2> 纬度 ddmm.mmmm
			                break;
			            case 2:             // <3> 纬度半球 N/S
			                break;
			            case GPGGA_LONGTITUDE:             // <4> 经度 dddmm.mmmm
			                break;
			            case 4:             // <5> 经度半球 E/W
			                break;
			            */
                    case 5:      // <6> GPS状态 0=未定位, 1=非差分定位, 2=差分定位, 6=正在估算
                           GPS_GGA_Data.PositionFix=SBuf;
                           break;
                    case 6:      // <7> 正在使用的卫星数量 00~12
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
                     case 7:         //<8> HDOP水平精度因子 0.5~99.9
                          GPS_GGA_Data.HDOP[NMEA_MsgBlockDatIndex]=SBuf;
                          break;
                     case 8:         //<9> 海拔高度 -9999.9~99999.9
                           GPS_GGA_Data.Altitude[NMEA_MsgBlockDatIndex]=SBuf;
                           break;
                  }
                  NMEA_MsgBlockDatIndex++;     //字段字符索引++, 指向下一个字符
                  break;
    }
}
//$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh
static int ParserGPRMC(char SBuf) {
    switch (SBuf) {
        case '*':
            //NMEA_Start=0;
            if(Parser_CallBack.gprmcCallback != NULL)
                Parser_CallBack.gprmcCallback(GPS_RMC_Data);
            break;
        case ',':
            NMEA_MsgBlock++;
            NMEA_MsgBlockDatIndex=0;
            break;
        default:
            switch (NMEA_MsgBlock){
                case 0:         // <1> UTC时间 hhmmss.mmm
                    switch (NMEA_MsgBlockDatIndex) {
                        case 0: // hh
                            GPS_RMC_Data.UTCDateTime[3]=(SBuf-'0')*10;
                            break;
                        case 1:
                            GPS_RMC_Data.UTCDateTime[3]+=(SBuf-'0');
                            GPS_RMC_Data.UTCDateTime[3]+=8;  // 源时区是 UTC, 转换成北京时区 +8, 下面要判断是否超过23小时
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
                case 1:         // <2> 定位状态 A=有效定位, V=无效定位
                    GPS_RMC_Data.Status=SBuf;
                    break;
                case GPRMC_LATITUDE:         // <3> 纬度 ddmm.mmmm
                    //GPS_RMC_Data.Latitude[NMEA_MsgBlockDatIndex]=SBuf;    //DEBUG
                    //nmea_base_msg.latitude[NMEA_MsgBlockDatIndex] = SBuf;
                    switch (NMEA_MsgBlockDatIndex) {           // 前导0也会输出, 分别转换成数值型
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
                        case 4:     // 小数点
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
                case 3:         //<4> 纬度半球 N/S
                    GPS_RMC_Data.NS=SBuf;
                    break;
                case GPRMC_LONGTITUDE:         //<5> 经度 dddmm.mmmm
                    //GPS_RMC_Data.Longitude[NMEA_MsgBlockDatIndex]=SBuf;   //DEBUG
                    //nmea_base_msg.longitude[NMEA_MsgBlockDatIndex] = SBuf;
                    switch (NMEA_MsgBlockDatIndex) {           // 前导0也会输出, 分别转换成数值型
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
                        case 5:     // 小数点
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
                case 5:         // <6> 经度半球 E/W
                    GPS_RMC_Data.EW=SBuf;
                    break;
                case 6:         // <7> 地面速率 000.0~999.9 节
                    //GPS_RMC_Data.sSpeed[NMEA_MsgBlockDatIndex]=SBuf;  //DEBUG
                    switch (NMEA_MsgBlockDatIndex) {    // 前导0也会输出, 转换成数值型, 数值x10
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
                case 7:         // <8> 地面航向 000.0~359.9 度, 以真北为参考基准
                    //GPS_RMC_Data.sCourse[NMEA_MsgBlockDatIndex]=SBuf; //DEBUG
                    switch (NMEA_MsgBlockDatIndex){    // 前导0也会输出, 转换成数值型, 数值x10
                    case 0:
                        GPS_RMC_Data.Course=(SBuf-'0')*1000;
                        break;
                    case 1:
                        GPS_RMC_Data.Course+=(SBuf-'0')*100;
                        break;
                    case 2:
                        GPS_RMC_Data.Course+=(SBuf-'0')*10;
                        break;
                    case 3: // 小数点, 忽略
                        break;
                    case 4:
                        GPS_RMC_Data.Course+=(SBuf-'0');
                        break;
                    }
                    break;
        case 8:         // <9> UTC日期 ddmmyy
            switch (NMEA_MsgBlockDatIndex)
            {
            case 0: // dd
                GPS_RMC_Data.UTCDateTime[2]=(SBuf-'0')*10;
                break;
            case 1:
                GPS_RMC_Data.UTCDateTime[2]+=(SBuf-'0');
                if (GPS_RMC_Data.UTCDateTime[3]>23)     // 如果小时超过23, 则表明日期要+1
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
                switch (GPS_RMC_Data.UTCDateTime[1])    // 根据大小月来判断日期是否溢出, 溢出则月份++
                {
                case 2:                             // 此处未考虑闰年29天的情况
                    if (GPS_RMC_Data.UTCDateTime[2]>28)
                    {
                        GPS_RMC_Data.UTCDateTime[2]-=28;
                        GPS_RMC_Data.UTCDateTime[1]++;
                    }
                    break;
                case 1:                             // 大月 31 天
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
                case 4:                             // 小月 30 天
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
                if (GPS_RMC_Data.UTCDateTime[1]>12)     // 如果月份超出, 则年 ++
                {
                    GPS_RMC_Data.UTCDateTime[1]=1;
                    GPS_RMC_Data.UTCDateTime[0]++;
                }
                break;
            }
            break;
        }
        NMEA_MsgBlockDatIndex++;  // 该字段字符序号 +1
    }
}

static void ParserGPGSA(char SBuf)
{
    switch (SBuf)
    {
    case '*':               // 语句数据结束, 紧跟2位校验值
        //NMEA_Start=0;
		if(Parser_CallBack.gpgsaCallback != NULL)
		Parser_CallBack.gpgsaCallback(GPS_GSA_Data);
        break;
    case ',':               // ,分隔符, 字段 +1, 字段内部字符编号归零
        NMEA_MsgBlock++;
        NMEA_MsgBlockDatIndex=0;
        //此处必须确保 GPS_GSA_Data.SatUsedList[] 初始化.
        break;
    default:
        switch (NMEA_MsgBlock)
        {
        case 0:         // <1>模式 M=手动, A=自动
            GPS_GSA_Data.Mode=SBuf;
            break;
        case 1:         // <2>定位型式 1=未定位, 2=二维定位, 3=三维定位
            GPS_GSA_Data.Mode2=SBuf;
            break;
        case 2:         // <3> PRN 01~32 使用中的卫星编号
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
            {           // 前导0也会输出, 转换成数值型
            case 0:
                GPS_GSA_Data.SatUsedList[NMEA_MsgBlock-2]=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSA_Data.SatUsedList[NMEA_MsgBlock-2]+=(SBuf-'0');
                break;
            }
            break;
        case 14:        // <4> PDOP 位置精度因子 0.5~99.9
            GPS_GSA_Data.PDOP[NMEA_MsgBlockDatIndex]=SBuf;
            break;
        case 15:        // <5> HDOP 水平精度因子 0.5~99.9
            GPS_GSA_Data.HDOP[NMEA_MsgBlockDatIndex]=SBuf;
            break;
        case 16:        // <6> VDOP 垂直精度因子 0.5~99.9
            GPS_GSA_Data.VDOP[NMEA_MsgBlockDatIndex]=SBuf;
            break;
        }
        NMEA_MsgBlockDatIndex++;  // 该字段字符序号 +1
    }
}

static void ParserGPGSV(char SBuf)
{
    switch (SBuf)
    {
    case '*':               // 语句数据结束, 紧跟2位校验值
        //NMEA_Start=0;
		SateInfoIndex=0;
		if(Parser_CallBack.gpgsvCallback != NULL)
		Parser_CallBack.gpgsvCallback(GPS_GSV_Data);
        break;
    case ',':               // ,分隔符, 字段 +1, 字段内部字符编号归零
        NMEA_MsgBlock++;
        NMEA_MsgBlockDatIndex=0;
        break;
    default:
        switch (NMEA_MsgBlock)
        {
            /*
            case 0:         // <1> GSV语句的总数
                break;
                */
        case 1:         // <2> 本句GSV的编号
            if (SBuf=='1') SateInfoIndex=0;
            //解析到第一句 GSV 语句 则判断卫星信息从新开始
            break;
        case 2:         // <3> 可见卫星的总数 00~12
            switch (NMEA_MsgBlockDatIndex)
            {           // 前导0也会输出, 转换成数值型
            case 0:
                GPS_GSV_Data.SatInView=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSV_Data.SatInView+=(SBuf-'0');
                break;
            }
            break;
        case 3:         // <4> 卫星编号 01~32
        case 7:
        case 11:
        case 15:
            switch (NMEA_MsgBlockDatIndex)
            {           // 前导0也会输出, 转换成数值型
            case 0:
                GPS_GSV_Data.SatInfo[SateInfoIndex].SatID=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSV_Data.SatInfo[SateInfoIndex].SatID+=(SBuf-'0');
                SateInfoIndex++;
                break;
            }
            break;
        case 4:         // <5>卫星仰角 00~90 度
        case 8:
        case 12:
        case 16:
            switch (NMEA_MsgBlockDatIndex)
            {           // 前导0也会输出, 转换成数值型
            case 0:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].Elevation=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].Elevation+=(SBuf-'0');
                break;
            }
            break;
        case 5:         // <6>卫星方位角 000~359 度
        case 9:
        case 13:
        case 17:
            switch (NMEA_MsgBlockDatIndex)
            {           // 前导0也会输出, 转换成数值型
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
        case 6:         // <7>讯号噪声比 C/No 00~99
        case 10:
        case 14:
        case 18:
            switch (NMEA_MsgBlockDatIndex)
            {           // 前导0也会输出, 转换成数值型
            case 0:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].SNR=(SBuf-'0')*10;
                break;
            case 1:
                GPS_GSV_Data.SatInfo[SateInfoIndex-1].SNR+=(SBuf-'0');
                break;
            }
            break;
        }
        NMEA_MsgBlockDatIndex++;  // 该字段字符序号 +1
    }
}
//str转换为数字
//buf:数字存储区
//返回值:转换后的数值
double Str2Num(u8 *buf)
{
    u8 *p=buf;
    double ires=0,fres=0;
    u8 ilen=0,flen=0,i;
    u8 mask=0;
    double res;
    i = strlen(buf);
    
    //printf("i=%d ",i);
    for(;i > 0;i--) {
        if(*p=='-'){mask|=0X02;p++;}//是负数
        if(*p=='.'){mask|=0X01;p++;}//遇到小数点了  
        if(mask&0X01)flen++;
        else ilen++;
        p++;
    }
    if(mask&0X02)buf++; //去掉负号
    for(i=0;i<ilen;i++) { //得到整数部分数据
        ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
    }
    if(flen>5)flen=5;   //最多取5位小数
    for(i=0;i<flen;i++) { //得到小数部分数据
        fres+=(double)(buf[ilen+1+i]-'0')/NMEA_Pow(10,1+i);
    } 
    res = ires + fres;
    if(mask&0X02)res=-res;
    //printf("s2n=%s,%d,%d,%.3f\n",buf,ilen,flen,res);
    return res;
}
//char dec[] ="326.22"; //326.22
char speedStr[10];
static void ParserGPVTG(char SBuf) {
    switch (SBuf){
        case '*':   //语句结束
            //NMEA_Start=0;
            //GPS_VTG_Data.dec = strtof(dec,NULL);
            if(Parser_CallBack.gpvtgCallback != NULL)
                Parser_CallBack.gpvtgCallback(GPS_VTG_Data);
            break;
        case ',':   //该字段结束
            NMEA_MsgBlock++;
            NMEA_MsgBlockDatIndex=0;
            break;
        default:    //字段字符
            switch (NMEA_MsgBlock) { // 判断当前处于哪个字段
                case GPVTG_DEC:      // <1> 运动角度，000 - 359，（前导位数不足则补0）
                    GPS_VTG_Data.dec = 20;
				    break;
                case GPVTG_SPK:
                    speedStr[NMEA_MsgBlockDatIndex] = SBuf;
                    break;
                case GPVTG_SPK_K:
                    GPS_VTG_Data.spk = Str2Num(speedStr);
                    //printf("spk=%.3f\n",GPS_VTG_Data.spk);
                    break;
            }
            NMEA_MsgBlockDatIndex++;	 //字段字符索引++, 指向下一个字符
            break;
    }
}

//Geographic Position（GLL）地理定位信息
static void ParserGPGLL(char SBuf) {
    switch (SBuf) {
        case '*':   //语句结束
            //NMEA_Start=0;
            if(Parser_CallBack.gpgllCallback != NULL)
                Parser_CallBack.gpgllCallback(GPS_GLL_Data);
            break;
        case ',':   //该字段结束
            NMEA_MsgBlock++;
            NMEA_MsgBlockDatIndex=0;
            break;
        default:    //字段字符
            switch (NMEA_MsgBlock) { // 判断当前处于哪个字段
                case GPGLL_LATITUDE:  //纬度ddmm.mmmm，度分格式（前导位数不足则补0）
                    break;
                case GPGLL_LONGTITUDE:
                    break;
            }
            NMEA_MsgBlockDatIndex++;	 //字段字符索引++, 指向下一个字符
            break;
    }
}

u8 NMEA_Parser(char SBuf) {
    u8 i;

    //printf("s=%d,n=%d\n",NMEA_Start,NMEA_TypeParsed);
    if (NMEA_Start) {  // 解析到以$开始的 NMEA 语句, 进入NMEA 解析流程:
        if (NMEA_TypeParsed == 1) { // NMEA 语句类型解析完毕, 根据类型调用解析函数
            switch (NMEA_MsgType) {
                case NMEA_GPGGA:
                    ParserGPGGA(SBuf);
                    GPS_Parse_Status &= 0xf0;
                    if(NMEA_MsgBlock == GPGGA_LATITUDE) {
                        GPS_Parse_Status |= GPS_PARSE_STATUS_LATITUDE;
                    } else if(NMEA_MsgBlock == GPGGA_LONGTITUDE){
                        GPS_Parse_Status |= GPS_PARSE_STATUS_LONGTITUDE;
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
                    GPS_Parse_Status &= 0xf0;
                    if(NMEA_MsgBlock == GPRMC_LATITUDE) {
                        GPS_Parse_Status |= GPS_PARSE_STATUS_LATITUDE;
                    } else if(NMEA_MsgBlock == GPRMC_LONGTITUDE) {
                        GPS_Parse_Status |= GPS_PARSE_STATUS_LONGTITUDE;
                    }
                    break;
                case NMEA_GPVTG:
                    ParserGPVTG(SBuf);
                    break;
                case NMEA_GPGLL:
                    ParserGPGLL(SBuf);
                    GPS_Parse_Status &= 0xf0;
                    if(NMEA_MsgBlock == GPGLL_LATITUDE) {
                        GPS_Parse_Status |= GPS_PARSE_STATUS_LATITUDE;
                    } else if(NMEA_MsgBlock == GPGLL_LONGTITUDE) {
                        GPS_Parse_Status |= GPS_PARSE_STATUS_LONGTITUDE;
                    }
                    break;
                default:    //无法识别的格式, 复位
                    NMEA_Start=0;
                    NMEA_TypeParsed=0;
                    NMEA_MsgType=NMEA_NULL;
                    NMEA_MsgTypeIndex=1;
                    break;
            }
            if(SBuf == '*') {
                NMEA_TypeParsed = 2;
                GPS_Parse_Status = GPS_Parse_Status & 0x0f | GPS_PARSE_STATUS_CRC_END;
            }
        } else if(NMEA_TypeParsed == 2) {   //CRC1 CRC2
            CRC_CNT++;
            if(CRC_CNT == 1) {
                GPS_Parse_Status = GPS_PARSE_STATUS_CRC_CRC1;
            } else if(CRC_CNT == 2) {
                GPS_Parse_Status = GPS_PARSE_STATUS_CRC_CRC2;
                CRC_CNT = 0;
                NMEA_TypeParsed = 3;
                NMEA_Start = 0;
            }

        } else {  // NMEA 语句类型未解析, 根据类型调用解析函数
            switch (SBuf) {
                case ',': // NMEA 语句类型字段结束,开始判断
                    // GPS 输出顺序 - 0
                    if(NMEA_MsgTypeBuff[3]=='G'&&NMEA_MsgTypeBuff[4]=='A') {
                        //初始化海拔数据
                        //初始化定位信息数据
                        //GPS_GGA_Data.PositionFix=0x00;
                        GPS_GGA_Data.SatUsed=0x00;
                        NMEA_MsgType=NMEA_GPGGA;
                    }
                    // GPS 输出顺序 - 1
                    if(NMEA_MsgTypeBuff[3]=='S'&&NMEA_MsgTypeBuff[4]=='A') {
                        //初始化使用中的卫星列表
                        for (i=0;i<12;i++) {
                            GPS_GSA_Data.SatUsedList[i]=0x00;
                        }
                        //初始化精度数据
                        for (i=0;i<5;i++){
                            GPS_GSA_Data.HDOP[i]=0x00;
                            GPS_GSA_Data.VDOP[i]=0x00;
                            GPS_GSA_Data.PDOP[i]=0x00;
                        }
                        //GPS_GSA_Data.Mode=0x00;
                        //GPS_GSA_Data.Mode2=0x00;
                        //由于GSV语句有三句, 下列初始化不能放于 GSV中,
                        //否则会导致前两句解析到的数据被初始化掉
                        //初始化卫星信号方位角数据
                        for (i=0;i<12;i++){
                            GPS_GSV_Data.SatInfo[i].SatID=0x00;
                            GPS_GSV_Data.SatInfo[i].SNR=0x00;
                            GPS_GSV_Data.SatInfo[i].Elevation=0x00;
                            GPS_GSV_Data.SatInfo[i].Azimuth=0x0000;
                        }
                        //GPS_GSV_Data.SatInView=0x00;
                        NMEA_MsgType=NMEA_GPGSA;
                    }
                    // GPS 输出顺序 - 2,3,4
                    if (NMEA_MsgTypeBuff[4]=='V'){
                        NMEA_MsgType=NMEA_GPGSV;
                    }
                    if(NMEA_MsgTypeBuff[3] == 'T' && NMEA_MsgTypeBuff[4] == 'G') {
                        NMEA_MsgType = NMEA_GPVTG;
                    }
                    if(NMEA_MsgTypeBuff[3] == 'L' && NMEA_MsgTypeBuff[4] == 'L') {
                        NMEA_MsgType = NMEA_GPGLL;
                        GPS_Parse_Status |= GPS_PARSE_STATUS_LATITUDE;
                    }
                    // GPS 输出顺序 - 5
                    if (NMEA_MsgTypeBuff[4]=='C'){
                        //GPS_RMC_Data.Status='-';
                        //初始化经纬度数据和速度,航向
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
                    //此处如果都不成立, 即语句不被识别, 则NMEA_MsgType为NULL或其他,
                    //则转为根据类型解析时会跳转到无法识别的格式, 而后复位
                    NMEA_TypeParsed=1;
                    NMEA_MsgTypeIndex=1;
                    NMEA_MsgBlock=0;
                    NMEA_MsgBlockDatIndex=0;
                    break;
                case '*':
                    NMEA_Start=0;
                    //GPS模块上电时输出
                    //$PSRF Model Name : J3S31_DGCB1_496 *45
                    //$PSRF *321.3*30
                    //$PSRF*17
                    //$PSRF*17
                    //$PSRF Product by J communications Co., Ltd *4C
                    //$PSRF Revision by Young Wook *69
                    //$PSRF www.jcomco.com *06
                    //导致判断失效,
                    break;
                default:  //处于第一个字段中, 继续接收
                    GPS_Parse_Status = GPS_PARSE_STATUS_CRC_START;
                    NMEA_MsgTypeBuff[NMEA_MsgTypeIndex]=SBuf;
                    NMEA_MsgTypeIndex++;
                    if (NMEA_MsgTypeIndex>5) NMEA_Start=0;
                    // NMEA 类型超过 5 个字符, (数组越界, 导致死机)
                    // 则判断不是正常的 NMEA 语句, 则略过此句.
                    break;
            }
        }
    } else { //未解析到$, 循环接收并判断 直到 $
        if (SBuf=='$') {           //接收到$, 下一个字符即为类型判断字符, 先进行相关变量初始化
            NMEA_Start = 1;         //下次调用则进入NMEA 解析流程:
            NMEA_MsgTypeIndex = 0;  //从头存放GPS类型字符到变量
            NMEA_TypeParsed = 0;
            NMEA_MsgType = NMEA_NULL;
            NMEA_MsgBlock = 0;
            NMEA_MsgBlockDatIndex = 0;
        }
        GPS_Parse_Status = 0;
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
            if(ubxType == UBX_TYPE_UNKOWN) {
                ubxType = HEADER_62;
                break;
            }
        case HEADER_62:
            if(ubxType == HEADER_62) {
                ubxType = UBX_TYPE_CLASS;
                break;
            }
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

