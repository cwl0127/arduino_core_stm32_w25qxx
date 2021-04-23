#include "w25qxx.h"
#include <Arduino.h>
#include <SPI.h>

#define W25QXX_DUMMY_BYTE         0xA5

w25qxx_t	w25qxx;
SPIClass spi3(PB5, PB4, PB3);
SPISettings spiSetting(SPI_SPEED_CLOCK_DIV2_MHZ, MSBFIRST, SPI_MODE3);

//spi3.transfer 读写一个字节
//Data:要写入的字节
//返回值:读取到的字节
// uint8_t	spi3.transfer(uint8_t	Data)
// {
// 	uint8_t	ret;
// 	HAL_SPI_TransmitReceive(&_W25QXX_SPI,&Data,&ret,1,100);
// 	return ret;	
// }

//读取芯片ID
//返回值如下:				   
//0XEF13,表示芯片型号为W25Q80  
//0XEF14,表示芯片型号为W25Q16    
//0XEF15,表示芯片型号为W25Q32  
//0XEF16,表示芯片型号为W25Q64 
//0XEF17,表示芯片型号为W25Q128 	  
//0XEF18,表示芯片型号为W25Q256
uint32_t W25QXX_ReadID(void)
{
	uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

    digitalWrite(W25QXX_CS_PIN, LOW);
    spi3.transfer(0x9F);
    Temp0 = spi3.transfer(0xA5);
    Temp1 = spi3.transfer(0xA5);
    Temp2 = spi3.transfer(0xA5);
    digitalWrite(W25QXX_CS_PIN, HIGH);
    Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
    
    return Temp;
}   

void W25qxx_ReadUniqID(void)
{
  digitalWrite(W25QXX_CS_PIN, LOW);
  spi3.transfer(0x4B);
	for(uint8_t	i=0;i<4;i++)
		spi3.transfer(W25QXX_DUMMY_BYTE);
	for(uint8_t	i=0;i<8;i++)
		w25qxx.UniqID[i] = spi3.transfer(W25QXX_DUMMY_BYTE);
  digitalWrite(W25QXX_CS_PIN, HIGH);
}

//4Kbytes为一个Sector
//16个扇区为1个Block
//W25Q256
//容量为32M字节,共有512个Block,8192个Sector 
													 
//初始化SPI FLASH的IO口
bool W25QXX_Init(void)
{ 
	pinMode(W25QXX_CS_PIN, OUTPUT);
	digitalWrite(W25QXX_CS_PIN, HIGH);
	spi3.beginTransaction(spiSetting);
    delay(100);
    
    uint32_t	id;
    
    id=W25QXX_ReadID();
    switch(id & 0x000000FF)
  {
		case 0x20:	// 	w25q512
			w25qxx.ID=W25Q512;
			w25qxx.BlockCount=1024;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q512\r\n");
			#endif
		break;
		case 0x19:	// 	w25q256
			w25qxx.ID=W25Q256;
			w25qxx.BlockCount=512;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q256\r\n");
			#endif
		break;
		case 0x18:	// 	w25q128
			w25qxx.ID=W25Q128;
			w25qxx.BlockCount=256;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q128\r\n");
			#endif
		break;
		case 0x17:	//	w25q64
			w25qxx.ID=W25Q64;
			w25qxx.BlockCount=128;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q64\r\n");
			#endif
		break;
		case 0x16:	//	w25q32
			w25qxx.ID=W25Q32;
			w25qxx.BlockCount=64;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q32\r\n");
			#endif
		break;
		case 0x15:	//	w25q16
			w25qxx.ID=W25Q16;
			w25qxx.BlockCount=32;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q16\r\n");
			#endif
		break;
		case 0x14:	//	w25q80
			w25qxx.ID=W25Q80;
			w25qxx.BlockCount=16;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q80\r\n");
			#endif
		break;
		case 0x13:	//	w25q40
			w25qxx.ID=W25Q40;
			w25qxx.BlockCount=8;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q40\r\n");
			#endif
		break;
		case 0x12:	//	w25q20
			w25qxx.ID=W25Q20;
			w25qxx.BlockCount=4;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q20\r\n");
			#endif
		break;
		case 0x11:	//	w25q10
			w25qxx.ID=W25Q10;
			w25qxx.BlockCount=2;
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Chip: w25q10\r\n");
			#endif
		break;
		default:
			#if (_W25QXX_DEBUG==1)
			Serial.printf("w25qxx Unknown ID\r\n");
			#endif	
			return false;
				
	}		
	w25qxx.PageSize=256;
	w25qxx.SectorSize=0x1000;
	w25qxx.SectorCount=w25qxx.BlockCount*16;
	w25qxx.PageCount=(w25qxx.SectorCount*w25qxx.SectorSize)/w25qxx.PageSize;
	w25qxx.BlockSize=w25qxx.SectorSize*16;
	w25qxx.CapacityInKiloByte=(w25qxx.SectorCount*w25qxx.SectorSize)/1024;
    W25qxx_ReadUniqID();
	W25QXX_ReadSR(1);
	W25QXX_ReadSR(2);
	W25QXX_ReadSR(3);
	#if (_W25QXX_DEBUG==1)
	Serial.printf("w25qxx Page Size: %d Bytes\r\n",w25qxx.PageSize);
	Serial.printf("w25qxx Page Count: %d\r\n",w25qxx.PageCount);
	Serial.printf("w25qxx Sector Size: %d Bytes\r\n",w25qxx.SectorSize);
	Serial.printf("w25qxx Sector Count: %d\r\n",w25qxx.SectorCount);
	Serial.printf("w25qxx Block Size: %d Bytes\r\n",w25qxx.BlockSize);
	Serial.printf("w25qxx Block Count: %d\r\n",w25qxx.BlockCount);
	Serial.printf("w25qxx Capacity: %d KiloBytes\r\n",w25qxx.CapacityInKiloByte);
	Serial.printf("w25qxx Init Done\r\n");
	#endif

	return true;
}  

//读取W25QXX的状态寄存器，W25QXX一共有3个状态寄存器
//状态寄存器1：
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
//状态寄存器2：
//BIT7  6   5   4   3   2   1   0
//SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
//状态寄存器3：
//BIT7      6    5    4   3   2   1   0
//HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
//regno:状态寄存器号，范:1~3
//返回值:状态寄存器值
uint8_t W25QXX_ReadSR(uint8_t regno)   
{  
	uint8_t byte=0,command=0; 
    switch(regno)
    {
        case 1:
            command=W25X_ReadStatusReg1;    //读状态寄存器1指令
            break;
        case 2:
            command=W25X_ReadStatusReg2;    //读状态寄存器2指令
            break;
        case 3:
            command=W25X_ReadStatusReg3;    //读状态寄存器3指令
            break;
        default:
            command=W25X_ReadStatusReg1;    
            break;
    }    
	digitalWrite(W25QXX_CS_PIN, LOW);                            //使能器件   
	spi3.transfer(command);            //发送读取状态寄存器命令    
	byte=spi3.transfer(0Xff);          //读取一个字节  
	digitalWrite(W25QXX_CS_PIN, HIGH);                            //取消片选     
	return byte;   
} 

//写W25QXX状态寄存器
void W25QXX_Write_SR(uint8_t regno,uint8_t sr)   
{   
    uint8_t command=0;
    switch(regno)
    {
        case 1:
            command=W25X_WriteStatusReg1;    //写状态寄存器1指令
            break;
        case 2:
            command=W25X_WriteStatusReg2;    //写状态寄存器2指令
            break;
        case 3:
            command=W25X_WriteStatusReg3;    //写状态寄存器3指令
            break;
        default:
            command=W25X_WriteStatusReg1;    
            break;
    }   
	digitalWrite(W25QXX_CS_PIN, LOW);                            //使能器件   
	spi3.transfer(command);            //发送写取状态寄存器命令    
	spi3.transfer(sr);                 //写入一个字节  
	digitalWrite(W25QXX_CS_PIN, HIGH);                            //取消片选     	      
}   

//W25QXX写使能	
//将WEL置位   
void W25QXX_Write_Enable(void)   
{
	digitalWrite(W25QXX_CS_PIN, LOW);                            //使能器件   
    spi3.transfer(W25X_WriteEnable);   //发送写使能  
	digitalWrite(W25QXX_CS_PIN, HIGH);                            //取消片选     	      
} 

//W25QXX写禁止	
//将WEL清零  
void W25QXX_Write_Disable(void)   
{  
	digitalWrite(W25QXX_CS_PIN, LOW);                            //使能器件   
    spi3.transfer(W25X_WriteDisable);  //发送写禁止指令    
	digitalWrite(W25QXX_CS_PIN, HIGH);                            //取消片选     	      
} 

//读取SPI FLASH  
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead)   
{ 
	#if (_W25QXX_DEBUG==1)
	uint32_t	StartTime=HAL_GetTick();
	Serial.printf("w25qxx ReadByte at address %d begin...\r\n",ReadAddr);
	#endif
 	uint16_t i;   										    
	digitalWrite(W25QXX_CS_PIN, LOW);                            //使能器件   
    spi3.transfer(W25X_ReadData);      //发送读取命令  
    if(w25qxx.ID >= W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        spi3.transfer((uint8_t)((ReadAddr)>>24));    
    }
    spi3.transfer((uint8_t)((ReadAddr)>>16));   //发送24bit地址    
    spi3.transfer((uint8_t)((ReadAddr)>>8));   
    spi3.transfer((uint8_t)ReadAddr);   
    for(i=0;i<NumByteToRead;i++)
	{ 
        pBuffer[i]=spi3.transfer(0XFF);    //循环读数  
    }
	digitalWrite(W25QXX_CS_PIN, HIGH); 				    	      
    #if (_W25QXX_DEBUG==1)
	Serial.printf("w25qxx ReadByte 0x%02X done after %d ms\r\n",*pBuffer,HAL_GetTick()-StartTime);
	#endif
}  

//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!	 
void W25QXX_Write_Page(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;  
    W25QXX_Write_Enable();                  //SET WEL 
	digitalWrite(W25QXX_CS_PIN, LOW);                            //使能器件   
    spi3.transfer(W25X_PageProgram);   //发送写页命令   
    if(w25qxx.ID >= W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        spi3.transfer((uint8_t)((WriteAddr)>>24)); 
    }
    spi3.transfer((uint8_t)((WriteAddr)>>16)); //发送24bit地址    
    spi3.transfer((uint8_t)((WriteAddr)>>8));   
    spi3.transfer((uint8_t)WriteAddr);   
    for(i=0;i<NumByteToWrite;i++)spi3.transfer(pBuffer[i]);//循环写数  
	digitalWrite(W25QXX_CS_PIN, HIGH);                            //取消片选 
	W25QXX_Wait_Busy();					   //等待写入结束
} 

//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 	  
	uint16_t pageremain;	   
	pageremain=256-WriteAddr%256; //单页剩余的字节数		 	    
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
	while(1)
	{	   
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;//写入结束了
	 	else //NumByteToWrite>pageremain
		{
			pBuffer+=pageremain;
			WriteAddr+=pageremain;	

			NumByteToWrite-=pageremain;			  //减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
			else pageremain=NumByteToWrite; 	  //不够256个字节了
		}
	};
} 

//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)						
//NumByteToWrite:要写入的字节数(最大65535)   
uint8_t W25QXX_BUFFER[4096];		 
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite)   
{ 
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;	   
 	uint16_t i;    
	uint8_t *W25QXX_BUF;
    
   	W25QXX_BUF = W25QXX_BUFFER;	     
 	secpos = WriteAddr/4096;    //扇区地址  
	secoff = WriteAddr%4096;    //在扇区内的偏移
	secremain = 4096 - secoff;  //扇区剩余空间大小 
    #if (_W25QXX_DEBUG==1)
 	Serial.printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
    #endif
 	if(NumByteToWrite <= secremain)
        secremain = NumByteToWrite;//不大于4096个字节
	while(1) 
	{	
		W25QXX_Read(W25QXX_BUF, secpos*4096, 4096);//读出整个扇区的内容
		for(i=0; i < secremain; i++)//校验数据
		{
			if(W25QXX_BUF[secoff+i] != 0XFF)
                break;//需要擦除  	  
		}
		if(i < secremain)//需要擦除
		{
			W25QXX_Erase_Sector(secpos);//擦除这个扇区
			for(i=0 ; i < secremain; i++)	   //复制
			{
				W25QXX_BUF[i+secoff]=pBuffer[i];	  
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//写入整个扇区  

		}
        else
        {
            W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间. 
        }            
		if(NumByteToWrite==secremain)
        {
            break;//写入结束了
        }
		else//写入未结束
		{
			secpos++;//扇区地址增1
			secoff=0;//偏移位置为0 	 

		   	pBuffer += secremain;  //指针偏移
			WriteAddr += secremain;//写地址偏移	   
		   	NumByteToWrite -= secremain;		    //字节数递减
			if(NumByteToWrite > 4096)
                secremain=4096;	                    //下一个扇区还是写不完
			else 
                secremain = NumByteToWrite;			//下一个扇区可以写完了
		}	 
	}
}

//擦除整个芯片		  
//等待时间超长...
void W25QXX_Erase_Chip(void)   
{     
    W25QXX_Write_Enable();                  //SET WEL 
    W25QXX_Wait_Busy();   
  	digitalWrite(W25QXX_CS_PIN, LOW);                            //使能器件   
    spi3.transfer(W25X_ChipErase);        //发送片擦除命令  
	digitalWrite(W25QXX_CS_PIN, HIGH);                            //取消片选     	      
	W25QXX_Wait_Busy();   				   //等待芯片擦除结束
}   

//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个扇区的最少时间:150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)   
{  
	//监视falsh擦除情况,测试用   
 	//Serial.printf("fe:%x\r\n",Dst_Addr);	  
 	Dst_Addr*=4096;
    W25QXX_Write_Enable();                  //SET WEL 	 
    W25QXX_Wait_Busy();   
  	digitalWrite(W25QXX_CS_PIN, LOW);                            //使能器件   
    spi3.transfer(W25X_SectorErase);   //发送扇区擦除指令 
    if(w25qxx.ID >= W25Q256)                //如果是W25Q256的话地址为4字节的，要发送最高8位
    {
        spi3.transfer((uint8_t)((Dst_Addr)>>24)); 
    }
    spi3.transfer((uint8_t)((Dst_Addr)>>16));  //发送24bit地址    
    spi3.transfer((uint8_t)((Dst_Addr)>>8));   
    spi3.transfer((uint8_t)Dst_Addr);  
	digitalWrite(W25QXX_CS_PIN, HIGH);                           //取消片选     	      
    W25QXX_Wait_Busy();   				    //等待擦除完成
} 

//等待空闲
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR(1)&0x01)==0x01);   // 等待BUSY位清空
}  

//进入掉电模式
void W25QXX_PowerDown(void)   
{ 
  	digitalWrite(W25QXX_CS_PIN, LOW);                           //使能器件   
    spi3.transfer(W25X_PowerDown);     //发送掉电命令  
	digitalWrite(W25QXX_CS_PIN, HIGH);                            //取消片选     	      
    HAL_Delay(1);                            //等待TPD  
}   

//唤醒
void W25QXX_WAKEUP(void)   
{  
  	digitalWrite(W25QXX_CS_PIN, LOW);                                //使能器件   
    spi3.transfer(W25X_ReleasePowerDown);  //  send W25X_PowerDown command 0xAB    
	digitalWrite(W25QXX_CS_PIN, HIGH);                                //取消片选     	      
    HAL_Delay(1);                                //等待TRES1
}   
