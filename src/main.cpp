#include <Arduino.h>
#include <w25qxx.h>

/* 获取缓冲区的长度 */
#define countof(a)      (sizeof(a) / sizeof(*(a)))
#define TxBufferSize1   (countof(TxBuffer1) - 1)
#define RxBufferSize1   (countof(TxBuffer1) - 1)
#define  BufferSize (countof(Tx_Buffer)-1)

#define  FLASH_WriteAddress     0x00000
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress

/* 发送缓冲区初始化 */
uint8_t Tx_Buffer[] = "w25qxx test\r\nhttp://www.sensertek.com";
uint8_t Rx_Buffer[BufferSize + 1] = {0};

// the setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println("SPI FLASH example");
  W25QXX_Init();
  Serial.printf("Start Write W25Q32....\r\n");
  W25QXX_Write(Tx_Buffer, w25qxx.PageSize-200, BufferSize);
  Serial.printf("W25Q32 Write Finished!\r\n");
    
  Serial.printf("Start Read W25Q32....\r\n");
  W25QXX_Read(Rx_Buffer, w25qxx.PageSize-200, BufferSize);
  Serial.printf("The Data Readed Is: %s\r\n", Rx_Buffer);
}

void loop()
{
  // nothing to do
}