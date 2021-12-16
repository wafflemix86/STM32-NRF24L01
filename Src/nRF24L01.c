#include "nRF24L01.h"
#include "string.h"
#include "stdlib.h"
#include "stm32l0xx_hal_rcc.h"
// hello test test
uint8_t pipe0_reading_address[5] = "12345";
const uint8_t child_pipe[6] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
const uint8_t child_pipe_enable[6] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};
uint16_t txDelay = 0;

// this is ABSOLUTELY only an approximation. not 100% accurate
void DelayUs(uint32_t n_us)
{
  uint32_t clks_per_us = HAL_RCC_GetSysClockFreq()/1000000;
  // account for # of cycles per instruction and while loop overhead
  volatile uint32_t nopcount = (clks_per_us *n_us)/12 - 17;
  while(nopcount-- > 0)
  {
    asm("NOP");
  }
}

void Delay15Us()
{
  uint32_t clks_per_us = HAL_RCC_GetSysClockFreq()/1000000;
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
  asm("NOP");
}

void WriteCommand(nRF_Handle_t* handle, uint8_t cmd)
{
  HAL_SPI_Transmit(handle->spihandle, &cmd, 1, 1000);
}

void WriteRegister(nRF_Handle_t* handle, uint8_t reg, uint8_t data)
{
  uint8_t msg[2] = { (reg | W_REGISTER), data };
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle->spihandle,msg,2U,2000);
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_SET);
}

void WriteRegisterBytes(nRF_Handle_t* handle, uint8_t reg, uint8_t* data, uint8_t len)
{
  uint8_t* txdata = malloc((len+1)*sizeof(uint8_t));
  txdata[0] = (reg | W_REGISTER);
  memcpy(&txdata[1], data, len*sizeof(uint8_t));
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle->spihandle, txdata, (len+1)*sizeof(uint8_t), 2000);
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_SET);
  free(txdata);
}

uint8_t ReadRegister(nRF_Handle_t* handle, uint8_t reg)
{
  uint8_t txdata[2] = { (reg | R_REGISTER), RF24_NOP};
  uint8_t rxdata[2] = { 0x00, 0x00};
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(handle->spihandle, txdata, rxdata, 2, 2000);
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_SET);
  return rxdata[1];
}

void ReadRegisterBytes(nRF_Handle_t* handle, uint8_t reg, uint8_t* rxdata, uint8_t len)
{
  uint8_t* txdata = malloc((len+1)*sizeof(uint8_t));
  txdata[0] = (reg | R_REGISTER);
  memset(&txdata[1], RF24_NOP, len*sizeof(uint8_t));
  memset(rxdata, 0x00, len*sizeof(uint8_t));
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(handle->spihandle, txdata, rxdata, (len+1)*sizeof(uint8_t), 2000);
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_SET);
  free(txdata);
}

uint8_t GetStatus(nRF_Handle_t* handle)
{
  uint8_t rxdata[2] = {0,0};
  uint8_t txdata[2] = {RF24_NOP,RF24_NOP};
  HAL_SPI_TransmitReceive(handle->spihandle, txdata, rxdata, 2, 2000);
  //ReadRegister(handle,NRF_STATUS);
  return rxdata[0];
}

// if return is non 0 there is an issue;
uint8_t VerifyHandle(nRF_Handle_t* pHandle)
{
  if(pHandle == NULL || 
     pHandle->spihandle == NULL ||
     pHandle->CSN_Port == NULL ||
     pHandle->CE_Port == NULL) return 1;
  else return 0;
}

uint8_t nRF_Startup(nRF_Handle_t* handle)
{
  uint8_t regval = 0;
  
  // make sure handle has valid pointers
  if(VerifyHandle(handle)) return 1;
  
  HAL_Delay(5);
  regval = ReadRegister(handle, NRF_CONFIG);
  WriteRegister(handle, NRF_CONFIG, regval & ~(1 << PWR_UP));
  HAL_Delay(10);
  
  // set up 5 retries with 1500us delay between
  regval = (5 << 4) | (5);
  WriteRegister(handle, SETUP_RETR, regval);
  
  // set data rate to 1mbps
  regval = ReadRegister(handle, RF_SETUP);
  regval &= ~((1 << 5) | (1 << 3));
  WriteRegister(handle, RF_SETUP, regval);
  txDelay = 280;
  if(ReadRegister(handle, RF_SETUP) != regval) return 1;
  
  // check special register pertainant to old hardware variant
  uint8_t before_toggle = ReadRegister(handle, FEATURE);
  WriteCommand(handle, ACTIVATE);
  WriteCommand(handle, 0x73);
  regval = ReadRegister(handle, FEATURE);
  if(regval)
  {
    if(regval == before_toggle)
    {
      // module did not experience power-on-reset (#401) idk wtf 0x73 is about but in the other libraries it fixes things
      WriteCommand(handle, ACTIVATE);
      WriteCommand(handle, 0x73);
    }
    WriteRegister(handle, FEATURE, 0);
  }
  
  // disable dynamic payload
  WriteRegister(handle, DYNPD, 0);
  
  // enable auto-ack on all pipes
  WriteRegister(handle, EN_AA, 0x3F);
  
  // enable receiving on pipes 0 and 1
  WriteRegister(handle, EN_RXADDR, 3);
  
  // set static payload size to 32 bytes for each pipe
  WriteRegister(handle, RX_PW_P0, 32);
  WriteRegister(handle, RX_PW_P1, 32);
  WriteRegister(handle, RX_PW_P2, 32);
  WriteRegister(handle, RX_PW_P3, 32);
  WriteRegister(handle, RX_PW_P4, 32);
  WriteRegister(handle, RX_PW_P5, 32);

  // set address width to 5 bytes
  WriteRegister(handle, SETUP_AW, 0x03);
  
  // set RF channel to 76
  WriteRegister(handle, RF_CH, 76);
  
  // reset status register
  WriteRegister(handle, NRF_STATUS, (1 << RX_DR)|(1 << TX_DS)|(1 << MAX_RT));
  
  // flush tx and rx buffers
  WriteCommand(handle, FLUSH_RX);
  WriteCommand(handle, FLUSH_TX);
  
  // set config to reflect all isr on isr pin, enable ptx, power up, 16 bit crc
  regval = (1 << EN_CRC) | (1 << CRCO) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT);
  WriteRegister(handle, NRF_CONFIG, regval);
  
  // set power up bit 
  regval |= 1 << PWR_UP;
  WriteRegister(handle, NRF_CONFIG, regval);
  
  // delay 5 ms after power up
  HAL_Delay(5);
  
  // verify config register is properly set
  regval = ReadRegister(handle, NRF_CONFIG);
  if(regval == (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP)) return 0;
  else return 1;
}

void nRF_OpenReadingPipe(nRF_Handle_t* handle, uint8_t pipe, uint8_t* address, uint8_t addr_width)
{
  // store address if pipe 0 to recall later for openwritingpipe
  if(pipe == 0)
  {
    memcpy(pipe0_reading_address, address, addr_width);
  }
  // only transmit one byte of address if pipe >= 2
  if(pipe < 2)
  {
    WriteRegisterBytes(handle, child_pipe[pipe], address, addr_width);
  }
  else
  {
    WriteRegisterBytes(handle, child_pipe[pipe], address, 1);
  }
  // enable rx pipe
  uint8_t regval = ReadRegister(handle, EN_RXADDR);
  WriteRegister(handle, EN_RXADDR, regval|(1 << child_pipe_enable[pipe]));
}

void nRF_CloseReadingPipe(nRF_Handle_t* handle, uint8_t pipe)
{
  uint8_t regval = ReadRegister(handle, EN_RXADDR);
  regval &= ~(1 << child_pipe_enable[pipe]);
  WriteRegister(handle, EN_RXADDR, regval);
}

void nRF_OpenWritingPipe(nRF_Handle_t* handle, uint8_t* address)
{
  WriteRegisterBytes(handle, RX_ADDR_P0, address, 5);
  WriteRegisterBytes(handle, TX_ADDR, address, 5);
}

void nRF_StartListening(nRF_Handle_t* handle)
{
  // clear interrupt bits
  WriteRegister(handle, NRF_STATUS, (1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));
  // set receive mode bit 
  uint8_t regval = ReadRegister(handle, NRF_CONFIG);
  WriteRegister(handle, NRF_CONFIG, regval | (1 << PRIM_RX));
  
  // restore pipe 0 address
  if(pipe0_reading_address[0] > 0)
  {
    WriteRegisterBytes(handle, RX_ADDR_P0, pipe0_reading_address, 5);
  }
  else
  {
    nRF_CloseReadingPipe(handle, 0);
  }
  
  // set chip enable 
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin, GPIO_PIN_SET);
  DelayUs(130);
  
  
}

void nRF_StopListening(nRF_Handle_t* handle)
{
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin, GPIO_PIN_RESET );
  HAL_Delay(1);
  WriteCommand(handle, FLUSH_TX);
  uint8_t regval = ReadRegister(handle, NRF_CONFIG);
  regval &= ~(1 << PRIM_RX);
  WriteRegister(handle, NRF_CONFIG, regval);
  regval = ReadRegister(handle, EN_RXADDR);
  regval |= (1 << child_pipe_enable[0]);
  WriteRegister(handle, EN_RXADDR, regval);
}
     
void nRF_SetPALevel(nRF_Handle_t* handle, uint8_t level)
{
  uint8_t regval = ReadRegister(handle, RF_SETUP) & 0xF8;
  regval |= ((level << 1) + 1);
  WriteRegister(handle, RF_SETUP, regval);
}

// returns 1 if data available and 0 otherwise
uint8_t nRF_DataAvailable(nRF_Handle_t* handle)
{
  uint8_t fifoReady = ReadRegister(handle, FIFO_STATUS) & (1 << RX_FULL);
  uint8_t status = (GetStatus(handle) >> RX_P_NO) && 0x07;
  if( status <= 5 && fifoReady) return 1;
  else return 0;
}

void nRF_WriteData(nRF_Handle_t* handle, uint8_t* data, uint8_t len)
{
  WriteRegisterBytes(handle, W_TX_PAYLOAD, data, len);
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin, GPIO_PIN_SET);
  Delay15Us();
  // wait until tx data sent flag is set or max number of retreies reached
  while(!(ReadRegister(handle, NRF_STATUS) & ((1 << TX_DS)|(1 << MAX_RT)))) 
  {
  }
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin, GPIO_PIN_RESET);
  Delay15Us();
  WriteRegister(handle,NRF_STATUS, ReadRegister(handle, NRF_STATUS) | (1 << TX_DS) | (1 << MAX_RT));
}
     
void nRF_ReadData(nRF_Handle_t* handle, uint8_t* rxbuff, uint8_t len)
{
  ReadRegisterBytes(handle, R_RX_PAYLOAD, rxbuff, len);
  // clear data rx interrupt flag
  WriteRegister(handle, NRF_STATUS, ReadRegister(handle,NRF_STATUS) | (1 << RX_DR));
}
