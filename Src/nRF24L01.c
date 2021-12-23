#include "nRF24L01.h"
#include "string.h"
#include "stdlib.h"
#include "stm32l0xx_hal_rcc.h"
//#include "stdio.h"

uint8_t pipe0_reading_address[5] = "12345";
uint8_t tempRxBuffer[33] = {0};
const uint8_t child_pipe[6] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
const uint8_t child_pipe_enable[6] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};
uint16_t txDelay = 0;

uint8_t nRF_GetStatus()
{
  uint8_t result[2] = {0};
  uint8_t msg[2] = { (RF24_NOP | W_REGISTER), RF24_NOP };
  HAL_GPIO_WritePin(nRFHandle.CSN_Port, nRFHandle.CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(nRFHandle.spihandle, msg, result, 1, 2000);
  HAL_GPIO_WritePin(nRFHandle.CSN_Port, nRFHandle.CSN_Pin, GPIO_PIN_SET);
  return result[0];
}

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
  HAL_SPI_TransmitReceive(handle->spihandle, txdata, tempRxBuffer, (len+1)*sizeof(uint8_t), 2000);
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, GPIO_PIN_SET);
  memcpy(rxdata, tempRxBuffer + 1, len*sizeof(uint8_t));
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
  
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin,0);
  HAL_GPIO_WritePin(handle->CSN_Port, handle->CSN_Pin, 1);
  HAL_Delay(200);
  // set up 5 retries with 1500us delay between
  regval = (5 << 4) | (5);
  WriteRegister(handle, SETUP_RETR, regval);
  
  // set RF Data Rate to 1Mbps and set the according txDelay
  regval = ReadRegister(handle, RF_SETUP);
  regval &= ~((1 << RF_DR_LOW)|(RF_DR_HIGH));
  if(HAL_RCC_GetSysClockFreq() >= 20000000)
  {
    txDelay = 280;
  }
  else txDelay = 85;
  WriteRegister(handle,RF_SETUP, regval);
  
  // check special register pertainant to old hardware variant
  uint8_t before_toggle = ReadRegister(handle, FEATURE);
  //WriteCommand(handle, ACTIVATE);
  WriteCommand(handle, 0x73);
  regval = ReadRegister(handle, FEATURE);
  if(regval)
  {
    if(regval == before_toggle)
    {
      // module did not experience power-on-reset (#401) idk wtf 0x73 is about but in the other libraries it fixes things
      //WriteCommand(handle, ACTIVATE);
      WriteCommand(handle, 0x73);
    }
    // WriteRegister(handle, FEATURE, 0);
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
  WriteRegister(handle, NRF_CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << MASK_TX_DS) | (1 << MASK_MAX_RT));
  uint8_t config_reg = ReadRegister(handle,NRF_CONFIG);
  if(!(config_reg & (1<<PWR_UP)))
  {
    config_reg |= (1 << PWR_UP);
    WriteRegister(handle,NRF_CONFIG,config_reg);
  }
  
  // delay 5 ms after power up
  HAL_Delay(5);
  
  // verify config register is properly set
  if(config_reg == ((1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP)| (1 << MASK_MAX_RT)| (1 << MASK_TX_DS))) return 0;
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
  // set receive mode bit 
  WriteRegister(handle, NRF_CONFIG, ReadRegister(handle, NRF_CONFIG) | (1 << PRIM_RX));
  // clear rx and tx interrupts
  WriteRegister(handle, NRF_STATUS, ReadRegister(handle, NRF_STATUS)|(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));
  // set chip enable high
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin, GPIO_PIN_SET);
  // restore pipe 0 address if it already exists
  if(pipe0_reading_address[0] > 0)
  {
    WriteRegisterBytes(handle, RX_ADDR_P0, pipe0_reading_address, 5);
  }
  else
  {
    nRF_CloseReadingPipe(handle, 0);
  }
  
  // wait at least 130us with CE high
  
}

void nRF_StopListening(nRF_Handle_t* handle)
{
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin, GPIO_PIN_RESET );
  DelayUs(txDelay + 50);
  //WriteCommand(handle, FLUSH_TX);
  WriteRegister(handle, NRF_CONFIG, ReadRegister(handle, NRF_CONFIG) & ~(1 << PRIM_RX));
  WriteRegister(handle, EN_RXADDR, ReadRegister(handle, EN_RXADDR)|(1 << child_pipe_enable[0]));
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
  uint8_t status = ((nRF_GetStatus() >> RX_P_NO) & 0x07);
  if( status > 5) { return 0; }
  else { return 1; }
}

uint8_t nRF_WriteData(nRF_Handle_t* handle, uint8_t* data, uint8_t len)
{
  uint8_t stat = 0;
  
  // fill tx fifo register with data
  WriteRegisterBytes(handle, W_TX_PAYLOAD, data, len);
  // pulse CE high for at least 10us
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin, GPIO_PIN_SET);
  // wait until tx data sent flag is set or max number of retreies reached
  do 
  {
    stat = nRF_GetStatus();
    HAL_Delay(1);
  }
  while(!(stat & ((1 << TX_DS)|(1 << MAX_RT))));
  // set CE pin low
  HAL_GPIO_WritePin(handle->CE_Port, handle->CE_Pin, GPIO_PIN_RESET);
  // clear interrupt flags
  WriteRegister(handle,NRF_STATUS, (1 << RX_DR)|(1 << TX_DS)|(1 << MAX_RT));
  
  // return send failure if max retry reached
  if(stat & (1 << MAX_RT))
  {
    WriteCommand(handle, FLUSH_TX);
    return NRF_ERROR;
  }
  else 
  {
    return NRF_SUCCESS;
  }
}
     
void nRF_ReadData(nRF_Handle_t* handle, uint8_t* rxbuff, uint8_t len)
{
  ReadRegisterBytes(handle, R_RX_PAYLOAD, rxbuff, len);
  // clear data rx interrupt flag
  WriteRegister(handle, NRF_STATUS, ReadRegister(handle,NRF_STATUS) | (1 << RX_DR));
}

void nRF_SwitchToRx(nRF_Handle_t* handle)
{
  nRF_OpenReadingPipe(handle,1,nRFHandle.address,5);
  nRF_StartListening(handle);
}

void nRF_SwitchToTx(nRF_Handle_t* handle)
{
  nRF_StopListening(handle);
  nRF_OpenWritingPipe(handle,nRFHandle.address);
  
}

