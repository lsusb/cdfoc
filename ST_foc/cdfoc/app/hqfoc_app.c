#include "hqfoc_app.h"
#include "main.h"
#include "user_interface.h"
#include "motor_control_protocol.h"

extern SPI_HandleTypeDef hspi3;

void FCP_SetClient(FCP_Handle_t *pHandle,
                   struct MCP_Handle_s *pClient,
                   FCP_SentFrameCallback_t pSentFrameCb,
                   FCP_ReceivedFrameCallback_t pReceviedFrameCb,
                   FCP_RxTimeoutCallback_t pRxTimeoutCb)
{
}

uint16_t drv_read_reg(uint8_t reg)
{
    uint16_t rx_val;
    uint16_t val = 0x8000 | reg << 11;

    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&val, (uint8_t *)&rx_val, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);
    return rx_val & 0x7ff;
}

void drv_write_reg(uint8_t reg, uint16_t val)
{
    val |= reg << 11;

    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, (uint8_t *)&val, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET);
}

void drv_init(void)
{
    drv_write_reg(0x05, 0x036d);                   // 400ns dead time

    drv_write_reg(0x03, 0x0300);                   // 10mA, 20mA
    drv_write_reg(0x04, 0x0700);                   // 10mA, 20mA

    drv_write_reg(0x02, 0x1 << 2);                 // COAST mode

    drv_write_reg(0x02, (1 << 10) | (1 << 7) | (1 << 5)); // shutdown all on err, otw err, 3x pwm mode
    
}
