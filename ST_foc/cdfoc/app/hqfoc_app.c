#include "hqfoc_app.h"
#include "main.h"
#include "user_interface.h"
#include "motor_control_protocol.h"

#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "mc_tuning.h"
#include "digital_output.h"
#include "state_machine.h"
#include "pwm_common.h"

#include "mc_tasks.h"
#include "parameters_conversion.h"


extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi1;

extern PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
extern CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
extern FOCVars_t FOCVars[NBR_OF_MOTORS];

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
    drv_write_reg(0x05, 0x036d); // 400ns dead time

    drv_write_reg(0x03, 0x0300); // 10mA, 20mA
    drv_write_reg(0x04, 0x0700); // 10mA, 20mA

    drv_write_reg(0x02, 0x1 << 2); // COAST mode

    drv_write_reg(0x02, (1 << 10) | (1 << 7) | (1 << 5)); // shutdown all on err, otw err, 3x pwm mode
}

#define SEN_CNT 2 // connection: MCU_MOSI -- resistor -- MCU_MISO -- TLE5012B_DATA
static volatile uint16_t sen_rx_val[2] = {0};
static uint16_t sen_tx_val[2] = {0x8021, 0};
uint16_t encoder_read(void)
{
    return 0xffff - (sen_rx_val[1] << 1);
}

void encoder_isr_prepare(void)
{
    __HAL_DMA_DISABLE(hspi1.hdmarx);
    hspi1.hdmarx->Instance->CNDTR = SEN_CNT;
    hspi1.hdmarx->Instance->CPAR = (uint32_t)&hspi1.Instance->DR;
    hspi1.hdmarx->Instance->CMAR = (uint32_t)sen_rx_val;
    __HAL_DMA_ENABLE(hspi1.hdmarx);

    __HAL_DMA_DISABLE(hspi1.hdmatx);
    hspi1.hdmatx->Instance->CNDTR = SEN_CNT;
    hspi1.hdmatx->Instance->CPAR = (uint32_t)&hspi1.Instance->DR;
    hspi1.hdmatx->Instance->CMAR = (uint32_t)sen_tx_val;
}

void tle_5012_init(void)
{
//    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);
//    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
//    encoder_isr_prepare();
	// __HAL_SPI_ENABLE(&hspi1);
	
//    __HAL_DMA_ENABLE(hspi1.hdmarx);
}

uint16_t encoder_reg_r(uint8_t addr)
{
    uint16_t buf[2];
    buf[0] = 0x8001 | (addr << 4);

    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, (uint8_t *)buf, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_SET);

    return buf[0];
}

void encoder_reg_w(uint8_t addr, uint16_t val)
{
    uint16_t buf[2];
    buf[0] = 0x0001 | (addr << 4);
    buf[1] = val;

    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_SET);
}


int16_t dbg_speed = 1;
qd_t dbg_Vqd_ref;
int16_t hElAngle;
uint16_t ori_tle5012_angle;
uint16_t HQ_FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;

  uint16_t hCodeError;
  SpeednPosFdbk_Handle_t *speedHandle;

//   HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_SET);
//   ori_tle5012_angle = encoder_read();
// 	encoder_isr_prepare();


// 	HAL_SPI_TransmitReceive_DMA(&hspi1,0x8021,&ori_tle5012_angle,1)
// 	HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_SET);
  
//  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
//  hElAngle = SPD_GetElAngle(speedHandle);
	
	hElAngle += 1;
	
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  RCM_ReadOngoingConv();
  RCM_ExecNextConv();
  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);
	
	Vqd = dbg_Vqd_ref;
//  Vqd.q = PI_Controller(pPIDIq[M1],
//            (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);

//  Vqd.d = PI_Controller(pPIDId[M1],
//            (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);

  Vqd = Circle_Limitation(pCLM[M1], Vqd);
  hElAngle += SPD_GetInstElSpeedDpp(speedHandle)*REV_PARK_ANGLE_COMPENSATION_FACTOR;
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
  hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;
	

  return(hCodeError);
}


void SPI1_DMA_RX_CPT_Callback(void)
{
    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_SET);
    ori_tle5012_angle = encoder_read();
    encoder_isr_prepare();
}


void encoder_isr(void)
{
    __HAL_DMA_ENABLE(hspi1.hdmatx);
}


uint16_t tle_5012_DMA_Read(void)
{
    uint16_t ang_reg_v = 0x8021, data_v;

    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_RESET);

//    __HAL_DMA_ENABLE(hspi1.hdmatx);
//    HAL_SPI_TransmitReceive_DMA(&hspi1,(uint8_t *)&ang_reg_v,(uint8_t *)&data_v,2);
		HAL_SPI_Transmit_DMA(&hspi1,(uint8_t *)&ang_reg_v,1);

//    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_SET);

//		 data_v = encoder_reg_r(2);
    return 0xffff - (data_v << 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_SET);
    ori_tle5012_angle = encoder_read();
    encoder_isr_prepare();
}
