#include "hqfoc_app.h"
#include "main.h"
#include "user_interface.h"
#include "motor_control_protocol.h"
#include <string.h>

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

extern PWMC_Handle_t *pwmcHandle[NBR_OF_MOTORS];
extern CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
extern FOCVars_t FOCVars[NBR_OF_MOTORS];

as5311_pack_t as5311;


static int16_t GetElAngle(void);

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

int16_t dbg_speed = 1;
qd_t dbg_Vqd_ref;
int16_t hElAngle;
uint16_t ori_tle5012_angle;
uint32_t ams5311_ori_reg;
uint16_t hElAngle_compensation = 1658;
uint16_t HQ_FOC_CurrControllerM1(void)
{
    qd_t Iqd, Vqd;
    ab_t Iab;
    alphabeta_t Ialphabeta, Valphabeta;

    uint16_t hCodeError;
    SpeednPosFdbk_Handle_t *speedHandle;

//    hElAngle = GetElAngle() + hElAngle_compensation;
	GetElAngle();
	hElAngle = (int16_t)((as5311.overloop_pos % ENC_COUNT_PER_POLE_PAIR) * 1.28f) + hElAngle_compensation;

//    hElAngle = -16384;
    //  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
    //  hElAngle = SPD_GetElAngle(speedHandle);

    //    hElAngle += 1;

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
    // hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * REV_PARK_ANGLE_COMPENSATION_FACTOR;
    Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
    FOCVars[M1].Vqd = Vqd;
    FOCVars[M1].Iab = Iab;
    FOCVars[M1].Ialphabeta = Ialphabeta;
    FOCVars[M1].Iqd = Iqd;
    FOCVars[M1].Valphabeta = Valphabeta;
    FOCVars[M1].hElAngle = hElAngle;

    return (hCodeError);
}

uint16_t rx_buff[2];

uint32_t ams5311_read(ams5311_frame_type_e frame_type)
{
    uint8_t cpol = SPI_POLARITY_LOW;
    uint32_t ams_reg;
    if (frame_type == POS_FRAME)
        cpol = SPI_POLARITY_HIGH;
    else
        cpol = SPI_POLARITY_LOW;

    if ((hspi1.Instance->CR1 & SPI_CR1_CPOL) != cpol)
    {
        change_spi1_cpol(cpol);
    }

    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Receive_DMA(&hspi1, (uint8_t *)&rx_buff, 2);

    ams_reg = ((rx_buff[0] & 0x3FF) << 10 | (rx_buff[1] & 0x3FF)) >> 1;

    return (ams_reg & 0x3FFFF);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_GPIO_WritePin(SEN_CS_GPIO_Port, SEN_CS_Pin, GPIO_PIN_SET);
}

void change_spi1_cpol(uint8_t cpol)
{
    uint32_t tmpreg = 0;
    /* Check the parameters */
    assert_param(IS_SPI_CPOL(cpol));

    /* Read the SPIx CR1 register */
    tmpreg = READ_REG(hspi1.Instance->CR1);

    /* Reset CPOL bit */
    tmpreg &= (~SPI_CR1_CPOL);

    /* Set new CPOL bit value */
    tmpreg |= cpol;

    /* Write to SPIx CR1 */
    WRITE_REG(hspi1.Instance->CR1, tmpreg);
}

static int32_t overloop_proccess(int16_t single_pos, int16_t single_max)
{
    static int16_t single_pos_last = 0;
    static int16_t cnt = 0;

    if (single_pos_last - single_pos > single_max / 2)
        cnt++;
    if (single_pos_last - single_pos < -single_max / 2)
        cnt--;
    int32_t overloop_pos = single_pos + cnt * single_max;
    single_pos_last = single_pos;

    return overloop_pos;
}

static int16_t GetElAngle(void)
{
    uint32_t reg_data = ams5311_read(POS_FRAME);

    as5311.even_par = reg_data & 0x01;
    as5311.mag_dec = (reg_data >> 1) & 0x01;
    as5311.mag_inc = (reg_data >> 2) & 0x01;
    as5311.mag_inc = (reg_data >> 3) & 0x01;
    as5311.cof = (reg_data >> 4) & 0x01;
    as5311.ocf = (reg_data >> 5) & 0x01;

    as5311.posdata = (uint16_t)(reg_data >> 6);
    as5311.overloop_pos = overloop_proccess(as5311.posdata,4096);
    // as5311.pos_mm = as5311.overloop_pos / ENC_COUNT_PER_MM;

    // float pole_pair = 2 * POLE_PITCH;
    // float eangle = (as5311.pos_mm - floor(as5311.pos_mm / pole_pair) * pole_pair) / (pole_pair);
    // eangle = LIMIT(eangle,0,1);

//    as5311.ElAngle = (int16_t)((as5311.overloop_pos % ENC_COUNT_PER_POLE_PAIR) * EANGLE_MANGLE_RATE);

    return as5311.ElAngle;
}
