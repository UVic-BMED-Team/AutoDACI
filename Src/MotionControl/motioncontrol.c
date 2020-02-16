#include "motioncontrol.h"

void rotate_left(Motor_HandleTypeDef* motor, uint32_t num_steps, e_speed speed)
{
	HAL_Delay(1);
	uint8_t delay_bw_steps = 2 * speed + 1;
	for(; num_steps > 0; num_steps--){
		HAL_GPIO_WritePin( motor->GPIO_Step_Port, motor->GPIO_Step_Pin, GPIO_PIN_SET );
		HAL_Delay(delay_bw_steps);
		HAL_GPIO_WritePin( motor->GPIO_Step_Port, motor->GPIO_Step_Pin, GPIO_PIN_RESET );
		HAL_Delay(delay_bw_steps);
	}
}

void rotate_right(Motor_HandleTypeDef* motor, uint32_t num_steps, e_speed speed)
{
	HAL_Delay(1);
	uint8_t delay_bw_steps = 2 * speed + 1;
	for(; num_steps > 0; num_steps--){
		HAL_GPIO_WritePin( motor->GPIO_Step_Port, motor->GPIO_Step_Pin, GPIO_PIN_SET );
		HAL_Delay(delay_bw_steps);
		HAL_GPIO_WritePin( motor->GPIO_Step_Port, motor->GPIO_Step_Pin, GPIO_PIN_RESET );
		HAL_Delay(delay_bw_steps);
	}
}

void motor_init(Motor_HandleTypeDef* motor)
{
	// Step/Dir pin for rotation
	GPIO_InitTypeDef SD_Step_InitStruct = {0};
	SD_Step_InitStruct.Pin = motor->GPIO_Step_Pin;
	SD_Step_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	SD_Step_InitStruct.Pull = GPIO_NOPULL;
	SD_Step_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(motor->GPIO_Step_Port, &SD_Step_InitStruct);
	HAL_GPIO_WritePin(motor->GPIO_Step_Port, motor->GPIO_Step_Pin, GPIO_PIN_RESET );

//	// Step/Dir pin for rotation
//	GPIO_InitTypeDef SD_Dir_InitStruct = {0};
//	SD_Dir_InitStruct.Pin = motor->GPIO_Dir_Pin;
//	SD_Dir_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	SD_Dir_InitStruct.Pull = GPIO_NOPULL;
//	SD_Dir_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(motor->GPIO_Dir_Port, &SD_Dir_InitStruct);
//	HAL_GPIO_WritePin(motor->GPIO_Dir_Port, motor->GPIO_Dir_Pin, GPIO_PIN_RESET );

	AD_TMC5160_Init(motor);
}

void AD_TMC5160_Init(Motor_HandleTypeDef* motor)
{
	unsigned char buf[5] = {0};

	/*
	 * Recently discovered the board only works in step/direction mode...no motion control possible
	 * */

	uint8_t step_size = motor->stepsize; // 0 is smallest (256 msteps) to 8 (FULL STEP)

	uint8_t cmd[][5] = {
//			  {0x80, 0x00, 0x00, 0x00, 0x80}, // GCONF Activate diag0_stall (Datasheet Page 31)
//			  {0xED, 0x00, 0x00, 0x00, 0x00}, // CoolConf Stallguard SGT (need adapt to get a StallGuard2 event) (datasheet page 52)
//			  {0x94, 0x00, 0x00, 0x00, 0x40}, // TCOOLTHRS -> TSTEP based threshold = 55 (Datasheet Page 38)
//			  {0x89, 0x00, 0x01, 0x06, 0x06}, // SHORTCONF
//			  {0x8A, 0x00, 0x08, 0x04, 0x00}, // DRV_CONF
//			  //{0x90, 0x00, 0x08, 0x05, 0x03},
//			  {0x90, 0x00, 0x08, 0x03, 0x03}, // IHOLD_IRUN
//			  {0x91, 0x00, 0x00, 0x00, 0x0A}, // TPOWERDOWN
//			  {0xAB, 0x00, 0x00, 0x00, 0x01}, // VSTOP
//			  {0xBA, 0x00, 0x00, 0x00, 0x01}, // ENC_CONST (datasheet page 44)
//			  {0xEC, 0x10 | step_size, 0x41, 0x01, 0x53}, // CHOPCONF (resolution is second nibble)
//			  {0xF0, 0xC4, 0x0C, 0x00, 0x1E}  // PWMCONF
			{0x80, 0x00, 0x00, 0x00, 0x80},
			{0xEC, 0x10 | step_size, 0x41, 0x01, 0x53}, // CHOPCONF
			{0xED, 0x00, 0x00, 0x00, 0x00},
			//  uint8_t cmd9[] = {0xAD, 0xFF, 0xFF, 0x38, 0x00};
			{0x90, 0x00, 0x08, 0x04, 0x03}, // IHOLD_IRUN
			//  uint8_t cmd10[] = {0x21, 0x00, 0x00, 0x00, 0x00};
			{0x91, 0x00, 0x00, 0x00, 0x0A}, // TPOWERDOWN
			//{0x80, 0x00, 0x00, 0x00, 0x04}, // EN_PWM_MODE = 1
			{0x93, 0x00, 0x00, 0x01, 0xF4},

	};

	unsigned int num_cmds = sizeof(cmd) / 5;

	for(unsigned int i = 0; i < num_cmds; i++){
		HAL_GPIO_WritePin(motor->GPIO_SS_Port, motor->GPIO_SS_Pin, GPIO_PIN_RESET );
		HAL_SPI_TransmitReceive(&hspi2, cmd[i], buf, 5, HAL_MAX_DELAY);
		while( hspi2.State == HAL_SPI_STATE_BUSY ){/* wait */};
		HAL_GPIO_WritePin(motor->GPIO_SS_Port, motor->GPIO_SS_Pin, GPIO_PIN_SET );
	}
}
