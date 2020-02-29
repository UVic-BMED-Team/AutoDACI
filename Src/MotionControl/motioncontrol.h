/*
 * motioncontrol.h
 *
 *  Created on: Feb 15, 2020
 *      Author: tbgiles
 */

#ifndef SRC_MOTIONCONTROL_MOTIONCONTROL_H_
#define SRC_MOTIONCONTROL_MOTIONCONTROL_H_

#include <stdlib.h>
#include "stm32f4xx_hal.h"

#define ROTATION_STEP_PORT GPIOC
#define ROTATION_STEP_PIN GPIO_PIN_0
#define ROTATION_DIR_PORT GPIOA
#define ROTATION_DIR_PIN GPIO_PIN_3

typedef enum{
	ROTATION,
	HEIGHT,
	DEPTH,
	NUM_MOTORS
} e_motor;
typedef enum {CW, CCW} e_dir;
typedef enum {FAST, MEDIUM, SLOW} e_speed;

extern SPI_HandleTypeDef hspi2;

typedef struct {
	e_motor motor;
	e_dir direction;
	uint8_t stepsize;

	GPIO_TypeDef* GPIO_Step_Port;
	uint16_t GPIO_Step_Pin;
	GPIO_TypeDef* GPIO_Dir_Port;
	uint16_t GPIO_Dir_Pin;

	GPIO_TypeDef* GPIO_SS_Port;
	uint16_t GPIO_SS_Pin;
} Motor_HandleTypeDef;

//static MotorRep_TypeDef motor_list[NUM_MOTORS];

void rotate_left(Motor_HandleTypeDef* motor, uint32_t num_steps, e_speed speed);
void rotate_right(Motor_HandleTypeDef* motor, uint32_t num_steps, e_speed speed);
void motor_init(Motor_HandleTypeDef* motor);

#endif /* SRC_MOTIONCONTROL_MOTIONCONTROL_H_ */
