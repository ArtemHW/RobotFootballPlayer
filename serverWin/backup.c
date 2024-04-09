/* USER CODE BEGIN Header_encoderR */
/**
* @brief Function implementing the EncoderR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoderR */
void encoderR(void const * argument)
{
  /* USER CODE BEGIN encoderR */
  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay(pdMS_TO_TICKS(ENCDELAY));

	  EncoderR.timeOld = EncoderR.timeNew;
	  EncoderR.timeNew = TIM17->CNT;
	  EncoderR.positionOld = EncoderR.positionNew;
	  EncoderR.positionNew = TIM1->CNT;

	  if(EncoderR.timeNew - EncoderR.timeOld == 0) {
		  continue;
	  }

	  switch (EncoderR.posCntUpdate + EncoderR.timeUpdate) {
		case 0:
			EncoderR.rpm = -((float)(((float)(EncoderR.positionNew - EncoderR.positionOld)) / ((float)(EncoderR.timeNew - EncoderR.timeOld)))*kToRpm); //(32*1000*60)/256;
			break;
		case POSUPDATED:
			if((EncoderR.positionOld >= 0) && (EncoderR.positionOld <= 32768) ) {
			  EncoderR.rpm = -((float)(((float)(EncoderR.positionNew - 65535 - EncoderR.positionOld)) / ((float)(EncoderR.timeNew - EncoderR.timeOld)))*kToRpm);
			} else {
			  EncoderR.rpm = -((float)(((float)(EncoderR.positionNew + (65535 - EncoderR.positionOld))) / ((float)(EncoderR.timeNew - EncoderR.timeOld)))*kToRpm);
			}
			EncoderR.posCntUpdate = 0;
			break;
		case TIMEUPDATED:
			EncoderR.rpm = -((float)(((float)(EncoderR.positionNew - EncoderR.positionOld)) / ((float)(EncoderR.timeNew + 65535 - EncoderR.timeOld)))*kToRpm);
			EncoderR.timeUpdate = 0;
			break;
		case (POSUPDATED + TIMEUPDATED):
			if((EncoderR.positionOld >= 0) && (EncoderR.positionOld <= 32768) ) {
			  EncoderR.rpm = -((float)(((float)(EncoderR.positionNew - 65535 - EncoderR.positionOld)) / ((float)(EncoderR.timeNew + 65535 - EncoderR.timeOld)))*kToRpm);
			} else {
			  EncoderR.rpm = -((float)(((float)(EncoderR.positionNew + (65535 - EncoderR.positionOld))) / ((float)(EncoderR.timeNew + 65535 - EncoderR.timeOld)))*kToRpm);
			}
			EncoderR.posCntUpdate = 0;
			EncoderR.timeUpdate = 0;
			break;
		default:
			break;
	}
	  __asm__ volatile("NOP");
  }
  /* USER CODE END encoderR */
}

/* USER CODE BEGIN Header_encoderL */
/**
* @brief Function implementing the EncoderL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoderL */
void encoderL(void const * argument)
{
  /* USER CODE BEGIN encoderL */
  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay(pdMS_TO_TICKS(ENCDELAY));

	  if(TIM2->CNT > TIM2->ARR) {
		  TIM2->EGR |= TIM_EGR_UG;
	  }

	  EncoderL.timeOld = EncoderL.timeNew;
	  EncoderL.timeNew = TIM17->CNT;
	  EncoderL.positionOld = EncoderL.positionNew;
	  EncoderL.positionNew = TIM2->CNT;

	  if(EncoderL.timeNew - EncoderL.timeOld == 0) {
		  continue;
	  }

	  switch (EncoderL.posCntUpdate + EncoderL.timeUpdate) {
		case 0:
			EncoderL.rpm = ((float)(((float)(EncoderL.positionNew - EncoderL.positionOld)) / ((float)(EncoderL.timeNew - EncoderL.timeOld)))*kToRpm); //(32*1000*60)/256;
			break;
		case POSUPDATED:
			if((EncoderL.positionOld >= 0) && (EncoderL.positionOld <= 32768) ) {
				EncoderL.rpm = ((float)(((float)(EncoderL.positionNew - 65535 - EncoderL.positionOld)) / ((float)(EncoderL.timeNew - EncoderL.timeOld)))*kToRpm);
			} else {
				EncoderL.rpm = ((float)(((float)(EncoderL.positionNew + (65535 - EncoderL.positionOld))) / ((float)(EncoderL.timeNew - EncoderL.timeOld)))*kToRpm);
			}
			EncoderL.posCntUpdate = 0;
			break;
		case TIMEUPDATED:
			EncoderL.rpm = ((float)(((float)(EncoderL.positionNew - EncoderL.positionOld)) / ((float)(EncoderL.timeNew + 65535 - EncoderL.timeOld)))*kToRpm);
			EncoderL.timeUpdate = 0;
			break;
		case (POSUPDATED + TIMEUPDATED):
			if((EncoderL.positionOld >= 0) && (EncoderL.positionOld <= 32768) ) {
				EncoderL.rpm = ((float)(((float)(EncoderL.positionNew - 65535 - EncoderL.positionOld)) / ((float)(EncoderL.timeNew + 65535 - EncoderL.timeOld)))*kToRpm);
			} else {
				EncoderL.rpm = ((float)(((float)(EncoderL.positionNew + (65535 - EncoderL.positionOld))) / ((float)(EncoderL.timeNew + 65535 - EncoderL.timeOld)))*kToRpm);
			}
			EncoderL.posCntUpdate = 0;
			EncoderL.timeUpdate = 0;
			break;
		default:
			break;
	}

	  __asm__ volatile("NOP");
  }
  /* USER CODE END encoderL */
}

/* USER CODE BEGIN Header_softwarePWMR */
/**
* @brief Function implementing the SoftwarePwmR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_softwarePWMR */
void softwarePWMR(void const * argument)
{
  /* USER CODE BEGIN softwarePWMR */
	int errorValue = 0;
	float sumValue = 0;
	float pValue = 0;
	float iValue = 0;
	float pwmFloatValue = 0;
	vTaskDelay( pdMS_TO_TICKS( 10 ) );
	GPIOA->ODR |= (1<<6); //EN34

	SoftPwmR.reqValue = 6000;
  /* Infinite loop */
  for(;;)
  {
	  float rWheelSpeed = tSpeed - aSpeed*DISBETWHEELS/2;
	  float reqValueTemp = (rWheelSpeed*60)/(2*3.14*RWHEEL);
	  if((reqValueTemp >= - 50) && (reqValueTemp <= 50)) {
		  reqValueTemp = 0;
	  } else if((reqValueTemp > MAXRPM)) {
		  reqValueTemp = MAXRPM;
	  } else if((reqValueTemp < -MAXRPM)) {
		  reqValueTemp = -MAXRPM;
	  }
	  SoftPwmR.reqValue = (int16_t)reqValueTemp;

	  SoftPwmR.curValue = EncoderR.rpm;
	  errorValue = SoftPwmR.reqValue - SoftPwmR.curValue;
	  pValue = KP * errorValue;
	  iValue += KI * errorValue;
	  if(iValue > ((float)MAXRPM)) iValue = MAXRPM;
	  else if(iValue < ((float)-MAXRPM)) iValue = -MAXRPM;
	  if((iValue <= 50) && (iValue >= -50)) {
		  sumValue = pValue;
	  } else {
		  sumValue = (pValue + iValue);
	  }
	  pwmFloatValue += ((((float)sumValue)/((float)MAXRPM))*PWMVAL);
	  if(pwmFloatValue > PWMVAL) pwmFloatValue = PWMVAL;
	  else if(pwmFloatValue < -PWMVAL) pwmFloatValue = -PWMVAL;
	  SoftPwmR.pwmValue = (int16_t)pwmFloatValue;
	  if(SoftPwmR.reqValue == 0) {
		  if(SoftPwmR.pwmValue > 0) {
			  pwmFloatValue -= 1;
		  }
		  if(SoftPwmR.pwmValue < 0) {
			  pwmFloatValue += 1;
		  }
	  }

	  if(SoftPwmR.pwmValue < 0) {
		  TIM15->CCR1 = TIM15->ARR - SoftPwmR.pwmValue*(-1);
		  SoftPwmR.status = 2;
	  } else if(SoftPwmR.pwmValue > 0){
		  TIM15->CCR1 = TIM15->ARR - SoftPwmR.pwmValue;
		  SoftPwmR.status = 1;
	  } else {
		  SoftPwmR.status = 0;
	  }

	  vTaskDelay(pdMS_TO_TICKS(ENCDELAY));
  }
  /* USER CODE END softwarePWMR */
}

/* USER CODE BEGIN Header_softwarePWML */
/**
* @brief Function implementing the SoftwarePwmL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_softwarePWML */
void softwarePWML(void const * argument)
{
  /* USER CODE BEGIN softwarePWML */
	int errorValue = 0;
	float sumValue = 0;
	float pValue = 0;
	float iValue = 0;
	float pwmFloatValue = 0;
	vTaskDelay( pdMS_TO_TICKS( 10 ) );
	GPIOA->ODR |= (1<<3); //EN12

	SoftPwmL.reqValue = 6000;
  /* Infinite loop */
  for(;;)
  {
	  float lWheelSpeed = tSpeed + aSpeed*DISBETWHEELS/2;
	  float reqValueTemp = (lWheelSpeed*60)/(2*3.14*RWHEEL);
	  if((reqValueTemp >= - 50) && (reqValueTemp <= 50)) {
		  reqValueTemp = 0;
	  } else if((reqValueTemp > MAXRPM)) {
		  reqValueTemp = MAXRPM;
	  } else if((reqValueTemp < -MAXRPM)) {
		  reqValueTemp = -MAXRPM;
	  }
	  SoftPwmL.reqValue = (int16_t)reqValueTemp;

	  SoftPwmL.curValue = EncoderL.rpm;
	  errorValue = SoftPwmL.reqValue - SoftPwmL.curValue;
	  pValue = KP * errorValue;
	  iValue += KI * errorValue;
	  if(iValue > ((float)MAXRPM)) iValue = MAXRPM;
	  else if(iValue < ((float)-MAXRPM)) iValue = -MAXRPM;
	  if((iValue <= 50) && (iValue >= -50)) {
		  sumValue = pValue;
	  } else {
		  sumValue = (pValue + iValue);
	  }
	  pwmFloatValue += ((((float)sumValue)/((float)MAXRPM))*PWMVAL);
	  if(pwmFloatValue > PWMVAL) pwmFloatValue = PWMVAL;
	  else if(pwmFloatValue < -PWMVAL) pwmFloatValue = -PWMVAL;
	  SoftPwmL.pwmValue = (int16_t)pwmFloatValue;
	  if(SoftPwmL.reqValue == 0) {
		  if(SoftPwmL.pwmValue > 0) {
			  pwmFloatValue -= 1;
		  }
		  if(SoftPwmL.pwmValue < 0) {
			  pwmFloatValue += 1;
		  }
	  }

	  if(SoftPwmL.pwmValue < 0) {
		  TIM15->CCR2 = TIM15->ARR - SoftPwmL.pwmValue*(-1);
		  SoftPwmL.status = 2;
	  } else if(SoftPwmL.pwmValue > 0) {
		  TIM15->CCR2 = TIM15->ARR - SoftPwmL.pwmValue;
		  SoftPwmL.status = 1;
	  } else {
		  SoftPwmL.status = 0;
	  }

	  vTaskDelay(pdMS_TO_TICKS(ENCDELAY));

  }
  /* USER CODE END softwarePWML */
}