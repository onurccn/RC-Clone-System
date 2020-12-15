#include <IR_Remote.h>

/////////////////////////////////////////////////////////////////////////////////

void send(uint16_t buf[], unsigned int len, unsigned long data, int nbits, decode_type_t protocol)
{
	if (protocol == UNKNOWN) {
		sendRaw(buf, len, 38);
	}
	else if (protocol == RC5) {
		sendRC5(data, nbits);
	}
	else if (protocol == RC6) {
		sendRC6(data, nbits);
	}
	else if (protocol == RC6) {
		sendRC6(data, nbits);
	}
	else if (protocol == NEC) {
		sendNEC(data, nbits);
	}
	else if (protocol == SONY) {
		sendSony(data, nbits);
	}
	else if (protocol == PANASONIC) {
		sendPanasonic(nbits, data);
	}
	else if (protocol == JVC) {
		sendJVC(data, nbits, 0);
	}
	else if (protocol == SAMSUNG) {
		sendSAMSUNG(data, nbits);
	}
	else if (protocol == WHYNTER) {
		sendWhynter(data, nbits);
	}
	else if (protocol == AIWA_RC_T501) {
		sendAiwaRCT501(data);
	}
	else if (protocol == LG) {
		sendLG(data, nbits);
	}
	else if (protocol == DENON) {
		sendDenon(data, nbits);
	}
}

void sendRaw(uint16_t buf[], unsigned int len, uint8_t hz)
{
	enableIROut(hz);
	int skipFirst = 0;
	if (buf[0] > 10000) {
		skipFirst = 1;
	}

	for(uint16_t i = skipFirst; i < len; i++)
	{
		if(i % 2 != skipFirst) space(buf[i]*USECPERTICK);
		else mark(buf[i]*USECPERTICK);
	}

	space(0);
}

void mark(unsigned int time)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	if (time > 0) custom_delay_usec(time);
}

void space(unsigned int time)
{
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	if(time > 0) custom_delay_usec(time);
}

void enableIROut(uint8_t khz)
{
	uint16_t pwm_freq = 0;
	uint16_t pwm_pulse = 0;
	pwm_freq = MYSYSCLOCK / (khz * 1000) - 1;
	pwm_pulse = pwm_freq / 3;

	HAL_TIM_Base_DeInit(&htim3);
	in_enabled = 0;

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = pwm_freq;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim3);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim3);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pwm_pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_MspPostInit(&htim3);

	out_enabled = 1;
}

void custom_delay_usec(unsigned long us)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // Every Count for this timer should be 1 microsec.
}
