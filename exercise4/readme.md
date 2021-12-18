### Exercise 4

![LED blinking when pressing button](https://github.com/RobotGrrl/MakingEmbeddedSystems/blob/master/exercise4/blinking1.gif?raw=true)


	GPIO_PinState val = HAL_GPIO_ReadPin(JOY_UP_GPIO_Port, JOY_UP_Pin);

	if(val == GPIO_PIN_SET) {
	LED1_GPIO_Port->BSRR = 0b0010000000000000 << 16; // on
	} else if(val == GPIO_PIN_RESET) {
	LED1_GPIO_Port->BSRR = 0b0010000000000000; // off
	}

