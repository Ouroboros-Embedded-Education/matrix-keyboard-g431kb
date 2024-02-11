
#include "mk_platform.h"

/* Insert here the include of gpio driver of your chipset*/
#include <stm32g4xx.h>

void _mk_gpio_write(mk_gpio_t *gpio, mk_gpio_state_e val){
	GPIO_TypeDef *GPIO;

	GPIO = (GPIO_TypeDef*)(gpio->GPIO);
    if (val == MK_GPIO_LOW){
        // write low value on the gpio port
    	HAL_GPIO_WritePin(GPIO, gpio->pin, GPIO_PIN_RESET);
    }
    else{
        // write high value on the gpio port
    	HAL_GPIO_WritePin(GPIO, gpio->pin, GPIO_PIN_SET);
    }
}

mk_gpio_state_e _mk_gpio_read(mk_gpio_t *gpio){
    // check the gpio state and return the value:
    // MK_GPIO_LOW if the state is low
    // MK_GPIO_HIGH if the state was high
	GPIO_TypeDef *GPIO;

	GPIO = (GPIO_TypeDef*)(gpio->GPIO);
	if (HAL_GPIO_ReadPin(GPIO, gpio->pin) == GPIO_PIN_RESET){
		return MK_GPIO_LOW;
	}
	else{
		return MK_GPIO_HIGH;
	}
}
