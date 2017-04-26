/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mbed_assert.h"
#include "pinmap.h"
#include "mbed_error.h"

void pin_function(PinName pin, int function) {

  MBED_ASSERT(pin != (PinName)NC);

  /* Enable AHB clock to the IOCON domain. */
  LPC_SYSCON->AHBCLKCTRLSET[0] = (0x1UL << 13);
  
  __IO uint32_t *reg = (__IO uint32_t*) (LPC_IOCON_BASE + (4 * pin));

  // pin function bits: [2:0] -> 111 = (0x7)
  *reg = (*reg & ~0x7) | (function & 0x7);


}

void pin_mode(PinName pin, PinMode mode) {
    MBED_ASSERT(pin != (PinName)NC);
    
    if ((pin >= P0_23) && (pin <= P0_28)) {
        // The true open-drain pins from PIO0_23 to PIO0_28 can be configured for different I2C-bus speeds.
        return;
    }

	/* Enable AHB clock to the IOCON domain. */
    LPC_SYSCON->AHBCLKCTRLSET[0] = (0x1UL << 13);
	
    __IO uint32_t *reg = (__IO uint32_t*)(LPC_IOCON_BASE + (pin * 4));
      
    if (mode == OpenDrain) {
        *reg |= (0x3 << 7);
    } else {
        uint32_t tmp = *reg;
        tmp &= ~(0x3 << 3);
        tmp |= (mode & 0x3) << 3;
        *reg = tmp;
    }
}
