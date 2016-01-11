#include "gd32f1x0.h"

void clk_init(void)
{
// disable external oscillator so we can use pins PF0 , PF1 
RCC_HSEConfig(RCC_HSE_OFF);
// this should be enabled already 
RCC_HSI_Enable(ENABLE);
// wait for HSI to stabilize
// this is redundant
while( !RCC_GetBitState(RCC_FLAG_HSISTB) );
// this should be enabled already 
RCC_CK_SYSConfig(RCC_SYSCLKSOURCE_HSI);

RCC_PLLConfig(RCC_PLLSOURCE_HSI_DIV2 , RCC_PLLMUL_12);
RCC_PLL_Enable(ENABLE);

// wait for pll to stablilize
// this is redundant
while( !RCC_GetBitState(RCC_FLAG_PLLSTB) );
RCC_CK_SYSConfig(RCC_SYSCLKSOURCE_PLLCLK);

// wait for clock to change to pll
while ((RCC->GCFGR & (uint32_t)RCC_GCFGR_SCSS) != (uint32_t)RCC_GCFGR_SCSS_PLL);

RCC_AHBConfig(RCC_SYSCLK_DIV1); // config ahb clock divider
RCC_APB1Config(RCC_APB1AHB_DIV1);
RCC_APB2Config(RCC_APB2AHB_DIV1);

			
}

