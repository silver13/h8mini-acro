#include "gd32f1x0.h"
#include "drv_adc.h"
#include "util.h"

uint16_t adcarray[4];

void adc_init(void)
{	  
    DMA_InitPara DMA_InitStructure;
    ADC_InitPara ADC_InitStructure;
    
    RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_ADC1, ENABLE);	  

    RCC_ADCCLKConfig(RCC_ADCCLK_APB2_DIV6);

    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);		     
	
    DMA_InitStructure.DMA_PeripheralBaseAddr = 0x4001244C; 
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcarray; 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PERIPHERALSRC;  
    DMA_InitStructure.DMA_BufferSize = 4;	
    DMA_InitStructure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;	
    DMA_InitStructure.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_HALFWORD; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_HALFWORD; 
    DMA_InitStructure.DMA_Mode = DMA_MODE_CIRCULAR; 
    DMA_InitStructure.DMA_Priority = DMA_PRIORITY_HIGH;
    DMA_InitStructure.DMA_MTOM = DMA_MEMTOMEM_DISABLE; 
		
    DMA_Init(DMA1_CHANNEL1, &DMA_InitStructure);	 
  
    DMA_Enable(DMA1_CHANNEL1, ENABLE);  
		
    ADC_InitStructure.ADC_Mode_Scan = ENABLE;		 
    ADC_InitStructure.ADC_Mode_Continuous = ENABLE;  
    ADC_InitStructure.ADC_Trig_External = ADC_EXTERNAL_TRIGGER_MODE_NONE;  
    ADC_InitStructure.ADC_Data_Align = ADC_DATAALIGN_RIGHT;	
    ADC_InitStructure.ADC_Channel_Number = 4;  
		
    ADC_Init(&ADC_InitStructure);  

    ADC_RegularChannel_Config(ADC_CHANNEL_7, 1, ADC_SAMPLETIME_239POINT5);
		// battery channel
    ADC_RegularChannel_Config(ADC_CHANNEL_5, 2, ADC_SAMPLETIME_239POINT5);

    ADC_RegularChannel_Config(ADC_CHANNEL_4, 3, ADC_SAMPLETIME_239POINT5);
    ADC_RegularChannel_Config(ADC_CHANNEL_1, 4, ADC_SAMPLETIME_239POINT5);

    ADC_DMA_Enable(ENABLE);	 
    
    ADC_Enable(ENABLE); 
		
    ADC_Calibration();
    
    ADC_SoftwareStartConv_Enable(ENABLE);  
}

float adc_read(int channel)
{
	switch(channel)
	{
		case 0:
		return adcarray[0] ;	
		
		case 1:
		return mapf( (float) adcarray[1] , 2727.0 , 3050.0 , 3.77 , 4.22);
		
		case 2:
		return adcarray[2];	
		
		case 3:
		return adcarray[3];	
		
		default:
			
	  return 0;
	}
	
	
}
