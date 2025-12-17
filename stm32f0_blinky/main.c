#define PERIPH_BASE (0X40000000UL)

#define AHB2PERIPH_OFFSET    (0X08000000UL)

#define AHB2PERIPH_BASE   (PERIPH_BASE + AHB2PERIPH_OFFSET)

#define GPIOC_OFFSET       (0X00000800UL)

#define GPIOC_BASE        (AHB2PERIPH_BASE + GPIOC_OFFSET)

#define AHB1PERIPH_OFFSET     (0X00020000UL)

#define AHB1PERIPH_BASE       (PERIPH_BASE + AHB1PERIPH_OFFSET)

#define RCC_OFFSET           (0X00001000UL)

#define  RCC_BASE            (AHB1PERIPH_BASE + RCC_OFFSET)

#define AHBEN_R_OFFSET        (0X14UL)
#define  RCC_AHBEN_R        (*(volatile unsigned int *)(RCC_BASE + AHBEN_R_OFFSET))

#define MODE_R_OFFSET        (0x00UL)
#define GPIOC_MODE_R        (*(volatile unsigned int *)(GPIOC_BASE + MODE_R_OFFSET))

#define OD_R_OFFSET          (0X14UL)
#define GPIOC_OD_R            (*(volatile unsigned int *)(GPIOC_BASE + OD_R_OFFSET))

#define   IOPCEN             (1U<<19) // ob 0000 0000 0000 1000 0000 0000 0000 0000

#define PIN9                 (1U<<9)
#define LED_PIN               PIN9

#define PIN8                 (1U<<8)
#define LED_PIN1               PIN8
/* (1U<<18)  // set bit 18 to 1
 *  &=~(1U<<19) //set bit 19 to 0
 */

int main()
{
	// 1. Enable clock access to GPIOC
		RCC_AHBEN_R |= IOPCEN;

		//2. Set PC8 as output pin
				GPIOC_MODE_R |= (1U<<16);
				GPIOC_MODE_R &=~(1U<<17);
	//2. Set PC9 as output pin
		GPIOC_MODE_R |= (1U<<18);
		GPIOC_MODE_R &=~(1U<<19);

	while(1)
	{
		//3. set pc9 high
		//GPIOC_OD_R |=LED_PIN;

		// 4. Toggle pc9
		GPIOC_OD_R ^=LED_PIN;
		for(int i=0;i<100000;i++){}
		GPIOC_OD_R ^=LED_PIN1;
		for(int i=0;i<200000;i++){}
	}
}

