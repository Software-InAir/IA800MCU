#include "stm32h723xx.h"

static void ltdc_bringup(void)
{

    RCC->APB3ENR |= RCC_APB3ENR_LTDCEN;
    
    
        // Sync
    LTDC->SSCR =
        ((H_SYNC - 1) << 16) |
        ((V_SYNC - 1) << 0);

    // Back porch
    LTDC->BPCR =
        ((H_SYNC + H_BACK - 1) << 16) |
        ((V_SYNC + V_BACK - 1) << 0);

    // Active
    LTDC->AWCR =
        ((H_SYNC + H_BACK + H_ACTIVE - 1) << 16) |
        ((V_SYNC + V_BACK + V_ACTIVE - 1) << 0);

    // Total
    LTDC->TWCR =
        ((H_SYNC + H_BACK + H_ACTIVE + H_FRONT - 1) << 16) |
        ((V_SYNC + V_BACK + V_ACTIVE + V_FRONT - 1) << 0);

    LTDC_Layer1->WHPCR =
        ((H_SYNC + H_BACK + H_ACTIVE - 1) << 16) |
        (H_SYNC + H_BACK);

    LTDC_Layer1->WVPCR =
        ((V_SYNC + V_BACK + V_ACTIVE - 1) << 16) |
        (V_SYNC + V_BACK);

    // Disable PLL3
    RCC->CR &= ~RCC_CR_PLL3ON;
    while (RCC->CR & RCC_CR_PLL3RDY);

    // M = 4 → 2 MHz input
    // N = 160 → 320 MHz VCO
    // R = 8 → 40 MHz pixel clock

    RCC->PLLCKSELR &= ~RCC_PLLCKSELR_DIVM3;
    RCC->PLLCKSELR |=  (4 << RCC_PLLCKSELR_DIVM3_Pos);

    RCC->PLL3DIVR =
        ((160 - 1) << RCC_PLL3DIVR_N3_Pos) |
        ((8   - 1) << RCC_PLL3DIVR_R3_Pos);

    RCC->PLLCFGR |= RCC_PLLCFGR_DIVR3EN;

    RCC->CR |= RCC_CR_PLL3ON;
    while (!(RCC->CR & RCC_CR_PLL3RDY));

    RCC->D1CCIPR &= ~RCC_D1CCIPR_LTDCSEL;
    RCC->D1CCIPR |=  (0x02 << RCC_D1CCIPR_LTDCSEL_Pos); // PLL3_R

    LTDC->GCR =
        (0 << LTDC_GCR_HSPOL_Pos) |
        (0 << LTDC_GCR_VSPOL_Pos) |
        (1 << LTDC_GCR_DEPOL_Pos) |   // <-- TRY THIS FIRST
        (0 << LTDC_GCR_PCPOL_Pos);

        
        LTDC_Layer1->DCCR = 0x00000000;

        LTDC_Layer1->BFCR = (0x6 << 8) | (0x7); // typical: CA + PA

        LTDC_Layer1->CFBAR = FB;  // framebuffer address
        LTDC_Layer1->CFBLR =
                        ((960 * 2) << 16) |   // pitch (bytes)
                        ((960 * 2) + 3);      // line length   // line length
        LTDC_Layer1->CFBLNR = 412;  // number of lines
        LTDC_Layer1->PFCR = 0x2;   // pixel format

        LTDC_Layer1->CR |= LTDC_LxCR_LEN;

        LTDC->SRCR = LTDC_SRCR_IMR;  // immediate reload
    
        
        LTDC->GCR |= LTDC_GCR_LTDCEN;


}

static void ltdc_gpio_setup(void)
{
    /////////////////////////////////////////////////////////// CLOCK setup
    /* Enable clocks */
    RCC->AHB4ENR  |= RCC_AHB4ENR_GPIOAEN |
                     RCC_AHB4ENR_GPIOBEN |
                     RCC_AHB4ENR_GPIOCEN |
                     RCC_AHB4ENR_GPIOGEN;

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /////////////////////////////////////////////////////////// GPIO setup

    // GPIO Pins (Blue channel)
    GPIOG->MODER &= ~(3 << (14 * 2));  ////// Blue 0
    GPIOG->MODER |=  (2 << (14 * 2));
    GPIOG->OSPEEDR &= ~(3 << (14 * 2));
    GPIOG->OSPEEDR |= (3 << (14 * 2));
    GPIOG->AFR[1] &= ~(0xF << ((14 - 8) * 4)); 
    GPIOG->AFR[1] |=  (0xE << ((14 - 8) * 4));

    GPIOA->MODER &= ~(3 << (10 * 2));   /// Blue 1
    GPIOA->MODER |=  (2 << (10 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (10 * 2));
    GPIOA->OSPEEDR |= (3 << (10 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((10 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0xE << ((10 - 8) * 4));

    GPIOC->MODER &= ~(3 << (9 * 2));   //// Blue 2
    GPIOC->MODER |=  (2 << (9 * 2)); 
    GPIOC->OSPEEDR &= ~(3 << (9 * 2));
    GPIOC->OSPEEDR |= (3 << (9 * 2));
    GPIOC->AFR[1] &= ~(0xF << ((9 - 8) * 4)); 
    GPIOC->AFR[1] |=  (0xE << ((9 - 8) * 4)); 

    GPIOA->MODER &= ~(3 << (8 * 2));   //// Blue 3
    GPIOA->MODER |=  (2 << (8 * 2));
    GPIOA->OSPEEDR &= ~(3 << (8 * 2));
    GPIOA->OSPEEDR |= (3 << (8 * 2)); 
    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0xD << ((8 - 8) * 4)); 

    GPIOC->MODER &= ~(3 << (11 * 2));   /// Blue 4
    GPIOC->MODER |=  (2 << (11 * 2)); 
    GPIOC->OSPEEDR &= ~(3 << (11 * 2));
    GPIOC->OSPEEDR |= (3 << (11 * 2));
    GPIOC->AFR[1] &= ~(0xF << ((11 - 8) * 4)); 
    GPIOC->AFR[1] |=  (0xE << ((11 - 8) * 4)); 

    GPIOA->MODER &= ~(3 << (3 * 2));  ///// Blue 5
    GPIOA->MODER |=  (2 << (3 * 2));
    GPIOA->OSPEEDR &= ~(3 << (3 * 2));
    GPIOA->OSPEEDR |= (3 << (3 * 2)); 
    GPIOA->AFR[0] &= ~(0xF << ((3) * 4)); 
    GPIOA->AFR[0] |=  (0xE << ((3) * 4)); 


    // GPIO Pins (Red channel)

    GPIOG->MODER &= ~(3 << (13 * 2));  ////// Red 0
    GPIOG->MODER |=  (2 << (13 * 2));
    GPIOG->OSPEEDR &= ~(3 << (13 * 2));
    GPIOG->OSPEEDR |= (3 << (13 * 2));
    GPIOG->AFR[1] &= ~(0xF << ((13 - 8) * 4)); 
    GPIOG->AFR[1] |=  (0xE << ((13 - 8) * 4));

    GPIOA->MODER &= ~(3 << (2 * 2));  ////// Red 1
    GPIOA->MODER |=  (2 << (2 * 2));
    GPIOA->OSPEEDR &= ~(3 << (2 * 2));
    GPIOA->OSPEEDR |= (3 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << ((2) * 4)); 
    GPIOA->AFR[0] |=  (0xD << ((2) * 4));

    GPIOC->MODER &= ~(3 << (10 * 2));  ///// Red 2
    GPIOC->MODER |=  (2 << (10 * 2)); 
    GPIOC->OSPEEDR &= ~(3 << (10 * 2));
    GPIOC->OSPEEDR |= (3 << (10 * 2));
    GPIOC->AFR[1] &= ~(0xF << ((10 - 8) * 4)); 
    GPIOC->AFR[1] |=  (0xE << ((10 - 8) * 4));

    GPIOA->MODER &= ~(3 << (15 * 2));   //// Red 3
    GPIOA->MODER |=  (2 << (15 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (15 * 2));
    GPIOA->OSPEEDR |= (3 << (15 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0x9 << ((15 - 8) * 4));

    GPIOA->MODER &= ~(3 << (11 * 2));  //// Red 4
    GPIOA->MODER |=  (2 << (11 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (11 * 2));
    GPIOA->OSPEEDR |= (3 << (11 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((11 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0xE << ((11 - 8) * 4));

    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |=  (2 << (9 * 2));
    GPIOA->OSPEEDR &= ~(3 << (9 * 2));
    GPIOA->OSPEEDR |= (3 << (9 * 2)); 
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0xE << ((9 - 8) * 4));


    // GPIO Pins (Green channel)

    GPIOB->MODER &= ~(3 << (0 * 2));   //// Green 1
    GPIOB->MODER |=  (2 << (0 * 2));
    GPIOB->OSPEEDR &= ~(3 << (0 * 2));
    GPIOB->OSPEEDR |= (3 << (0 * 2));
    GPIOB->AFR[0] &= ~(0xF << ((0) * 4)); 
    GPIOB->AFR[0] |=  (0xE << ((0) * 4));

    GPIOB->MODER &= ~(3 << (1 * 2));  //// Green 0
    GPIOB->MODER |=  (2 << (1 * 2));
    GPIOB->OSPEEDR &= ~(3 << (1 * 2));
    GPIOB->OSPEEDR |= (3 << (1 * 2));
    GPIOB->AFR[0] &= ~(0xF << ((1) * 4)); 
    GPIOB->AFR[0] |=  (0xE << ((1) * 4));

    GPIOA->MODER &= ~(3 << (6 * 2)); //// Green 2
    GPIOA->MODER |=  (2 << (6 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (6 * 2));
    GPIOA->OSPEEDR |= (3 << (6 * 2));
    GPIOA->AFR[0] &= ~(0xF << ((6) * 4)); 
    GPIOA->AFR[0] |=  (0xE << ((6) * 4));

    GPIOG->MODER &= ~(3 << (10 * 2));  /// Green 3
    GPIOG->MODER |=  (2 << (10 * 2));
    GPIOG->OSPEEDR &= ~(3 << (10 * 2));
    GPIOG->OSPEEDR |= (3 << (10 * 2));
    GPIOG->AFR[1] &= ~(0xF << ((10 - 8) * 4)); 
    GPIOG->AFR[1] |=  (0x9 << ((10 - 8) * 4)); 

    GPIOB->MODER &= ~(3 << (10 * 2)); /// Green 4
    GPIOB->MODER |=  (2 << (10 * 2)); 
    GPIOB->OSPEEDR &= ~(3 << (10 * 2));
    GPIOB->OSPEEDR |= (3 << (10 * 2));
    GPIOB->AFR[1] &= ~(0xF << ((10 - 8) * 4)); 
    GPIOB->AFR[1] |=  (0xE << ((10 - 8) * 4));

    GPIOB->MODER &= ~(3 << (11 * 2)); // Green 5
    GPIOB->MODER |=  (2 << (11 * 2)); // AF MODE
    GPIOB->OSPEEDR &= ~(3 << (11 * 2));
    GPIOB->OSPEEDR |= (3 << (11 * 2)); // HIGH SPEED
    GPIOB->AFR[1] &= ~(0xF << ((11 - 8) * 4)); // Alternate Function clear
    GPIOB->AFR[1] |=  (0xE << ((11 - 8) * 4)); // Alternate Functoon AF13 (LTDC)

    ////////////////////////////////////////////////// LTDC Clock Signals

    GPIOC->MODER &= ~(3 << (6 * 2));  //// HSYNC
    GPIOC->MODER |=  (2 << (6 * 2));
    GPIOC->OSPEEDR &= ~(3 << (6 * 2));
    GPIOC->OSPEEDR |= (3 << (6 * 2));
    GPIOC->AFR[0] &= ~(0xF << ((6) * 4)); 
    GPIOC->AFR[0] |=  (0xE << ((6) * 4));

    GPIOA->MODER &= ~(3 << (4 * 2)); //// VSYNC
    GPIOA->MODER |=  (2 << (4 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (4 * 2));
    GPIOA->OSPEEDR |= (3 << (4 * 2));
    GPIOA->AFR[0] &= ~(0xF << ((4) * 4)); 
    GPIOA->AFR[0] |=  (0xE << ((4) * 4));

    GPIOB->MODER &= ~(3 << (14 * 2));  /// LCD_CLK
    GPIOB->MODER |=  (2 << (14 * 2));
    GPIOB->OSPEEDR &= ~(3 << (14 * 2));
    GPIOB->OSPEEDR |= (3 << (14 * 2));
    GPIOB->AFR[1] &= ~(0xF << ((14 - 8) * 4)); 
    GPIOB->AFR[1] |=  (0xE << ((14 - 8) * 4)); 

    GPIOC->MODER &= ~(3 << (5 * 2)); /// LCD_DE
    GPIOC->MODER |=  (2 << (5 * 2)); 
    GPIOC->OSPEEDR &= ~(3 << (5 * 2));
    GPIOC->OSPEEDR |= (3 << (5 * 2));
    GPIOC->AFR[0] &= ~(0xF << ((5) * 4)); 
    GPIOC->AFR[0] |=  (0xE << ((5) * 4));
}