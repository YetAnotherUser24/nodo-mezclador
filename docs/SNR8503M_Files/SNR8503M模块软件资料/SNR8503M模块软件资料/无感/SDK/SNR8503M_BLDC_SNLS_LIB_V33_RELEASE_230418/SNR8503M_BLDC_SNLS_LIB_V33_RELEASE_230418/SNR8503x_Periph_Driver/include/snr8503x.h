#ifndef _GONI_H_
#define _GONI_H_
//-----------------------------------------------------------
#include "basic.h"
//#include "system_cm0.h"
#define ROM_TAB_BASE        0xF0000000

//-----------------------------------------------------------
#define MISC_BASE           0x40000000

#define SYS_AFE_ADC         REG32(MISC_BASE + 0x00) 
#define SYS_AFE_INFO        REG32(MISC_BASE + 0x04) 
#define SYS_AFE_DBG         REG32(MISC_BASE + 0x08) 
#define SYS_OPA_SEL         REG32(MISC_BASE + 0x0C)

#define SYS_AFE_REG0        REG32(MISC_BASE + 0x10) 
#define SYS_AFE_REG1        REG32(MISC_BASE + 0x14)  
#define SYS_AFE_REG2        REG32(MISC_BASE + 0x18)  
#define SYS_AFE_REG3        REG32(MISC_BASE + 0x1C)  
#define SYS_AFE_REG4        REG32(MISC_BASE + 0x20)  
#define SYS_AFE_REG5        REG32(MISC_BASE + 0x24)  
#define SYS_AFE_REG6        REG32(MISC_BASE + 0x28)  
#define SYS_AFE_DAC         REG32(MISC_BASE + 0x2C)  

#define SYS_CLK_CFG         REG32(MISC_BASE + 0x80)  
#define SYS_IO_CFG          REG32(MISC_BASE + 0x84)  
#define SYS_DBG_CFG         REG32(MISC_BASE + 0x88)  

#define SYS_CLK_DIV0        REG32(MISC_BASE + 0x90)  // SPI clock div
#define SYS_CLK_DIV1        REG32(MISC_BASE + 0x94)  // I2C clock div
#define SYS_CLK_DIV2        REG32(MISC_BASE + 0x98)  // UART0/1 clock div
#define SYS_CLK_FEN         REG32(MISC_BASE + 0x9C)  // perapheral clock enable
#define SYS_TRIM            REG32(MISC_BASE + 0xA0)  
#define SYS_SFT_RST         REG32(MISC_BASE + 0xA4)  
#define SYS_WR_PROTECT      REG32(MISC_BASE + 0xA8)
#define SYS_PROTECT         REG32(MISC_BASE + 0xA8)
#define SYS_PROT            REG32(MISC_BASE + 0xA8)
#define SYS_MBIST           REG32(MISC_BASE + 0xC0)
#define SYS_FLSE            REG32(MISC_BASE + 0xD0)
#define SYS_FLSP            REG32(MISC_BASE + 0xD4)
#define SYS_FLST            REG32(MISC_BASE + 0xD8)
#define SYS_TEST            REG32(MISC_BASE + 0xFC)

#define PSW_SYS_CLR_RST     0xCA40
#define PSW_SYS_SET_IAP     0x3720 // lsb must be ZERO
#define PSW_SYS_PROTECT     0x7a83
#define PSW_PROTECT         0x7a83
#define PSW_PROT            0x7a83

#define FLASH_PASSEN        0xB714AD9C
#define FLASH_PASS_NVR_EN   0x7531FDB9
#define FLASH_PASS_PROG     0x2468ACE0
#define FLASH_PASS_ERASE    0x7654DCBA
#define FLASH_PASS_CONFEN   0x13579BDF

#define FLS_CR_BASE         0x00010000
#define FLASH_CFG           REG32(FLS_CR_BASE + 0x00)
#define FLASH_ADDR          REG32(FLS_CR_BASE + 0x04) 
#define FLASH_WDATA         REG32(FLS_CR_BASE + 0x08)
#define FLASH_RDATA         REG32(FLS_CR_BASE + 0x0C)

#define FLASH_ERASE         REG32(FLS_CR_BASE + 0x10)
#define FLASH_PROTECT       REG32(FLS_CR_BASE + 0x14)
#define FLASH_READY         REG32(FLS_CR_BASE + 0x18)

#define SPI_BASE 	        0x40010000
#define SPI                 ((SPI_TypeDef *) SPI_BASE)

#define SPI_CFG             REG32(SPI_BASE + 0x00)
#define SPI_IE              REG32(SPI_BASE + 0x04)
#define SPI_DIV             REG32(SPI_BASE + 0x08)
#define SPI_TX_DATA         REG32(SPI_BASE + 0x0C)
#define SPI_RX_DATA         REG32(SPI_BASE + 0x10)
#define SPI_SIZE            REG32(SPI_BASE + 0x14)

#define I2C_BASE 	        0x40010100
#define I2C                 ((I2C_TypeDef *) I2C_BASE)

#define I2C_ADDR 	        REG32(I2C_BASE + 0x00) 
#define I2C_CFG 	        REG32(I2C_BASE + 0x04)
#define I2C_SCR 	        REG32(I2C_BASE + 0x08)
#define I2C_DATA 	        REG32(I2C_BASE + 0x0C)
#define I2C_MSCR 	        REG32(I2C_BASE + 0x10)
#define I2C_BCR             REG32(I2C_BASE + 0x14)
#define I2C_BSIZE           REG32(I2C_BASE + 0x18)

#define CMP_BASE            0x40010200
#define CMP                 ((CMP_TypeDef *) CMP_BASE)

#define CMP_IE              REG32(CMP_BASE + 0x00)
#define CMP_IF              REG32(CMP_BASE + 0x04)
#define CMP_TCLK            REG32(CMP_BASE + 0x08)
#define CMP_CFG             REG32(CMP_BASE + 0x0C)
#define CMP_BLCWIN          REG32(CMP_BASE + 0x10)
#define CMP_DATA            REG32(CMP_BASE + 0x14)

#define HALL_BASE 	        0x40010300

#define HALL0               ((HALL_TypeDef *) HALL_BASE)

#define HALL_CFG            REG32(HALL_BASE + 0x00) 
#define HALL_INFO           REG32(HALL_BASE + 0x04) 
#define HALL_WIDTH          REG32(HALL_BASE + 0x08) 
#define HALL_TH             REG32(HALL_BASE + 0x0C)
#define HALL_CNT            REG32(HALL_BASE + 0x10) 

#define ADC_BASE            0x40010400
#define ADC                 ((ADC_TypeDef *) ADC_BASE)
#define ADC_DAT0            SREG16(ADC_BASE + 0x00)
#define ADC_DAT1            SREG16(ADC_BASE + 0x04)
#define ADC_DAT2            SREG16(ADC_BASE + 0x08)
#define ADC_DAT3            SREG16(ADC_BASE + 0x0C)
#define ADC_DAT4            SREG16(ADC_BASE + 0x10)
#define ADC_DAT5            SREG16(ADC_BASE + 0x14)
#define ADC_DAT6            SREG16(ADC_BASE + 0x18)
#define ADC_DAT7            SREG16(ADC_BASE + 0x1C)
#define ADC_DAT8            SREG16(ADC_BASE + 0x20)
#define ADC_DAT9            SREG16(ADC_BASE + 0x24)

#define ADC_LTH             REG32(ADC_BASE + 0x30)
#define ADC_HTH             REG32(ADC_BASE + 0x34)
#define ADC_GEN             REG32(ADC_BASE + 0x38)

#define ADC_CHN0            REG32(ADC_BASE + 0x40)
#define ADC_CHN1            REG32(ADC_BASE + 0x44)
#define ADC_CHN2            REG32(ADC_BASE + 0x48)
#define ADC_CHNT            REG32(ADC_BASE + 0x50)
#define ADC_CFG             REG32(ADC_BASE + 0x54)
#define ADC_SWT             REG32(ADC_BASE + 0x58)

#define ADC_DC              REG32(ADC_BASE + 0x60)
#define ADC_AMC             REG32(ADC_BASE + 0x64)
#define ADC_IE              REG32(ADC_BASE + 0x68)
#define ADC_IF              REG32(ADC_BASE + 0x6C)

#define TIMER0_BASE         0x40010500
#define TIMER1_BASE         0x40010600

#define TIMER0              ((TIM_TimerTypeDef *) TIMER0_BASE)
#define TIMER1              ((TIM_TimerTypeDef *) TIMER1_BASE)

#define UTIMER0_CFG         REG32(TIMER0_BASE + 0x00)
#define UTIMER0_TH          REG32(TIMER0_BASE + 0x04)
#define UTIMER0_CNT         REG32(TIMER0_BASE + 0x08)
#define UTIMER0_CMP0        REG32(TIMER0_BASE + 0x0C)
#define UTIMER0_CMP1        REG32(TIMER0_BASE + 0x10)
#define UTIMER0_EVT         REG32(TIMER0_BASE + 0x14)
#define UTIMER0_FLT         REG32(TIMER0_BASE + 0x18)
#define UTIMER0_IE          REG32(TIMER0_BASE + 0x1C)
#define UTIMER0_IF          REG32(TIMER0_BASE + 0x20)

#define UTIMER1_CFG         REG32(TIMER1_BASE + 0x00)
#define UTIMER1_TH          REG32(TIMER1_BASE + 0x04)
#define UTIMER1_CNT         REG32(TIMER1_BASE + 0x08)
#define UTIMER1_CMP0        REG32(TIMER1_BASE + 0x0C)
#define UTIMER1_CMP1        REG32(TIMER1_BASE + 0x10)
#define UTIMER1_EVT         REG32(TIMER1_BASE + 0x14)
#define UTIMER1_FLT         REG32(TIMER1_BASE + 0x18)
#define UTIMER1_IE          REG32(TIMER1_BASE + 0x1C)
#define UTIMER1_IF          REG32(TIMER1_BASE + 0x20)

#define MCPWM_BASE          0x40010700
#define MCPWM0              ((MCPWM_REG_TypeDef *) MCPWM_BASE)

#define MCPWM_TH00          SREG16(MCPWM_BASE + 0x00)
#define MCPWM_TH01          SREG16(MCPWM_BASE + 0x04)
#define MCPWM_TH10          SREG16(MCPWM_BASE + 0x08)
#define MCPWM_TH11          SREG16(MCPWM_BASE + 0x0C)
#define MCPWM_TH20          SREG16(MCPWM_BASE + 0x10)
#define MCPWM_TH21          SREG16(MCPWM_BASE + 0x14)
#define MCPWM_TH30          SREG16(MCPWM_BASE + 0x18)
#define MCPWM_TH31          SREG16(MCPWM_BASE + 0x1C)
#define MCPWM_TMR0          SREG16(MCPWM_BASE + 0x20)
#define MCPWM_TMR1          SREG16(MCPWM_BASE + 0x24)
#define MCPWM_TMR2          SREG16(MCPWM_BASE + 0x28)
#define MCPWM_TMR3          SREG16(MCPWM_BASE + 0x2C)
#define MCPWM_TH0           REG32(MCPWM_BASE + 0x30)
#define MCPWM_TH1           REG32(MCPWM_BASE + 0x34)
#define MCPWM_CNT0          REG32(MCPWM_BASE + 0x38)
#define MCPWM_CNT1          REG32(MCPWM_BASE + 0x3C)
#define MCPWM_UPDATE        REG32(MCPWM_BASE + 0x40)
#define MCPWM_FCNT          REG32(MCPWM_BASE + 0x44)
#define MCPWM_EVT0          REG32(MCPWM_BASE + 0x48)
#define MCPWM_EVT1          REG32(MCPWM_BASE + 0x4C)
#define MCPWM_DTH0          REG32(MCPWM_BASE + 0x50)
#define MCPWM_DTH1          REG32(MCPWM_BASE + 0x54)

#define MCPWM_FLT           REG32(MCPWM_BASE + 0x70)
#define MCPWM_SDCFG         REG32(MCPWM_BASE + 0x74)
#define MCPWM_AUEN          REG32(MCPWM_BASE + 0x78)
#define MCPWM_TCLK          REG32(MCPWM_BASE + 0x7C)
#define MCPWM_IE0           REG32(MCPWM_BASE + 0x80)
#define MCPWM_IF0           REG32(MCPWM_BASE + 0x84)
#define MCPWM_IE1           REG32(MCPWM_BASE + 0x88)
#define MCPWM_IF1           REG32(MCPWM_BASE + 0x8C)
#define MCPWM_EIE           REG32(MCPWM_BASE + 0x90)
#define MCPWM_EIF           REG32(MCPWM_BASE + 0x94)
#define MCPWM_RE            REG32(MCPWM_BASE + 0x98)
#define MCPWM_PP            REG32(MCPWM_BASE + 0x9C)
#define MCPWM_IO01          REG32(MCPWM_BASE + 0xA0)
#define MCPWM_IO23          REG32(MCPWM_BASE + 0xA4)
                        
#define MCPWM_FAIL012       REG32(MCPWM_BASE + 0xA8)
#define MCPWM_FAIL3         REG32(MCPWM_BASE + 0xAC)
#define MCPWM_PRT           REG32(MCPWM_BASE + 0xB0)
#define MCPWM_SWAP          REG32(MCPWM_BASE + 0xB4)
#define MCPWM_CH_MASK       REG32(MCPWM_BASE + 0xB8)

#define GPIO0               ((GPIO_TypeDef *) GPIO0_BASE)
#define GPIO1               ((GPIO_TypeDef *) GPIO1_BASE)

#define GPIO0_BASE          0x40010800

#define GPIO0_PIE           REG32(GPIO0_BASE + 0x00)
#define GPIO0_POE           REG32(GPIO0_BASE + 0x04)
#define GPIO0_PDI           REG32(GPIO0_BASE + 0x08)
#define GPIO0_PDO           REG32(GPIO0_BASE + 0x0C)
#define GPIO0_PUE           REG32(GPIO0_BASE + 0x10)
#define GPIO0_PODE          REG32(GPIO0_BASE + 0x18)
#define GPIO0_PFLT          REG32(GPIO0_BASE + 0x1C)
#define GPIO0_F3210         REG32(GPIO0_BASE + 0x20)
#define GPIO0_F7654         REG32(GPIO0_BASE + 0x24)
#define GPIO0_FBA98         REG32(GPIO0_BASE + 0x28)
#define GPIO0_FFEDC         REG32(GPIO0_BASE + 0x2C)
#define GPIO0_BSRR          REG32(GPIO0_BASE + 0x30)
#define GPIO0_BRR           REG32(GPIO0_BASE + 0x34)

#define GPIO1_BASE          0x40010840
#define GPIO1_PIE           REG32(GPIO1_BASE + 0x00)
#define GPIO1_POE           REG32(GPIO1_BASE + 0x04)
#define GPIO1_PDI           REG32(GPIO1_BASE + 0x08)
#define GPIO1_PDO           REG32(GPIO1_BASE + 0x0C)
#define GPIO1_PUE           REG32(GPIO1_BASE + 0x10)
#define GPIO1_PODE          REG32(GPIO1_BASE + 0x18)
#define GPIO1_PFLT          REG32(GPIO1_BASE + 0x1C)
#define GPIO1_F3210         REG32(GPIO1_BASE + 0x20)
#define GPIO1_F7654         REG32(GPIO1_BASE + 0x24)
#define GPIO1_FBA98         REG32(GPIO1_BASE + 0x28)
#define GPIO1_FFEDC         REG32(GPIO1_BASE + 0x2C)
#define GPIO1_BSRR          REG32(GPIO1_BASE + 0x30)
#define GPIO1_BRR           REG32(GPIO1_BASE + 0x34)

#define EXTI_BASE           0x40010880
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)

#define EXTI_CR0            REG32(EXTI_BASE  + 0x00)
#define EXTI_CR1            REG32(EXTI_BASE  + 0x04)
#define EXTI_IE             REG32(EXTI_BASE  + 0x08)
#define EXTI_IF             REG32(EXTI_BASE  + 0x0C)
#define CLKO_SEL            REG32(EXTI_BASE  + 0x10)

#define UART_BASE           0x40010900

#define UART0               ((UART_TypeDef *) UART_BASE)

#define UART_CTRL           REG32(UART_BASE + 0x00) 
#define UART_DIVH           REG32(UART_BASE + 0x04)
#define UART_DIVL           REG32(UART_BASE + 0x08)
#define UART_BUFF           REG32(UART_BASE + 0x0C)
#define UART_ADR            REG32(UART_BASE + 0x10)
#define UART_STT            REG32(UART_BASE + 0x14)
#define UART_RE             REG32(UART_BASE + 0x18)
#define UART_IE             REG32(UART_BASE + 0x1C)
#define UART_IF             REG32(UART_BASE + 0x20)
#define UART_IOC            REG32(UART_BASE + 0x24)

#define DMA_BASE            0x40010A00
#define DMA_CCR0            REG32(DMA_BASE + 0x00)
#define DMA_REN0            REG32(DMA_BASE + 0x04)
#define DMA_CTMS0           REG32(DMA_BASE + 0x08)
#define DMA_SADR0           REG32(DMA_BASE + 0x0C)
#define DMA_DADR0           REG32(DMA_BASE + 0x10)

#define DMA_CCR1            REG32(DMA_BASE + 0x20)
#define DMA_REN1            REG32(DMA_BASE + 0x24)
#define DMA_CTMS1           REG32(DMA_BASE + 0x28)
#define DMA_SADR1           REG32(DMA_BASE + 0x2C)
#define DMA_DADR1           REG32(DMA_BASE + 0x30)

#define DMA_CCR2            REG32(DMA_BASE + 0x40)
#define DMA_REN2            REG32(DMA_BASE + 0x44)
#define DMA_CTMS2           REG32(DMA_BASE + 0x48)
#define DMA_SADR2           REG32(DMA_BASE + 0x4C)
#define DMA_DADR2           REG32(DMA_BASE + 0x50)

#define DMA_CCR3            REG32(DMA_BASE + 0x60)
#define DMA_REN3            REG32(DMA_BASE + 0x64)
#define DMA_CTMS3           REG32(DMA_BASE + 0x68)
#define DMA_SADR3           REG32(DMA_BASE + 0x6C)
#define DMA_DADR3           REG32(DMA_BASE + 0x70)

#define DMA_CTRL            REG32(DMA_BASE + 0x80)
#define DMA_IE              REG32(DMA_BASE + 0x84)
#define DMA_IF              REG32(DMA_BASE + 0x88)

#define DSP_BASE            0x40010B00
#define DSP_DID             REG32(DSP_BASE + 0x20)
#define DSP_DIS             REG32(DSP_BASE + 0x24)
#define DSP_QUO             REG32(DSP_BASE + 0x28)
#define DSP_REM             REG32(DSP_BASE + 0x2C)
#define DSP_RAD             REG32(DSP_BASE + 0x30)
#define DSP_SQRT            REG32(DSP_BASE + 0x34)

#define AON_BASE            0x40010C00
#define IWDG_PSW            REG32(AON_BASE + 0x00)
#define IWDG_CFG            REG32(AON_BASE + 0x04)
#define IWDG_CLR            REG32(AON_BASE + 0x08)
#define IWDG_WTH            REG32(AON_BASE + 0x0C)
#define IWDG_RTH            REG32(AON_BASE + 0x10)
#define IWDG_CNT            REG32(AON_BASE + 0x14)
#define PSW_IWDG_PRE        0xA6B4
#define PSW_IWDG_CLR        0x798D
#define IWDG               ((IWDG_TypeDef *) AON_BASE)

#define AON_PWR_CFG         REG32(AON_BASE + 0x20)
#define AON_EVT_RCD         REG32(AON_BASE + 0x24)
#define AON_IO_WAKE_POL     REG32(AON_BASE + 0x28)
#define AON_IO_WAKE_EN      REG32(AON_BASE + 0x2C)
#define PSW_EVT_CLR         0xCA40

// SYS_CLK_FEN
#define MCLK_RCG_B_I2C      0
#define MCLK_RCG_B_HALL     1
#define MCLK_RCG_B_UART     2
#define MCLK_RCG_B_CMP      3
#define MCLK_RCG_B_MCPWM    4
#define MCLK_RCG_B_TIMER0   5
#define MCLK_RCG_B_TIMER1   6
#define MCLK_RCG_B_GPIO     7
#define MCLK_RCG_B_DSP      8
#define MCLK_RCG_B_ADC      9
#define MCLK_RCG_B_SPI      10
#define MCLK_RCG_B_DMA      11

#endif // _GONI_H_
