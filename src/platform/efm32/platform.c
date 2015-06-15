// Platform-dependent functions

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include "stacks.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "utils.h"
#include "common.h"
#include "elua_adc.h"
#include "platform_conf.h"
#include "lrotable.h"
#include "buf.h"

// Platform includes
#include <em_usart.h>
#include <em_timer.h>

#define SYSTICKHZ             10

static void clock_init(void);
static void pio_init(void);
static void uart_init(void);
static void timer_init(void);
uint32_t cmsis_get_cpu_frequency();


// ****************************************************************************
// Platform initialization

int platform_init()
{
	/* Chip errota */
	CHIP_Init();

	// Set up microcontroller system and SystemCoreClock variable
	SystemInit();

	clock_init();
	uart_init();
	timer_init();

	// System timer setup
	cmn_systimer_set_base_freq(SystemCoreClockGet());
	cmn_systimer_set_interrupt_freq(SYSTICKHZ);

	// Enable SysTick
	SystemCoreClockGet();
	SysTick_Config(SystemCoreClockGet() / SYSTICKHZ);

	NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
	// Common platform initialization code
	cmn_platform_init();

	return PLATFORM_OK;
}

uint32_t platform_s_cpu_get_frequency()
{
	return SystemCoreClockGet();
}

uint32_t cmsis_get_cpu_frequency()
{
	return SystemCoreClockGet();
}

/**
 * @brief Enable HFX0:48MHz for system clock
 * @details Enable HFX0:48MHz for system clock. Also enable clocks to
 * 			USART1, UART1, GPIO, TIME0, TIME1, TIME2
 */
static void clock_init(void)
{
	/* Enable HFXO */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

	/* Enable clock to GPIO, USART1, UART, TIMER0 */
	CMU->HFPERCLKEN0 |= (CMU_HFPERCLKEN0_USART1 | CMU_HFPERCLKEN0_GPIO |
	                     CMU_HFPERCLKEN0_UART1  | CMU_HFPERCLKEN0_TIMER0 |
	                     CMU_HFPERCLKEN0_TIMER1 | CMU_HFPERCLKEN0_TIMER2);

}


/**
 * @brief USART1 initialization
 * @details Init USART1 as 115200 baud rate, PD0-TXD as push pull mode,
 * 			PD1-RXD as input
 */
static void uart_init(void)
{
	USART_TypeDef          *uart = USART1;
	USART_InitAsync_TypeDef init  = USART_INITASYNC_DEFAULT;

	/* Set TXD pin to push-pull, RXD pin to input.
	 To avoid false start, configure output as high */
	GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 1);

	/* Configure UART for basic async operation */
	init.enable = usartDisable;
	USART_InitAsync(uart, &init);

	/* Enable pins at USART1 location #1 */
	uart->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC1;

	/* Finally enable it */
	USART_Enable(uart, usartEnable);
}

// SysTick interrupt handler
void SysTick_Handler()
{
	// Handle virtual timers
	cmn_virtual_timer_cb();

	// Handle system timer call
	cmn_systimer_periodic();
}


// ****************************************************************************
// PIO section

/**
 * @brief Convert pinmask to pin number
 * @details The pinmask is bit mask of the GPIO register, we need to change it
 *     to pin number so that low level GPIO function can use it directly.
 *
 * @param pinmask: bit mask of the GPIO register.
 * @return pin number
 */
pio_type convert_pinmask2pin(pio_type pinmask)
{
	pio_type i;
	for (i = 0; i < 16; i++) {
		if (pinmask & 0x01) {
			break;
		}
		pinmask >>= 1;
	}
	return i;
}

/**
 * @brief PIO port or pin operation
 * @details Set/clear pin/port status and set direction of the port/pin
 *
 * @param port: Port number
 * @param pinmask: The pin mask of each port
 * @param op: The operation of the pin/port
 * @return pin/port value whe op is get value. return 0 for no supported op.
 */
pio_type platform_pio_op(unsigned port, pio_type pinmask, int op)
{
	pio_type retval = 1, pin;

	pin = convert_pinmask2pin(pinmask);
	switch (op) {
	// pin setting
	case PLATFORM_IO_PIN_SET:
		GPIO_PinOutSet(port, pin);
		break;
	case PLATFORM_IO_PIN_CLEAR:
		GPIO_PinOutClear(port, pin);
		break;
	case PLATFORM_IO_PIN_DIR_OUTPUT:
		GPIO_PinModeSet(port, pin, gpioModePushPull, 1);
		break;
	case PLATFORM_IO_PIN_DIR_INPUT:
		GPIO_PinModeSet(port, pin, gpioModeInputPull, 1);
		break;
	case PLATFORM_IO_PIN_GET:
		retval = GPIO_PinInGet(port, pin);
		break;
	// Port setting
	case PLATFORM_IO_PORT_SET_VALUE:
		GPIO_PortOutSetVal(port, pinmask, 0xFFFF);
		break;
	case PLATFORM_IO_PORT_DIR_OUTPUT:
		GPIO->P[port].DOUTSET = 0xFFFF;
		// Set as push-pull for each pin, the value is 0x04
		GPIO->P[port].MODEL = 0x44444444;
		GPIO->P[port].MODEH = 0x44444444;
		break;
	case PLATFORM_IO_PORT_DIR_INPUT:
		GPIO->P[port].DOUTSET = 0xFFFF;
		// set as input-pull for each pin, the value is 0x02
		GPIO->P[port].MODEL = 0x22222222;
		GPIO->P[port].MODEH = 0x22222222;
		break;
	case PLATFORM_IO_PORT_GET_VALUE:
		retval = GPIO_PortInGet(port);
		break;
	default:
		retval = 0;
		break;
	}
	return retval;
}


// ****************************************************************************
// UART section

// USART1: RX = PD1, TX = PD0
// The other UARTs have assignable Rx/Tx pins and thus have to be configured
// by the user

static USART_TypeDef *const uart[] = {USART1, USART1};

uint32_t platform_uart_setup(unsigned id, uint32_t baud, int databits, int parity, int stopbits)
{
	USART_InitAsync_TypeDef init   = USART_INITASYNC_DEFAULT;

	/* Configure UART for basic async operation */
	init.enable = usartDisable;

	USART_InitAsync(uart[id], &init);

	/* Enable pins at USART1 location #1 */
	uart[id]->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC1;

	/* Finally enable it */
	USART_Enable(uart[id], usartEnable);

	return baud; // FIXME: find a way to actually get baud
}

/**
 * @brief UART send byte data
 * @details UART send byte data
 *
 * @param id: UART index
 * @param data: byte data
 */
void platform_s_uart_send(unsigned id, u8 data)
{
	USART_Tx(uart[id], data);
}

/**
 * @brief UART receive byte data
 * @details UART receive byte data
 *
 * @param id: UART index
 * @param timeout: if it is 0, it will return even no data ready. If it is none
 * 	zero value, it will wait for data coming for ever.
 *
 * @return received byte data
 */
int platform_s_uart_recv(unsigned id, timer_data_type timeout)
{
	u8 buffer;

	if (timeout == 0) {
		if (!(uart[id]->STATUS & USART_STATUS_RXDATAV)) {
			return -1;
		} else {
			return (int)(uart[id]->RXDATA);
		}
	}
	buffer = USART_Rx(uart[id]);
	return (int)buffer;
}

int platform_s_uart_set_flow_control(unsigned id, int type)
{
	return PLATFORM_OK;
}

// ****************************************************************************
// Timer section

static TIMER_TypeDef *tmr[] = {TIMER0, TIMER1, TIMER2};

/**
 * @brief Timer init function
 * @details Init all timers, the HFXO is 48Mhz, the timer is 16-bit, 64K maximum
 * 	value, so prescale 1024 is the best option, 48Mhz/1024= 48K.
 */
static void timer_init(void)
{
	int32_t i;
	/* Select TIMER parameters */
	TIMER_Init_TypeDef timerInit = {
		.enable     = true,
		.debugRun   = true,
		.prescale   = timerPrescale1024,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};

	for (i = 0; i < NUM_TIMER; i++) {
		TIMER_TopSet(tmr[i], 0xFFFF);
		/* Configure TIMER */
		TIMER_Init(tmr[i], &timerInit);
	}
}

/**
 * @brief Get timer clock
 * @details get timer clock according the value in bit field PRESC of CTRL
 * register.
 *
 * @param id: Timer index
 * @return the timer clock
 */
static uint32_t platform_timer_get_clock(unsigned id)
{
	uint32_t clk, pre;
	pre = (tmr[id]->CTRL & 0x0F000000) >> 24;
	clk = SystemHFClockGet() / (1 << pre);
	return clk;
}


/**
 * @brief Timer delay function,
 * @details it delays in micro second.
 *
 * @param id: timer index
 * @param delay_us time to be delayed
 */
void platform_s_timer_delay(unsigned id, timer_data_type delay_us)
{
	timer_data_type final;
	final = ((u64)delay_us * platform_timer_get_clock(id)) / 1000000;
	TIMER_CounterSet(tmr[id], 0);
	while (TIMER_CounterGet(tmr[id]) < final);
}

/**
 * @brief Timer operation
 * @details Timer start, read, max value.
 *
 * @param id: Timer index
 * @param op: The operation of the Timer
 * @param data: it is useless in this function
 * @return value of each operation.
 */
timer_data_type platform_s_timer_op(unsigned id, int op, timer_data_type data)
{
	uint32_t res = 0;

	switch (op) {
	case PLATFORM_TIMER_OP_START:
		TIMER_Enable(tmr[id], 1);
		break;

	case PLATFORM_TIMER_OP_READ:
		res = TIMER_CounterGet(tmr[id]);
		break;

	case PLATFORM_TIMER_OP_SET_CLOCK:
	case PLATFORM_TIMER_OP_GET_CLOCK:
		res = platform_timer_get_clock(id);
		break;

	case PLATFORM_TIMER_OP_GET_MAX_CNT:
		res = 0xFFFF;
		break;
	}
	return res;
}

u64 platform_timer_sys_raw_read()
{
	return SysTick->LOAD - SysTick->VAL;
}

void platform_timer_sys_disable_int()
{
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

void platform_timer_sys_enable_int()
{
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

timer_data_type platform_timer_read_sys()
{
	return cmn_systimer_get();
}


// ****************************************************************************
// Platform specific modules go here

#ifdef PS_LIB_TABLE_NAME

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

#ifdef BUILD_LCD
extern const LUA_REG_TYPE disp_map[];
#endif
#ifdef BUILD_RTC
extern const LUA_REG_TYPE rtc_map[];
#endif

#if LUA_OPTIMIZE_MEMORY == 0
#define LROVAL(x)    x
#endif

const LUA_REG_TYPE platform_map[] = {
#if LUA_OPTIMIZE_MEMORY > 0
#ifdef BUILD_LCD
	{ LSTRKEY("disp"), LROVAL(disp_map) },
#endif
#ifdef BUILD_RTC
	{ LSTRKEY("rtc"), LROVAL(rtc_map) },
#endif
#endif
	{ LNILKEY, LNILVAL }
};


LUALIB_API int luaopen_platform(lua_State *L)
{
#if LUA_OPTIMIZE_MEMORY > 0
	return 0;
#else // #if LUA_OPTIMIZE_MEMORY > 0
	luaL_register(L, PS_LIB_TABLE_NAME, platform_map);

	// Setup the new tables inside platform table
#ifdef BUILD_LCD
	lua_newtable(L);
	luaL_register(L, NULL, disp_map);
	lua_setfield(L, -2, "disp");
#endif
#ifdef BUILD_RTC
	lua_newtable(L);
	luaL_register(L, NULL, rtc_map);
	lua_setfield(L, -2, "rtc");
#endif

	return 1;
#endif // #if LUA_OPTIMIZE_MEMORY > 0
}


#endif // #ifdef PS_LIB_TABLE_NAME



