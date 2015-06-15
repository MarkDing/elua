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
#include <bsp.h>
#include <em_usart.h>
#include <em_timer.h>
#include <tftamapped.h>
#include <glib.h>
#include "graphics.h"

/** Graphics context */
GLIB_Context_t    gc;

#define SYSTICKHZ             10

static void clock_init(void);
static void pio_init(void);
static void uart_init(void);
static void disp_init(void);
static void timer_init(void);
uint32_t cmsis_get_cpu_frequency();
static void peripheral_test(void);


// ****************************************************************************
// Platform initialization

int platform_init()
{
	/* Chip errota */
	CHIP_Init();
	/* Initialize DK board register access */

	BSP_Init(BSP_INIT_DEFAULT);

	// Set up microcontroller system and SystemCoreClock variable
	SystemInit();

	clock_init();
	pio_init();
	uart_init();
	disp_init();
	timer_init();
	// peripheral_test();

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
 * @brief Display initialization function
 * @details Initialize EFM32GG-DK3750 TFT display driver.
 */
static void disp_init()
{
	static char *efm32_hello = "48MHz / SSD2119 address mapped TFT\n";
	GLIB_Rectangle_t rect = {
		.xMin =   0,
		.yMin =   0,
		.xMax = 319,
		.yMax = 239,
	};

	TFT_AddressMappedInit();
	GLIB_contextInit(&gc);
	/* Clear framebuffer */
	gc.foregroundColor = GLIB_rgbColor(20, 40, 20);
	GLIB_drawRectFilled(&gc, &rect);
	GLIB_drawString(&gc, efm32_hello, strlen(efm32_hello), 0, 0, 1);
}

static void pio_init(void)
{
	BSP_RegisterWrite(BSP_LED_PORT, 0);
}

static void LEDs_ToggleLEDs(const uint8_t LEDMask)
{
	uint32_t tmp;
	tmp = BSP_RegisterRead(BSP_LED_PORT) & 0xFF;
	tmp ^= 0xFF;
	BSP_RegisterWrite(BSP_LED_PORT, tmp);
}

static void uart_init(void)
{
	USART_TypeDef          *uart = UART1;
	USART_InitAsync_TypeDef init  = USART_INITASYNC_DEFAULT;

	/* Set TXD pin to push-pull, RXD pin to input.
	 To avoid false start, configure output as high */
	GPIO_PinModeSet(gpioPortB, 9, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);

	/* Enable EFM32GG_DK3750 RS232/UART switch */
	BSP_PeripheralAccess(BSP_RS232_UART, true);

	/* Configure UART for basic async operation */
	init.enable = usartDisable;
	USART_InitAsync(uart, &init);

	/* Enable pins at UART1 location #2 */
	uart->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC2;

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

int BSP_LedSetMask(int ledMask)
{
	uint32_t tmp;

	if ((ledMask >= 0) && (ledMask < BSP_LED_MASK)) {
		tmp = BSP_RegisterRead(BSP_LED_PORT) & BSP_LED_MASK;
		tmp |= ledMask;
		BSP_RegisterWrite(BSP_LED_PORT, tmp);
		return BSP_STATUS_OK;
	}
	return BSP_STATUS_ILLEGAL_PARAM;
}

int BSP_LedClearMask(int ledMask)
{
	uint32_t tmp;

	if ((ledMask >= 0) && (ledMask < BSP_LED_MASK)) {
		tmp = BSP_RegisterRead(BSP_LED_PORT) & BSP_LED_MASK;
		tmp &= ~(ledMask);
		BSP_RegisterWrite(BSP_LED_PORT, tmp);
		return BSP_STATUS_OK;
	}
	return BSP_STATUS_ILLEGAL_PARAM;
}

int BSP_LedGetMask(int ledMask)
{
	if ((ledMask >= 0) && (ledMask < BSP_LED_MASK)) {
		if (BSP_RegisterRead(BSP_LED_PORT) & BSP_LED_MASK & (1 << ledMask))
			return 1;

		return 0;
	}
	return BSP_STATUS_ILLEGAL_PARAM;
}
// ****************************************************************************
// PIO section
// We make virtual port for DK3750 board,
// Port0:[0-31] LEDs[0-15], JoyStick[DLURC], PB1-PB4, DIP

// The platform I/O functions
pio_type platform_pio_op(unsigned port, pio_type pinmask, int op)
{
	pio_type retval = 1;

	switch (op) {
	case PLATFORM_IO_PORT_SET_VALUE:
		// GPIO_PortOutSet(port, pinmask);
		break;

	case PLATFORM_IO_PIN_SET:
		BSP_LedSetMask(pinmask);
		// GPIO_PinOutSet(port, pinmask);
		break;

	case PLATFORM_IO_PIN_CLEAR:
		BSP_LedClearMask(pinmask);
		// GPIO_PinOutClear(port, pinmask);
		break;

	case PLATFORM_IO_PORT_DIR_OUTPUT:
		// GPIO_PinModeSet(port, 0xFFFF, gpioModePushPull, 0);
		break;

	case PLATFORM_IO_PIN_DIR_OUTPUT:
		// GPIO_PinModeSet(port, pinmask, gpioModePushPull, 1);
		break;

	case PLATFORM_IO_PORT_DIR_INPUT:
		// GPIO_PinModeSet(port, 0xFFFF, gpioModeInputPull, 1);
		break;

	case PLATFORM_IO_PIN_DIR_INPUT:
		// GPIO_PinModeSet(port, pinmask, gpioModeInputPull, 1);
		break;

	case PLATFORM_IO_PORT_GET_VALUE:
		// retval = GPIO_PortInGet(port);
		break;

	case PLATFORM_IO_PIN_GET:
		BSP_LedsInit();
		if (pinmask < (1 << 16)) { // LED
			retval = BSP_LedGetMask(pinmask);
		} else if (pinmask < (1 << (16 + 5))) {  // Joystick
			retval = BSP_JoystickGet() & (pinmask >> 16);
		} else if (pinmask < (1 << (16 + 5 + 4))) { // Buttons
			retval = BSP_PushButtonsGet() & (pinmask >> (16 + 5));
		}
		if (retval) {
			BSP_LedSetMask(pinmask >> 16);
		}
		// retval = GPIO_PinInGet(port, pinmask);
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

static USART_TypeDef *const uart[] = {UART1, UART1};

uint32_t platform_uart_setup(unsigned id, uint32_t baud, int databits, int parity, int stopbits)
{
	USART_InitAsync_TypeDef init   = USART_INITASYNC_DEFAULT;

	/* Enable EFM32GG_DK3750 RS232/UART switch */
	BSP_PeripheralAccess(BSP_RS232_UART, true);

	/* Configure UART for basic async operation */
	init.enable = usartDisable;

	USART_InitAsync(uart[id], &init);

	/* Enable pins at UART1 location #2 */
	uart[id]->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC2;

	/* Finally enable it */
	USART_Enable(uart[id], usartEnable);

	return baud; // FIXME: find a way to actually get baud
}

void platform_s_uart_send(unsigned id, u8 data)
{
	USART_Tx(uart[id], data);
}

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

/* 13671 Hz -> 14Mhz (clock frequency) / 1024 (prescaler) */
#define ONE_SECOND_COUNT 13671
static TIMER_TypeDef *tmr[] = {TIMER0, TIMER1, TIMER2};

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

// Helper function: get timer clock
static uint32_t platform_timer_get_clock(unsigned id)
{
	uint32_t clk, pre;
	pre = (tmr[id]->CTRL & 0x0F000000) >> 24;
	clk = SystemHFClockGet() / (1 << pre);
	return clk;
}

// Helper function: set timer clock
static uint32_t platform_timer_set_clock(unsigned id, uint32_t clk)
{
	uint32_t factor, pre, i;
	uint32_t preset[11] = {0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};
	factor = SystemHFClockGet() / clk;
	for (i = 0; i < 11; i++) {
		if (factor > preset[i]) {
			pre = i;
		} else {
			break;
		}
	}
	/* Select TIMER parameters */
	TIMER_Init_TypeDef timerInit = {
		.enable     = true,
		.debugRun   = true,
		.prescale   = pre,
		.clkSel     = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode       = timerModeUp,
		.dmaClrAct  = false,
		.quadModeX4 = false,
		.oneShot    = false,
		.sync       = false,
	};

	clk = SystemHFClockGet() / (1 << pre);
	/* Configure TIMER */
	TIMER_Init(tmr[id], &timerInit);
	return clk;
}


void platform_s_timer_delay(unsigned id, timer_data_type delay_us)
{
	timer_data_type final;
	final = ((u64)delay_us * platform_timer_get_clock(id)) / 1000000;
	TIMER_CounterSet(tmr[id], 0);
	while (TIMER_CounterGet(tmr[id]) < final);
}

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
		res = platform_timer_set_clock(id, data);
		break;

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

static void peripheral_test(void)
{
	while (1) {
		LEDs_ToggleLEDs(0x55aa);
		platform_s_timer_delay(0, 1000000);
	}
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



