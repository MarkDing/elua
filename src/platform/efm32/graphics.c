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
#include "platform_conf.h"
#include "lrotable.h"
#include "buf.h"
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "lrotable.h"
#include "graphics.h"

// Platform includes
#include <bsp.h>
#include <glib.h>
#include "dmd/ssd2119/dmd_ssd2119.h"
#include "dmd/ssd2119/dmd_ssd2119_registers.h"

#define PSRAM 		 0x84000000
#define PSRAM_SIZE	 0x3FFFFF

#define FRAMEBUFFER_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT * 2)

extern GLIB_Context_t    gc;

/**
 * @brief Fill rectagle on screen
 * @details
 *
 * @param L lua parameters list
 * @return 0: sucess; others: Lua error
 */
static int disp_fillRect(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc != 5) return luaL_error(L, "wrong number of arguments!");

	GLIB_Rectangle_t rect;

	rect.xMin = luaL_checkint(L, 1);
	rect.yMin = luaL_checkint(L, 2);
	rect.xMax = rect.xMin + luaL_checkint(L, 3);
	rect.yMax = rect.yMin + luaL_checkint(L, 4);
	gc.foregroundColor = luaL_checkint(L, 5);

	GLIB_drawRectFilled(&gc, &rect);

	return 0;
}

/**
 * @brief Draw strings on screen
 * @details
 *
 * @param L lua parameters list
 * @return 0: sucess; others: Lua error
 */
static int disp_drawString(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc != 4 ) return luaL_error(L, "wrong number of arguments!");

	int x = luaL_checkint(L, 1);
	int y = luaL_checkint(L, 2);
	const char *str = luaL_checkstring(L, 3);
	gc.foregroundColor = luaL_checkint(L, 4);
	int len = strlen(str);

	GLIB_drawString(&gc, (char *)str, len, x, y, 1);

	return 0;
}

/**
 * @brief Draw line on screen
 * @details
 *
 * @param L lua parameters list
 * @return 0: sucess; others: Lua error
 */
static int disp_drawLine(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc != 5) return luaL_error(L, "wrong number of arguments!");

	int x1 = luaL_checkint(L, 1);
	int y1 = luaL_checkint(L, 2);
	int x2 = luaL_checkint(L, 3);
	int y2 = luaL_checkint(L, 4);
	gc.foregroundColor = luaL_checkint(L, 5);

	GLIB_drawLine(&gc, x1, y1, x2, y2);

	return 0;
}

/**
 * @brief Draw Bitmap on screen
 * @details
 *
 * @param L lua parameters list
 * @return 0: sucess; others: Lua error
 */
static int disp_drawBitmap(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc != 5) return luaL_error(L, "wrong number of arguments!");

	int x1 = luaL_checkint(L, 1);
	int y1 = luaL_checkint(L, 2);
	int width = luaL_checkint(L, 3);
	int height = luaL_checkint(L, 4);
	const char *img = luaL_checkstring(L, 5);

	GLIB_drawBitmap(&gc, x1, y1, width, height, (uint8_t *)img);

	return 0;
}

/**
 * @brief Draw pixel on screen
 * @details
 *
 * @param L lua parameters list
 * @return 0: sucess; others: Lua error
 */
static int disp_drawPixel(lua_State *L)
{
	int argc = lua_gettop(L);
	if (argc != 3) return luaL_error(L, "wrong number of arguments!");
	int x = luaL_checkint(L, 1);
	int y = luaL_checkint(L, 2);
	gc.foregroundColor = luaL_checkint(L, 3);

	GLIB_drawPixel(&gc, x, y);
	return 0;
}


/**
 * @brief clear screen
 * @details
 *
 * @param L lua parameters list
 * @return 0: sucess; others: Lua error
 */
static int disp_clear(lua_State *L)
{
	GLIB_clear(&gc);
	return 0;
}


#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

// Module function map
const LUA_REG_TYPE disp_map[] = {
	{ LSTRKEY("blit"), LFUNCVAL(disp_drawBitmap) },
	{ LSTRKEY("clear"), LFUNCVAL(disp_clear) },
	{ LSTRKEY("drawBitmap"), LFUNCVAL(disp_drawBitmap) },
	{ LSTRKEY("drawLine"), LFUNCVAL(disp_drawLine) },
	{ LSTRKEY("fillRect"), LFUNCVAL(disp_fillRect) },
	{ LSTRKEY("pixel"), LFUNCVAL(disp_drawPixel) },
	{ LSTRKEY("print"), LFUNCVAL(disp_drawString) },
	{ LNILKEY, LNILVAL }
};

