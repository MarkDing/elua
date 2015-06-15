-- eLua blinking led example, the Hello World of embedded :)

local uartid, invert, ledpin = 0, false

print("hello world!")
if pd.board() == "EFM32GG-DK3750" then
  ledpin = pio.PA_3
else
  print( "\nError: Unknown board " .. pd.board() .. " !" )
  return
end

function cycle()
  pio.pin.sethigh( ledpin )
  tmr.delay( 0, 500000 )
  pio.pin.setlow( ledpin )
  tmr.delay( 0, 500000 )
end

-- pio.pin.setdir( pio.OUTPUT, ledpin )
print( "Hello from eLua on " .. pd.board() )
print "Watch your LED blinking :)"
print "Enjoy eLua !"
print "Press any key to end this demo.\n"

-- efm32.disp.print(0,50, "Hello from lua!", 0xff0000)

keys = {
	{ "C", "sin", "cos", "tan" },
	{ "1/x", "x^2", "sqr", "/" },
	{ "7", "8", "9", "*" },
	{ "4", "5", "6", "-" },
	{ "1", "2", "3", "+" },
	{ "0", "+/-", ".", "=" } }

color = 0x00FF00

function drawRect(x0, y0, w, h)
	efm32.disp.drawLine(x0, y0, x0+w, y0, color)
	efm32.disp.drawLine(x0, y0, x0, y0+h, color)
	efm32.disp.drawLine(x0+w, y0, x0+w, y0+h, color)
	efm32.disp.drawLine(x0+w, y0+h, x0, y0+h, color)
end

yk = 0
w = 40
h = 30
text = "0"
drawRect(w, 0, w * 4, h-2)
efm32.disp.print(w+4, 4, text, color)
for yindex,line in ipairs(keys) do
	for xindex,key in ipairs(line) do
		x0 = xindex * w
		y0 = yindex * h
		drawRect(x0, y0, w, h)
		if xindex == x and yindex == y then
			efm32.disp.fillRect(x0, y0, w, h, color)
			foreground = 0x000000
		else
			foreground = color
		end
		efm32.disp.print(x0 + 5, y0 + 5, key, foreground)
	end
end


clockOfs = 120
clockWidth = 100
clockTextPosition = 85
clockBigMarkWidth = 7
clockSmallMarkWidth = 3
x0 = clockOfs
y0 = clockOfs - clockWidth
pi = 4*math.atan(1)
color = 0xFF0000
for i=0,60 do
	x1 = math.sin(pi-i/60*2*pi) * clockWidth + clockOfs
	y1 = math.cos(pi-i/60*2*pi) * clockWidth + clockOfs
	efm32.disp.drawLine(x0, y0, x1, y1, color)
	xv = (x1 - clockOfs) / clockWidth
	yv = (y1 - clockOfs) / clockWidth
	if math.mod(i, 5) == 0 then
		xt = xv * clockTextPosition + clockOfs
		yt = yv * clockTextPosition + clockOfs
		value = math.ceil(i / 5)
		if value == 0 then
			value = 12
		end
		efm32.disp.print(xt, yt, value, color)
		xv = xv * (clockWidth - clockBigMarkWidth) + clockOfs
		yv = yv * (clockWidth - clockBigMarkWidth) + clockOfs
		efm32.disp.drawLine(x1, y1, xv, yv, color)
	else
		xv = xv * (clockWidth - clockSmallMarkWidth) + clockOfs
		yv = yv * (clockWidth - clockSmallMarkWidth) + clockOfs
		efm32.disp.drawLine(x1, y1, xv, yv, color)
	end
	x0 = x1
	y0 = y1
end
efm32.disp.print(4, 4, "os.date: ", color)
efm32.disp.print(4, 14, "digital: ", color)

while uart.getchar( uartid, 0 ) == "" do
  cycle()
end

-- efm32.disp.clear()
