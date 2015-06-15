local led0, led1, btn0, btn1

print("hello world!")

led0 = pio.PE_2
led1 = pio.PE_3

btn0 = pio.PB_9
btn1 = pio.PB_10

-- two LEDs gpio set as output
pio.pin.setdir(pio.OUTPUT, led0)
pio.pin.setdir(pio.OUTPUT, led1)

-- Two buttons GPIO set as input
pio.pin.setdir(pio.INPUT, btn0)
pio.pin.setdir(pio.INPUT, btn1)

-- Default LEDs on
pio.pin.sethigh(led0)
pio.pin.sethigh(led1)


-- LED and button test code
-- while uart.getchar(0, 0) == "" do
-- 	if pio.pin.getval(btn0) == 0 then
-- 		pio.pin.setlow(led0)
-- 	else
-- 		if pio.pin.getval(btn1) == 0 then
-- 			pio.pin.setlow(led1)
-- 		else
-- 			pio.pin.sethigh(led0)
-- 			pio.pin.sethigh(led1)
-- 		end
-- 	end
-- end


--  Timer testing code
-- while uart.getchar(0, 0) == "" do
-- 	pio.pin.sethigh(led0)
-- 	tmr.delay(0, 500000)
-- 	pio.pin.setlow(led0)
-- 	tmr.delay(0, 500000)
-- end

-- EM2 wakeup test code

function enter_EM2()
	pio.pin.sethigh(pio.PD_2)
end

function exit_EM2()
	pio.pin.setlow(pio.PD_2)
end

function is_EM2()
	state = 0
	repeats = 0

	while repeats < 3 do
		if state == pio.pin.getval(pio.PD_3) then
			repeats = repeats + 1
			tmr.delay(0, 10000)
		else
			state = pio.pin.getval(pio.PD_3)
			repeats = 0
		end
	end

	if state == 0 then
		return 1
	else
		return 0
	end
end

-- Set PD2 as push-pull, to make target enter EM2 mode
-- Set PD3 as input, to know the target is in EM2 status or not
local count, fails
count = 0
fails = 0

pio.pin.setdir(pio.OUTPUT, pio.PD_2)
pio.pin.setdir(pio.INPUT, pio.PD_3)

while uart.getchar(0, 0) == "" do
	if count == 2000 then
		break;
	end
	while true do
		count = count + 1

		enter_EM2()
		-- tmr.delay(0, 10000)
		if (is_EM2() == 0) then
			fails = fails + 1
			print("Doesn't enter EM2 mode")
			exit_EM2()
			break
		end
		tmr.delay(0, 10000)
		exit_EM2()
		if (is_EM2() == 1) then
			fails = fails + 1
			print("Doesn't exit EM2 mode")
		end
		tmr.delay(0, 10000)
		if (count % 50 == 0) then
			print(string.format("count=%d, fails=%d", count, fails))
		end
		break
	end
end

print(string.format("count=%d, fails=%d", count, fails))
print("Done")

