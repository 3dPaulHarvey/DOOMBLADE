import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

import asyncio
import math
from gpiozero import RGBLED, Button
import moteus

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
speed_value = 1.0
speed_mapped = 1.0

# Initialize Moteus controller and GPIO
c = moteus.Controller()
led = RGBLED(red=12, green=13, blue=19)
home_button = Button(6)
extend_button = Button(5)
activate_button = Button(4)
safety_button = Button(3)  

# LED colors
BLUE = (0, 0, 1)
GREEN = (0, 1, 0)
RED = (1, 0, 0)

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

async def check_safety_button():
    if not safety_button.is_pressed:
        print("Safety button not engaged. System is locked.")
        return False
    return True

async def homing():
    obstruction_encountered = False

    await c.set_stop()
    await c.set_rezero(rezero=0.0, query=False)
    await asyncio.sleep(0.1)

    while not home_button.is_pressed:
        c_data = await c.query()
        c_torque = c_data.values.get(moteus.Register.TORQUE, 0)

        if c_torque > 1:
            obstruction_encountered = True
            led.color = RED
            
            await c.set_position(position=math.nan, velocity=-1, kp_scale=1, maximum_torque=None, watchdog_timeout=math.nan)
            await asyncio.sleep(.25) 
            
            c_data = await c.query()
            current_position = c_data.values[moteus.Register.POSITION]
            await c.set_position(position=current_position, velocity=0.0, kp_scale=10, maximum_torque=None, watchdog_timeout=math.nan)
            return obstruction_encountered

        await c.set_position(position=math.nan, velocity=1.0, kp_scale=0.8, maximum_torque=None, watchdog_timeout=math.nan, query=True)
        await asyncio.sleep(0.02)

    led.color = GREEN


    c_data = await c.query()
    home_position = c_data.values[moteus.Register.POSITION]
    await c.set_position(position=home_position, velocity=0.0, kp_scale=10, maximum_torque=None, watchdog_timeout=math.nan)
    await asyncio.sleep(.02)
    
    return obstruction_encountered


async def extend(extend_speed):
    c_data = await c.query()
    position_10 = c_data.values[moteus.Register.POSITION]
    desired_pos = position_10 - 1.75 #1.75
    kp = None
    feedforward = None
    max_torque = 0.0

    while True:
        c_data = await c.query()
        if extend_button.is_pressed:
            led.color = BLUE  # Indicate that the limit switch for extending has been hit
            c_data = await c.query()
            current_position = c_data.values[moteus.Register.POSITION]
            await c.set_position(position=current_position, velocity=0.0, kp_scale=10, maximum_torque=None, watchdog_timeout=math.nan)
            return

        result = await c.set_position(
            position=math.nan,
            stop_position=desired_pos,
            velocity=extend_speed, #15 #10
            maximum_torque=None,
            velocity_limit=30.0,
            watchdog_timeout=math.nan,
            feedforward_torque=feedforward,
            kp_scale=1.3,
            kd_scale=1,
            query=True)

        error_cm = (result.values[moteus.Register.POSITION] - desired_pos)
        if abs(error_cm) < 0.05:
            break

        await asyncio.sleep(0.02)

    #clear fault if we went to fast or over voltage
    c_data = await c.query()
    await asyncio.sleep(0.1)
    await c.set_stop()
    await asyncio.sleep(0.1)
    await c.set_position(position=math.nan, velocity=0.0, kp_scale=10, maximum_torque=None, watchdog_timeout=math.nan)
    

async def sheath(sheath_speed):
    c_data = await c.query()
    position_10 = c_data.values[moteus.Register.POSITION]
    desired_pos = position_10 + 1.55  #1.55 Opposite direction
    obstruction_encountered = False

    while True:
        c_data = await c.query()
        c_torque = c_data.values.get(moteus.Register.TORQUE, 0)

        if c_torque > 1:
            obstruction_encountered = True
            led.color = RED
            
            await c.set_position(position=math.nan, velocity=-1, kp_scale=1, maximum_torque=None, watchdog_timeout=math.nan)
            await asyncio.sleep(.50) 
            
            c_data = await c.query()
            current_position = c_data.values[moteus.Register.POSITION]
            await c.set_position(position=current_position, velocity=0.0, kp_scale=10, maximum_torque=None, watchdog_timeout=math.nan)
            await asyncio.sleep(1) 
            return obstruction_encountered

        result = await c.set_position(
            position=math.nan,
            stop_position=desired_pos,
            velocity=sheath_speed, #15
            maximum_torque=None,
            velocity_limit=30.0,
            watchdog_timeout=math.nan,
            kp_scale=1.3,
            kd_scale=1,
            query=True)

        error_cm = (result.values[moteus.Register.POSITION] - desired_pos)
        if abs(error_cm) < 0.05:
            break
        await asyncio.sleep(0.02)

    # Clear fault if we went too fast or over voltage
    c_data = await c.query()
    await asyncio.sleep(0.1)
    await c.set_stop()
    await asyncio.sleep(0.1)
    await c.set_position(position=math.nan, velocity=0.0, kp_scale=10, maximum_torque=None, watchdog_timeout=math.nan)

    return obstruction_encountered

async def main():
    state = "initial"

    while True:
        # Safety button check
        safety_ok = await check_safety_button()
        if not safety_ok:
            state = "safety_lockout"

        if state == "waiting_to_home":
            print("Waiting to home...")
            led.color = (1, 0.5, 0)  # Orange
            
            # Check if the activate button is pressed to transition to the homing state
            if activate_button.is_pressed:
                state = "homing"
        
        elif state == "homing":
            print("Homing...")
            led.color = (0.5, 0, 0.5)  # Purple
            
            # Call the homing function
            obstruction_encountered = await homing()

            # Transition to the appropriate state after homing
            if obstruction_encountered:
                led.color = (1, 0, 0)  # Red
                state = "waiting_to_home"
            else:
                led.color = (0, 1, 0)  # Green
                state = "homed" 

        elif state == "homed":
            print("System is homed, ready for next command.")
            led.color = (0, 1, 0)  # Green
            #await asyncio.sleep(1)  # Waiting in the homed state
            state = "waiting_to_extend"

        elif state == "waiting_to_extend":
            print("Waiting to extend...")
            led.color = (0, 1, 1)  # Cyan
            
            # Check if the activate button is pressed to transition to the extending state
            if activate_button.is_pressed:
                state = "extending"

        elif state == "extending":
            print("Extending...")
            led.color = (0, 0, 1)  # Blue
            
            # Call the extend function
            await extend(speed_mapped)

            for delayTime in range (1,10):
                await c.set_position(position=math.nan, velocity=0.0, kp_scale=10, maximum_torque=None, watchdog_timeout=math.nan)
                print(delayTime)
                await asyncio.sleep(.1)

            
            # Check if the extend limit switch button is pressed
            if extend_button.is_pressed:
                state = "extended"
            else:
                state = "extend_error"

        elif state == "extended":
            print("System has been successfully extended.")
            led.color = (0, 0, 1)  # Blue
            # Wait for the activate button to be pressed to transition to the sheathing state
            if activate_button.is_pressed:
                state = "sheathing"
            #await asyncio.sleep(0.1)  # Short delay to prevent rapid state changes

        elif state == "sheathing":
            print("Sheathing...")
            led.color = (1, 0.5, 0)  # Orange
            # Call the sheath function
            obstruction_encountered = await sheath(speed_mapped)
            if obstruction_encountered:
                led.color = (1, 0, 0)  # Red
                state = "waiting_to_home"
            else:
                led.color = (0, 1, 0)  # Green
                state = "homing"

        elif state == "extend_error":
            print("Extension error: extend limit switch was not activated.")
            led.color = (1, 1, 0)  # Yellow
            # Handle the error appropriately
            # For now, we'll go back to the initial state
            await asyncio.sleep(0.1)
            state = "initial" 
       
        elif state == "initial":
            print("Initial state. Checking system status...")
            await asyncio.sleep(0.1)
            if not home_button.is_pressed:
                state = "waiting_to_home"
            if home_button.is_pressed:
                state = "homing"            
            # Other conditions can be added here to transition to other states

        elif state == "safety_lockout":
            print("SAFETY LOCKOUT - The system is in a locked state until the safety button is engaged.")
            # Wait in safety lockout state until the safety button is pressed
            while not safety_button.is_pressed:
                await asyncio.sleep(0.1)
            # Once the safety button is engaged again, you can decide to which state to transition
            state = "initial"  # For example, go back to initial state

        else:
            pass

        await asyncio.sleep(0.1)  # Small delay to prevent CPU overload in the loop
        speed_value = mcp.read_adc(0)    
        print(speed_value)
        speed_mapped = map_value(speed_value,0, 1023, 1.0, 12.0)
        print(speed_mapped)

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())


    

