import asyncio
import math
import time
import moteus
async def main():
    c = moteus.Controller()
    c_data = await c.query()
    print (c_data)
    await c.set_stop()
    print (await c.set_stop(query=True))

asyncio.run(main())