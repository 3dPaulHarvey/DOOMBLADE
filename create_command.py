import moteus
import math

import asyncio
import math
import moteus

def hexify(data2):
    # print("hexify")
    # print(data)
    # data2 = ''.join(['{:02x}'.format(x) for x in data])
    # print("data2")
    # print(data2)
    return ''.join(['{:02x}'.format(x) for x in data2])


c = moteus.Controller()





# await c.set_stop()
# await c.set_output_exact(position=5.0)



    # #Set Exact
    # print("set exact hex")
    # data2 = c.make_diagnostic_write(b'c cfg-set-exact 0.0\n').data.hex()
    # print(data2)
    # print("set exact hex latin")
    # data3 = c.make_diagnostic_write(b'c cfg-set-exact 0.0\n').data.hex().encode('latin1')
    # print(data3)
    # print("set exact latin hexified")
    # data3 = hexify(data3)
    # print(data3)

    # while True:

    #     state = await c.set_position(position=math.nan, query=True)
    #     #print(state)
    #     print("Position:", state.values[moteus.Register.POSITION])
    #     print()


    #     await asyncio.sleep(0.02)


#Set Exact
print("exact output hex")
print(c.make_set_output_exact(position=1.0).data.hex())
print("exact output hex latin")
data4 = c.make_set_output_exact(position=1.0).data.hex().encode('latin1')
print(data4)
print("exact output latin hexified")
data4 = hexify(data4)
print(data4)

# #Velocity Command 2
# print("make position hex query")
# print(c.make_position(position=math.nan, velocity=0.2, query=True).data.hex())
# print("make position hex latin query")
# data4 = c.make_position(position=math.nan, velocity=0.2, query=True).data.hex().encode('latin1')
# print(data4)
# print("make position latin hexified query")
# data4 = hexify(data4)
# print(data4)

# #Position Command
# print("set position hex")
# print(c.set_position(position=math.nan, velocity=0.2).data.hex())
# print("set position hex latin")
# data4 = c.set_position(position=math.nan, velocity=0.2).data.hex().encode('latin1')
# print(data4)
# print("set position latin hexified")
# data4 = hexify(data4)
# print(data4)

# if __name__ == '__main__':
#     asyncio.run(main())