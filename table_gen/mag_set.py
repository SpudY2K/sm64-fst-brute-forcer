import itertools as itool
import numpy as np
import math
import struct


def to_float32(x: float):
    return struct.unpack('f', struct.pack('f', x))[0]

magCheck = [False for _ in range(4097)]
magSet = []

for (x, y) in itool.product(range(-128, 128, 1), range(-128, 128, 1)):
    xs = x - 6 if x >= 8 else (x + 6 if x <= -8 else 0)
    ys = y - 6 if y >= 8 else (y + 6 if y <= -8 else 0)
    
    mag2 = min(xs*xs + ys*ys, 4096)
    magCheck[mag2] = True

for (i, x) in enumerate(magCheck):
    if x != 0:
        mag = np.float32(math.sqrt(i))
        mag = (mag / np.float32(64.0)) * (mag / 64.0) * np.float32(32.0)
        magSet.append(mag)

print(f"constant float magSet[{len(magSet)}] = {{")
print(", ".join([f"{i:10f}" for i in magSet]))
print("}")
        