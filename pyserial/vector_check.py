import struct

with open("blinky.bin", "rb") as f:
    data = f.read()

# Read first 8 bytes (SP and Reset vector)
sp = struct.unpack('<I', data[0:4])[0]
reset = struct.unpack('<I', data[4:8])[0]

print(f"Stack Pointer: 0x{sp:08X}")
print(f"Reset Handler: 0x{reset:08X}")

# Validate
if sp < 0x20000000 or sp > 0x20002000:
    print("❌ INVALID STACK POINTER!")
else:
    print("✓ Stack pointer looks valid")

if not (reset & 0x1):
    print("❌ INVALID RESET VECTOR (Thumb bit not set)!")
else:
    print("✓ Reset vector has Thumb bit set")
    print(f"  Actual address: 0x{reset & ~0x1:08X}")