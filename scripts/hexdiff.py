"""Compare two firmware bin files and show differences."""
import os
import sys

build = r'cmake_builds\tc_ble_simple_b80_sdk\TC32-GCC_Toolchain'
file1 = os.path.join(build, 'b80_ble_cat_laser_toy.bin')
file2 = os.path.join(build, 'b80_ble_cat_laser_toy_ABC.bin')

with open(file1, 'rb') as f:
    a = f.read()
with open(file2, 'rb') as f:
    b = f.read()

print(f'File 1 (OLD): {len(a)} bytes - {os.path.basename(file1)}')
print(f'File 2 (NEW): {len(b)} bytes - {os.path.basename(file2)}')
print()

# Show firmware size at offset 0x18
fw_size_a = int.from_bytes(a[0x18:0x1C], 'little')
fw_size_b = int.from_bytes(b[0x18:0x1C], 'little')
print(f'fw_size @0x18: OLD=0x{fw_size_a:06X}({fw_size_a})  NEW=0x{fw_size_b:06X}({fw_size_b})')
print()

# Show last 16 bytes of each file
print(f'OLD last 16 bytes: {a[-16:].hex()}')
print(f'NEW last 16 bytes: {b[-16:].hex()}')
print()

# If sizes differ, show extra bytes
if len(b) > len(a):
    print(f'NEW has {len(b)-len(a)} extra bytes at end: {b[len(a):].hex()}')
    if len(b) - len(a) == 4:
        crc = int.from_bytes(b[len(a):], 'little')
        print(f'  → As u32 LE: 0x{crc:08X} (looks like a CRC32!)')
elif len(a) > len(b):
    print(f'OLD has {len(a)-len(b)} extra bytes at end: {a[len(b):].hex()}')
print()

# Show all differences with context
print('All byte differences:')
print(f'{"Offset":>8}  {"OLD":>8}  {"NEW":>8}  {"Description":>20}')
print('-' * 50)
min_len = min(len(a), len(b))
count = 0
for i in range(min_len):
    if a[i] != b[i]:
        desc = ''
        if i == 0x18:
            desc = 'firmware_size field'
        elif 0x170 <= i <= 0x175:
            desc = 'code area (branch offset?)'
        elif 0x6F10 <= i <= 0x6F15:
            desc = 'code area'
        elif 0xAD50 <= i <= 0xAD60:
            desc = 'near version string?'
        elif i >= min_len - 20:
            desc = 'near file end'
        print(f'  0x{i:06X}    0x{a[i]:02X}     0x{b[i]:02X}    {desc}')
        count += 1
        if count > 20:
            print(f'  ... ({count} total differences, showing first 20)')
            break

# Show the version string area (usually near the end of .rodata)
print()
print('Looking for version string in both files...')
for name, data in [('OLD', a), ('NEW', b)]:
    # Find "Get firmware version" strings
    idx = data.find(b'Get firmware version')
    if idx >= 0:
        ctx = data[max(0,idx-4):idx+40]
        print(f'  {name} @0x{idx:06X}: {ctx}')
