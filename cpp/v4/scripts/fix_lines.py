"""Fix double line endings in gcpc.cpp"""

with open('src/envelope/gcpc.cpp', 'rb') as f:
    raw = f.read()

# The file has \r\r\n between lines due to encoding fix script bug. Fix to \r\n.
fixed = raw.replace(b'\r\r\n', b'\r\n')
print(f'Before: {len(raw)} bytes, After: {len(fixed)} bytes')

# Check for remaining garbled chars
garbled_count = fixed.count('鈺'.encode('utf-8'))
print(f'Remaining garbled chars: {garbled_count}')

with open('src/envelope/gcpc.cpp', 'wb') as f:
    f.write(fixed)

lines = fixed.decode('utf-8').split('\r\n')
print(f'Line count: {len(lines)}')
