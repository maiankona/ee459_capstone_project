# Read raw 8-bit unsigned audio data and convert to 4-bit packed format
data = open('dog_bark.raw','rb').read()
packed = bytearray()

for i in range(0, len(data), 2):
    # Convert 8-bit samples (0-255) to 4-bit (0-15)
    # First center around 128, then scale to 4 bits
    sample1 = ((data[i] - 128) // 8) + 8  # Center and scale to 0-15
    # Handle the second sample, or use middle value (8) if at the end
    sample2 = ((data[i+1] - 128) // 8) + 8 if i+1 < len(data) else 8
    
    # Clamp values to 0-15 range
    sample1 = max(0, min(15, sample1))
    sample2 = max(0, min(15, sample2))
    
    # Pack two 4-bit samples into one byte
    packed.append((sample1 << 4) | sample2)

# Write the packed data as a C array
with open('bark_packed.h','w') as f:
    f.write('const uint8_t bark_data[] PROGMEM = {\n')
    for i, b in enumerate(packed):
        f.write(f' 0x{b:02X},')
        if i%16==15: f.write('\n')
    f.write('\n};\n')
