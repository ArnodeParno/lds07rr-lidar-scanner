import sys, lds
d = open(sys.argv[1], 'rb').read()
i = 0; rows = []; valid = total = 0
while i < len(d) - 22:
    if d[i] == 0xFA and 0xA0 <= d[i+1] <= 0xF9 and lds.neato_checksum(d[i:i+22]) == (d[i+20] | d[i+21] << 8):
        p = d[i:i+22]; rows.append((p[2] | p[3] << 8) / 64)
        for k in range(4):
            total += 1; valid += not (p[5+k*4] & 0x80)
        i += 22
    else:
        i += 1
n = len(rows)
print(len(d), "bytes, 0xAA status packets:", d.count(b'\xaa\x2a\x00'))
for part in range(10):
    seg = rows[part*n//10:(part+1)*n//10]
    if seg: print(f"part {part+1}/10: {sum(seg)/len(seg):6.0f} rpm")
print(f"packets {n}, valid measurements {valid}/{total} ({100*valid/max(total,1):.0f}%)")
