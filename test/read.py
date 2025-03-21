#!/usr/bin/python3
import time
import sys
import mmap

FILE_SIZE = 4096

def test_read(file_name: str):
    with open(file_name, 'r+b') as f:
        mmio = mmap.mmap(f.fileno(), FILE_SIZE, mmap.MAP_SHARED)
        try:
            while True:
                print(int.from_bytes(mmio[:8], byteorder='little', signed=True))
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            mmio.close()

if __name__ == '__main__':
    test_read(sys.argv[1])
