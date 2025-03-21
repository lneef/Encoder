#!/usr/bin/python3
import time
import sys
import mmap

def test_read(file_name: str):
    with open(file_name, 'r+b') as f:
        mmio = mmap.mmap(f.fileno(), 16)
        try:
            while True:
                print(mmio[:8])
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            mmio.close()

if __name__ == '__main__':
    test_read(sys.argv[1])
