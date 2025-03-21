import time
import sys
import mmap

def test_read(file_name: str):
    with open(file_name, 'w+') as f:
        mmio = mmap.mmap(f.fileno(), 16)
        while True:
            try:
                print(mmio[:8])
            except KeyboardInterrupt:
                break
            time.sleep(1)

        mmio.close()


if __name__ == '__main__':
    test_read(sys.argv[1])
