#!/usr/bin/python3
import sys

DATA_LEN = 4

IRQ_ENABLE = 0x1
IRQ_DISABLE = 0x0


def enable_irq(fd):
    fd.write(int.to_bytes(IRQ_ENABLE, DATA_LEN, 'little'))

def disable_irq(fd):
    fd.write(int.to_bytes(IRQ_DISABLE, DATA_LEN, 'little'))

def test_irq(name: str) -> None:
    with open(name, "wb+", buffering=0) as uio:
        while True:
            irq_cnt = uio.read(DATA_LEN)
            irq_value = int.from_bytes(irq_cnt)

            print(f'Read Interrupt: {irq_value}\n')

if __name__ == "__main__":
    filename = sys.argv[1]
    try:
        test_irq(filename)
    except KeyboardInterrupt:
        pass





