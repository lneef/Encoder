import os
import select
from typing import Callable

DATA_LEN = 4

class IRQDescriptor:

    def __init__(self, fname :str, callback: Callable[[int], None]) -> None:
        self.fd = os.open(fname, os.O_RDWR | os.O_NONBLOCK);
        self.callback = callback
        self.enable_irq(self.fd)

    def handle_irq(self) -> None:
        while True:
            try:
                irqno = os.read(self.fd, DATA_LEN)
                self.callback(int(irqno))
            except BlockingIOError:
                pass

    @staticmethod
    def enable_irq(fd: int) -> None:
        while True:
            try: 
                os.write(fd, b'0001')
            except BlockingIOError:
                pass



def poll_irq(self, fds: list[IRQDescriptor]) -> None:
    p = select.epoll()
    for fd in fds:
        p.register(fd.fd, select.EPOLLIN | select.EPOLLPRI)
    while True:
        events = self.p.poll()
        for fd, event in events:
            if event & select.EPOLLIN or event & select.EPOLLPRI:
                fd.handle_irq()
            elif event & select.EPOLLHUP:
                p.unregister(fd.fd)
                fd.close()
                fds.remove(fd)
            elif event & select.EPOLLERR:
                p.unregister(fd.fd)
                fd.close()
                fds.remove(fd)
                raise Exception("Error on {fd.fd}")





