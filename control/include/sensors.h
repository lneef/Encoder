#pragma once

#include <array>
#include <cstdint>
#include <fcntl.h>
#include <string_view>
#include <sys/epoll.h>
#include <sys/mman.h>
#include <unistd.h>

struct SensorData {
  int64_t cartPosition, pendulumAngle;
  bool leftBoundary, rightBoundary;
};

class EncoderController {
public:
  EncoderController(std::string_view path, std::array<int64_t, 2> init = {})
      : psize(_SC_PAGESIZE) {
    int fd = open(path.data(), O_RDWR);
    regs = reinterpret_cast<volatile MMIO *>(
        mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
    regs->encoder[0].position = init[0];
    regs->encoder[1].position = init[1];
#ifdef __aarch64__
    #include <arm_acle.h>
    __dmb(_ARM_BARRIER_SY);
#endif
    close(fd);
  }
  EncoderController(EncoderController &&other)
      : psize(other.psize), regs(other.regs) {
    other.regs = nullptr;
  }
  ~EncoderController() {
    if (regs) {
      munmap(reinterpret_cast<void *>(const_cast<MMIO *>(regs)), psize);
    }
  }

  void handle_impl(SensorData &data) {
    data.cartPosition = regs->encoder[0].value;
    data.pendulumAngle = regs->encoder[1].value;
  }
private:
  struct MMIO {
    struct {
      int64_t value;
      int64_t position;
    } encoder[2];
    int32_t pwm;
  };
  size_t psize;
  volatile MMIO *regs;
};

class ButtonController {
public:
  ButtonController(std::string_view f1, std::string_view f2) {
    epollfd = epoll_create1(0);
    fd[0] = open(f1.data(), O_RDWR | O_NONBLOCK);
    fd[1] = open(f2.data(), O_RDWR | O_NONBLOCK);
    irqs[0] = 0;
    irqs[1] = 0;
    for (int i = 0; i < 2; i++) {
      struct epoll_event ev;
      ev.events = EPOLLIN;
      ev.data.fd = fd[i];
      epoll_ctl(epollfd, EPOLL_CTL_ADD, fd[i], &ev);
    }
  }

  ButtonController(ButtonController &&other)
      : epollfd(other.epollfd), fd(other.fd), irqs(other.irqs) {
    other.epollfd = -1;
    other.fd.fill(-1);
  }

  ~ButtonController() {
    if (epollfd != -1) {
      close(epollfd);
    }
    if (fd[0] != -1) {
      close(fd[0]);
    }
    if (fd[1] != -1) {
      close(fd[1]);
    }
  }

  void handle_impl(SensorData &data) {
    struct epoll_event ev;
    data.leftBoundary = false;
    data.rightBoundary = false;
    int ret = epoll_wait(epollfd, &ev, 1, 0);
    if (ret <= 0) 
        return;
    if (ev.events & EPOLLIN) {
      if (ev.data.fd == fd[0]) {
        read(fd[0], &irqs[0], sizeof(int));
        data.leftBoundary = true;
      }
      if (ev.data.fd == fd[1]) {
        read(fd[1], &irqs[1], sizeof(int));
        data.rightBoundary = true;
      }
    }
  }
private:
  int epollfd;
  std::array<int, 2> fd;
  std::array<int, 2> irqs;
};

template <typename... C>
class SensorController: public C... {
    public:
        SensorController(C&&... controllers) : C(std::forward<C>(controllers))... {}

        void handle(SensorData &data){
            (static_cast<C&>(*this).handle_impl(data), ...);
        }
};
