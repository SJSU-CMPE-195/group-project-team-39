// IPC Test — POSIX shared memory (mmap) + named semaphores
// Both containers share /dev/shm via ipc:host in docker-compose.yml.
// Three receive methods are demonstrated sequentially on the main thread
// while a writer thread sends a counter byte every 500 ms.

#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

// ─────────────────────────────────────────────────────────────────────────────
// Shared memory layout  (64 bytes total, extensible via _reserved)
// This struct must stay byte-for-byte identical to the layout in pysvc/main.py.
// ─────────────────────────────────────────────────────────────────────────────
struct IPCBlock {
  volatile uint8_t py_ready;  // pysvc writes 1 when py_data is valid
  volatile uint8_t py_data;   // payload byte: pysvc → cppsvc
  volatile uint8_t cpp_ready; // cppsvc writes 1 when cpp_data is valid
  volatile uint8_t cpp_data;  // payload byte: cppsvc → pysvc
  uint8_t
      _reserved[60]; // reserved for future fields (coordinates, status, etc.)
};
static_assert(sizeof(IPCBlock) == 64, "IPCBlock must be exactly 64 bytes");

static constexpr const char *SHM_NAME = "/ipc_block";
static constexpr const char *SEM_PY_CPP = "/sem_py_to_cpp";
static constexpr const char *SEM_CPP_PY = "/sem_cpp_to_py";
static constexpr size_t SHM_SIZE = sizeof(IPCBlock);

static std::atomic<bool> g_running{true};
void handle_signal(int) { g_running = false; }

// ─────────────────────────────────────────────────────────────────────────────
// Helper: read py_data and clear py_ready (flag-based consume)
// __sync_synchronize() issues a full memory barrier to prevent the CPU or
// compiler from reordering the flag read relative to the data read.
// ─────────────────────────────────────────────────────────────────────────────
static inline uint8_t consume_py(IPCBlock *blk) {
  uint8_t val = blk->py_data;
  __sync_synchronize();
  blk->py_ready = 0;
  return val;
}

// ─────────────────────────────────────────────────────────────────────────────
// Receive Method 1 — Polling
// Sleeps 1 ms between flag checks. Low CPU cost; ~1 ms latency floor.
// ─────────────────────────────────────────────────────────────────────────────
uint8_t poll_receive(IPCBlock *blk) {
  while (g_running) {
    __sync_synchronize();
    if (blk->py_ready)
      return consume_py(blk);
    usleep(1000);
  }
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Receive Method 2 — Spin (busy-wait)
// Loops with no sleep. Lowest possible latency; consumes a full CPU core.
// ─────────────────────────────────────────────────────────────────────────────
uint8_t spin_receive(IPCBlock *blk) {
  while (g_running) {
    __sync_synchronize();
    if (blk->py_ready)
      return consume_py(blk);
  }
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Receive Method 3 — Non-blocking semaphore (trywait + yield)
// sem_trywait returns immediately with EAGAIN if the semaphore count is 0.
// sched_yield() surrenders the time-slice so the writer can progress.
// ─────────────────────────────────────────────────────────────────────────────
uint8_t semaphore_nonblocking_receive(IPCBlock *blk, sem_t *sem) {
  while (g_running) {
    if (sem_trywait(sem) == 0) {
      __sync_synchronize();
      return consume_py(blk);
    }
    sched_yield();
  }
  return 0;
}

int main() {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  // ── Open / create shared memory ──────────────────────────────────────────
  /**
   * EXPLAINATION: The shm_open() function creates a POSIX shared memory object
   * then returns a file decriptor to work with (so we can do the basic file
   * operations for example on the share memory object). The first parameter is
   * the name of the shared memory block. The second parameter are given flags.
   * O_CREAT means to create the shared object if it doesn't exist already.
   * O_RDWR means open the shared memory as a file descriptor for read and
   * write. The third parameter gives permission. 0666 gives the following
   * permissions:Owner: read + write, Group: read + write, Others: read + write
   */
  int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
  if (fd < 0) {
    perror("shm_open");
    return 1;
  }

  // ftruncate is a no-op if size is already set (safe to call from both sides)
  if (ftruncate(fd, static_cast<off_t>(SHM_SIZE)) < 0) {
    perror("ftruncate");
    close(fd);
    return 1;
  }

  auto *blk = static_cast<IPCBlock *>(
      mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));
  if (blk == MAP_FAILED) {
    perror("mmap");
    close(fd);
    return 1;
  }
  close(fd); // fd is no longer needed once the mapping is established

  // ── Open / create named semaphores ───────────────────────────────────────
  // initial_value=0 means the semaphore starts "not ready"
  sem_t *sem_py_cpp = sem_open(SEM_PY_CPP, O_CREAT, 0666, 0);
  sem_t *sem_cpp_py = sem_open(SEM_CPP_PY, O_CREAT, 0666, 0);
  if (sem_py_cpp == SEM_FAILED || sem_cpp_py == SEM_FAILED) {
    perror("sem_open");
    return 1;
  }

  std::cout << "[cppsvc] IPC ready  shm=" << SHM_NAME << "\n";

  // ── Writer thread: send a counter byte to pysvc every 500 ms ─────────────
  std::thread writer([&] {
    uint8_t counter = 0;
    while (g_running) {
      blk->cpp_data = counter;
      __sync_synchronize(); // data must be visible before the flag
      blk->cpp_ready = 1;
      sem_post(sem_cpp_py);
      std::cout << "[cppsvc] SENT        byte=" << static_cast<int>(counter)
                << "\n";
      counter = static_cast<uint8_t>((counter + 1) % 256);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  // ── Reader: demonstrate all three receive methods in sequence ─────────────
  std::cout << "[cppsvc] Waiting for pysvc data (POLL)...\n";
  uint8_t v1 = poll_receive(blk);
  std::cout << "[cppsvc] POLL        received byte=" << static_cast<int>(v1)
            << "\n";

  std::cout << "[cppsvc] Waiting for pysvc data (SPIN)...\n";
  uint8_t v2 = spin_receive(blk);
  std::cout << "[cppsvc] SPIN        received byte=" << static_cast<int>(v2)
            << "\n";

  std::cout << "[cppsvc] Waiting for pysvc data (SEM NON-BLOCK)...\n";
  uint8_t v3 = semaphore_nonblocking_receive(blk, sem_py_cpp);
  std::cout << "[cppsvc] SEM         received byte=" << static_cast<int>(v3)
            << "\n";

  std::cout << "[cppsvc] All three receive methods completed. Exiting.\n";
  g_running = false;
  writer.join();

  munmap(blk, SHM_SIZE);
  sem_close(sem_py_cpp);
  sem_close(sem_cpp_py);
  // Note: shm_unlink / sem_unlink are intentionally omitted so the other
  // container can finish accessing the objects after this one exits.
  return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Original GPIO / gantry code — preserved below, commented out
// ─────────────────────────────────────────────────────────────────────────────

// #include <arpa/inet.h>
// #include <netinet/in.h>
// #include <stdio.h>
// #include <sys/socket.h>
// #include <string>
// #include "gantry.hpp"
// #include "iSV57T.hpp"
// #include "include/gantry.hpp"
// #include "include/iSV57T.hpp"
// #include "limitSwitch.hpp"
//
// int main() {
//   const char *gpio_chip4_char = "/dev/gpiochip4";
//   const char *gpio_chip0_char = "/dev/gpiochip0";
//
//   const unsigned m1_dir_line = 8;
//   const unsigned m1_pul_line = 9;
//   const unsigned m2_dir_line = 12;
//   const unsigned m2_pul_line = 13;
//   const unsigned sw1_line = 1;
//   const unsigned sw2_line = 2;
//   const uint16_t pulse_per_rev = 2000;
//
//   gpiod_chip *chip0 = gpiod_chip_open(gpio_chip0_char);
//   if (!chip0) {
//     fprintf(stderr, "Failed to open %s for motor pins: %s\n",
//             gpio_chip0_char, strerror(errno));
//     gpiod_chip_close(chip0);
//     return 0;
//   }
//
//   gpiod_chip *chip4 = gpiod_chip_open(gpio_chip4_char);
//   if (!chip4) {
//     fprintf(stderr, "Failed to open %s for limit switches: %s\n",
//             gpio_chip4_char, strerror(errno));
//     gpiod_chip_close(chip4);
//     return 0;
//   }
//
//   limitSwitch sw_y(chip4, sw1_line);
//   limitSwitch sw_x(chip4, sw2_line);
//
//   iSV57T m_lower = iSV57T(chip0, m1_dir_line, m1_pul_line, pulse_per_rev);
//   iSV57T m_upper = iSV57T(chip0, m2_dir_line, m2_pul_line, pulse_per_rev);
//   m_lower.set_target_rpm(500);
//   m_upper.set_target_rpm(500);
//
//   gantry g = gantry(m_lower, m_upper, sw_x, sw_y);
//
//   // g.move_to_origin();
//   // g.move_to_coord(x, y);
//
//   // std::thread sw1_thread([&]() {
//   //   while (sw1.read() != 1) {}
//   //   std::cout << "Limit switch 1 (Y-axis) triggered!\n";
//   // });
//   // std::thread sw2_thread([&]() {
//   //   while (sw2.read() != 1) {}
//   //   std::cout << "Limit switch 2 (X-axis) triggered!\n";
//   // });
//   // g.rotate_motors(720, iSV57T::CCW, iSV57T::CW,
//   gantry::MotorSelect::BOTH); // North
//   // g.rotate_motors(720, iSV57T::CW,  iSV57T::CCW,
//   gantry::MotorSelect::BOTH); // South
//   // g.rotate_motors(720, iSV57T::CCW, iSV57T::CCW,
//   gantry::MotorSelect::BOTH); // East
//   // g.rotate_motors(720, iSV57T::CW,  iSV57T::CW,
//   gantry::MotorSelect::BOTH); // West
//
//   gpiod_chip_close(chip4);
//   gpiod_chip_close(chip0);
//   return 0;
// }

// 63.7919 left to right mm = 7900 degrees
// 7800 degrees top to bottom
// 125 mm diagonal == 43 rotation of 360 = 15480 degrees
// Golden Ratio: 123.84 degrees per 1 mm
