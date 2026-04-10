#include <arpa/inet.h>

#include <csignal>
#include <cstdint>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>

#include "iSV57T.hpp"
#include "include/iSV57T.hpp"

static void send_all(int fd, const std::string &data) {
  const char *p = data.c_str();
  size_t left = data.size();
  while (left > 0) {
    ssize_t n = ::send(fd, p, left, 0);
    if (n <= 0)
      return;
    p += n;
    left -= (size_t)n;
  }
}

/*
[[deprecated("Only a test main() function to test communication with the "
             "Python script.")]] int
main() {
  const int port = 8080;

  int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) {
    perror("socket");
    return 1;
  }

  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);

  if (::bind(server_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    return 1;
  }
  if (::listen(server_fd, 16) < 0) {
    perror("listen");
    return 1;
  }

  std::cout << "[cppsvc] Listening on : " << port << " (GET /ping)\n";

  while (true) {
    sockaddr_in client{};
    socklen_t len = sizeof(client);
    int fd = ::accept(server_fd, (sockaddr *)&client, &len);
    if (fd < 0)
      continue;

    char buf[2048];
    ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) {
      ::close(fd);
      continue;
    }
    buf[n] = '\0';

    std::string req(buf);
    bool is_ping = (req.rfind("GET /ping", 0) == 0);

    std::string body = is_ping ? "pong\n" : "not found\n";
    int code = is_ping ? 200 : 404;

    std::string resp = "HTTP/1.1 " + std::to_string(code) +
                       (code == 200 ? " OK\r\n" : " Not Found\r\n") +
                       "Content-Type: text/plain\r\n"
                       "Connection: close\r\n"
                       "Content-Length: " +
                       std::to_string(body.size()) + "\r\n\r\n" + body;

    send_all(fd, resp);
    ::close(fd);
  }
}
*/

int main() {
  const char *gpio_chip_char = "/dev/gpiochip2";
  const unsigned dir_line = 8; // GPIO1; M40_GPIO0_00; Pin 13
  const unsigned pul_line = 9; // GPIO2; M40_GPIO0_01; Pin 14
  const uint16_t pulse_per_rev = 2000;

  gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
  if (!chip) {
    fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char, strerror(errno));
    std::cerr << "Something bad happened\n";
    return 0;
  }

  std::cout << "Initializing motor object...\n";
  // try {
  //   iSV57T motor = iSV57T(chip, dir_line, pul_line, pulse_per_rev);
  // } catch (const std::runtime_error &e) {
  //   std::cerr << e.what() << "\n";
  // }
  iSV57T motor = iSV57T(chip, dir_line, pul_line, pulse_per_rev);

  motor.set_target_rpm(3000);

  std::cout << "Finished motor object initialization!\n";

  std::cout << "Starting motor rotations...\n";
  motor.rotate(iSV57T::CW, 360);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  motor.rotate(iSV57T::CCW, 360);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << "Ending motor rotations...\n";

  // gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
  // if (!chip) {
  //   fprintf(stderr, "Failed to open %s: %s\n", gpio_chip_char,
  //   strerror(errno)); std::cerr << "Something bad happened\n"; return 0;
  // }

  // // gpiod_line *line = gpiod_chip_get_line(chip, pul_line);
  // gpiod_line *line = gpiod_chip_get_line(chip, dir_line);
  // if (!line) {
  //   fprintf(stderr, "Failed to get line: %s\n", strerror(errno));
  //   std::cerr << "Something bad happened\n";
  //   return 0;
  // }

  // // Set the GPIO pin values to default LOW
  // if (gpiod_line_request_output(line, "led", 0) < 0) {
  //   fprintf(stderr, "Failed to default: %s\n", strerror(errno));
  //   std::cerr << "Something bad happened\n";
  //   return 0;
  // }

  // std::cout << "Starting toggle...\n";
  // for (int i = 0; i < 5; ++i) {
  //   std::cout << "HIGH\n";
  //   if (gpiod_line_set_value(line, 1) < 0) {
  //     fprintf(stderr, "Failed to set line: %s\n", strerror(errno));
  //     std::cerr << "Something bad happened\n";
  //     return 0;
  //   }

  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));

  //   std::cout << "LOW\n";
  //   if (gpiod_line_set_value(line, 0) < 0) {
  //     fprintf(stderr, "Failed to set line: %s\n", strerror(errno));
  //     std::cerr << "Something bad happened\n";
  //     return 0;
  //   }

  //   std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // }
  // std::cout << "Ending toggle...\n";

  // gpiod_line_release(line);
  gpiod_chip_close(chip);

  return 0;
}