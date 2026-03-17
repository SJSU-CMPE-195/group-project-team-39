#include <arpa/inet.h>
#include <cstdint>
#include <netinet/in.h>
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
  const char *gpio_chip_char = "/dev/gpiochip0";
  const unsigned dur_line = 8; // GPIO1; M40_GPIO0_00
  const unsigned pul_line = 9; // GPIO2; M40_GPIO0_01
  const uint16_t pulse_per_rev = 2000;

  gpiod_chip *chip = gpiod_chip_open(gpio_chip_char);
  if (!chip)
    std::cerr << "Something bad happened\n";

  iSV57T motor = iSV57T(chip, dur_line, pul_line, pulse_per_rev);

  motor.rotate(iSV57T::CCW, 360.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  motor.rotate(iSV57T::CW, 360.0f);

  gpiod_chip_close(chip);

  return 0;
}