#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>

static void send_all(int fd, const std::string &data) {
  const char* p = data.c_str();
  size_t left = data.size();
  while (left > 0) {
    ssize_t n = ::send(fd, p, left, 0);
    if (n <= 0) return;
    p += n;
    left -= (size_t)n;
  }
}

int main() {
  const int port = 8080;

  int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0) { perror("socket"); return 1; }

  int opt = 1;
  setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port);

  if (::bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }
  if (::listen(server_fd, 16) < 0) { perror("listen"); return 1; }

  std::cout << "[cppsvc] Listening on : " << port << " (GET /ping)\n";

  while (true) {
    sockaddr_in client{};
    socklen_t len = sizeof(client);
    int fd = ::accept(server_fd, (sockaddr*)&client, &len);
    if (fd < 0) continue;

    char buf[2048];
    ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) { ::close(fd); continue; }
    buf[n] = '\0';

    std::string req(buf);
    bool is_ping = (req.rfind("GET /ping", 0) == 0);

    std::string body = is_ping ? "pong\n" : "not found\n";
    int code = is_ping ? 200 : 404;

    std::string resp =
      "HTTP/1.1 " + std::to_string(code) + (code == 200 ? " OK\r\n" : " Not Found\r\n") +
      "Content-Type: text/plain\r\n"
      "Connection: close\r\n"
      "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n" +
      body;

    send_all(fd, resp);
    ::close(fd);
  }
}

int main() {
    std::cout
}
