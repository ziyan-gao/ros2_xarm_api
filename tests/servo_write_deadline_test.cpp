// Offline fault-injection tests. Never connects to a robot: mocked transport
// and an ephemeral 127.0.0.1 server only. Compile against the patched SDK.
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "xarm/core/instruction/uxbus_cmd.h"
#include "xarm/core/instruction/uxbus_cmd_tcp.h"

class FakeCommand : public UxbusCmd {
public:
  int sends = 0, receives = 0, last_timeout = 0;
  int send_result = 9, reply_result = 0;
  int send_delay_ms = 0, reply_delay_ms = 0;
  std::vector<unsigned char> bytes;
  int function = -1;
  void hold_lock(std::atomic<bool>& held, std::atomic<bool>& release) {
    std::lock_guard<std::mutex> guard(mutex_);
    held = true;
    while (!release) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
private:
  int _send_modbus_request(unsigned char unit, unsigned char* data,
                           unsigned short len, int) override {
    ++sends;
    function = unit;
    bytes.assign(data, data + len);
    std::this_thread::sleep_for(std::chrono::milliseconds(send_delay_ms));
    return send_result;
  }
  int _recv_modbus_response(unsigned char, unsigned short, unsigned char*,
                            unsigned short, int timeout, int) override {
    ++receives;
    last_timeout = timeout;
    std::this_thread::sleep_for(std::chrono::milliseconds(reply_delay_ms));
    return reply_result;
  }
};

static float joints[7] = {.1f,.2f,.3f,.4f,.5f,.6f,0.f};
static int servo(UxbusCmd& cmd) { return cmd.move_servoj(joints, .3f, .5f, 0.f); }

static void mock_tests() {
  {
    FakeCommand cmd;
    assert(servo(cmd) == 0 && servo(cmd) == 0);
    assert(cmd.sends == 2 && cmd.receives == 2);
    assert(cmd.function == UXBUS_RG::MOVE_SERVOJ && cmd.bytes.size() == 40);
    float expected[10] = {.1f,.2f,.3f,.4f,.5f,.6f,0.f,.3f,.5f,0.f};
    unsigned char bytes[40];
    nfp32_to_hex(expected, bytes, 10);
    assert(std::memcmp(bytes, cmd.bytes.data(), 40) == 0);
    assert(cmd.last_timeout > 0 && cmd.last_timeout <= 50);
  }
  {
    FakeCommand cmd;
    std::atomic<bool> held{false}, release{false};
    std::thread owner([&] { cmd.hold_lock(held, release); });
    while (!held) std::this_thread::yield();
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT);
    release = true;
    owner.join();
    assert(cmd.sends == 0 && cmd.receives == 0);
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT && cmd.sends == 0);
  }
  {
    FakeCommand cmd;
    cmd.send_result = -1;
    assert(servo(cmd) == UXBUS_STATE::ERR_NOTTCP && cmd.receives == 0);
    cmd.send_result = 9;
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT && cmd.sends == 1);
  }
  {
    FakeCommand cmd;
    cmd.send_delay_ms = 60;  // Scheduler stall after obtaining command lock.
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT && cmd.receives == 0);
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT && cmd.sends == 1);
  }
  {
    FakeCommand cmd;
    cmd.reply_delay_ms = 60;  // Even a late SUCCESS must not authorize resume.
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT);
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT && cmd.sends == 1);
  }
  {
    FakeCommand cmd;
    cmd.reply_result = UXBUS_STATE::ERR_TOUT;
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT);
    cmd.reply_result = 0;
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT && cmd.sends == 1);
    // Fault containment must not prevent the driver requesting STOP.
    assert(cmd.set_state(4) == 0 && cmd.function == UXBUS_RG::SET_STATE);
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT && cmd.sends == 2);
  }
  {
    FakeCommand cmd;
    cmd.reply_result = UXBUS_STATE::WAR_CODE;
    assert(servo(cmd) == UXBUS_STATE::WAR_CODE);
    cmd.reply_result = 0;
    assert(servo(cmd) == 0);  // Preserve upstream warning semantics.
  }
  {
    FakeCommand cmd;
    cmd.send_delay_ms = 10;
    assert(servo(cmd) == 0);
    assert(cmd.last_timeout > 0 && cmd.last_timeout <= 40);
  }
}

static void tcp_test(bool delay_reply) {
  const int listener = socket(AF_INET, SOCK_STREAM, 0);
  assert(listener >= 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  assert(bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
  socklen_t size = sizeof(address);
  assert(getsockname(listener, reinterpret_cast<sockaddr*>(&address), &size) == 0);
  assert(listen(listener, 1) == 0);
  std::atomic<int> requests{0};
  std::atomic<bool> release{false};
  std::thread server([&] {
    int peer = accept(listener, nullptr, nullptr);
    assert(peer >= 0);
    unsigned char request[47];
    assert(recv(peer, request, sizeof(request), MSG_WAITALL) == sizeof(request));
    ++requests;
    assert(request[6] == UXBUS_RG::MOVE_SERVOJ);
    // Preserve transaction ID and private protocol; zero means motion-ready.
    unsigned char response[8] = {request[0],request[1],0,2,0,2,request[6],0};
    if (delay_reply) std::this_thread::sleep_for(std::chrono::milliseconds(220));
    assert(send(peer, response, sizeof(response), MSG_NOSIGNAL) == sizeof(response));
    while (!release) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    shutdown(peer, SHUT_RDWR);
    close(peer);
  });
  auto port = std::make_shared<SocketPort>("127.0.0.1", ntohs(address.sin_port), 8, 128);
  UxbusCmdTcp cmd(port);
  const auto started = std::chrono::steady_clock::now();
  const int result = servo(cmd);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started).count();
  if (delay_reply) {
    assert(result == UXBUS_STATE::ERR_TOUT);
    assert(elapsed < 150);  // CI scheduling allowance; reply takes 220 ms.
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    assert(servo(cmd) == UXBUS_STATE::ERR_TOUT);  // Late ACK cannot unlatch.
  } else {
    assert(result == 0 && cmd.state_is_ready);
  }
  assert(requests == 1);
  release = true;
  server.join();
  port->disconnect();
  close(listener);
}

static void tcp_backpressure_test() {
  const int listener = socket(AF_INET, SOCK_STREAM, 0);
  assert(listener >= 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  assert(bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
  socklen_t size = sizeof(address);
  assert(getsockname(listener, reinterpret_cast<sockaddr*>(&address), &size) == 0);
  assert(listen(listener, 1) == 0);
  std::atomic<bool> release{false};
  std::thread server([&] {
    const int peer = accept(listener, nullptr, nullptr);
    assert(peer >= 0);
    // Intentionally never drain the socket, forcing a partial send/EAGAIN.
    while (!release) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    shutdown(peer, SHUT_RDWR);
    close(peer);
  });
  auto port = std::make_shared<SocketPort>("127.0.0.1", ntohs(address.sin_port), 8, 128);
  std::vector<unsigned char> bytes(16 * 1024 * 1024, 0);
  const auto started = std::chrono::steady_clock::now();
  assert(port->write_frame_bounded(bytes.data(), bytes.size(), 5) == -1);
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started).count();
  assert(elapsed < 150);
  assert(!port->is_connected());  // No new frame after an incomplete frame.
  assert(port->write_frame_bounded(bytes.data(), 47, 5) == -1);
  release = true;
  server.join();
  close(listener);
}

int main() {
  mock_tests();
  tcp_test(false);
  tcp_test(true);
  tcp_backpressure_test();
  std::cout << "Servo-J deadline: 8 mock cases and 3 loopback cases passed\n";
}
