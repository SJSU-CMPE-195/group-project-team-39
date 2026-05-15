#pragma once

#include <cstdint>
#include <semaphore.h>

// ── Shared memory layout (32 bytes, naturally aligned, no implicit padding) ────
//
//  Offset  Field           Type      Description
//  ──────  ──────────────  ───────── ────────────────────────────────────────────
//   0      target_x_mm     float     Gantry target X  (pysvc -> cppsvc)
//   4      target_y_mm     float     Gantry target Y
//   8      command         uint8_t   IDLE/INTERCEPT/PUSH_NORTH/PUSH_OUT
//   9      priority        uint8_t   LOW/MEDIUM/HIGH
//  10      _pad1           uint16_t  (alignment pad)
//  12      command_seq     uint32_t  Incremented per new command
//  16      current_x_mm    float     Current mallet X (cppsvc -> pysvc)
//  20      current_y_mm    float     Current mallet Y
//  24      state           uint8_t   IDLE/MOVING/HOMING/ERROR
//  25      ready           uint8_t   1 = gantry ready for next command
//  26      _pad2           uint16_t  (alignment pad)
//  28      status_seq      uint32_t  Incremented per status update
//  ─────────────────────────────────────────────────────────────────────────────
//                          Total     32 bytes

struct AirHockeyIPC {
    float    target_x_mm;
    float    target_y_mm;
    uint8_t  command;
    uint8_t  priority;
    uint16_t _pad1;
    uint32_t command_seq;

    float    current_x_mm;
    float    current_y_mm;
    uint8_t  state;
    uint8_t  ready;
    uint16_t _pad2;
    uint32_t status_seq;
};
static_assert(sizeof(AirHockeyIPC) == 32, "AirHockeyIPC size mismatch");

// ── Enumerations ───────────────────────────────────────────────────────────────
namespace AirHockey {

enum Command : uint8_t {
    IDLE       = 0,
    INTERCEPT  = 1,
    PUSH_NORTH = 2,
    PUSH_OUT   = 3,
};

enum Priority : uint8_t {
    LOW    = 0,
    MEDIUM = 1,
    HIGH   = 2,
};

enum State : uint8_t {
    STATE_IDLE = 0,
    MOVING     = 1,
    HOMING     = 2,
    ERROR      = 3,
};

} // namespace AirHockey

// ── IPCHandler ─────────────────────────────────────────────────────────────────
class IPCHandler {
public:
    struct CommandData {
        float    target_x_mm;
        float    target_y_mm;
        uint8_t  command;
        uint8_t  priority;
        uint32_t seq;
    };

    IPCHandler();
    ~IPCHandler();

    // Block until pysvc signals a new command, then copy it into `out`.
    void wait_for_command(CommandData& out);

    // Write gantry status into shared memory (acquires mutex internally).
    void write_status(float x_mm, float y_mm, uint8_t state, uint8_t ready);

private:
    int           m_shm_fd;
    AirHockeyIPC* m_shm;
    sem_t*        m_mutex;
    sem_t*        m_cmd;
    uint32_t      m_status_seq;
};
