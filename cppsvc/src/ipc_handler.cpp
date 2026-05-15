#include "ipc_handler.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

static constexpr const char* SHM_NAME   = "/airhockey_shm";
static constexpr const char* MUTEX_NAME = "/airhockey_mutex";
static constexpr const char* CMD_NAME   = "/airhockey_cmd";

// ── Constructor ─────────────────────────────────────────────────────────────────
IPCHandler::IPCHandler() : m_shm_fd(-1), m_shm(nullptr),
                           m_mutex(SEM_FAILED), m_cmd(SEM_FAILED),
                           m_status_seq(0) {
    // Force umask to 0 so the explicit 0666 modes on shm_open/sem_open
    // actually take effect.  Without this, a default umask of 022/077 masks
    // the perms down to 0644/0600 -- which locks pysvc out when the two
    // services run as different uids.  Restored before this ctor returns.
    mode_t old_umask = umask(0);

    // Open (or create) shared memory
    m_shm_fd = shm_open(SHM_NAME, O_RDWR | O_CREAT, 0666);
    if (m_shm_fd < 0) {
        umask(old_umask);
        throw std::runtime_error(std::string("shm_open failed: ") + strerror(errno));
    }

    // Belt-and-braces: force perms regardless of umask or pre-existing file.
    if (fchmod(m_shm_fd, 0666) < 0) {
        // Non-fatal -- log and continue; only matters across uid boundaries.
        // Most likely cause: shm was created by another uid first.
    }

    if (ftruncate(m_shm_fd, sizeof(AirHockeyIPC)) < 0) {
        umask(old_umask);
        throw std::runtime_error(std::string("ftruncate failed: ") + strerror(errno));
    }

    void* ptr = mmap(nullptr, sizeof(AirHockeyIPC),
                     PROT_READ | PROT_WRITE, MAP_SHARED, m_shm_fd, 0);
    if (ptr == MAP_FAILED) {
        umask(old_umask);
        throw std::runtime_error(std::string("mmap failed: ") + strerror(errno));
    }

    m_shm = static_cast<AirHockeyIPC*>(ptr);

    // Open (or create) semaphores
    // mutex: binary, starts at 1 (unlocked)
    m_mutex = sem_open(MUTEX_NAME, O_CREAT, 0666, 1);
    if (m_mutex == SEM_FAILED) {
        umask(old_umask);
        throw std::runtime_error(std::string("sem_open mutex failed: ") + strerror(errno));
    }

    // cmd: counting, starts at 0 (no pending command)
    m_cmd = sem_open(CMD_NAME, O_CREAT, 0666, 0);
    if (m_cmd == SEM_FAILED) {
        umask(old_umask);
        throw std::runtime_error(std::string("sem_open cmd failed: ") + strerror(errno));
    }

    umask(old_umask);
}

// ── Destructor ──────────────────────────────────────────────────────────────────
IPCHandler::~IPCHandler() {
    if (m_shm && m_shm != MAP_FAILED)
        munmap(m_shm, sizeof(AirHockeyIPC));
    if (m_shm_fd >= 0)
        close(m_shm_fd);
    if (m_mutex != SEM_FAILED)
        sem_close(m_mutex);
    if (m_cmd != SEM_FAILED)
        sem_close(m_cmd);
}

// ── wait_for_command ────────────────────────────────────────────────────────────
void IPCHandler::wait_for_command(CommandData& out) {
    // Block until pysvc posts the cmd semaphore
    sem_wait(m_cmd);

    // Acquire mutex, snapshot command fields, release mutex
    sem_wait(m_mutex);
    out.target_x_mm = m_shm->target_x_mm;
    out.target_y_mm = m_shm->target_y_mm;
    out.command     = m_shm->command;
    out.priority    = m_shm->priority;
    out.seq         = m_shm->command_seq;
    sem_post(m_mutex);
}

// ── write_status ────────────────────────────────────────────────────────────────
void IPCHandler::write_status(float x_mm, float y_mm, uint8_t state, uint8_t ready) {
    sem_wait(m_mutex);
    m_shm->current_x_mm = x_mm;
    m_shm->current_y_mm = y_mm;
    m_shm->state        = state;
    m_shm->ready        = ready;
    m_shm->status_seq   = ++m_status_seq;
    sem_post(m_mutex);
}
