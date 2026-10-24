/**
 * @file IpcManager.h
 * @brief Management of Inter-Process Communication (IPC).
 *
 * macOS port: POSIX message queues (mqueue.h) and eventfd (sys/eventfd.h) are
 * Linux-only and unavailable on macOS.  The IMU channel is replaced with a
 * Unix-domain SOCK_DGRAM socket.  shm_open / mmap / sem_open are POSIX and
 * work on macOS without modification.
 */

#ifndef IPC_MANAGER_H
#define IPC_MANAGER_H

#include <string>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <cstdio>
#include <cerrno>
#include <ctime>

#include "DataTypes.h"

/** @brief Path of the IMU Unix-domain socket. */
const char* const IMU_SOCK_PATH  = "/tmp/vins_imu.sock";
/** @brief Name of the camera shared memory. */
const char* const CAM_SHM_NAME  = "/vins_cam_shm";
/** @brief Name of the camera semaphore. */
const char* const CAM_SEM_NAME  = "/vins_cam_sem";
/** @brief Name of the readiness semaphore. */
const char* const READY_SEM_NAME = "/vins_ready_sem";

/**
 * @class IpcManager
 * @brief Class responsible for setting up and managing IPC resources.
 *
 * IMU transport: Unix-domain SOCK_DGRAM socket (replaces POSIX mqueue).
 *   - Server (is_server=true)  binds to IMU_SOCK_PATH and reads datagrams.
 *   - Client (is_server=false) connects to IMU_SOCK_PATH and writes datagrams.
 *
 * Camera transport: POSIX shared memory + named semaphore (unchanged).
 */
class IpcManager {
public:
    /** @brief Constructor. */
    IpcManager() = default;

    /** @brief Destructor. Resources are unlinked explicitly via cleanup_all(). */
    ~IpcManager() = default;

    // -------------------------------------------------------------------------
    // IMU channel — Unix-domain SOCK_DGRAM
    // -------------------------------------------------------------------------

    /**
     * @brief Create or connect to the IMU Unix-domain socket.
     *
     * Server side binds to IMU_SOCK_PATH (removes any stale socket first).
     * Client side creates an unbound socket and connects to the server path so
     * that send() can be used without specifying the destination each time.
     *
     * @param is_server  True for the receiving (VIO) process; false for the
     *                   sending (sensor) process.
     * @return           Socket file descriptor (>= 0).
     * @throws std::runtime_error on any failure.
     */
    int open_imu_mq(bool is_server) {
        int fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
        if (fd == -1) {
            perror("socket imu");
            throw std::runtime_error("Failed to create IMU socket");
        }

        struct sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, IMU_SOCK_PATH, sizeof(addr.sun_path) - 1);

        if (is_server) {
            // Remove stale socket file so bind() succeeds.
            ::unlink(IMU_SOCK_PATH);
            if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr),
                       sizeof(addr)) == -1) {
                perror("bind imu");
                ::close(fd);
                throw std::runtime_error("Failed to bind IMU socket");
            }
        } else {
            // Client: connect so we can use send() instead of sendto().
            if (::connect(fd, reinterpret_cast<struct sockaddr*>(&addr),
                          sizeof(addr)) == -1) {
                perror("connect imu");
                ::close(fd);
                throw std::runtime_error("Failed to connect IMU socket");
            }
        }

        return fd;
    }

    /**
     * @brief Send one ImuDataPacket over the socket (client side).
     * @param sock_fd  Socket fd returned by open_imu_mq(false).
     * @param pkt      Packet to send.
     * @throws std::runtime_error on send failure.
     */
    void send_imu(int sock_fd, const ImuDataPacket& pkt) {
        ssize_t n = ::send(sock_fd, &pkt, sizeof(pkt), 0);
        if (n == -1) {
            perror("send imu");
            throw std::runtime_error("Failed to send IMU packet");
        }
    }

    /**
     * @brief Receive one ImuDataPacket from the socket with a timeout (server side).
     *
     * Uses select() for the timeout, which is portable across macOS and Linux
     * (replaces the Linux-only sem_timedwait / mq_timedreceive pattern).
     *
     * @param sock_fd        Socket fd returned by open_imu_mq(true).
     * @param pkt            Output packet.
     * @param timeout_ms     Milliseconds to wait before returning false.
     * @return true if a packet was received; false on timeout.
     * @throws std::runtime_error on a hard socket error.
     */
    bool recv_imu(int sock_fd, ImuDataPacket& pkt, int timeout_ms = 100) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock_fd, &rfds);

        struct timeval tv{};
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = ::select(sock_fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret == -1) {
            perror("select imu");
            throw std::runtime_error("select() failed on IMU socket");
        }
        if (ret == 0) {
            return false;  // timeout
        }

        ssize_t n = ::recv(sock_fd, &pkt, sizeof(pkt), 0);
        if (n == -1) {
            perror("recv imu");
            throw std::runtime_error("Failed to receive IMU packet");
        }
        if (static_cast<size_t>(n) != sizeof(pkt)) {
            throw std::runtime_error("IMU packet size mismatch");
        }
        return true;
    }

    // -------------------------------------------------------------------------
    // Camera channel — POSIX shared memory (unchanged, works on macOS)
    // -------------------------------------------------------------------------

    /**
     * @brief Create or open the camera shared memory.
     * @param is_server  True if this is the creating process.
     * @param size       Size of the shared memory region in bytes.
     * @return           Pointer to the mapped shared memory region.
     * @throws std::runtime_error on any failure.
     */
    void* open_cam_shm(bool is_server, size_t size) {
        int flags = is_server ? (O_CREAT | O_RDWR) : O_RDWR;
        if (is_server) shm_unlink(CAM_SHM_NAME);

        int shm_fd = shm_open(CAM_SHM_NAME, flags, 0666);
        if (shm_fd == -1) {
            perror("shm_open camera");
            throw std::runtime_error("Failed to open Camera shared memory");
        }

        if (is_server && ftruncate(shm_fd, static_cast<off_t>(size)) == -1) {
            perror("ftruncate camera");
            ::close(shm_fd);
            throw std::runtime_error("Failed to ftruncate Camera shared memory");
        }

        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (ptr == MAP_FAILED) {
            perror("mmap camera");
            ::close(shm_fd);
            throw std::runtime_error("Failed to mmap Camera shared memory");
        }

        ::close(shm_fd);
        return ptr;
    }

    // -------------------------------------------------------------------------
    // Semaphores (unchanged, work on macOS)
    // -------------------------------------------------------------------------

    /**
     * @brief Create or open the camera semaphore.
     * @param is_server  True if this is the creating process.
     * @return           Pointer to the opened semaphore.
     * @throws std::runtime_error on failure.
     */
    sem_t* open_cam_sem(bool is_server) {
        if (is_server) sem_unlink(CAM_SEM_NAME);
        sem_t* sem = sem_open(CAM_SEM_NAME, is_server ? O_CREAT : 0, 0666, 0);
        if (sem == SEM_FAILED) {
            perror("sem_open camera");
            throw std::runtime_error("Failed to open Camera semaphore");
        }
        return sem;
    }

    /**
     * @brief Create or open the readiness semaphore.
     * @param is_server  True if this is the creating process.
     * @return           Pointer to the opened semaphore.
     * @throws std::runtime_error on failure.
     */
    sem_t* open_ready_sem(bool is_server) {
        if (is_server) sem_unlink(READY_SEM_NAME);
        sem_t* sem = sem_open(READY_SEM_NAME, is_server ? O_CREAT : 0, 0666, 0);
        if (sem == SEM_FAILED) {
            perror("sem_open readiness");
            throw std::runtime_error("Failed to open Readiness semaphore");
        }
        return sem;
    }

    /**
     * @brief Portable timed semaphore wait.
     *
     * macOS does not implement sem_timedwait(3).  This helper polls with
     * sem_trywait() in short sleep intervals up to the requested deadline,
     * which is sufficient for the low-frequency signalling used here.
     *
     * @param sem         Named semaphore pointer.
     * @param timeout_ms  Maximum wait time in milliseconds.
     * @return  0 on success, -1 on timeout (errno = ETIMEDOUT) or error.
     */
    static int sem_timedwait_portable(sem_t* sem, int timeout_ms) {
        const int sleep_us = 1000;  // poll every 1 ms
        int elapsed_us = 0;
        const int limit_us = timeout_ms * 1000;

        while (elapsed_us < limit_us) {
            if (sem_trywait(sem) == 0) {
                return 0;
            }
            if (errno != EAGAIN) {
                return -1;  // real error
            }
            ::usleep(static_cast<useconds_t>(sleep_us));
            elapsed_us += sleep_us;
        }
        errno = ETIMEDOUT;
        return -1;
    }

    // -------------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------------

    /**
     * @brief Unlink all IPC resources.
     *
     * Should be called by the server process on shutdown.  The socket file is
     * removed with unlink(); named POSIX objects are removed with their
     * respective *_unlink() calls.
     */
    static void cleanup_all() {
        ::unlink(IMU_SOCK_PATH);
        shm_unlink(CAM_SHM_NAME);
        sem_unlink(CAM_SEM_NAME);
        sem_unlink(READY_SEM_NAME);
    }
};

#endif // IPC_MANAGER_H
