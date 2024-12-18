#include "shared_memory.hpp"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>
#include <iomanip> // std::fixed, std::setprecision

using namespace std;

void motor_control()
{
    // 공유 메모리 열기
    int shm_fd = shm_open(SHARED_MEMORY_NAME, O_RDWR, 0666);
    if (shm_fd == -1)
    {
        throw std::runtime_error("Failed to open shared memory.");
    }

    // 메모리 매핑
    SharedMemoryData *shared_memory = static_cast<SharedMemoryData *>(mmap(
        0, sizeof(SharedMemoryData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (shared_memory == MAP_FAILED)
    {
        throw std::runtime_error("Failed to map shared memory.");
    }

    while (true)
    {
        pthread_mutex_lock(&shared_memory->mutex);

        // 데이터 준비 상태 확인
        while (!shared_memory->data_ready)
        {
            pthread_cond_wait(&shared_memory->condition, &shared_memory->mutex);
        }

        // 데이터 읽기
        float target_position = shared_memory->data;
        shared_memory->data_ready = false; // 플래그 초기화

        pthread_mutex_unlock(&shared_memory->mutex);

        // 모터 제어 로직
        std::cout << std::fixed << std::setprecision(1) << "Motor Control: Received Target Position = " << target_position << std::endl;
    }

    // 공유 메모리 해제
    munmap(shared_memory, sizeof(SharedMemoryData));
    close(shm_fd);
}

int main()
{
    try
    {
        std::cout << "Motor Control Program Started." << std::endl;
        motor_control();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

