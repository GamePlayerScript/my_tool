#pragma once

#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <semaphore.h>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include "sempore.h"

class SharedMemory {
public:
    SharedMemory() {
		
	}
    ~SharedMemory() {
        // 释放信号量
        // sem_close(mutex_);
        // sem_unlink(mutex_key.c_str());
        SemporeTransmitter::DelSemvalue(shm_id_);
        SemporeTransmitter::DelSemvalue(sempore_condition_id_);
         if (shmctl(shm_id_, IPC_RMID, NULL) == -1) {
            perror("shmctl IPC_RMID");
            // 处理错误
        }
    }
    void cleanupSemaphore(const std::string& mutex_key) {
        sem_t* semaphore = sem_open(mutex_key.c_str(), 0);

        if (semaphore != SEM_FAILED) {
            // 信号量存在，关闭并删除
            sem_close(semaphore);
            sem_unlink(mutex_key.c_str());
            std::cout << "Cleaned up existing semaphore: " << mutex_key << std::endl;
        }
}
    int Init(key_t key, std::string mutex_key, int size) {
        memory_size = size;
        shm_id_ = shmget(key, size, IPC_CREAT | 0644);
        memory_ptr_ = (u_char*)shmat(shm_id_, NULL, 0);
        std::cout << "mutex_key: " << mutex_key << std::endl;

        sempore_id_ = SemporeTransmitter::InitSemvalue(mutex_key);
        std::cout << "sempore_id_--------------: " << sempore_id_ << std::endl;
        sempore_condition_id_ = SemporeTransmitter::InitSemvalue(mutex_key + "condition"); 
        std::cout << "sempore_condition_id_-----------------: " << sempore_condition_id_ << std::endl;
        return 0;
    }

    int WriteData(u_char* data, int data_len) {
        memcpy(memory_ptr_, data, data_len);
        
        return 0;
    }

    u_char* GetMemory() { 
		return memory_ptr_; 
    }

    void MemoryClear() {
        memset(memory_ptr_, 0, memory_size);
    }

    void MemoryFree() {
        if (shmctl(shm_id_, IPC_RMID, NULL) == -1) {
            perror("shmctl IPC_RMID");
            // 处理错误
        }
    }

    void MemoryWait() {

        std::cout << "sempore_id_" << sempore_id_ << std::endl;
        SemporeTransmitter::Semaphore_p(sempore_id_);
    }

    void MemoryUnlock() {
        std::cout << "sempore_id_v_" << sempore_id_<< std::endl;
        SemporeTransmitter::Semaphore_v(sempore_id_);
    }

    void MemoryConditionWait(int num) {
        std::cout << "sempore_id_" << sempore_id_;
        SemporeTransmitter::Semaphore_num_p(sempore_condition_id_, num);
    }

    void MemoryConditionUnlock(int num) {
        std::cout << "sempore_id_v_" << sempore_id_;
        SemporeTransmitter::Semaphore_num_v(sempore_condition_id_, num);
    }

    sem_t* GetMutex() {
        return mutex_;
    }

private:
    int memory_size;
    key_t key;
    u_char* memory_ptr_ = nullptr;
    int shm_id_;
    sem_t* mutex_;
    std::string mutex_key;  // 保存mutex_key，以便在析构函数中释放信号量
	boost::interprocess::named_mutex *mutex;
    boost::interprocess::named_condition *data_written_condition_;
    int sempore_id_;
    int sempore_condition_id_;
    
};

