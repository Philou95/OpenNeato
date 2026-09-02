#include "fs_lock.h"

// Function-local static so the mutex exists before the first caller, whatever
// the translation-unit initialisation order turns out to be. Both tasks that
// take it are started well after static init, so the one-time guard is never
// contended.
SemaphoreHandle_t fsMutex() {
    static SemaphoreHandle_t mutex = xSemaphoreCreateRecursiveMutex();
    return mutex;
}
