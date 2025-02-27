#include "terminate_benchmark.h"

void benchmark_success()
{
    __asm__ volatile("jalr x0, 124(x0)");     //jump to 0x7c to signal success (Custom use interrupt, will not be called).  Jumps to these addresses are handled by verilator_main.cpp
}

void benchmark_failure()
{
    __asm__ volatile("jalr x0, 120(x0)");   //jump to 0x78 to signal failure caused by mismatched test output  Other interrupts are caught by 0x74 to signal a problem
}
