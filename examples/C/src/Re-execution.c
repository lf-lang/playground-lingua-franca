#include <stdio.h>
#include <math.h>

int main() {
    // Given values
    double lambda_cpu_per_hour = 1e-6; // λ_cpu: CPU fault rate per hour.
    double lambda_mem_per_hour = 1e-7; // λ_mem: Memory fault rate per hour.
    double clock_frequency = 100e6; // f = 100 MHz (100 million cycles per second).
    double k = 3600 * clock_frequency; // k = 1hour * f
    int exposure_cpu = 1000;   // ε_(i, cpu): CPU exposure time in cycles (equal to WCET).
    int exposure_mem = 10000;  // ε_(i, mem): Memory exposure time in cycles (full period).

    // double p_req = 1e-9; // Required failure rate per hour. Gets 0 re-execution.
     double p_req = 1e-15; // Stricter failure probability requirement. Gets 1 re-execution.
    return 0;
}
