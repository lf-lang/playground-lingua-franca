#include <stdio.h>
#include <math.h>

// Get the per-cycle fault probability λ' = 1 - (1 -λ)^(1/k)
double compute_lambda_per_cycle(double lambda_per_hour, double k) {
    // return 1 - pow(1 - lambda_per_hour, 1.0 / k); // Same with below.
    return -log(1 - lambda_per_hour) / k; 
}

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

    // Get λ' to compute the natural failure.
    // λ'_cpu
    double lambda_cpu_per_cycle = compute_lambda_per_cycle(lambda_cpu_per_hour, k); 

    return 0;
}
