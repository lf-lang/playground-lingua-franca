#include <stdio.h>
#include <math.h>

// Get the per-cycle fault probability λ' = 1 - (1 -λ)^(1/k)
double compute_lambda_per_cycle(double lambda_per_hour, double k) {
    // return 1 - pow(1 - lambda_per_hour, 1.0 / k); // Same with below.
    return -log(1 - lambda_per_hour) / k; 
}

// Compute the natural failure probability for a task
double compute_natural_failure(double lambda_cpu, double lambda_mem, int exposure_cpu, int exposure_mem) {
    // double p_F_cpu = 1 - pow(1 - lambda_cpu, exposure_cpu); 
    // The original equation above does not work because too small values.
    // Same with below mathematically.
    // (1-λ)^k ≈ e^(-λk) for small λ (approximation).
    double p_F_cpu = 1 - exp(-lambda_cpu * exposure_cpu); // Failure probability from cpu faults.

    // double p_F_cpu = 1 - pow(1 - lambda_cpu, exposure_cpu); // Same with below.
    double p_F_mem = 1 - exp(-lambda_mem * exposure_mem); // Failure probability from memory faults.
    
    // Combined probability of failure.
    // 1 - Π (1 - λ')^(ε_(i, resource)
    return 1 - (1 - p_F_cpu) * (1 - p_F_mem); 
}

// Calculate the required number of re-executions.
int calculate_reexecutions(double p_F, double p_req) {
    if (p_F <= p_req) {
        return 0; // No re-execution needed
    }
    // N_(re-exec,i) = max(0, ⌈log (p_(req,i)) / log (p_F_i)⌉ − 1)
    return (int) fmax(0, ceil(log(p_req) / log(p_F)) - 1);
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
    // λ'_mem
    double lambda_mem_per_cycle = compute_lambda_per_cycle(lambda_mem_per_hour, k);

    // Get natural failure rate p_F.
    double p_F = compute_natural_failure(lambda_cpu_per_cycle, lambda_mem_per_cycle, exposure_cpu, exposure_mem);

    int reexecutions = calculate_reexecutions(p_F, p_req);

    printf("Computed per-cycle CPU fault probability: %e\n", lambda_cpu_per_cycle);
    printf("Computed per-cycle Memory fault probability: %e\n", lambda_mem_per_cycle);
    printf("Natural failure probability per job: %e\n", p_F);
    printf("Required number of re-executions: %d\n", reexecutions);

    return 0;
}
