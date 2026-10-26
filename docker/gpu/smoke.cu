// GPU-3010: compile with nvcc, run a kernel on the GPU, check the result.
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <cuda_runtime.h>

#define CHECK(call)                                                   \
    do {                                                              \
        cudaError_t err_ = (call);                                    \
        if (err_ != cudaSuccess) {                                    \
            std::fprintf(stderr, "CUDA error %s at line %d: %s\n",    \
                         #call, __LINE__, cudaGetErrorString(err_));  \
            return 1;                                                 \
        }                                                             \
    } while (0)

__global__ void square(int* out, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < n) out[i] = i * i;
}

int main() {
    int count = 0;
    CHECK(cudaGetDeviceCount(&count));
    if (count < 1) {
        std::fprintf(stderr, "no CUDA device\n");
        return 1;
    }
    cudaDeviceProp prop;
    CHECK(cudaGetDeviceProperties(&prop, 0));
    std::printf("device %s compute %d.%d\n", prop.name, prop.major,
                prop.minor);

    const int n = 1024;
    int* dev = nullptr;
    CHECK(cudaMalloc(&dev, n * sizeof(int)));
    square<<<(n + 255) / 256, 256>>>(dev, n);
    CHECK(cudaGetLastError());
    std::vector<int> host(n);
    CHECK(cudaMemcpy(host.data(), dev, n * sizeof(int),
                     cudaMemcpyDeviceToHost));
    CHECK(cudaFree(dev));
    for (int i = 0; i < n; ++i) {
        if (host[i] != i * i) {
            std::fprintf(stderr, "mismatch at %d: %d\n", i, host[i]);
            return 1;
        }
    }
    std::printf("kernel ok: %d squares\n", n);
    return 0;
}
