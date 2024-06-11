#include <algorithm>
#include <array>
#include <cuda_runtime.h>
#include <stdio.h>

constexpr int N = 512;
constexpr int MAX_BLOCK_SIZE = 1024; // Typical max block size

__global__ void add(int *a, int *b, int *c)
{
    int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid < N)
        c[tid] = a[tid] + b[tid];
}

class GpuArray
{
public:
    explicit GpuArray(size_t n) : size_(n * sizeof(int))
    {
        cudaMalloc(&data_, size_);
    }

    ~GpuArray()
    {
        cudaFree(data_);
    }

    void copyToDevice(const std::array<int, N> &host_array)
    {
        cudaMemcpy(data_, host_array.data(), size_, cudaMemcpyHostToDevice);
    }

    void copyToHost(std::array<int, N> &host_array)
    {
        cudaMemcpy(host_array.data(), data_, size_, cudaMemcpyDeviceToHost);
    }

    int *data() const
    {
        return data_;
    }

private:
    int *data_ = nullptr;
    size_t size_;
};

int main()
{
    std::array<int, N> a, b, c;

    // fill with random numbers
    for (auto &val : a)
    {
        val = rand() % 100;
    }
    for (auto &val : b)
    {
        val = rand() % 100;
    }

    GpuArray d_a(N), d_b(N), d_c(N);

    d_a.copyToDevice(a);
    d_b.copyToDevice(b);

    constexpr int block_size = (N < MAX_BLOCK_SIZE) ? N : MAX_BLOCK_SIZE;
    constexpr int grid_size = (N + block_size - 1) / block_size;
    add<<<grid_size, block_size>>>(d_a.data(), d_b.data(), d_c.data());

    // check for errors
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        fprintf(stderr, "Failed to launch kernel: %s\n", cudaGetErrorString(err));
        return 1;
    }

    d_c.copyToHost(c);

    // print results
    for (int i = 0; i < N; i++)
    {
        printf("%d + %d = %d\n", a[i], b[i], c[i]);
    }

    return 0;
}
