#include "util.h"
namespace ra {
namespace util {
void parallel_for(size_t start, size_t end, const std::function<void(size_t)>& func)
{
    size_t num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    size_t chunk_size = (end - start + num_threads - 1) / num_threads;
    for (size_t t = 0; t < num_threads; ++t) {
        size_t chunk_start = start + t * chunk_size;
        size_t chunk_end = std::min(chunk_start + chunk_size, end);

        threads.emplace_back([=]() {
            for (size_t i = chunk_start; i < chunk_end; ++i) {
                func(i);
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }
}
}
}