#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <thread>
#include <vector>
namespace ra {
namespace util {
void parallel_for(size_t start, size_t end, const std::function<void(size_t)>& func);

template <typename Iterator, typename Func>
void parallel_for(Iterator begin, Iterator end, Func func)
{
    size_t num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    size_t total_elements = std::distance(begin, end);
    size_t chunk_size = (total_elements + num_threads - 1) / num_threads;

    auto chunk_begin = begin;
    for (size_t t = 0; t < num_threads; ++t) {
        auto chunk_end = chunk_begin;
        std::advance(chunk_end, std::min(chunk_size, static_cast<size_t>(std::distance(chunk_begin, end))));

        threads.emplace_back([chunk_begin, chunk_end, &func]() {
            for (auto it = chunk_begin; it != chunk_end; ++it) {
                func(*it);
            }
        });

        chunk_begin = chunk_end;
        if (chunk_begin == end) {
            break;
        }
    }

    for (auto& thread : threads) {
        thread.join();
    }
}
}
namespace math {
/**
 * 计算一个容器内数据的均值与矩阵形式协方差
 * @tparam C    容器类型
 * @tparam int 　数据维度
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个Eigen::Matrix<double, dim,1> 矢量类型
 */
template <typename C, int dim, typename Getter>
void ComputeMeanAndCov(
    const C& data, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov, Getter&& getter)
{
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = data.size();
    assert(len > 1);

    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                        [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                        [&mean, &getter](const E& sum, const auto& data) -> E {
                            auto value = getter(data).eval();
                            D v = value - mean;
                            return sum + v * v.transpose();
                        }) / (len - 1);
    // clang-format on
}
}
}
