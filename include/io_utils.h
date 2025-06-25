#pragma once
#include <exception>
#include <string>
#include <functional>
#include <fstream>
#include <optional>
#include "common.h"
namespace ra {

namespace utils {
namespace io {
template <typename T>
class TxtIO {
public:
    TxtIO(const std::string& filePath)
        : fin(filePath)
    {
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open file: " + filePath);
        }
    }
    using ProcessFuncType = std::function<void(const T&)>;
    using ParseFuncType = std::function<std::optional<T>(const std::string&)>;

    ~TxtIO() = default;

    // 加载文件
    void Process()
    {
        if (!fin.is_open()) {
            LOG(ERROR) << "File is not open.";
        }
        if (!processFunc) {
            LOG(ERROR) << "Process function is not set.";
            return;
        }
        if (!parseFunc) {
            LOG(ERROR) << "Parse function is not set.";
            return;
        }
        while (!fin.eof()) {
            std::string line;
            std::getline(fin, line);
            if (line.empty()) {
                continue;
            }

            if (line[0] == '#') {
                // 以#开头的是注释
                continue;
            }

            // load data from line
            std::stringstream ss;
            ss << line;
            auto data = parseFunc(line);
            if (data) {
                processFunc(*data);
            }
        }

        LOG(INFO) << "done.";
    }

    // 处理文件
    TxtIO& SetProcessFunc(ProcessFuncType func)
    {
        processFunc = std::move(func); // 如果用户传入的是一个临时函数会更高效
        return *this;
    }
    TxtIO& SetParseFunc(ParseFuncType func)
    {
        parseFunc = std::move(func);
        return *this;
    }

private:
    std::ifstream fin;           // 文件输入流
    ProcessFuncType processFunc; // 处理函数
    ParseFuncType parseFunc;     // 解析函数
};

} // namespace ra::utils::io

} // namespace ra::utils

} // namespace ra
