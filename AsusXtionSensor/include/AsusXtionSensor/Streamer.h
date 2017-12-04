#pragma once

#include <functional>

namespace ar {
namespace AsusXtionSensor {

template <typename T>
using ReadTFunction = std::function<void(T&)>;

template<typename T>
class Streamer_
{
public:
    Streamer_()
        : m_readFunction(static_cast<ReadTFunction<T>>(NULL))
    {}

    Streamer_(ReadTFunction<T> readFunction)
        : m_readFunction(readFunction)
    {}
    ~Streamer_() = default;
    
    void read(T& target)
    {
        m_readFunction(target);
    }

protected:
    ReadTFunction<T> m_readFunction;
};

template<typename T>
Streamer_<T>& operator>>(Streamer_<T>& stream, T& target)
{
    stream.read(target);
    return stream;
}

} // namespace ar
} // namespace AsusXtionSensor
