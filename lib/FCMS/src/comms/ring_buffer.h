#include <array>
#include <cstdint>
#include <cstring>
#include <optional>

class RingBuffer
{
public:
    static constexpr std::size_t BUFFER_MAX_SIZE = 256;

    bool push(uint8_t data)
    {
        if (isFull())
        {
            ++start_;
        }
        buffer[end_++] = data;
        if (end_ >= BUFFER_MAX_SIZE)
        {
            end_ = 0;
            overlap_ = true;
        }
        // advance(end_);
        return true;
    }

    std::optional<uint8_t> pop()
    {
        if (isEmpty())
        {
            return std::nullopt;
        }
        uint8_t data = buffer[start_++];
        if (start_ >= BUFFER_MAX_SIZE)
        {
            start_ = 0;
            overlap_ = false;
        }
        // advance(start_);
        return data;
    }

    bool isEmpty() const
    {
        return start_ == end_ && !overlap_;
    }

    bool isFull() const
    {
        return size() == BUFFER_MAX_SIZE;
    }

    void clear()
    {
        start_ = 0;
        end_ = 0;
        overlap_ = false;
    }

    std::optional<uint8_t> get(std::size_t idx) const
    {
        if (idx >= size())
        {
            return std::nullopt;
        }
        std::size_t pos = (start_ + idx) % BUFFER_MAX_SIZE;
        return buffer[pos];
    }

    std::optional<uint8_t> peek() const
    {
        return get(0);
    }

    bool erase(std::size_t len)
    {
        if (len > size())
        {
            return false;
        }
        start_ = (start_ + len) % BUFFER_MAX_SIZE;
        if (start_ < end_)
        {
            overlap_ = false;
        }
        return true;
    }

    bool copy(uint8_t *dest, std::size_t size) const
    {
        if (size > this->size())
        {
            return false;
        }

        std::size_t firstChunk = std::min(size, BUFFER_MAX_SIZE - start_);
        std::memcpy(dest, buffer.data() + start_, firstChunk);
        if (size > firstChunk)
        {
            std::memcpy(dest + firstChunk, buffer.data(), size - firstChunk);
        }
        return true;
    }

    std::size_t size() const
    {
        if (overlap_)
        {
            return BUFFER_MAX_SIZE - start_ + end_;
        }
        return end_ - start_;
    }

private:
    std::array<uint8_t, BUFFER_MAX_SIZE> buffer{};
    std::size_t start_ = 0;
    std::size_t end_ = 0;
    bool overlap_ = false;

    void advance(std::size_t &index)
    {
        index = (index + 1) % BUFFER_MAX_SIZE;
        if (index == start_)
        {
            overlap_ = !overlap_;
        }
    }
};
