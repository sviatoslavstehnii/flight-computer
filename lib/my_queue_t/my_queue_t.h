#include <queue>


template <typename T>
class my_queue_t {
public:
    my_queue_t(size_t max_size) : max_size(max_size), queue_() {};

    void push(const T& value) {
        if (queue_.size() >= max_size) {
            queue_.pop();
        }
        queue_.push(value);
    }

    T pop() {
        T value = queue_.front();
        queue_.pop();
        return value;
    }

    bool empty() const {
        return queue_.empty();
    }

    size_t size() const {
        return queue_.size();
    }

    void clear() {
        while (!queue_.empty()) {
            queue_.pop();
        }
    }

private:
    std::queue<T> queue_;
    size_t max_size;
};
