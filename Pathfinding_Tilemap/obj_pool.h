#ifndef OBJ_POOL_HPP
#define OBJ_POOL_HPP

#include <vector>


template <typename T>
class ObjectPool {
public:
    void init(int n) {
        if (!pool.empty()) {
            return;
        }
        pool.reserve(n);
        for (int i = 0; i < n; ++i) {
            pool.push_back(std::make_shared<T>());
        }
    }

    std::shared_ptr<T> acquire() {
        if (!pool.empty()) {
            auto obj = pool.back();
            pool.pop_back();
            return obj;
        } else {
            return std::make_shared<T>();
        }
    }

    void release(std::shared_ptr<T> obj) {
        pool.push_back(obj);
    }

private:
    std::vector<std::shared_ptr<T>> pool;
};

#endif
