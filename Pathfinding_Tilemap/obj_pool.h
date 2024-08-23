//see https://stackoverflow.com/questions/27827923/c-object-pool-that-provides-items-as-smart-pointers-that-are-returned-to-pool
#ifndef OBJ_POOL_HPP
#define OBJ_POOL_HPP

#include <stack>
#include <typeindex>


class MultiTypeObjectPool {
public:
    template <typename T>
    void init(int n) {
        auto& pool = getPool<T>();
        if (!pool.empty()) {
            return;
        }
        pool.reserve(n);
        for (int i = 0; i < n; ++i) {
            pool.push(std::make_shared<T>());
        }
    }
    
    template <typename T>
    std::shared_ptr<T> acquire() {
        auto& pool = getPool<T>();
        if (!pool.empty()) {
            auto obj = pool.top();
            pool.pop();
            return std::shared_ptr<T>(obj.get(), [this](T* ptr){ release(std::shared_ptr<T>(ptr)); });
        } else {
            return std::shared_ptr<T>(new T, [this](T* ptr){ release(std::shared_ptr<T>(ptr)); });
        }
    }

    template <typename T>
    void release(std::shared_ptr<T> obj) {
        auto& pool = getPool<T>();
        pool.push(obj);
    }

private:
    template <typename T>
    std::stack<std::shared_ptr<T>>& getPool() {
        auto typeId = std::type_index(typeid(T));
        if (pools.find(typeId) == pools.end()) {
            pools[typeId] = std::make_unique<std::stack<std::shared_ptr<T>>>();
        }
        return *std::static_pointer_cast<std::stack<std::shared_ptr<T>>>(pools[typeId]);
    }

    std::unordered_map<std::type_index, std::unique_ptr<void>> pools;
};

#endif
