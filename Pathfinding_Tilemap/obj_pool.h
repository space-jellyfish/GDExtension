//see https://stackoverflow.com/questions/27827923/c-object-pool-that-provides-items-as-smart-pointers-that-are-returned-to-pool
#ifndef OBJ_POOL_HPP
#define OBJ_POOL_HPP

#include <stack>
#include <memory>
#include <typeindex>
#include <any>
#include <godot_cpp/variant/utility_functions.hpp>


class MultiTypeObjectPool {
public:
    template <typename T>
    void init(int n) {
        auto& pool = getPool<T>();
        if (!pool.empty()) {
            return;
        }
        for (int i = 0; i < n; ++i) {
            pool.push(std::shared_ptr<T>(static_cast<T*>(nullptr), Deleter(*this)));
        }
    }
    
    template <typename T>
    std::shared_ptr<T> acquire() {
        auto& pool = getPool<T>();
        if (!pool.empty()) {
            auto obj = pool.top();
            pool.pop();
            assert(obj != nullptr);
            return std::move(obj);
        } else {
            return std::shared_ptr<T>(static_cast<T*>(nullptr), Deleter(*this));
        }
    }

    template <typename T>
    void add(std::shared_ptr<T> obj) {
        auto& pool = getPool<T>();
        pool.push(std::move(obj));
    }

private:
    struct Deleter {
        Deleter(MultiTypeObjectPool& _p) : p(_p) {}

        template <typename T>
        void operator()(T* ptr) {
            p.add(std::shared_ptr<T>(ptr, Deleter(*this)));
        }
    private:
        MultiTypeObjectPool& p;
    };

    template <typename T>
    std::stack<std::shared_ptr<T>>& getPool() {
        auto typeId = std::type_index(typeid(T));
        if (pools.find(typeId) == pools.end()) {
            pools[typeId] = std::make_any<std::stack<std::shared_ptr<T>>>();
        }
        return std::any_cast<std::stack<std::shared_ptr<T>>&>(pools[typeId]);
    }

    std::unordered_map<std::type_index, std::any> pools;
};

#endif
