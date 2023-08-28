#ifndef PRIORITY_STACK_H
#define PRIORITY_STACK_H

#include <vector>


template <class T, class Container = std::vector<T>, class Compare = std::less<typename Container::value_type>>
class priority_stack {
public:
    using value_type = typename Container::value_type;
    using reference = typename Container::reference;
    using const_reference = typename Container::const_reference;
    using size_type = typename Container::size_type;
    using container_type = Container;
    using value_compare = Compare;

    static_assert(is_same_v<T, value_type>, "container adaptors require consistent types");
    static_assert(is_object_v<T>, "container adaptors of non-object types forbidden because of [container.requirements].");

    priority_stack() = default;
};

#endif