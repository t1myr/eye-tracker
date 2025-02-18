#pragma once

#include <type_traits>
#include <exception>

struct bad_optional_access : std::exception
{
    const char* what() const noexcept override
    {
        return "bad optional access";
    }
};

struct nullopt_t 
{
    enum class _Construct { _Token };
    constexpr explicit nullopt_t(_Construct) noexcept {}
};

constexpr nullopt_t nullopt{nullopt_t::_Construct::_Token};

struct in_place_t
{
    explicit in_place_t() = default;
};

namespace detail
{
    template <typename T, typename U>
    struct constructible;

    template <typename T, typename U>
    struct convertible;

    template <typename T, typename U>
    struct assignable;

    template<typename T, typename U>
    struct ctor_convert_assign;
}

template<class T, typename = void>
struct optional_storage
{
    union 
    {
        char _dummy;
        T _val;
    };

    bool _engaged;
    
    ~optional_storage() 
    {
        if(_engaged) _val.~T();
    }
};

template<class T>
struct optional_storage<T, typename std::enable_if<std::is_trivially_destructible<T>::value>::type>
{
    union 
    {
        char _dummy;
        T _val;
    };

    bool _engaged;
    
    ~optional_storage() = default;
};

template<class T>
struct optional 
{
    /**
     * @brief 
     */
    constexpr optional() noexcept : _storage{._dummy = 0, ._engaged = false}
    {}

    /**
     * @brief 
     */
    constexpr optional(nullopt_t) noexcept : _storage{._dummy = 0, ._engaged = false}
    {}

    /**
     * @brief 
     */
    template <typename U = T, std::enable_if_t<!std::is_trivially_copy_constructible<U>::value>>
    optional(const optional& other)
    {
        if(other._storage._engaged)
        {
            new(&_storage._val) T(other._storage._val);
            _storage._engaged = true;
        }else
        {
            _storage._dummy = 0;
            _storage._engaged = false;
        }
    }

    template <typename U = T, std::enable_if_t<!std::is_copy_constructible<U>::value>>
    optional(const optional& other) = delete;

    optional(const optional& other) = default;

    /**
     * @brief 
     */
    template <typename U = T, std::enable_if_t<!std::is_trivially_move_constructible<U>::value>>
    optional(optional&& other) noexcept(std::is_nothrow_move_constructible<U>::value)
    {
        if(other._storage._engaged)
        {
            new(&_storage._val) T(std::move(other._storage._val));
            _storage._engaged = true;
            other._storage._engaged = false;
        }else
        {
            _storage._dummy = 0;
            _storage._engaged = false;
        }
    }

    template <typename U = T, typename std::enable_if_t<!std::is_move_constructible<U>::value, bool> = 0>
    optional(optional&& other) = delete;

    optional(optional&& other) = default;

    /**
     * @brief 
     */
    template<class U, std::enable_if_t<(!detail::convertible<T, U>::value && !detail::constructible<T, U>::value) ||
                                                                     std::is_constructible<T, const U&>::value>>
    constexpr optional(const optional<U>& other)
    {
        if(other._storage._engaged)
        {
            new(&_storage._val) T(other._storage._val);
            _storage._engaged = true;
        }else
        {
            _storage._dummy = 0;
            _storage._engaged = false;
        }
    }

    /**
     * @brief
     */
    template<class U, std::enable_if_t<(!detail::convertible<T, U>::value && !detail::constructible<T, U>::value) ||
                                                                    std::is_constructible<T, U&&>::value>>
    constexpr optional(optional<U>&& other)
    {
        if(other._storage._engaged)
        {
            new(&_storage._val) T(std::move(other._storage._val));
            _storage._engaged = true;
            other._storage._engaged = false;
        }else
        {
            _storage._dummy = 0;
            _storage._engaged = false;
        }
    }

    template<class... Args, std::enable_if_t<std::is_constructible<T, Args...>::value>>
    constexpr explicit optional(in_place_t, Args&&... args)
    {
        new(&_storage._val) T(std::forward<Args>(args)...);
        _storage._engaged = true;
    }

    template<class U, class... Args, std::enable_if_t<std::is_constructible<T, std::initializer_list<U>&, Args...>::value>>
    constexpr explicit optional(in_place_t, std::initializer_list<U> il, Args&&... args)
    {
        new(&_storage._val) T(il, std::forward<Args>(args)...);
        _storage._engaged = true;
    }

    /**
     * @brief
     */
    template <typename U, std::enable_if_t<
                                            std::is_constructible<T, U>::value &&
                                            !std::is_same<optional<T>, typename std::decay_t<U>>::value && 
                                            !std::is_same<in_place_t, typename std::decay_t<U>>::value &&
                                            (
                                                std::is_same<typename std::decay_t<T>, bool>::value && 
                                                !std::is_same<optional, typename std::decay_t<U>>::value
                                            )
                                            >
            >
    constexpr optional(U&& value)
    {
        new(&_storage._val) T(std::forward<U>(value));
        _storage._engaged = true;
    }

    /**
     * @brief Проверяем, инициализирован ли объект
     */
    bool has_value() const noexcept
    {
        return _storage._engaged;
    }

    template<class... Args>
    T& emplace(Args&&... args)
    {   
        if(_storage._engaged)
        {
            _storage._val.~T();
            _storage._engaged = false;
        }

        new(&_storage._val) T(std::forward<Args>(args)...);
        _storage._engaged = true;

        return _storage._val;
    }

    template<class U, class... Args, std::enable_if_t<std::is_constructible<T, std::initializer_list<U>&, Args&&...>::value>>
    T& emplace(std::initializer_list<U> il, Args&&... args)
    {
        if(_storage._engaged)
        {
            _storage._val.~T();
            _storage._engaged = false;
        }

        new(&_storage._val) T(il, std::forward<Args>(args)...);
        _storage._engaged = true;

        return _storage._val;
    }

    T& value() &
    {
        return _storage._engaged ? _storage._val : throw bad_optional_access();
    }

    const T& value() const &
    {
        return _storage._engaged ? _storage._val : throw bad_optional_access();
    }

    T&& value() &&
    {
        return _storage._engaged ? std::move(_storage._val) : throw bad_optional_access();
    }

    const T&& value() const &&
    {
        return _storage._engaged ? std::move(_storage._val) : throw bad_optional_access();
    }

    template<class U>
    T value_or(U&& u) &&
    {
        return _storage._engaged ? 
                std::move(_storage._val) : static_cast<T>(std::forward<U>(u));
    }

    template<class U>
    T value_or(U&& u) const &
    {
        return _storage._engaged ? 
                std::move(_storage._val) : static_cast<T>(std::forward<U>(u));
    }

    optional& operator=(nullopt_t) noexcept
    {
        if(_storage._engaged)
        {
            _storage._val.~T();
            _storage._engaged = false;
        }
        return *this;
    }

    template<typename U = T, std::enable_if_t<!std::is_trivially_copy_constructible<U>::value ||
                                                        !std::is_trivially_copy_assignable<U>::value || 
                                                        !std::is_trivially_destructible<U>::value>>
    constexpr optional& operator=(const optional& other)
    {
        if(this == &other) return *this;

        if(_storage._engaged && other._storage._engaged)
        {
            _storage._val = other._storage._val;
        }else if(_storage._engaged && !other._storage._engaged)
        {
            _storage._val.~T();
            _storage._engaged = false;
        }else if(!_storage._engaged && other._storage._engaged)
        {
            new(&_storage._val) T(other._storage._val);
            _storage._engaged = true;
        }

        return *this;
    }

    template<typename U = T, std::enable_if_t<!std::is_copy_constructible<U>::value || 
                                                        !std::is_copy_assignable<U>::value>>
    constexpr optional& operator=(const optional& other) = delete;

    constexpr optional& operator=(const optional& other) = default;

    template<typename U = T, std::enable_if_t<!std::is_trivially_move_constructible<U>::value ||
                                                        !std::is_trivially_move_assignable<U>::value || 
                                                        !std::is_trivially_destructible<U>::value>>
    constexpr optional& operator=(optional&& other) noexcept(std::is_nothrow_move_assignable<T>::value &&
                                                             std::is_nothrow_move_constructible<T>::value)
    {
        if(*this == &other) return *this;
        
        if(_storage._engaged && other._storage._engaged)
        {
            _storage._val = std::move(other._storage._val);
            other._storage._engaged = false;
        }else if(_storage._engaged && !other._storage._engaged)
        {
            _storage._val.~T();
            _storage._engaged = false;
        }else if(!_storage._engaged && other._storage._engaged)
        {
            new(&_storage._val) T(std::move(other._storage._val));
            _storage._engaged = true;
            other._storage._engaged = false;
        }
        return *this;
    }

    template<typename U = T, std::enable_if_t<!std::is_move_constructible<U>::value || 
                                                        !std::is_move_assignable<U>::value>>
    constexpr optional& operator=(optional&& other) = delete;

    constexpr optional& operator=(optional&& other) = default;

    template <typename U,
        std::enable_if_t<
            !std::is_same<optional, typename std::decay_t<U>>::value && 
            std::is_constructible<T, U>::value &&                          
            std::is_assignable<T&, U>::value &&                          
            (!std::is_scalar<T>::value ||                               
            !std::is_same<typename std::decay_t<U>, T>::value)      
        >>
    optional& operator=(U&& other)
    {
        if (_storage._engaged) {
            _storage._val = std::forward<U>(value); 
        } else {
            new (&_storage._val) T(std::forward<U>(value)); 
            _storage._engaged = true;
        }
        return *this;
    }

    template<class U, std::enable_if_t<!detail::ctor_convert_assign<T, U>::value || 
                        std::is_constructible<T, const U&>::value && std::is_assignable<T&, const U&>::value>>
    optional& operator=(const optional<U>& other)
    {
        if (this == &other)
            return *this;

        if(_storage._engaged && other._storage._engaged)
        {
            _storage._val = other._storage._val;
        }else if(_storage._engaged && !other._storage._engaged)
        {
            _storage._val.~T();
            _storage._engaged = false;
        }else if(!_storage._engaged && other._storage._engaged)
        {
            new(&_storage._val) T(other._storage._val);
            _storage._engaged = true;
        }

        return *this;
    }
 
    template<class U, std::enable_if_t<!detail::ctor_convert_assign<T, U>::value ||
                        std::is_constructible<T, U>::value && std::is_assignable<T&, U>::value>>
    optional& operator=(optional<U>&& other)
    {
        if(*this == &other) return *this;

        if(_storage._engaged && other._storage._engaged)
        {
            _storage._val = std::move(other._storage._val);
            other._storage._engaged = false;
        }else if(_storage._engaged && !other._storage._engaged)
        {
            _storage._val.~T();
            _storage._engaged = false;
        }else if(!_storage._engaged && other._storage._engaged)
        {
            new(&_storage._val) T(std::move(other._storage._val));
            _storage._engaged = true;
            other._storage._engaged = false;
        }

        return *this;
    }
    
    /**
     * @brief Оператор разыменования
     * @details Если объект не инициализирован, результат UB
     */
    constexpr T& operator*() &
    {
        return _storage._val;
    }

    /**
     * @brief Оператор разыменования
     * @details Если объект не инициализирован, результат UB
     */
    constexpr const T& operator*() const &
    {
        return _storage._val;
    }

    /**
     * @brief Оператор разыменования
     * @details Если объект не инициализирован, результат UB
     */
    constexpr T&& operator*() && 
    {
        return std::move(_storage._val);
    }

    /**
     * @brief Оператор разыменования
     * @details Если объект не инициализирован, результат UB
     */
    constexpr const T&& operator*() const && 
    {
        return std::move(_storage._val);
    }

    /**
     * @brief Pointer-like operator
     * @details Если объект не инициализирован, результат UB
     */
    constexpr T& operator->() &
    {
        return _storage._val;
    }

    /**
     * @brief Pointer-like operator
     * @details Если объект не инициализирован, результат UB
     */
    constexpr const T& operator->() const &
    {
        return _storage._val;
    }

    /**
     * @brief Pointer-like operator
     * @details Если объект не инициализирован, результат UB
     */
    constexpr T&& operator->() &&
    {
        return std::move(_storage._val);
    }

    /**
     * @brief Pointer-like operator
     * @details Если объект не инициализирован, результат UB
     */
    constexpr const T&& operator->() const &&
    {
        return std::move(_storage._val);
    }

    /**
     * @brief Оператор приведения к bool
     */
    explicit operator bool() const noexcept
    {
        return _storage._engaged;
    }

    /**
     * @brief Деструктор
     * @details По стандарту деструктор обязан быть тривиальным
     */
    ~optional() = default;

private:
    optional_storage<T> _storage;
};

namespace detail
{
    template <typename T, typename U>
    struct constructible
    {
        static constexpr bool value = std::is_constructible<T, optional<U>&>::value && 
                                        std::is_constructible<T, const optional<U>&>::value &&
                                        std::is_constructible<T, optional<U>&&>::value &&
                                        std::is_constructible<T, const optional<U>&&>::value;
    };

    template <typename T, typename U>
    struct convertible
    {
        static constexpr bool value = std::is_convertible<optional<U>&, T>::value && 
                                        std::is_convertible<const optional<U>&, T>::value &&
                                        std::is_convertible<optional<U>&&, T>::value &&
                                        std::is_convertible<const optional<U>&&, T>::value;
    };

    template <typename T, typename U>
    struct assignable
    {
        static constexpr bool value = std::is_assignable<T&, optional<U>&>::value && 
                                        std::is_assignable<T&, const optional<U>&>::value &&
                                        std::is_assignable<T&, optional<U>&&>::value &&
                                        std::is_assignable<T&, const optional<U>&&>::value;
    };

    template <typename T, typename U>
    struct ctor_convert_assign
    {
        static constexpr bool value = constructible<T, U>::value && convertible<T, U>::value && assignable<T, U>::value;
    };
}

//optional vs optional
template< class T, class U >
constexpr bool operator==( const optional<T>& lhs, const optional<U>& rhs )
{
    return (!lhs && !rhs) || 
            (lhs && rhs && *lhs == *rhs);
}

template< class T, class U >
constexpr bool operator!=( const optional<T>& lhs, const optional<U>& rhs )
{
    return !(lhs == rhs);
}

template< class T, class U >
constexpr bool operator<( const optional<T>& lhs, const optional<U>& rhs )
{
    return !lhs ? rhs : (rhs && *lhs < *rhs);
}

template< class T, class U >
constexpr bool operator<=( const optional<T>& lhs, const optional<U>& rhs )
{
    return !lhs ? rhs : (rhs && *lhs <= *rhs);
}

template< class T, class U >
constexpr bool operator>( const optional<T>& lhs, const optional<U>& rhs )
{
    return !rhs ? lhs : (lhs && *rhs < *lhs);
}

template< class T, class U >
constexpr bool operator>=( const optional<T>& lhs, const optional<U>& rhs )
{
    return !rhs ? lhs : (lhs && *rhs <= *lhs);
}



//optional vs nullopt
template< class T >
constexpr bool operator==( const optional<T>& opt, nullopt_t ) noexcept
{
    return !opt._storage._engaged;
}

template< class T >
constexpr bool operator==( nullopt_t, const optional<T>& opt ) noexcept
{
    return !opt._storage._engaged;
}

template< class T >
constexpr bool operator!=( const optional<T>& opt, nullopt_t ) noexcept
{
    return opt._storage._engaged;
}

template< class T >
constexpr bool operator!=( nullopt_t, const optional<T>& opt ) noexcept
{
    return opt._storage._engaged;
}

template< class T >
constexpr bool operator<( const optional<T>& opt, nullopt_t ) noexcept
{
    return !opt._storage._engaged;
}

template< class T >
constexpr bool operator<( nullopt_t, const optional<T>& opt ) noexcept
{
    return opt._storage._engaged;
}

template< class T >
constexpr bool operator<=( const optional<T>& opt, nullopt_t ) noexcept
{
    return !opt._storage._engaged;
}

template< class T >
constexpr bool operator<=( nullopt_t, const optional<T>& opt ) noexcept
{
    return opt._storage._engaged;
}

template< class T >
constexpr bool operator>( const optional<T>& opt, nullopt_t ) noexcept
{
    return opt._storage._engaged;
}

template< class T >
constexpr bool operator>( nullopt_t, const optional<T>& opt ) noexcept
{
    return !opt._storage._engaged;
}

template< class T >
constexpr bool operator>=( const optional<T>& opt, nullopt_t ) noexcept
{
    return opt._storage._engaged;
}

template< class T >
constexpr bool operator>=( nullopt_t, const optional<T>& opt ) noexcept
{
    return !opt._storage._engaged;
}


/////////////////////////////////////////////////////////////////////
template< class T, class U >
constexpr bool operator==( const optional<T>& opt, const U& value )
{
    return opt._storage._engaged && opt._storage._val == value;
}

template< class U, class T >
constexpr bool operator==( const U& value, const optional<T>& opt )
{
    return opt._storage._engaged && opt._storage._val == value;
}

template< class T, class U >
constexpr bool operator!=( const optional<T>& opt, const U& value )
{
    return !opt._storage._engaged || opt._storage._val != value;
}

template< class U, class T >
constexpr bool operator!=( const U& value, const optional<T>& opt )
{
    return !opt._storage._engaged || opt._storage._val != value;
}

template< class T, class U >
constexpr bool operator<( const optional<T>& opt, const U& value )
{
    return !opt._storage._engaged || opt._storage._val < value;
}

template< class U, class T >
constexpr bool operator<( const U& value, const optional<T>& opt )
{
    return opt._storage._engaged || value < opt._storage._val;
}

template< class T, class U >
constexpr bool operator<=( const optional<T>& opt, const U& value )
{
    return !opt._storage._engaged || opt._storage._val <= value;
}

template< class U, class T >
constexpr bool operator<=( const U& value, const optional<T>& opt )
{
    return opt._storage._engaged || value <= opt._storage._val;
}

template< class T, class U >
constexpr bool operator>( const optional<T>& opt, const U& value )
{
    return opt._storage._engaged || opt._storage._val > value;
}

template< class U, class T >
constexpr bool operator>( const U& value, const optional<T>& opt )
{
    return !opt._storage._engaged || value > opt._storage._val;
}

template< class T, class U >
constexpr bool operator>=( const optional<T>& opt, const U& value )
{
    return opt._storage._engaged || opt._storage._val >= value;
}

template< class U, class T >
constexpr bool operator>=( const U& value, const optional<T>& opt )
{
    return !opt._storage._engaged || value >= opt._storage._val;
}
