#ifndef _CAST_HPP_
#define _CAST_HPP_

#include "assert.hpp"
#include "../logging/logger.hpp"

// Проверка и аварийное завершение при неудаче (общая логика)
inline void reportBadCast(const char* file, int line,
                          const char* fromType, const char* toType)
{
    spdlog::critical("[CRIT] dynamic_cast failed at {}:{} from {} to {}", file, line, fromType, toType);
    std::abort();
}

/// @brief Для указателей
template<typename To, typename From>
To* dynamicCastPointerImpl(From* value, const char* file, int line)
{
    To* result = dynamic_cast<To*>(value);
    if (!result)
    {
        reportBadCast(file, line, typeid(From).name(), typeid(To).name());
    }
    return result;
}

/// @brief Для ссылок
template<typename To, typename From>
To& dynamicCastRefImpl(From& value, const char* file, int line)
{
    try
    {
        return dynamic_cast<To&>(value);
    }
    catch (const std::bad_cast&)
    {
        reportBadCast(file, line, typeid(From).name(), typeid(To).name());
        throw; // avoid compiler warning
    }
}

#define dynamicCastPtr(To, FromVal) \
    dynamicCastPointerImpl<To>(FromVal, __FILE__, __LINE__)

#define dynamicCastRef(To, FromVal) \
    dynamicCastRefImpl<To>(FromVal, __FILE__, __LINE__)

#endif