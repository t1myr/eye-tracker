#include <cstdint>

template <char... chars>
using str_t = std::integer_sequence<char, chars...>;


template <typename T, T... chars>
constexpr str_t<chars...> operator""_tstr() {
    return {};
}

#define str_to_literal(str) decltype(str##_tstr)


struct SuperBase
{
    using value_t = uint8_t;

    SuperBase(value_t val) : value(val) {}

    value_t value;

    virtual void pureVirtualFunc() = 0;

    virtual const char* to_string() const {return "Unknown";}

    virtual ~SuperBase() {}
};

template<uint8_t value, typename...>
struct Base;

template<uint8_t value, char... elements, typename Payload>
struct Base<value, str_t<elements...>, Payload> : public SuperBase
{
    Base() : SuperBase(value) {}
    virtual ~Base() {}

    virtual void pureVirtualFunc() override {}

    const char* to_string() const override final 
    {
        static constexpr char str[sizeof...(elements) + 1] = { elements..., '\0' };
        return str;
    }

    Payload payload;
};

template<uint8_t value, char... elements>
struct Base<value, str_t<elements...>> : public SuperBase
{
    Base() : SuperBase(value) {}
    virtual ~Base() {}

    virtual void pureVirtualFunc() override {}

    const char* to_string() const override final 
    {
        static constexpr char str[sizeof...(elements) + 1] = { elements..., '\0' };
        return str;
    }
};

template<uint8_t value, typename...>
struct Derived;

template<uint8_t value, char... elements, typename Payload>
struct Derived<value, str_t<elements...>, Payload> : public Base<value, str_t<elements...>, Payload>
{
    Derived() : Base<value, str_t<elements...>, Payload>() {}
    virtual ~Derived() {}

    virtual void pureVirtualFunc() override final {}

    Payload payload;
};

template<uint8_t value, char... elements>
struct Derived<value, str_t<elements...>> : public Base<value, str_t<elements...>>
{
    Derived() : Base<value, str_t<elements...>>() {}
    virtual ~Derived() {}

    virtual void pureVirtualFunc() override final {}
};