#include <cstdint>
#include <type_traits>

template <typename T, typename = typename std::enable_if<std::is_enum<T>::value, T>::type>
class auto_bool
{
    T val_;

  public:
    constexpr auto_bool(T val) : val_(val) {}
    constexpr operator T() const { return val_; }
    constexpr explicit operator bool() const
    {
        return static_cast<std::underlying_type_t<T>>(val_) != 0;
    }
};

template <typename T, typename = typename std::enable_if<std::is_enum<T>::value, T>::type>
constexpr auto_bool<T> operator&(T lhs, T rhs)
{
    return static_cast<T>(
        static_cast<typename std::underlying_type<T>::type>(lhs) &
        static_cast<typename std::underlying_type<T>::type>(rhs));
}

template <typename T, typename = typename std::enable_if<std::is_enum<T>::value, T>::type>
constexpr T operator|(T lhs, T rhs)
{
    return static_cast<T>(
        static_cast<typename std::underlying_type<T>::type>(lhs) |
        static_cast<typename std::underlying_type<T>::type>(rhs));
}

template <typename T, typename = typename std::enable_if<std::is_enum<T>::value, T>::type>
constexpr T operator^(T lhs, T rhs)
{
    return static_cast<T>(
        static_cast<typename std::underlying_type<T>::type>(lhs) ^
        static_cast<typename std::underlying_type<T>::type>(rhs));
}

template <typename T, typename = typename std::enable_if<std::is_enum<T>::value, T>::type>
constexpr T operator~(T op)
{
    return static_cast<T>(
        ~static_cast<typename std::underlying_type<T>::type>(op));
}