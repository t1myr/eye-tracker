#pragma once

#include <bits/stdc++.h>
#include "all.hpp"


/**
 * @brief Класс для работы с набором битов (аналог std::bitset, только с поддержкой разных типов данных). 
 * @tparam Integral Тип, который хранится в наборе. Должен быть беззнаковым.
 * @tparam N Количество битов в наборе. Должно быть меньше или равно количеству битов в типе, который хранится в наборе.
 */
template<typename Integral, typename std::size_t N = sizeof(Integral) * 8>
struct BitSet
{
	using type_t = Integral;

	static_assert(std::is_unsigned<Integral>::value, "BitSet type cannot be signed");

	/**
	 * @brief Конструктор по умолчанию. Создает пустой набор.
	 */
	constexpr BitSet() = default;

	/**
	 * @brief Конструктор от значения типа, который хранится в наборе
	 * @param value Значение типа, который хранится в наборе
	 * @tparam T Тип значения. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	explicit constexpr BitSet(T value) noexcept
	{
		//когда присваиваем значение обрезаем лишние биты
		type_t mask = ~static_cast<type_t>(0);
		mask >>= sizeof(type_t) * allignment - N;
		_value = value & mask;
	}

	/**
	 * @brief Конструктор копирования
	 * @param other другой BitSet
	 */
	BitSet(const BitSet& other) = default;

	/**
	 * @brief Конструктор перемещения
	 * @param other другой BitSet
	 */
	BitSet(BitSet&& other) = default;

	/**
	 * @brief Приводим BitSet к значению типа, который хранится в наборе
	 * @return Значение типа, который хранится в наборе
	 */
	inline operator type_t() const
	{
		return _value;
	}

	/**
	 * @brief Добавляем элемент в набор. Данный оператор создает копию объекта и возвращает её.
	 * @param index Бит, который нужно добавить в набора
	 * @tparam T Тип элемента. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	inline BitSet operator+(T index) const noexcept
	{
		BitSet newSet(*this);
		newSet.set(index);
		return newSet;
	}

	/**
	 * @brief Удаляем элемент из набора. Данный оператор создает копию объекта и возвращает её.
	 * @param index Бит, который нужно сбросить в наборе
	 * @tparam T Тип элемента. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	inline BitSet operator-(T index) const noexcept
	{
		BitSet newSet(*this);
		newSet.reset(index);
		return newSet;
	}

	/**
	 * @brief Добавляем элемент в набор
	 * @param index Бит, который нужно выставить в наборе
	 * @tparam T Тип элемента. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	inline BitSet& operator+=(T index) noexcept
	{
		set_impl(index);
		return *this;
	}

	/**
	 * @brief Удаляем элемент из набора
	 * @param index Бит, который нужно сбросить в наборе
	 * @tparam T Тип элемента. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	inline BitSet& operator-=(T index) noexcept
	{
		reset_impl(index);
		return *this;
	}

	/**
	 * @brief Проверяем, выставлены ли в наборе все биты в true для переданных индексов
	 * @param args Индексы, которые проверяем на то, выставлены в true они, или нет
	 * @param Args Parameter pack из индексов, которые проверяем на включение в текущий набор. Должны быть такого же типа, как и элементы набора.
	 * @return true, если по всем переданным индексам биты выставлены, иначе false
	 */	
	template<typename... Args, typename = typename std::enable_if<all_unsigned<Integral, Args...>::value>::type>
	inline bool has(Args&&... args) const noexcept
	{
		for(const auto& arg : {args...})
		{
			if(!has_impl(arg)) return false;
		}
		return true;
	}

	/**
	 * @brief Проверяем, выставлены ли все биты из второго набора в первом
	 * @param other BitSet, который проверяем на включение в текущий набор
	 * @return true, если текущий набор включает все элементы другого набора, иначе false
	 */
	inline bool has(const BitSet& other) const noexcept
	{
		return (_value & other._value) == other._value;
	}

	/**
	 * @brief Проверяем, выставлен ли в наборе хотя бы один бит
	 * @return true, если в наборе есть хотя бы один элемент, иначе false
	 */
	inline bool any() const noexcept
	{
		return _value;
	}

	/**
	 * @brief Проверяем, выставлены ли все биты в наборе
	 * @return true, если все биты в наборе выставлены, иначе false
	 */
	inline bool all() const noexcept
	{
		type_t mask = ~static_cast<type_t>(0);
		mask >>= sizeof(type_t) * allignment - N;
		return _value == mask;
	}

	/**
	 * @brief Проверяем, что в наборе не выставлены ни один бит
	 * @return true, если в наборе не выставлены ни один бит, иначе false
	 */
	inline bool empty() const noexcept
	{
		return !_value;		
	}

	/**
	 * @brief Выставляем биты в наборе
	 * @param args Индексы битов, которые мы хотим выставить в true в наборе
	 * @tparam Args Parameter pack значений, которые нужно установить в наборе, должен быть того же типа, что и Enum
	 */
	template<typename... Args, typename = typename std::enable_if<all_unsigned<Args...>::value>::type>
	inline void set(Args&&... args) noexcept
	{
		for(const auto& arg : {args...})
		{
			set_impl(arg);
		}
	}

	/**
	 * @brief Сбрасываем биты в наборе
	 * @param args Индексы битов, которые мы хотим выставить в false в наборе
	 * @tparam Args Parameter pack значений, которые нужно сбросить в наборе, должен быть того же типа, что и Enum
	 */
	template<typename... Args, typename = typename std::enable_if<all_unsigned<Args...>::value>::type>
	inline void reset(Args&&... args) noexcept
	{
		//проверяем количество аргументов
		if(!sizeof...(args))
		{
			//если аргументов нет, сбрасываем значение
			_value = 0;
			return;
		}

		//если аргументы есть - вызываем хелпер
		for(const auto arg : {args...})
			reset_impl(arg);
	}

	/**
	 * @brief Возвращаем значение набора
	 * @return Значение набора
	 */
	inline type_t value() const noexcept
	{
		return _value;
	}

	/**
	 * @brief Возвращаем размер набора в битах
	 * @return Размер набора в битах
	 */
	inline constexpr type_t size() const noexcept
	{
		return N;
	}

	/**
	 * @brief Считаем количество бит, выставленных в 1 в наборе
	 * @return Количество бит, выставленных в 1 в наборе
	 */
	inline constexpr type_t count() const noexcept
	{
		//https://stackoverflow.com/questions/109023/count-the-number-of-set-bits-in-a-32-bit-integer/109025#109025
		return __builtin_popcount(_value);
	}

	/**
	 * @brief Возвращаем строковое представление набора
	 * @return Строковое представление набора в бинарном виде
	 */
	const char* to_string() const noexcept
	{
		char str[N];
		for(type_t i = 0; i < N; ++i)
		{
			type_t mask = static_cast<type_t>(1) << i;
			str[i] = (_value & mask) ? '1' : '0';
		}
		return str;
	}

private:

	static constexpr type_t allignment = 8;

	static_assert(N <= sizeof(type_t) * allignment, "size exceeds the bit width of the underlying type");

	/**
	 * @brief Класс для работы с отдельным битом в наборе
	 */
	struct reference
    {
    public:
		/**
		 * @brief В конструкторе ссылки сохраняем ссылку на основной контейнер и индекс бита, с которым будем работать
		 */
        reference(type_t& value, Integral index) 
            : _value(value), _index(index) {}

		/**
		 * @brief Оператор приведения к bool для проверки значения бита в наборе
		 */
        operator bool() const noexcept
        {
            return _value & (static_cast<type_t>(1) << static_cast<type_t>(_index));
        }

		/**
		 * @brief Оператор присваивания для установки значения бита в наборе
		 * @param flag Значение, которое нужно установить в бите
		 */
        reference& operator=(bool flag) noexcept
        {
            if (flag)
                _value |= (static_cast<type_t>(1) << static_cast<type_t>(_index));
            else
                _value &= ~(static_cast<type_t>(1) << static_cast<type_t>(_index));
            return *this;
        }

		/**
		 * @brief Оператор присваивания для копирования значения из другой ссылки
		 * @param other Ссылка, из которой нужно скопировать значение
		 */
        reference& operator=(const reference& other) noexcept
        {
            return *this = static_cast<bool>(other);
        }

    private:
        type_t& _value;
        Integral _index;
    };

	type_t _value{0};

	/**
	 * @brief Проверяем выставлен ли бит номер index в наборе
	 * @param index Номер бита, который нужно проверить
	 * @return true, если бит выставлен, иначе false
	 */
	inline bool has_impl(Integral index) const noexcept
	{
		return _value & (static_cast<type_t>(1) << static_cast<type_t>(index));
	}

	/**
	 * @brief Устанавливаем бит номер index в наборе
	 * @param index Номер бита, который нужно установить
	 */
	inline void set_impl(Integral index) noexcept
	{
		_value |= (static_cast<type_t>(1) << static_cast<type_t>(index));
	}

	/**
	 * @brief Сбрасываем бит номер index в наборе
	 * @param index Номер бита, который нужно сбросить
	 */
	inline void reset_impl(Integral index) noexcept
	{
		_value &= ~(static_cast<type_t>(1) << static_cast<type_t>(index));
	}
public:

	/**
	 * Небезопасный оператор, который генерирует UB при индексе, выходящем из диапазона значений
	 * @brief Оператор доступа к биту по индексу 
	 * @param index Индекс бита, к которому нужно получить доступ
	 * @tparam T Тип индекса, должен быть таким же, как и Enum
	 * @return Ссылка на бит по указанному индексу
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	inline reference operator[](T index) noexcept
	{
		return reference(_value, index);
	}

	/**
	 * Небезопасный оператор, который генерирует UB при индексе, выходящем из диапазона значений
	 * @brief Оператор доступа к биту по индексу 
	 * @param index Индекс бита, к которому нужно получить доступ
	 * @tparam T Тип индекса, должен быть таким же, как и Enum
	 * @return Ссылка на бит по указанному индексу
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	inline const reference operator[](T index) const noexcept
	{
		return reference(_value, index);
	}

	/**
	 * @brief Метод доступа к биту по индексу 
	 * @param index Индекс бита, к которому нужно получить доступ
	 * @tparam T Тип индекса, должен быть таким же, как и Enum
	 * @return Ссылка на бит по указанному индексу
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	inline reference at(T index) noexcept
	{
		if(index >= N) throw std::out_of_range("BitSet::at: index out of range");
		return reference(_value, index);
	}

	/**
	 * @brief Метод доступа к биту по индексу 
	 * @param index Индекс бита, к которому нужно получить доступ
	 * @tparam T Тип индекса, должен быть таким же, как и Enum
	 * @return Ссылка на бит по указанному индексу
	 */
	template<typename T, typename = typename std::enable_if<std::is_unsigned<T>::value>::type>
	inline const reference at(T index) const noexcept
	{
		if(index >= N) throw std::out_of_range("BitSet::at: index out of range");
		return reference(_value, index);
	}
};

