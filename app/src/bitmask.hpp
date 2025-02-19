#pragma once

#include <bits/stdc++.h>
#include "all.hpp"

/**
 * @brief Класс, описывающий битовую маску. Работает только с перечислением.
 * @tparam Enum Перечисление, для которого создаётся маска.
 * @tparam N Количество элементов в перечислении.
 */
template<typename Enum, typename std::underlying_type<Enum>::type N>
struct BitMask
{
	using type_t = typename std::underlying_type<Enum>::type;

	static_assert(std::is_enum<Enum>::value, "BitMask can be used only with enum types");
	static_assert(std::is_unsigned<typename std::underlying_type<Enum>::type>::value, "Enum's underlying type must be unsigned");

	/**
	 * @brief Конструктор по умолчанию. Создает пустой набор.
	 */
	constexpr BitMask() = default;

	/**
	 * @brief Конструктор копирования
	 * @param other другой BitMask
	 */
	constexpr BitMask(const BitMask& other) = default;

	/**
	 * @brief Конструктор перемещения
	 * @param other другой BitMask
	 */
	constexpr BitMask(BitMask&& other) = default;

	/**
	 * @brief Конструктор с parameter pack
	 * @param indexes Список индексов, биты котоорых нужно установить в наборе
	 * @tparam Args Parameter pack из индексов битов, которые нужно выставить в наборе. Должен быть того же типа, что и Enum
	 */
	template<typename... Args, typename = typename std::enable_if<all_same<Enum, Args...>::value>::type>
	explicit constexpr BitMask(Args&&... indexes) noexcept
	{
		for(const auto& index : {indexes...})
		{
			set_impl(index);
		}
	}

	/**
	 * @brief Конструктор от значения типа, который хранится в наборе
	 * @param value Значение типа, который хранится в наборе
	 * @tparam T Тип значения. Должен быть таким же, как и тип элемента набора
	 */
	explicit constexpr BitMask(type_t value) noexcept
	{
		_value = value & get_strip_mask();
	}

	BitMask& operator=(const BitMask& other) = default;

	BitMask& operator=(BitMask&& other) = default;

	/**
	 * @brief Оператор списковой инициализации
	 * @param list Список из индексов битов, которые нужно выставить в 1
	 * @tparam T Тип элемента списка. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	BitMask& operator=(const std::initializer_list<T>& list) noexcept
	{
		// если список пустой значение можно сбросить
		if(!list.size())
			_value = 0;

		for(const auto& index : list)
		{
			_value |= static_cast<type_t>(1) << static_cast<type_t>(index);
		}

		// удаляем  лишние биты
		_value = _value &  get_strip_mask();
		return *this;
	}

	/**
	 * @brief Оператор присваивания от целого
	 * @param value Значение типа, который хранится в наборе
	 * @tparam T Тип значения. Должен быть таким же, как и тип элемента набора
	 */
	BitMask& operator=(type_t value) noexcept
	{
		_value = value & get_strip_mask();
		return *this;
	}

	/**
	 * @brief Приводим BitMask к значению типа, который хранится в наборе
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

#include <iostream>
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline BitMask operator+(T index) const noexcept
	{
		BitMask newSet(*this);
		newSet.set(index);
		return newSet;
	}

	/**
	 * @brief Удаляем элемент из набора. Данный оператор создает копию объекта и возвращает её.
	 * @param index Бит, который нужно сбросить в наборе
	 * @tparam T Тип элемента. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline BitMask operator-(T index) const noexcept
	{
		BitMask newSet(*this);
		newSet.reset(index);
		return newSet;
	}

	/**
	 * @brief Добавляем элемент в набор
	 * @param index Бит, который нужно выставить в наборе
	 * @tparam T Тип элемента. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline BitMask& operator+=(T index) noexcept
	{
		set_impl(index);
		return *this;
	}

	/**
	 * @brief Удаляем элемент из набора
	 * @param index Бит, который нужно сбросить в наборе
	 * @tparam T Тип элемента. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline BitMask& operator-=(T index) noexcept
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
	template<typename T, typename... Args, typename = typename std::enable_if<(sizeof...(Args) > 0)>::type>
	inline bool has(T index, Args&&... args) const noexcept
	{
		static_assert(std::is_same<Enum, T>::value, "has parameters type must be same as Enum");
		static_assert(all_same<Enum, Args...>::value, "has parameters type must be same as Enum");

		if(!has_impl(index)) return false;

		for(const auto& arg : {args...})
		{
			if(!has_impl(arg)) return false;
		}
		return true;
	}

	/**
	 * @brief Проверяем, выставлен ли в наборе бит в true для переданного индекса
	 * @param index Индекс
	 * @return true, если по переданному индексу бит выставлен
	 */	
	template<typename T>
	inline bool has(T index) const noexcept
	{
		static_assert(std::is_same<Enum, T>::value, "has parameters type must be same as Enum");

		return has_impl(index);
	}

	/**
	 * @brief Проверяем, выставлены ли все биты из второго набора в первом
	 * @param other BitMask, который проверяем на включение в текущий набор
	 * @return true, если текущий набор включает все элементы другого набора, иначе false
	 */
	inline bool has(const BitMask& other) const noexcept
	{
		return (_value & other._value) == other._value;
	}

	/**
	 * @brief Проверяем, выставлен ли в наборе хотя бы один бит
	 * @return true, если в наборе выставлен хотя бы один бит, иначе false
	 */
	inline bool any() const noexcept
	{
		return _value;
	}

	/**
	 * @brief Проверяем, выставлены ли все биты в наборе. Корректно работает только если ваше перечисление подразумевает инкрементную последовательность
	 * @return true, если все биты в наборе выставлены, иначе false
	 */
	inline bool all() const noexcept
	{
		return _value == get_strip_mask();
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
	 * @brief Устанавливаем значение в маску
	 * @param value Значение типа, который хранится в наборе
	 * @tparam T Тип значения. Должен быть таким же, как и тип элемента набора
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline void set(T index) noexcept
	{
		set_impl(index);
	}

	/**
	 * @brief Сбрасываем бит в маске
	 * @param index Индекс бита, который мы хотим выставить в false в маске
	 * @tparam T тип параметра, должен совпадать с Enum
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline void reset(T index) noexcept
	{
		reset_impl(index);
	}

	/**
	 * @brief Сбрасываем биты в наборе
	 * @param args Индексы битов, которые мы хотим выставить в false в наборе
	 * @tparam Args Parameter pack значений, которые нужно сбросить в наборе, должен быть того же типа, что и Enum
	 */
	inline void reset() noexcept
	{
		_value = 0;
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
		static char str[sizeof(type_t) * allignment + 1];
		for(type_t i = 0; i < sizeof(type_t) * allignment; ++i)
		{
			str[i] = (_value & (static_cast<type_t>(1) << (sizeof(type_t) * 8 - i - 1))) ? '1' : '0';
		}
		str[sizeof(type_t) * allignment] = '\0';
		return str;
	}

private:

	static constexpr type_t allignment = 8;
	
	/**
	 * @brief Класс для работы с отдельным битом в наборе
	 */
	struct reference
    {
    public:
		/**
		 * @brief В конструкторе ссылки сохраняем ссылку на основной контейнер и индекс бита, с которым будем работать
		 */
        reference(type_t& value, Enum index) 
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
        Enum _index;
    };

	type_t _value{0};

	/**
	 * @brief Проверяем выставлен ли бит номер index в наборе
	 * @param index Номер бита, который нужно проверить
	 * @return true, если бит выставлен, иначе false
	 */
	inline bool has_impl(Enum index) const noexcept
	{
		return _value & (static_cast<type_t>(1) << static_cast<type_t>(index));
	}

	/**
	 * @brief Устанавливаем бит номер index в наборе
	 * @param index Номер бита, который нужно установить
	 */
	inline void set_impl(Enum index) noexcept
	{
		_value |= (static_cast<type_t>(1) << static_cast<type_t>(index));
	}

	/**
	 * @brief Сбрасываем бит номер index в наборе
	 * @param index Номер бита, который нужно сбросить
	 */
	inline void reset_impl(Enum index) noexcept
	{
		_value &= ~(static_cast<type_t>(1) << static_cast<type_t>(index));
	}

	/**
	 * @brief Получаем маску
	 * @return маска для обрезки бит
	 */
	static constexpr type_t get_strip_mask() noexcept
	{
		type_t mask = ~static_cast<type_t>(0);
		mask >>= sizeof(type_t) * allignment - N;
		return mask;
	}

public:

	// Оператор [] и метод at абсолютно одинаковы в данном случае, потому что безопасность по переполнению не нужна из за строгости типа Enum

	/**
	 * @brief Оператор доступа к биту по индексу 
	 * @param index Индекс бита, к которому нужно получить доступ
	 * @tparam T Тип индекса, должен быть таким же, как и Enum
	 * @return Ссылка на бит по указанному индексу
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline reference operator[](T index) noexcept
	{
		return reference(_value, index);
	}

	/**
	 * @brief Оператор доступа к биту по индексу 
	 * @param index Индекс бита, к которому нужно получить доступ
	 * @tparam T Тип индекса, должен быть таким же, как и Enum
	 * @return Ссылка на бит по указанному индексу
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
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
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline reference at(T index) noexcept
	{
		return reference(_value, index);
	}

	/**
	 * @brief Метод доступа к биту по индексу 
	 * @param index Индекс бита, к которому нужно получить доступ
	 * @tparam T Тип индекса, должен быть таким же, как и Enum
	 * @return Ссылка на бит по указанному индексу
	 */
	template<typename T, typename = typename std::enable_if<std::is_same<Enum, T>::value>::type>
	inline const reference at(T index) const noexcept
	{
		return reference(_value, index);
	}
};
