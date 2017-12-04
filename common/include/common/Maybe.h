#pragma once

#include <ostream>

namespace ar {
namespace common {

	template<typename T>
	class Maybe
	{
	public:
		static Maybe Just(const T& just)
		{
			return Maybe(just);
		}

		static Maybe Just()
		{
			T empty;
			return Maybe(empty);
		}

		static Maybe Nothing()
		{
			return Maybe();
		}

		Maybe(const Maybe<T>& copyFrom)
			: m_valid(false)
		{
			if(copyFrom)
			{
				just(copyFrom.value());
			}
		}
		
		Maybe(const T& just)
			: m_value(just)
			, m_valid(true)
		{}

		template <typename C>
		Maybe(Maybe<C>& castFrom) //Converting constructor 
		: m_valid(false)
		{
			C* source = castFrom.ptr();
			T* target = dynamic_cast<T*>(source);
			if(target)
			{
				just(*target);
			}
		}

		Maybe()
			: m_valid(false)
		{}

		~Maybe()
		{}

		const T& value() const
		{
			return m_value;
		}

		T* ptr()
		{
			if (m_valid)
			{
				return &m_value;
			}
			else
			{
				return nullptr;
			}
		}

		const T* operator->() const
		{
			if (m_valid)
			{
				return &m_value;
			}
			else
			{
				return nullptr;
			}
		}

		T* operator->()
		{
			if (m_valid)
			{
				return &m_value;
			}
			else
			{
				return nullptr;
			}
		}

		explicit operator bool() const 
		{
			return containsValue();
		}

		bool containsNothing() const
		{
			return !m_valid;
		}

		bool containsValue() const
		{
			return m_valid;
		}

		bool isValid() const
		{
			return m_valid;
		}

		bool isEmpty() const
		{
			return !m_valid;
		}

		void clear()
		{
			T empty;
			m_value = empty;
			m_valid = false;
		}

		T& just(const T& just)
		{
			m_valid = true;
			m_value = just;
			return m_value;
		}

		T& just()
		{
			m_valid = true;
			return m_value;
		}

	protected:
		bool m_valid;
		T m_value;
	};
	
	template<typename T>
	std::ostream& operator<<(std::ostream& os, const Maybe<T>& maybe)
	{
		os 
			<< "m_valid" << std::endl << maybe.isValid() << std::endl << std::endl
			<< "m_value" << std::endl << maybe.value() << std::endl << std::endl
			;
		return os;
	}

} // namespace common
} // namespace ar