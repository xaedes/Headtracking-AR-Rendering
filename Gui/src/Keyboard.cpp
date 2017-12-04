#include "Keyboard.h"
#include <QKeyEvent>

namespace ar {
namespace Gui {

	Keyboard::Keyboard()
	{}

	Keyboard::~Keyboard()
	{}

	bool Keyboard::isKeyDown(int key)
	{
		if (m_isKeyDown.count(key) > 0)
		{
			return m_isKeyDown[key];
		}
		else
		{
			return false;
		}
	}

	bool Keyboard::isKeyUp(int key)
	{
		return !isKeyDown(key);
	}


	void Keyboard::keyPressEvent(QKeyEvent *event)
	{
		m_isKeyDown[event->key()] = true;
	}
	void Keyboard::keyReleaseEvent(QKeyEvent *event)
	{
		m_isKeyDown[event->key()] = false;
	}


} // namespace Gui
} // namespace ar
