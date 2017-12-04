#pragma once

namespace ar {
namespace Gui {

		// forward declaration
	class App;

	class KeyboardControl
	{
	public:
		KeyboardControl(App& app);
		~KeyboardControl();

		void update(float dt);
	protected:

		App& m_app;
	};

} // namespace Gui
} // namespace ar
