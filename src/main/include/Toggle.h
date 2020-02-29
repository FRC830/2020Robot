class Toggle {

private:

	bool prev_state;

	bool toggle_state;

public:

	explicit Toggle(bool toggle_state = false):prev_state(false), toggle_state(toggle_state){}

	bool toggle (bool button_state) {

		if (button_state && prev_state != button_state) {

			toggle_state = !toggle_state;

		}

		prev_state = button_state;

		return toggle_state;

	}

	bool rising_edge(bool button_state) {
		// if true and previously false
		bool is_rising_edge = (button_state && !prev_state);
		prev_state = button_state;
		return is_rising_edge;	
	}

	bool operator= (const bool state) {

		toggle_state = state;

		return toggle_state;

	}



	operator bool () {

		return toggle_state;

	}



};

