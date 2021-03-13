#pragma once


template<typename T>
std::vector<Joint_Action> Search_Method::extract_actions(size_t goal_id, const std::vector<T>& states) const {
	size_t current_id = goal_id;
	std::vector<Joint_Action> result_actions;
	while (current_id != 0) {
		auto temp_state = states.at(current_id);
		result_actions.push_back(temp_state.action);
		current_id = temp_state.parent_id;
	}

	std::vector<Joint_Action> reverse_actions;
	for (auto it = result_actions.rbegin(); it < result_actions.rend(); ++it) {
		reverse_actions.push_back(*it);
	}
	return reverse_actions;
}