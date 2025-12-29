➜  ~ ros2 interface show rcl_interfaces/srv/SetParameters                           
# A list of parameters to set.
Parameter[] parameters
	string name
	ParameterValue value
		uint8 type
		bool bool_value
		int64 integer_value
		float64 double_value
		string string_value
		byte[] byte_array_value
		bool[] bool_array_value
		int64[] integer_array_value
		float64[] double_array_value
		string[] string_array_value

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
	bool successful
	string reason
