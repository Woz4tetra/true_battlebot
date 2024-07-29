std_msgs/Header header
int32 channel  # motor channel
float64 velocity  # commanded velocity in relative units
string filename  # sample filename containing the data
bool valid  # true if the sample should be used
float64 feedback  # sensed motor velocity in real units (Hz, RPM, etc)
