# Defines a Task Message. 

string       name          # the name to be associated with this task instantiation
uint16       priority      # the task priority (1 is highest)
bool         visible       # whether or not the task should be visible initially
bool         active        # whether or not the task should be active initially
bool         monitored     # whether or not the task should be monitored initially
string[]     def_params    # the task definition name and a list of strings that is passed to the init function of the task class
string[]     dyn_params    # the name of the task dynamics along with its parameters