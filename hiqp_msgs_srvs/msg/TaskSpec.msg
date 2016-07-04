string    task                  # the name of the task class (e.g. TaskPoP)
string    behaviour             # the name of the taks behaviour class (e.g. TaskBehaviourFirstOrder)
string[]  behaviour_parameters  # the parameters sent to init() of the behaviour class
uint16    priority              # the task priority (1 is highest)
bool      visibility            # true if visualization fo this task should be turned on
string[]  parameters            # a list of strings that is passed to the init function of the task class
