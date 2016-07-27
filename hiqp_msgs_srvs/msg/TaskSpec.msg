# The HiQP Control Framework, an optimal control framework targeted at robotics
# Copyright (C) 2016 Marcus A Johansson
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.





string    task                  # the name of the task class (e.g. TaskPoP)
string    behaviour             # the name of the taks behaviour class (e.g. TaskBehaviourFirstOrder)
string[]  behaviour_parameters  # the parameters sent to init() of the behaviour class
string    name                  # the name to be associated with this task instantiation
uint16    priority              # the task priority (1 is highest)
bool      visibility            # true if visualization fo this task should be turned on
string[]  parameters            # a list of strings that is passed to the init function of the task class
