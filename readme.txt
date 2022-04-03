install python 3.7.9 (32 bit) at C:/Python/, including debug binaries (https://www.python.org/downloads/release/python-379/)
Add C:\Python to Path system variable
at multi-agent_collaboration\gym-cooking-fork\gym_cooking run "python setup.py install"

If python is installed in another location, adjust in VS mac_interface->properties->C/C++/Additional Include Directories/ "C\Python\include"

The gym_cooking and mac_interface projects must be run, respectively compiled, in x32

Note that Planner_Mac_One is litterally a copy-pasta of Planner_Mac (with a single line changed in calculate_infos), and any changes to one agent will therefore not change the other.

Note that the gym-cooking-fork submodule links to an old commit, so after pulling, one must maunally check out the master branch. 
