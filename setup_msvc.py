from distutils.core import setup, Extension

module1 = Extension('mac_interface',
                    include_dirs = ['multi-agent_collaboration'],
                    extra_compile_args = ['/std:c++17', '/O2'],
                    sources = ['mac_interface/Mac_interface.cpp',
                               'multi-agent_collaboration/A_Star.cpp',
                               'multi-agent_collaboration/Bayesian_Recogniser.cpp',
                               'multi-agent_collaboration/BFS.cpp',
                               'multi-agent_collaboration/Core.cpp',
                               'multi-agent_collaboration/Environment.cpp',
                               'multi-agent_collaboration/Heuristic.cpp',
                               'multi-agent_collaboration/Planner_Mac.cpp',
                               'multi-agent_collaboration/Planner_Still.cpp',
                               'multi-agent_collaboration/Search_Trimmer.cpp',
                               'multi-agent_collaboration/Sliding_Recogniser.cpp',
                               'multi-agent_collaboration/State.cpp',
                               'multi-agent_collaboration/Utils.cpp'])


setup (name = 'mac_interface',
       version = '1.0',
       description = 'This is a demo package',
       ext_modules = [module1])
