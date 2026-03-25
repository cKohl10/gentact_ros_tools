#!/usr/bin/env python3

import sys
import os
import subprocess

def create_script(venv_path, script_name):

    current_direc = os.getcwd
    

    with open ('run.sh', 'w') as rsh:
        rsh.write(f'''\
        #! /bin/bash
        # Get the directory where this script is located
        SCRIPT_DIR="$(cd "$(dirname "${{{current_direc}}}")" && pwd)"
                  
        # Path to the virtual environment
        VENV_PATH={venv_path}
        
        # Activate the virtual environment
        source "$VENV_PATH/bin/activate"
                  
        # Run the Python script with the virtual environment's Python
        exec "$VENV_PATH/bin/python3" "$SCRIPT_DIR/{script_name}.py" "$@" 
        ''')


def main():
    # Get the path to the virtual environment
    venv_path = "/home/caleb/ros2_ws/.venv"
    venv_python = os.path.join(venv_path, "bin", "python3")
    
    # Get the path to the actual franky_relay script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    franky_relay_script = os.path.join(script_dir, "franky_relay.py")
    

    # Set up environment variables
    env = os.environ.copy()
    env['VIRTUAL_ENV'] = venv_path
    env['PATH'] = os.path.join(venv_path, 'bin') + ':' + env.get('PATH', '')
    
    # Add virtual environment's site-packages to PYTHONPATH
    site_packages = os.path.join(venv_path, "lib", "python3.8", "site-packages")
    pythonpath = env.get('PYTHONPATH', '')
    if pythonpath:
        env['PYTHONPATH'] = site_packages + ':' + pythonpath
    else:
        env['PYTHONPATH'] = site_packages
    
    # Run the franky_relay script with the virtual environment's Python
    try:
        venv_path = "/home/caleb/ros2_ws/.venv"
        script_name = "franky_relay.py"
        franky_relay_script = create_script(venv_path, script_name)
        result = subprocess.run([venv_python, franky_relay_script] + sys.argv[1:], 
                              env=env, check=True)
        return result.returncode
    except subprocess.CalledProcessError as e:
        print(f"Error running franky_relay: {e}")
        return e.returncode
    except FileNotFoundError:
        print(f"Virtual environment Python not found at: {venv_python}")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 