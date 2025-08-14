import subprocess
import sys

def get_installed_packages():
    """Gets a set of all currently installed packages."""
    try:
        # Use pip to get a list of installed packages and their versions
        # and parse the output
        installed_output = subprocess.check_output(
            [sys.executable, "-m", "pip", "freeze"]
        ).decode("utf-8")
        return {
            line.split("==")[0]
            for line in installed_output.strip().split("\n")
            if line
        }
    except subprocess.CalledProcessError:
        return set()

def install_dependencies(requirements_file):
    """
    Installs packages from a requirements file,
    skipping those that are already installed.
    """
    try:
        with open(requirements_file, "r") as f:
            required_packages = {line.strip().split("==")[0] for line in f if line.strip()}
    except FileNotFoundError:
        print(f"Error: The file '{requirements_file}' was not found.")
        return

    installed_packages = get_installed_packages()
    
    # Find packages that are required but not installed
    to_install = required_packages - installed_packages
    
    if not to_install:
        print("All required packages are already installed. 🎉")
        return

    print("Installing new dependencies...")
    try:
        # Pass the list of packages to install to pip
        subprocess.check_call(
            [sys.executable, "-m", "pip", "install", *list(to_install)]
        )
        print("Dependencies installed successfully. ✅")
    except subprocess.CalledProcessError as e:
        print(f"Error installing dependencies: {e}")

if __name__ == "__main__":
    install_dependencies("requirements.txt")