import sys
import os

# Add the parent directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Now import the modules
import activations
import main
from Tests import test_runner

print("Initialized python modules")
