import sys

def select_debug_config():
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg == "test-sensor-model" or  arg == "test-sensor-sampling":
            return "Python: Test Sensor Model"
    
    # Default debug configuration if no valid argument is provided
    return "Python: Current File"

if __name__ == "__main__":
    debug_config = select_debug_config()
    print(f"Selected Debug Configuration: {debug_config}")
