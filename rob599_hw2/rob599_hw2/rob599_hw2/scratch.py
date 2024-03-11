import numpy as np

def limit_value(value, constraint):
    """
    Limits the incoming value to the absolute of the input constraint.
    
    Parameters:
        value (float or numpy.ndarray): The incoming value or array of values.
        constraint (float): The constraint to limit the value against.
    
    Returns:
        float or numpy.ndarray: The limited value or array of limited values.
    """
    return np.clip(value, -np.abs(constraint), np.abs(constraint))

# Example usage:
incoming_value = np.array([-5, 3, 8, -10, 12])
constraint = 2
limited_value = limit_value(incoming_value, constraint)
print("Original value:", incoming_value)
print("Limited value:", limited_value)
