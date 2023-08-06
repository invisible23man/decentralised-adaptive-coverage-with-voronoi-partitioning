import json
import numpy as np

def encode_data(data):
    """
    Encodes data into a JSON string. Supports numpy ndarrays, lists, and strings.

    Parameters:
    data (numpy.ndarray, list, str): The data to encode.

    Returns:
    str: A JSON string representation of the data.
    """
    if isinstance(data, np.ndarray):
        return json.dumps({"data": data.tolist(), "type": "ndarray"})
    elif isinstance(data, list):
        return json.dumps({"data": data, "type": "list"})
    elif isinstance(data, str):
        return json.dumps({"data": data, "type": "str"})
    else:
        raise ValueError("Unsupported data type")

def decode_data(json_str):
    """
    Decodes data from a JSON string. Supports numpy ndarrays, lists, and strings.

    Parameters:
    json_str (str): The JSON string to decode.

    Returns:
    numpy.ndarray, list, str: The decoded data.
    """
    python_dict = json.loads(json_str.data)
    data_type = python_dict["type"]
    if data_type == "ndarray":
        return np.array(python_dict["data"])
    elif data_type == "list":
        return python_dict["data"]
    elif data_type == "str":
        return python_dict["data"]
    else:
        raise ValueError("Unsupported data type")
