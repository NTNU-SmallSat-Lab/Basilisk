from Basilisk.simulation import vizInterface

def create_rotation_message(slider_rotations):
    """
    Creates a vizMessage to transmit the rotation of sliders inducing satellite rotation.

    Args:
        slider_rotations (list): A list of rotation values for the sliders.

    Returns:
        vizInterface.VizMessage: The constructed vizMessage.
    """
    message = vizInterface.VizMessage()
    for i, rotation in enumerate(slider_rotations):
        slider = message.sliders.add()
        slider.id = i
        slider.rotation = rotation
    return message

# Example usage
slider_rotations = [3, 3, 3]  # Example slider rotations
rotation_message = create_rotation_message(slider_rotations)