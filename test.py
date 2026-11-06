def generate_heights(start_height, num_drones, difference):
    return [start_height + i * difference for i in range(num_drones)]

print(generate_heights(300,3,10))