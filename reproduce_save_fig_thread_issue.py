import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
import os

# Function to create and save a random image
def create_and_save_image(filename):
    # Create a random image
    data = np.random.rand(10, 10)
    # plt.imshow(data, cmap='jet')
    # plt.imshow(data)

    plt.plot(data)

    # plt.colorbar()

    # time.sleep(np.random.uniform(1.0, 6.0))
    # Save the image
    plt.savefig(filename)
    plt.close()

# Function to be run by each thread
def thread_function(thread_id):
    filename = f'random_image_{thread_id}.png'
    create_and_save_image(filename)
    print(f'Thread {thread_id} saved {filename}')

# Number of threads
num_threads = 5

# Create and start threads
threads = []
for i in range(num_threads):
    t = threading.Thread(target=thread_function, args=(i,))
    threads.append(t)
    t.start()

# Wait for all threads to complete
for t in threads:
    t.join()

print("All threads have finished execution.")