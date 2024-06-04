# Use ROS Noetic as a parent image
FROM ros:noetic

# Install any needed packages specified in requirements.txt
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libgtest-dev \
    libvtk7-dev \
    libompl-dev \
    libyaml-cpp-dev \
    x11-apps

# Copy the current directory contents into the container at /app
COPY . /app

# Define environment variable
ENV NAME World
ENV DISPLAY :0

# Remove build directory if it exists and then create a new one
RUN rm -rf build && mkdir -p build
WORKDIR /app/build

# Run cmake when the container launches
RUN cmake ..

# Run make when the container launches
RUN make -j8

# Run the output program from the previous step
CMD ["./run_planner", "../test/environment.yaml", "../test/test_cases.yaml"]