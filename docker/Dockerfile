FROM nvcr.io/nvidia/isaac-sim:2022.2.1

WORKDIR /workspace

# Install C++ build tools
USER root
RUN apt-get update && apt-get install -y build-essential cmake

# Copy project files
COPY . /workspace

# Build the C++ project
RUN mkdir build && cd build && \
    cmake .. && \
    make

# Set default command
ENTRYPOINT ["./build/isaac_synth"]
