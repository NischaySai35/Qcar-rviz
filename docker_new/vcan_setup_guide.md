# Podman Host-Container vcan0 Setup Guide

This guide provides the complete, step-by-step procedure to set up a virtual CAN (`vcan0`) interface on an **Ubuntu 24.04** host machine, build and run a **Podman**, **Docker** container with host network access, and verify the bidirectional data flow using `can-utils`.

---

## Step 1: Configure the Host Machine

Run these commands in a terminal on your **host machine** to initialize the virtual CAN interface in the Linux kernel.

```bash
# 1. Load the virtual CAN kernel module
sudo modprobe vcan

# 2. Create the virtual network interface named vcan0
sudo ip link add dev vcan0 type vcan

# 3. Bring the interface UP
sudo ip link set up vcan0

# 4. Verify the interface is active (should show <UP,LOWER_UP>)
ip link show vcan0
```

---

## Step 2: Build and Run the Container

Build the container image and execute it with permissions to access the host's networking namespace.

### Build the Image

```bash
podman build -t esync-sdk .
```

### Run the Container

Include `--net=host` to share network interfaces and `--cap-add=NET_ADMIN` to manage them.

```bash
podman run -d --name esync --net=host --cap-add=NET_ADMIN -v ~/Downloads/esync-campaigns:/downloads esync-sdk qcar123
```

---

## Step 3: Verify Traffic Flow Inside the Container

### 1. Monitor the Bus from Inside the Container

Open a terminal window on your host and attach a live `candump` monitor directly into the container namespace:

```bash
podman exec -it esync candump vcan0
```

_(The terminal window will sit idle, waiting for packets.)_

### 2. Inject Dummy Traffic from the Host Machine

Open a **second, separate terminal window** on your host machine and push a packet onto the bus:

```bash
# Send a single frame with ID 123 and hex payload AABBCCDD
cansend vcan0 123#AABBCCDD

# Alternative: Generate a continuous stream of random frames for stress testing
cangen vcan0 -v
```

### 3. Check the Results

Go back to your first terminal window (running the `podman exec` command). You should immediately see the data appearing live inside the container space:

```text
  vcan0  123   [4]  AA BB CC DD
```
