# Jetson Nano Setup

Steps to run this project on an NVIDIA Jetson Nano (JetPack 4.x, Ubuntu 18.04, aarch64).
The Nano's default Python is 3.6. This project is developed on Python 3.8, so these steps install 3.8 and use a virtual environment.

## 1. Update the system

```bash
sudo apt update && sudo apt upgrade -y
```

## 2. Install system packages

```bash
sudo apt install -y git build-essential gfortran \
    libopenblas-dev liblapack-dev libatlas-base-dev \
    libgeos-dev libffi-dev libssl-dev libfreetype6-dev libpng-dev \
    python3-pip
```

## 3. Install Python 3.8

```bash
sudo apt install -y python3.8 python3.8-venv python3.8-dev
python3.8 --version
```

## 4. Get the code

```bash
git clone <your-repo-url> swarm_tasks_mutli_platform
cd swarm_tasks_mutli_platform
git checkout hybrid
```

## 5. Create a virtual environment and install dependencies

```bash
python3.8 -m venv venv
source venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt
```

numpy, scipy, pandas and matplotlib may build from source on aarch64. This can take 30+ minutes, and it helps to add swap first (see Troubleshooting).

If you'd rather stay on the Nano's default Python 3.6, use `python3 -m venv venv` in place of `python3.8`. `requirements.txt` pins 3.6-compatible versions automatically.

## 6. Verify the install

```bash
python -c "import numpy, scipy, pandas, matplotlib, shapely, yaml, geopy, simplekml, netifaces, dronekit, pymavlink; print('OK')"
```

## 7. Run

```bash
source venv/bin/activate
python main.py
```

On a headless Jetson (no display), use a non-GUI matplotlib backend:

```bash
export MPLBACKEND=Agg
```

## Connecting to the flight controller

- Serial (Pixhawk/ArduPilot on USB or UART): add your user to the serial group, then log out and back in.
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- Typical device paths are `/dev/ttyACM0` (USB) and `/dev/ttyTHS1` (Nano UART).
- Check the device exists with `ls /dev/ttyACM* /dev/ttyTHS*`.
- Check the Jetson's network interface and IP with `ip addr`. The project's UDP and config server use it for swarm communication.

## Troubleshooting

**Out of memory during pip install.** Add swap:
```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
```

**netifaces fails to build.** Install `python3.8-dev` and `build-essential` (steps 2 and 3).

**shapely cannot find GEOS.** Install `libgeos-dev`, then reinstall: `pip install --no-cache-dir shapely`.

**`ImportError: libopenblas`.** Install `libopenblas-dev`.

**dronekit error `collections.MutableMapping`.** You are on Python 3.10 or newer. Use Python 3.8 or earlier.

**Run on boot (optional).** Create a systemd service that runs `venv/bin/python main.py` from the project directory.

## Notes

- `gui/` has its own dependencies and is not needed on the Jetson.
- Performance: `sudo nvpmodel -m 0 && sudo jetson_clocks` sets maximum power mode.
