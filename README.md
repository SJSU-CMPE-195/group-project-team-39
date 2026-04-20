# RoboMallet

Source code for the ROBOTMALLET robotic system's control logic.

## Team

| Name            | GitHub                                     | Email                    |
| --------------- | ------------------------------------------ | ------------------------ |
| Kenny Tran      | [@ktran1121](https://github.com/ktran1121) | kenny.k.tran@sjsu.edu    |
| Kshitij Shirule | [@K-Shirule](https://github.com/K-Shirule) | kshitij.shirule@sjsu.edu |
| Nam Nguyen      | [@namng2](https://github.com/namng2)       | nam.h.nguyen04@sjsu.edu  |
| Yathien Thai    | [@yyaatt0](https://github.com/yyaatt0)     | yathien.thai@sjsu.edu    |

**Advisor:** [Dr. Wencen Wu]

## Prerequisites

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) installed and running
- A Docker Hub account (required to build and push container images)
- Docker Buildx enabled (included by default in Docker Desktop)
- SSH or terminal access to the target MCU/board ([PuTTY](https://www.putty.org/) recommended for Windows users)

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/<your-org>/RoboMallet.git
   cd RoboMallet
   ```

2. Log in to your Docker Hub account:

   ```bash
   docker login -u yathien
   ```

3. Build and push the C++ service container:

   ```bash
   docker buildx build --platform linux/arm64 -t yathien/cppsvc:dev --push ./cppsvc
   ```

4. Build and push the Python service container:

   ```bash
   docker buildx build --platform linux/arm64 -t yathien/pysvc:dev --push ./pysvc
   ```

## Configuration

Before running the application, ensure the following hardware is connected to the MCU:

- Motor drivers (iSV57T) wired to the appropriate GPIO pins for DIR and PUL signals
- Limit switches wired to the designated GPIO input pins
- All power supplies are connected and within operating voltage range

Refer to the Ixora carrier board datasheet and the pin mapping in the project documentation for exact GPIO assignments.

## Running the Application

SSH into the target board and run:

```bash
cd ~/RoboMallet
docker compose pull
docker compose up -d
```

To view logs:

```bash
docker compose logs -f
```

To stop the application:

```bash
docker compose down
```

## Usage

Once the containers are running, the robotic system will initialize the motor drivers and limit switches on startup. The system listens for commands to control mallet rotation and positioning.

## Project Structure

```
RoboMallet/
├── cppsvc/                 # C++ service (motor control, GPIO interface)
│   ├── include/            # Header files (.hpp)
│   ├── src/                # Source files (.cpp)
│   ├── main.cpp            # Entry point
│   ├── Dockerfile          # ARM64 container build
│   └── CMakeLists.txt      # Build configuration
├── pysvc/                  # Python service
│   ├── Dockerfile          # ARM64 container build
│   └── main.py             # Entry point
├── docker-compose.yml      # Multi-container orchestration
└── README.md
```

## Demo

🎥 **Project Demo Video**  
[CMPE195B RoboMallet Group39 Demo 1](https://youtu.be/watch?v=AesrC-o85-U)
[CMPE195B RoboMallet Group39 Demo 2](https://youtu.be/WAsmsQb9nug)
