
# ROS2 Ground Control Station (GCS) Setup and Run Guide

This guide will walk you through the steps required to set up and run the GCS web application, which communicates with ROS using `rosbridge_suite` and `roslib`. 

### Prerequisites
1. **Ubuntu 22 with ROS Humble** installed.
2. **Node.js** (version 16 or higher).
3. **NPM** (comes with Node.js) and **nvm** (Node Version Manager), in case you need to manage Node.js versions.

### Steps

#### 1. Install `rosbridge_suite` for ROS
The `rosbridge_suite` package enables ROS to communicate with non-ROS systems, like the GCS web application.

```bash
sudo apt install ros-humble-rosbridge-suite
```

#### 2. Install `roslib` in the GCS Application
This library will allow the GCS to interact with ROS topics, services, and parameters.

- If Node.js version is below 16, upgrade to version 16 using `nvm`:

```
wget -qO- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
```

  ```bash
  nvm install 16
  nvm use 16
  nvm alias default 16
  ```

- With Node.js 16 active, install `roslib` and `react` and `react-dom` in the project folder:
  ```bash
  npm install roslib
  npm install react react-dom

  ```

#### 3. Install Project Dependencies
Navigate to the `src` directory in your project and install all dependencies listed in `package.json`:

```bash
cd /path/to/your/project/src
npm install
```

#### 4. Launch `rosbridge_websocket`
Open another terminal to launch the `rosbridge_websocket` server, which provides a WebSocket connection for the GCS to communicate with ROS:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```


#### 5. Start the GCS Application
In the `src` directory, start the GCS application:

```bash
npm start
```

This should start the development server and open the GCS application in your browser. You should now be able to interact with ROS topics and services through the GCS web interface.

---

### Troubleshooting

- Takes about 5 min for the first time start up
- **Missing `react-scripts`**: Run `npm install` in the project’s root directory to install any missing dependencies.
- **Incorrect Node.js Version**: Use `nvm use 16` to switch to Node.js 16 if another version is active.
https://stackoverflow.com/questions/58743333/react-app-stuck-on-starting-the-development-server
https://stackoverflow.com/questions/71511921/failed-to-load-plugin-react-declared-in-package-json-eslint-config-react-ap
This completes the setup and run procedure for the GCS application.