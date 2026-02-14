# Running ROS GUI tools with Xpra in containers

This tutorial will guide you through using Xpra to access GUI applications in Docker containers via a web browser. Xpra is useful when you don't have an X11 server available (e.g., on macOS, Windows, or Codespaces) or when you want a more portable solution than X11 forwarding.

## What is Xpra?

Xpra is a "screen for X11" that allows you to run GUI applications in a container and access them through a web browser. Unlike X11 forwarding, Xpra doesn't require an X11 server on your local machine and works seamlessly over HTTP, making it ideal for remote development environments and cloud-based workflows.

## Quick Start

### Using the devcontainer

1. **Open the devcontainer** in VS Code
2. **Start Xpra** in the integrated terminal:
   ```bash
   /start-xpra.sh
   ```
3. **Open your browser** and navigate to `http://localhost:14500`
4. **Launch a GUI application**:
   ```bash
   rviz2
   # or
   rqt
   # or
   gazebo
   ```

The application window will appear in your browser.

### Running from Docker directly

If you're not using the devcontainer, you can run Xpra directly from Docker:

```bash
docker run -it -p 14500:14500 \
  ghcr.io/dr-qp/jazzy-ros-desktop:edge \
  bash -c "/start-xpra.sh & sleep 2 && bash"
```

Then open `http://localhost:14500` in your browser.

## Configuration

The `/start-xpra.sh` script accepts the following options:

- `--port PORT`: Change the web server port (default: 14500)
- `--display DISPLAY`: Specify the X display number (default: 100)

Example:
```bash
/start-xpra.sh --port 8080 --display 101
```

## GPU Acceleration

For GPU-accelerated rendering (e.g., in Gazebo or RViz), Xpra uses VirtualGL to forward GPU commands.

### Automatic GPU detection

The container automatically detects GPU availability. If a GPU is present, it will be used automatically.

### Manual GPU acceleration

To explicitly use GPU acceleration with an application:

```bash
vglrun rviz2
vglrun gazebo
```

### Software rendering fallback

If no GPU is available, the container automatically falls back to software rendering using Mesa/llvmpipe. This is slower but works on any system.

## Comparison with X11 forwarding

| Feature | Xpra | X11 Forwarding |
|---------|------|---|
| Requires X11 server | No | Yes |
| Works in browser | Yes | No |
| Works on macOS/Windows | Yes | Requires XQuartz/Xming |
| Works in Codespaces | Yes | No |
| Latency | Higher | Lower (on LAN) |
| Setup complexity | Simple | Moderate |

For more details on X11 forwarding, see [Running ROS GUI tools remotely using X11 forwarding](remote-x11-tools.md).

## Troubleshooting

### Port already in use

If port 14500 is already in use, specify a different port:

```bash
/start-xpra.sh --port 14501
```

### Display conflict

If you get a display conflict error, try a different display number:

```bash
/start-xpra.sh --display 101
```

### Black screen or no window appears

1. Check that the application is running: `ps aux | grep rviz2`
2. Check Xpra logs: `tail -f ~/.xpra/logs/`
3. Try launching the app again in the browser's terminal

### Performance issues

- Use GPU acceleration with `vglrun` for 3D applications
- Reduce window size or resolution if experiencing lag
- Check system resources: `top` or `htop`

