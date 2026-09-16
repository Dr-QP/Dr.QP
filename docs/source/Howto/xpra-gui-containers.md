# Running ROS GUI tools with Xpra in containers

This tutorial will guide you through using Xpra to access GUI applications in Docker containers via a web browser. Xpra is useful when you don't have an X11 server available (e.g., on macOS, Windows, or Codespaces) or when you want a more portable solution than X11 forwarding.

## What is Xpra?

Xpra is a "screen for X11" that allows you to run GUI applications in a container and access them through a web browser. Unlike X11 forwarding, Xpra doesn't require an X11 server on your local machine and works seamlessly over HTTP, making it ideal for remote development environments and cloud-based workflows.

## Quick Start

### Using the devcontainer

1. **Open the devcontainer** in VS Code. Xpra will be started automatically in the background by the devcontainer configuration.
2. **(Optional) Manually start or restart Xpra** in the integrated terminal if needed (for example, to change the port or display, or for troubleshooting):
   ```bash
   /start-xpra.sh
   ```
3. **Open your browser** and navigate to the forwarded address. Xpra always listens on port **14500** inside the container; VS Code forwards it and picks a free port on your machine, which it shows in the **Ports** panel. See **Multiple devcontainer instances** below.
4. **Launch a GUI application**:
   ```bash
   rviz2
   # or
   rqt
   # or
   gazebo
   ```

The application window will appear in your browser.

#### Multiple devcontainer instances

When multiple devcontainer instances run (e.g., different worktrees or workspaces), every instance listens on **14500** inside its own container — container ports are per-container, so they cannot collide. Conflicts can only happen on the host side, and VS Code resolves them by forwarding each container's 14500 to a different free host port.

- Find the host address in VS Code's **Ports** panel; it is forwarded automatically by the devcontainer configuration.
- The first instance normally lands on `http://localhost:14500`, later ones on whatever VS Code picks.

> Earlier versions derived a unique container port from the devcontainer ID and forwarded the range 14500-14599. Host-side forwarding supersedes that, so the container port is now fixed.

### Running from Docker directly

If you're not using the devcontainer, you can run Xpra directly from Docker:

```bash
docker run -it -p 14500:14500 \
  ghcr.io/dr-qp/jazzy-ros-desktop:edge \
   bash -c "/start-xpra.sh --host 0.0.0.0 & sleep 2 && bash"
```

Then open `http://localhost:14500` in your browser.

When using Docker port publishing (`-p`), pass `--host 0.0.0.0` so Xpra listens on the container interface. For devcontainer or host-network workflows, `127.0.0.1` remains the safer default.

## Configuration

The `/start-xpra.sh` script accepts the following options:

- `--host HOST`: Host/IP for Xpra TCP bind (default: `127.0.0.1`; use `0.0.0.0` with Docker `-p` port publishing)
- `--port PORT`: HTTP/TCP port for the HTML5 client (default: `14500`)
- `--display DISPLAY`: X display number (accepts `:100` or `100`; default: `:100`)
- `--background`: Start Xpra in the background and exit immediately (used by devcontainer auto-start)
- `--stop`: Stop the Xpra session for the selected display and clean up

Example:

```bash
/start-xpra.sh --port 8080 --display :101
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

| Feature                | Xpra   | X11 Forwarding         |
| ---------------------- | ------ | ---------------------- |
| Requires X11 server    | No     | Yes                    |
| Works in browser       | Yes    | No                     |
| Works on macOS/Windows | Yes    | Requires XQuartz/Xming |
| Works in Codespaces    | Yes    | No                     |
| Latency                | Higher | Lower (on LAN)         |
| Setup complexity       | Simple | Moderate               |

For more details on X11 forwarding, see [Running ROS GUI tools remotely using X11 forwarding](remote-x11-tools.md).

## Troubleshooting

### Which address is my instance using?

Inside the container the port is always 14500. What varies is the host port VS Code forwards it to — read that from the **Ports** panel, where the entry is labelled "Xpra HTML5".

### Port already in use

Host-side collisions are handled by VS Code's forwarding. If the container port itself is occupied — usually a previous Xpra session that did not shut down — stop it and start again:

```bash
/start-xpra.sh --stop
/start-xpra.sh --background
```

To run a second server alongside the first, give it both its own port and its own display:

```bash
/start-xpra.sh --port 14501 --display :101
```

A port other than 14500 is not forwarded by the devcontainer configuration, so add it in the **Ports** panel yourself.

### Display conflict

If you get a display conflict error, try a different display number:

```bash
/start-xpra.sh --display :101
```

### Black screen or no window appears

1. Check that the application is running: `ps aux | grep rviz2`
2. Check Xpra logs: `tail -f ~/.xpra/logs/`
3. Try launching the app again in the browser's terminal

### Performance issues

- Use GPU acceleration with `vglrun` for 3D applications
- Reduce window size or resolution if experiencing lag
- Check system resources: `btop`
