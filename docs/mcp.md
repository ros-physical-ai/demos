# MCP Interface

Connect the SO-ARM101 to an AI agent running the [ROS-MCP server](https://github.com/robotmcp/ros-mcp-server),
so the agent can control, introspect, or debug the robot. This works across all three bringups
(Gazebo, MuJoCo, and real hardware).

## Step 1: Set up the AI Client

Follow `Step 1` of the [ROS-MCP installation guide](https://github.com/robotmcp/ros-mcp-server/blob/main/docs/install/installation.md), which has instructions for adding the MCP server to your client. (Claude Code, Codex, Gemini, local agent harnesses, etc.)

> Only the `Step 1: Set up the AI Client` section on that page is needed. The remaining steps are replaced by the corresponding steps in this guide.

<details>
<summary>For Claude Code, `Step 1` is a single line command (expand for details)</summary>

Having [`uv`](https://docs.astral.sh/uv/) installed is a prerequisite for the MCP server:

```bash
claude mcp add ros-mcp -- uvx ros-mcp --transport=stdio
```
</details>

## Step 2: Enable the MCP interface

The MCP interface is off by default. Enable it on any bringup by passing `mcp:=true`:

```bash
pixi run so-arm-gz mcp:=true
# or: so-arm-mujoco / so-arm-real
```

This opens the connection the ROS-MCP server uses to reach the robot on port `9090` (override with
`mcp_port:=<port>`). It is bound on all network interfaces, so the agent can run on another machine
on your network.

## Step 3: Interact with the robot
Now that your AI client has the MCP server configured and the demo is running with the `mcp:=true` flag, you're ready to connect.

### 3.1. Connect

Open your AI client and tell it to connect to the robot:

```
Connect to the robot on localhost
```
The MCP server will report that the IP is reachable and the rosbridge port is open — this means you're connected.


> Replace `localhost` with your robot's IP address on the local network (e.g., `192.168.1.42`) if you are connecting from a different machine on the local network. Make sure the rosbridge port (default 9090) is not blocked by a firewall on the robot's machine.

### 3.2. Explore

Once connected, try asking your AI client to explore the ROS system:

```
What topics and services are available on the robot?
```

Or give it a command:

```
Make the robot do a dance
```

The MCP server will query and return the results from the robot's ROS environment, and command it as well.
