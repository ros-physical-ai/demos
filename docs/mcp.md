# MCP Interface

Connect the SO-ARM101 to an AI agent running the [ROS-MCP server](https://github.com/robotmcp/ros-mcp-server),
so the agent can control, introspect, or debug the robot. This works across all three bringups
(Gazebo, MuJoCo, and real hardware).

## Enabling the interface

The MCP interface is off by default. Enable it on any bringup by passing `mcp:=true`:

```bash
pixi run so-arm-gz mcp:=true
# or: so-arm-mujoco / so-arm-real
```

This opens the connection the ROS-MCP server uses to reach the robot on port `9090` (override with
`mcp_port:=<port>`). It is bound on all network interfaces, so the agent can run on another machine
on your network.

## Connecting an agent

To connect [Claude Code](https://docs.claude.com/en/docs/claude-code) as the agent, install the
ROS-MCP server (requires [`uv`](https://docs.astral.sh/uv/)) with a single command:

```bash
claude mcp add ros-mcp -- uvx ros-mcp --transport=stdio
```

For instructions on all other AI models, see the [ROS-MCP installation guide](https://github.com/robotmcp/ros-mcp-server/blob/main/docs/install/installation.md).
