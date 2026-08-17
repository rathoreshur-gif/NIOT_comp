"""Pool water disturbance, driven by zero-mean Gauss-Markov processes.

Publishes a water velocity that the Gazebo Hydrodynamics plugin consumes on its
/ocean_current input. Because the plugin computes drag from the vehicle's
velocity RELATIVE to the water, this is a physically real disturbance: an AUV
drifting with the water feels nothing, one holding station feels the full push.
That is the reason this publishes a velocity rather than a wrench - a wrench
would keep shoving the vehicle even once it was moving with the flow.

The model is a pool, not a river. Each axis carries two independent zero-mean
Ornstein-Uhlenbeck (first-order Gauss-Markov) processes summed together:

    drift - large stddev, long time constant: the pool's lazy circulation
    gust  - smaller stddev, short time constant: eddies, wake, pump wash

Zero-mean is the whole point. The water wanders and can sit off-centre for a
minute at a time, which reads as a slow drift, but averaged over a run there is
no preferred heading. A steady directional flow is available via `bias`, which
defaults to zero.

Each process is advanced with the exact discretisation of the OU process,

    x <- mean + (x - mean) * exp(-dt/tau) + stddev * sqrt(1 - exp(-2*dt/tau)) * N(0,1)

rather than an Euler step. It is unconditionally stable for any dt and its
stationary distribution is exactly N(mean, stddev^2), so the stddev you write in
the config is the stddev you actually get - no dependence on the update rate.

Time is taken from the node clock, so with use_sim_time the disturbance follows
simulation time: pausing Gazebo pauses the water, and a run replayed from the
same seed sees the same water.
"""

import math

from geometry_msgs.msg import Vector3
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Empty
import yaml

CURRENT_TOPIC = '/model/auv/ocean_current'

DEFAULTS = {
    'enabled': True,
    'update_rate': 20.0,
    'intensity_scale': 1.0,
    'bias': [0.0, 0.0, 0.0],
    'drift': {'stddev': [0.030, 0.030, 0.005], 'time_constant': 45.0},
    'gust': {'stddev': [0.020, 0.020, 0.008], 'time_constant': 3.0},
}


class OrnsteinUhlenbeck:
    """Zero-mean OU process on three independent axes."""

    def __init__(self, stddev, time_constant, rng):
        self.stddev = np.asarray(stddev, dtype=float)
        self.tau = max(float(time_constant), 1e-3)
        self.rng = rng
        # Start from the stationary distribution rather than zero, so the water
        # is already moving at t=0 instead of ramping up over the first minute.
        self.state = self.rng.normal(0.0, 1.0, 3) * self.stddev

    def step(self, dt):
        decay = math.exp(-dt / self.tau)
        # Exact stationary-variance diffusion for this step length.
        spread = self.stddev * math.sqrt(max(0.0, 1.0 - decay * decay))
        self.state = self.state * decay + spread * self.rng.normal(0.0, 1.0, 3)
        return self.state

    def reset(self):
        self.state = np.zeros(3)


class OceanCurrent(Node):

    def __init__(self):
        super().__init__('ocean_current')

        self.declare_parameter('config_file', '')
        self.declare_parameter('seed', -1)
        config = self._load_config(
            self.get_parameter('config_file').get_parameter_value().string_value)

        drift = {**DEFAULTS['drift'], **(config.get('drift') or {})}
        gust = {**DEFAULTS['gust'], **(config.get('gust') or {})}

        # Everything below is a live ROS parameter, so the water can be retuned
        # mid-run without restarting the simulator.
        self.declare_parameter('enabled', bool(config.get('enabled', DEFAULTS['enabled'])))
        self.declare_parameter('intensity_scale',
                               float(config.get('intensity_scale', DEFAULTS['intensity_scale'])))
        self.declare_parameter('bias', [float(v) for v in config.get('bias', DEFAULTS['bias'])])
        self.declare_parameter('drift_stddev', [float(v) for v in drift['stddev']])
        self.declare_parameter('drift_time_constant', float(drift['time_constant']))
        self.declare_parameter('gust_stddev', [float(v) for v in gust['stddev']])
        self.declare_parameter('gust_time_constant', float(gust['time_constant']))

        seed = self.get_parameter('seed').get_parameter_value().integer_value
        # A negative seed means "don't care"; numpy then seeds from the OS.
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        self.drift = OrnsteinUhlenbeck(
            self.get_parameter('drift_stddev').value,
            self.get_parameter('drift_time_constant').value, self.rng)
        self.gust = OrnsteinUhlenbeck(
            self.get_parameter('gust_stddev').value,
            self.get_parameter('gust_time_constant').value, self.rng)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.current_pub = self.create_publisher(Vector3, CURRENT_TOPIC, 10)
        self.enable_srv = self.create_service(
            Empty, '/simulator/enable_current', self.handle_enable_current)
        self.disable_srv = self.create_service(
            Empty, '/simulator/disable_current', self.handle_disable_current)

        rate = float(config.get('update_rate', DEFAULTS['update_rate']))
        self.period = 1.0 / max(rate, 0.1)
        self.last_time = None
        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(
            f'Ocean current {"enabled" if self.enabled else "disabled"} '
            f'(seed {seed if seed >= 0 else "random"}, {rate:.1f} Hz) -> {CURRENT_TOPIC}')

    # -- configuration ------------------------------------------------------

    def _load_config(self, path):
        """Pull the `current:` block out of the competition config, if given."""
        if not path:
            return {}
        try:
            with open(path) as handle:
                loaded = yaml.safe_load(handle) or {}
        except OSError as exc:
            self.get_logger().warn(f'Could not read {path}: {exc}; using defaults.')
            return {}
        return loaded.get('current') or {}

    @property
    def enabled(self):
        return self.get_parameter('enabled').get_parameter_value().bool_value

    def _on_set_parameters(self, params):
        """Apply live retunes; the OU objects hold their own copies of the gains."""
        for param in params:
            if param.name == 'drift_stddev':
                self.drift.stddev = np.asarray(param.value, dtype=float)
            elif param.name == 'drift_time_constant':
                self.drift.tau = max(float(param.value), 1e-3)
            elif param.name == 'gust_stddev':
                self.gust.stddev = np.asarray(param.value, dtype=float)
            elif param.name == 'gust_time_constant':
                self.gust.tau = max(float(param.value), 1e-3)
            elif param.name == 'enabled' and not param.value:
                self.drift.reset()
                self.gust.reset()
        return SetParametersResult(successful=True)

    # -- services -----------------------------------------------------------

    def handle_enable_current(self, request, response):
        self.set_parameters([rclpy.Parameter('enabled', rclpy.Parameter.Type.BOOL, True)])
        self.get_logger().info('Ocean current enabled.')
        return response

    def handle_disable_current(self, request, response):
        self.set_parameters([rclpy.Parameter('enabled', rclpy.Parameter.Type.BOOL, False)])
        self.get_logger().info('Ocean current disabled.')
        return response

    # -- main loop ----------------------------------------------------------

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            # Sim time paused or stepped backwards; hold the water where it is.
            return

        if not self.enabled:
            # Publish zeros rather than going quiet: the plugin latches the last
            # value it received, so silence would leave the old current running.
            self.current_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
            return

        scale = self.get_parameter('intensity_scale').get_parameter_value().double_value
        bias = np.asarray(self.get_parameter('bias').value, dtype=float)
        velocity = bias + scale * (self.drift.step(dt) + self.gust.step(dt))

        self.current_pub.publish(
            Vector3(x=float(velocity[0]), y=float(velocity[1]), z=float(velocity[2])))


def main(args=None):
    rclpy.init(args=args)
    node = OceanCurrent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
